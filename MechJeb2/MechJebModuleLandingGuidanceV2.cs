extern alias JetBrainsAnnotations;
using System;
using System.Globalization;
using System.IO;
using System.Linq;
using JetBrainsAnnotations::JetBrains.Annotations;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// V2 foundation module.  It is intentionally passive: it captures snapshots,
    /// estimates airless impacts, assesses a delta-V lower bound, and emits opt-in
    /// structured trace records.  It does not own warp, attitude, RCS, or throttle.
    /// </summary>
    public class MechJebModuleLandingGuidanceV2 : ComputerModule
    {
        private const double RefreshInterval = 0.5;
        private double _nextRefreshUT;
        private long _snapshotVersion;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool PreviewEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool StructuredTraceEnabled;

        public LandingGuidanceV2Preflight Preflight { get; private set; }

        public bool IsPreviewOnly => true;

        public MechJebModuleLandingGuidanceV2(MechJebCore core) : base(core)
        {
            Enabled = true;
        }

        public override void OnFixedUpdate()
        {
            if (!PreviewEnabled || !HighLogic.LoadedSceneIsFlight || !Core.Target.PositionTargetExists)
                return;

            if (VesselState.Time < _nextRefreshUT)
                return;

            _nextRefreshUT = VesselState.Time + RefreshInterval;
            RefreshPreflight();
        }

        public void RefreshPreflight()
        {
            if (!HighLogic.LoadedSceneIsFlight || !Core.Target.PositionTargetExists || Vessel == null || MainBody == null)
            {
                Preflight = null;
                return;
            }

            LandingGuidanceV2Snapshot snapshot = CaptureSnapshot();
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(snapshot);

            // Cancelling the predicted inertial impact velocity is a physical lower bound,
            // not a landing feasibility claim.  A real plan must additionally account for
            // gravity losses, finite-throttle constraints, terrain, trims, and reserve.
            double brakingLowerBound = estimate.HasImpact ? estimate.ImpactVelocity.magnitude : double.NaN;
            Preflight = new LandingGuidanceV2Preflight(snapshot, estimate, brakingLowerBound);
            WriteTrace(Preflight);
        }

        private LandingGuidanceV2Snapshot CaptureSnapshot()
        {
            Core.StageStats.RequestUpdate();
            double availableDeltaV = Core.StageStats.VacStats.Sum(s => s.DeltaV);

            return new LandingGuidanceV2Snapshot(++_snapshotVersion, VesselState.Time, MainBody,
                VesselState.OrbitalPosition, VesselState.OrbitalVelocity, VesselState.Mass, availableDeltaV,
                VesselState.LimitedMaxThrustAcceleration, Core.Target.targetLatitude, Core.Target.targetLongitude);
        }

        private void WriteTrace(LandingGuidanceV2Preflight preflight)
        {
            if (!StructuredTraceEnabled || preflight?.Snapshot == null || preflight.Estimate == null)
                return;

            try
            {
                string path = MuUtils.GetCfgPath("LandingGuidanceV2.trace.jsonl");
                MuUtils.FileExistsCreateDirectory(path);
                LandingGuidanceV2Snapshot snapshot = preflight.Snapshot;
                LandingGuidanceV2Estimate estimate = preflight.Estimate;
                string line = string.Format(CultureInfo.InvariantCulture,
                    "{{\"snapshotVersion\":{0},\"ut\":{1},\"body\":\"{2}\",\"targetLat\":{3},\"targetLon\":{4}," +
                    "\"position\":[{5},{6},{7}],\"velocity\":[{8},{9},{10}]," +
                    "\"availableDeltaV\":{11},\"maxAcceleration\":{12},\"outcome\":\"{13}\",\"impactUT\":{14}," +
                    "\"targetError\":{15},\"brakingDeltaVLowerBound\":{16},\"deltaVAboveLowerBound\":{17}}}",
                    snapshot.Version, JsonNumber(snapshot.UT), EscapeJson(snapshot.Body.bodyName), JsonNumber(snapshot.TargetLatitude),
                    JsonNumber(snapshot.TargetLongitude), JsonNumber(snapshot.Position.x), JsonNumber(snapshot.Position.y),
                    JsonNumber(snapshot.Position.z), JsonNumber(snapshot.Velocity.x), JsonNumber(snapshot.Velocity.y),
                    JsonNumber(snapshot.Velocity.z), JsonNumber(snapshot.AvailableDeltaV), JsonNumber(snapshot.MaximumAcceleration),
                    estimate.Outcome, JsonNumber(estimate.ImpactUT), JsonNumber(estimate.TargetError),
                    JsonNumber(preflight.BrakingDeltaVLowerBound), JsonNumber(preflight.DeltaVAboveLowerBound));
                File.AppendAllText(path, line + Environment.NewLine);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[MechJebLandingGuidanceV2] Could not write structured trace: " + ex.GetType().Name);
            }
        }

        private static string EscapeJson(string value) => (value ?? string.Empty).Replace("\\", "\\\\").Replace("\"", "\\\"");

        private static string JsonNumber(double value) => double.IsNaN(value) || double.IsInfinity(value)
            ? "null"
            : value.ToString("F6", CultureInfo.InvariantCulture);
    }
}
