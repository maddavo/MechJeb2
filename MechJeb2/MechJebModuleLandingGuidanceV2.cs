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
        private long _v1PredictionVersion;
        private ReentrySimulation.Result _lastV1Prediction;
        private string _lastV1Phase;
        private bool? _lastV1Burning;
        private bool? _lastWarped;

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
            WriteCorrelatedTrace(Preflight);
        }

        private LandingGuidanceV2Snapshot CaptureSnapshot()
        {
            Core.StageStats.RequestUpdate();
            double availableDeltaV = Core.StageStats.VacStats.Sum(s => s.DeltaV);

            return new LandingGuidanceV2Snapshot(++_snapshotVersion, VesselState.Time, MainBody,
                VesselState.OrbitalPosition, VesselState.OrbitalVelocity, VesselState.Mass, availableDeltaV,
                VesselState.LimitedMaxThrustAcceleration, Core.Target.targetLatitude, Core.Target.targetLongitude);
        }

        private void WriteCorrelatedTrace(LandingGuidanceV2Preflight preflight)
        {
            if (!StructuredTraceEnabled || preflight?.Snapshot == null || preflight.Estimate == null)
                return;

            try
            {
                string path = MuUtils.GetCfgPath("LandingGuidanceV2.trace.jsonl");
                MuUtils.FileExistsCreateDirectory(path);
                LandingGuidanceV2Snapshot snapshot = preflight.Snapshot;
                LandingGuidanceV2Estimate estimate = preflight.Estimate;
                MechJebModuleLandingAutopilot landing = Core.Landing;
                AutopilotStep step = landing?.CurrentStep;
                string phase = step?.GetType().Name;
                string status = landing?.Status;
                double warpRate = TimeWarp.CurrentRate;

                ReentrySimulation.Result prediction = Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Result;
                if (!ReferenceEquals(prediction, _lastV1Prediction))
                {
                    _lastV1Prediction = prediction;
                    _v1PredictionVersion++;
                }

                string predictionOutcome = prediction?.Outcome.ToString();
                double predictionLat = prediction == null ? double.NaN : prediction.EndPosition.Latitude;
                double predictionLon = prediction == null ? double.NaN : prediction.EndPosition.Longitude;
                double predictionError = prediction == null || prediction.Body == null || !Core.Target.PositionTargetExists || Core.Target.targetBody != prediction.Body
                    ? double.NaN
                    : Vector3d.Distance(
                        prediction.Body.GetWorldSurfacePosition(predictionLat, predictionLon, 0),
                        prediction.Body.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0));

                bool landed = Vessel != null && Vessel.LandedOrSplashed;
                bool estimateApplicable = !landed && !snapshot.Body.atmosphere;
                string validityReason = landed ? "landed_or_splashed" : snapshot.Body.atmosphere ? "atmospheric_estimator_not_applicable" : "flight_snapshot";
                Vector3d rcsCommand = Vessel?.ctrlState == null ? Vector3d.zero : new Vector3d(Vessel.ctrlState.X, Vessel.ctrlState.Y, Vessel.ctrlState.Z);
                bool mechjebRcsEnabled = Core.RCS != null && Core.RCS.Enabled;
                bool rcsActionGroupEnabled = Vessel != null && Vessel.ActionGroups[KSPActionGroup.RCS];
                double commandedThrottle = Core.Thrust?.LastThrottle ?? 0;
                double actualThrustAcceleration = VesselState.Mass > 0
                    ? VesselState.ThrustVectorLastFrame.magnitude / VesselState.Mass
                    : double.NaN;
                bool burning = commandedThrottle > 0.001 || actualThrustAcceleration > 0.001;
                string v1Phase = JsonString(phase);
                string v1Status = JsonString(status);
                string predOutcome = JsonString(predictionOutcome);
                string baseFields = string.Format(CultureInfo.InvariantCulture,
                    "\"snapshotVersion\":{0},\"ut\":{1},\"body\":\"{2}\",\"targetLat\":{3},\"targetLon\":{4}," +
                    "\"position\":[{5},{6},{7}],\"velocity\":[{8},{9},{10}],\"mass\":{11}," +
                    "\"availableDeltaV\":{12},\"maxAcceleration\":{13},\"outcome\":\"{14}\",\"impactUT\":{15}," +
                    "\"targetError\":{16},\"brakingDeltaVLowerBound\":{17},\"deltaVAboveLowerBound\":{18}," +
                    "\"v1Phase\":{19},\"v1Status\":{20},\"v1PredictionVersion\":{21},\"v1PredictionOutcome\":{22}," +
                    "\"v1PredictionEndLat\":{23},\"v1PredictionEndLon\":{24},\"v1PredictionEndUT\":{25},\"v1TargetError\":{26}," +
                    "\"warpRate\":{27},\"attitudeErrorDegrees\":{28},\"commandedThrottle\":{29},\"flightControlThrottle\":{30}," +
                    "\"actualThrustAcceleration\":{31},\"forwardThrustAcceleration\":{32},\"mechjebRcsEnabled\":{33}," +
                    "\"rcsActionGroupEnabled\":{34},\"rcsCommand\":[{35},{36},{37}],\"burnActive\":{38}," +
                    "\"isLandedOrSplashed\":{39},\"estimatorApplicable\":{40},\"flightDataValid\":{41},\"validityReason\":{42}",
                    snapshot.Version, JsonNumber(snapshot.UT), EscapeJson(snapshot.Body.bodyName), JsonNumber(snapshot.TargetLatitude),
                    JsonNumber(snapshot.TargetLongitude), JsonNumber(snapshot.Position.x), JsonNumber(snapshot.Position.y),
                    JsonNumber(snapshot.Position.z), JsonNumber(snapshot.Velocity.x), JsonNumber(snapshot.Velocity.y),
                    JsonNumber(snapshot.Velocity.z), JsonNumber(snapshot.Mass), JsonNumber(snapshot.AvailableDeltaV),
                    JsonNumber(snapshot.MaximumAcceleration), estimate.Outcome, JsonNumber(estimate.ImpactUT), JsonNumber(estimate.TargetError),
                    JsonNumber(preflight.BrakingDeltaVLowerBound), JsonNumber(preflight.DeltaVAboveLowerBound), v1Phase, v1Status,
                    _v1PredictionVersion, predOutcome, JsonNumber(predictionLat), JsonNumber(predictionLon),
                    JsonNumber(prediction?.EndUT ?? double.NaN), JsonNumber(predictionError), JsonNumber(warpRate),
                    JsonNumber(Core.Attitude?.attitudeError ?? double.NaN), JsonNumber(commandedThrottle),
                    JsonNumber(Vessel?.ctrlState?.mainThrottle ?? float.NaN), JsonNumber(actualThrustAcceleration),
                    JsonNumber(VesselState.CurrentThrustAcceleration), mechjebRcsEnabled ? "true" : "false",
                    rcsActionGroupEnabled ? "true" : "false", JsonNumber(rcsCommand.x), JsonNumber(rcsCommand.y),
                    JsonNumber(rcsCommand.z), burning ? "true" : "false",
                    landed ? "true" : "false", estimateApplicable ? "true" : "false", !landed ? "true" : "false", JsonString(validityReason));

                var lines = new System.Collections.Generic.List<string>();
                if (_lastV1Phase != phase)
                    lines.Add(TraceRecord(baseFields, "phase_transition", _lastV1Phase, phase));
                if (_lastV1Burning != burning && (_lastV1Burning.HasValue || burning))
                    lines.Add(TraceRecord(baseFields, burning ? "burn_start" : "burn_end", _lastV1Burning.HasValue && _lastV1Burning.Value ? "burning" : "not_burning", burning ? "burning" : "not_burning"));
                bool warped = warpRate > 1.0;
                if (_lastWarped != warped && (_lastWarped.HasValue || warped))
                    lines.Add(TraceRecord(baseFields, warped ? "warp_enter" : "warp_exit", _lastWarped.HasValue && _lastWarped.Value ? "warped" : "1x", warped ? "warped" : "1x"));
                _lastV1Phase = phase;
                _lastV1Burning = burning;
                _lastWarped = warped;
                lines.Add("{\"recordType\":\"sample\"," + baseFields + "}");
                File.AppendAllText(path, string.Join(Environment.NewLine, lines) + Environment.NewLine);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[MechJebLandingGuidanceV2] Could not write structured trace: " + ex.GetType().Name);
            }
        }

        private static string TraceRecord(string fields, string eventName, string from, string to) =>
            "{\"recordType\":\"event\",\"event\":\"" + EscapeJson(eventName) + "\",\"eventFrom\":" + JsonString(from) +
            ",\"eventTo\":" + JsonString(to) + "," + fields + "}";

        private static string JsonString(string value) => value == null ? "null" : "\"" + EscapeJson(value) + "\"";

        private static string EscapeJson(string value) => (value ?? string.Empty)
            .Replace("\\", "\\\\")
            .Replace("\"", "\\\"")
            .Replace("\r", "\\r")
            .Replace("\n", "\\n")
            .Replace("\t", "\\t");

        private static string JsonNumber(double value) => double.IsNaN(value) || double.IsInfinity(value)
            ? "null"
            : value.ToString("F6", CultureInfo.InvariantCulture);
    }
}
