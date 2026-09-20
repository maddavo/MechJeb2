using System;
using UnityEngine;

namespace MuMech
{
    public static class AirlessLandingPlanner
    {
        public static AirlessLandingPlan Plan(LandingGuidanceV2Snapshot s)
        {
            if (s == null || s.Body == null || s.IsLandedOrSplashed || s.Body.atmosphere)
                return Reject(s, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null, "An airborne airless snapshot is required.");
            try
            {
                var o = new Orbit(); o.UpdateFromStateVectors(s.Position, s.Velocity, s.Body, s.UT);
                Vector3d up = s.Position.normalized, horizontal = Vector3d.Exclude(up, s.Velocity);
                Vector3d periapsisBurn = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(o, s.UT, s.Body.Radius * 0.9);
                Orbit trial = o.PerturbedOrbit(s.UT, periapsisBurn);
                double trialImpactUT = trial.NextTimeOfRadius(s.UT, s.Body.Radius);
                if (!Finite(trialImpactUT) || trialImpactUT < s.UT)
                    return Reject(s, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null, "The initial deorbit geometry does not reach the sea-level surface.");
                Vector3d direction = Vector3d.Exclude(up, TargetAt(s, trialImpactUT) - s.Position);
                if (direction.sqrMagnitude < 1e-9)
                    return Reject(s, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null, "The target direction is degenerate at this planning epoch.");
                Vector3d burn = (horizontal + periapsisBurn).magnitude * direction.normalized - horizontal;
                var candidateSnapshot = new LandingGuidanceV2Snapshot(s.Version, s.UT, s.Body, s.Position, s.Velocity + burn,
                    s.Mass, s.AvailableDeltaV, s.MaximumAcceleration, s.TargetLatitude, s.TargetLongitude, false);
                LandingGuidanceV2Estimate e = AirlessImpactEstimator.Estimate(candidateSnapshot);
                if (!e.HasImpact) return Reject(s, burn, double.NaN, double.NaN, double.NaN, double.NaN, e, "The strategic deorbit candidate has no valid impact trajectory.");
                Vector3d delta = e.ImpactPosition - TargetAt(s, e.ImpactUT);
                Vector3d down = Vector3d.Exclude(e.ImpactPosition.normalized, e.ImpactVelocity).normalized;
                double downrange = Vector3d.Dot(delta, down), cross = Math.Sqrt(Math.Max(0, delta.sqrMagnitude - downrange * downrange));
                double limit = Math.Max(100, s.Body.Radius * 0.002), terminal = e.ImpactVelocity.magnitude;
                if (downrange < 0) return Reject(s, burn, terminal, downrange, cross, limit, e, "Candidate is on the short side of the target.");
                if (downrange > limit || cross > limit) return Reject(s, burn, terminal, downrange, cross, limit, e, "Candidate is outside the permitted long-side or cross-range corridor.");
                if (s.AvailableDeltaV < burn.magnitude + terminal) return Reject(s, burn, terminal, downrange, cross, limit, e, "Usable delta-V is below the strategic-deorbit plus impact-cancellation lower bound.");
                return new AirlessLandingPlan(s.Version, AirlessLandingPlanState.Candidate, burn, terminal, downrange, cross, limit, e, s.AvailableDeltaV, "Candidate meets the current long-side corridor and lower-bound budget; trim, reserve, and terminal profile remain unplanned.");
            }
            catch (Exception ex) { return Reject(s, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null, "Airless strategic-deorbit planning failed: " + ex.GetType().Name); }
        }
        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot s, Vector3d burn, double terminal, double down, double cross, double limit, LandingGuidanceV2Estimate e, string reason) =>
            new AirlessLandingPlan(s?.Version ?? -1, AirlessLandingPlanState.Rejected, burn, terminal, down, cross, limit, e, s?.AvailableDeltaV ?? double.NaN, reason);
        private static Vector3d TargetAt(LandingGuidanceV2Snapshot s, double ut)
        {
            Vector3d target = s.Body.GetWorldSurfacePosition(s.TargetLatitude, s.TargetLongitude, 0) - s.Body.position;
            return Quaternion.AngleAxis((float)(360d * (ut - s.UT) / s.Body.rotationPeriod), s.Body.angularVelocity) * target;
        }
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
