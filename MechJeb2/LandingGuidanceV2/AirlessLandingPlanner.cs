using System;
using UnityEngine;

namespace MuMech
{
    /// <summary>Builds one finite airless strategic-deorbit candidate without actuator authority.</summary>
    public static class AirlessLandingPlanner
    {
        public static AirlessLandingPlan Plan(LandingGuidanceV2Snapshot snapshot)
        {
            if (snapshot == null || snapshot.Body == null || snapshot.IsLandedOrSplashed || snapshot.Body.atmosphere)
                return Reject(snapshot, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null,
                    "An airborne airless snapshot is required.");

            try
            {
                var orbit = new Orbit();
                orbit.UpdateFromStateVectors(snapshot.Position, snapshot.Velocity, snapshot.Body, snapshot.UT);
                Vector3d up = snapshot.Position.normalized;
                Vector3d horizontalVelocity = Vector3d.Exclude(up, snapshot.Velocity);
                Vector3d periapsisBurn = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, snapshot.UT,
                    snapshot.Body.Radius * 0.9);
                Orbit trialOrbit = orbit.PerturbedOrbit(snapshot.UT, periapsisBurn);
                double trialImpactUT = trialOrbit.NextTimeOfRadius(snapshot.UT, snapshot.Body.Radius);
                if (!Finite(trialImpactUT) || trialImpactUT < snapshot.UT)
                    return Reject(snapshot, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null,
                        "The initial deorbit geometry does not reach the sea-level surface.");

                Vector3d targetDirection = Vector3d.Exclude(up, TargetPositionAtUT(snapshot, trialImpactUT) - snapshot.Position);
                if (targetDirection.sqrMagnitude < 1e-9)
                    return Reject(snapshot, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null,
                        "The target direction is degenerate at this planning epoch.");

                Vector3d desiredHorizontalVelocity = (horizontalVelocity + periapsisBurn).magnitude * targetDirection.normalized;
                Vector3d strategicBurn = desiredHorizontalVelocity - horizontalVelocity;
                var candidateSnapshot = new LandingGuidanceV2Snapshot(snapshot.Version, snapshot.UT, snapshot.Body,
                    snapshot.Position, snapshot.Velocity + strategicBurn, snapshot.Mass, snapshot.AvailableDeltaV,
                    snapshot.MaximumAcceleration, snapshot.TargetLatitude, snapshot.TargetLongitude, false);
                LandingGuidanceV2Estimate candidate = AirlessImpactEstimator.Estimate(candidateSnapshot);
                if (!candidate.HasImpact)
                    return Reject(snapshot, strategicBurn, double.NaN, double.NaN, double.NaN, double.NaN, candidate,
                        "The strategic deorbit candidate has no valid impact trajectory.");

                Vector3d delta = candidate.ImpactPosition - TargetPositionAtUT(snapshot, candidate.ImpactUT);
                Vector3d impactUp = candidate.ImpactPosition.normalized;
                Vector3d downrangeDirection = Vector3d.Exclude(impactUp, candidate.ImpactVelocity).normalized;
                double signedDownrange = Vector3d.Dot(delta, downrangeDirection);
                double crossRange = Math.Sqrt(Math.Max(0, delta.sqrMagnitude - signedDownrange * signedDownrange));
                double corridorLimit = Math.Max(100, snapshot.Body.Radius * 0.002);
                double terminalLowerBound = candidate.ImpactVelocity.magnitude;

                if (signedDownrange < 0)
                    return Reject(snapshot, strategicBurn, terminalLowerBound, signedDownrange, crossRange, corridorLimit,
                        candidate, "Candidate is on the short side of the target.");
                if (signedDownrange > corridorLimit || crossRange > corridorLimit)
                    return Reject(snapshot, strategicBurn, terminalLowerBound, signedDownrange, crossRange, corridorLimit,
                        candidate, "Candidate is outside the permitted long-side or cross-range corridor.");
                if (snapshot.AvailableDeltaV < strategicBurn.magnitude + terminalLowerBound)
                    return Reject(snapshot, strategicBurn, terminalLowerBound, signedDownrange, crossRange, corridorLimit,
                        candidate, "Usable delta-V is below the strategic-deorbit plus impact-cancellation lower bound.");

                return new AirlessLandingPlan(snapshot.Version, AirlessLandingPlanState.Candidate, strategicBurn,
                    terminalLowerBound, signedDownrange, crossRange, corridorLimit, candidate, snapshot.AvailableDeltaV,
                    "Candidate meets the current long-side corridor and lower-bound budget; trim, reserve, and terminal profile remain unplanned.");
            }
            catch (Exception ex)
            {
                return Reject(snapshot, Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null,
                    "Airless strategic-deorbit planning failed: " + ex.GetType().Name);
            }
        }

        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot snapshot, Vector3d burn, double terminalLowerBound,
            double downrange, double crossRange, double corridorLimit, LandingGuidanceV2Estimate estimate, string reason) =>
            new AirlessLandingPlan(snapshot?.Version ?? -1, AirlessLandingPlanState.Rejected, burn, terminalLowerBound,
                downrange, crossRange, corridorLimit, estimate, snapshot?.AvailableDeltaV ?? double.NaN, reason);

        private static Vector3d TargetPositionAtUT(LandingGuidanceV2Snapshot snapshot, double ut)
        {
            Vector3d target = snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0) - snapshot.Body.position;
            double rotationDegrees = 360d * (ut - snapshot.UT) / snapshot.Body.rotationPeriod;
            return Quaternion.AngleAxis((float)rotationDegrees, snapshot.Body.angularVelocity) * target;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
