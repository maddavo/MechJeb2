using System;

namespace MuMech
{
    /// <summary>
    /// Applies fast snapshot lower-bound checks alongside the complete planner.
    /// It is diagnostic context only; the AirlessLandingPlan or
    /// AtmosphericLandingPlan is the V2 feasibility authority.
    /// </summary>
    public static class LandingGuidanceV2PreflightEvaluator
    {
        public static LandingGuidanceV2PreflightAssessment Evaluate(LandingGuidanceV2Snapshot snapshot,
            LandingGuidanceV2Estimate estimate, double brakingDeltaVLowerBound)
        {
            long version = snapshot?.Version ?? -1;
            if (snapshot == null || estimate == null)
                return New(version, LandingGuidanceV2PreflightState.WaitingForImpactTrajectory, double.NaN,
                    brakingDeltaVLowerBound, double.NaN, "A valid V2 snapshot and estimate are required.");

            if (snapshot.IsLandedOrSplashed)
                return New(version, LandingGuidanceV2PreflightState.NotFlight, double.NaN,
                    brakingDeltaVLowerBound, double.NaN, "Landed or splashed snapshots cannot form a flight plan.");

            if (snapshot.Body.atmosphere)
                return New(version, LandingGuidanceV2PreflightState.UnsupportedBody, double.NaN,
                    brakingDeltaVLowerBound, double.NaN, "Atmospheric lower-bound check is deferred to the entry-corridor plan.");

            if (!estimate.HasImpact)
                return New(version, LandingGuidanceV2PreflightState.WaitingForImpactTrajectory, double.NaN,
                    brakingDeltaVLowerBound, double.NaN, "A valid airless impact trajectory is required before planning.");

            double radiusSquared = snapshot.Position.sqrMagnitude;
            double localGravity = radiusSquared > 0 ? snapshot.Body.gravParameter / radiusSquared : double.NaN;
            double margin = snapshot.AvailableDeltaV - brakingDeltaVLowerBound;
            if (!Finite(snapshot.AvailableDeltaV) || !Finite(brakingDeltaVLowerBound) || !Finite(localGravity))
                return New(version, LandingGuidanceV2PreflightState.Rejected, localGravity, brakingDeltaVLowerBound,
                    margin, "Usable delta-V, braking lower bound, or local gravity is unavailable.");

            if (snapshot.MaximumAcceleration <= localGravity)
                return New(version, LandingGuidanceV2PreflightState.Rejected, localGravity, brakingDeltaVLowerBound,
                    margin, "Available thrust acceleration does not exceed local gravity.");

            if (margin < 0)
                return New(version, LandingGuidanceV2PreflightState.Rejected, localGravity, brakingDeltaVLowerBound,
                    margin, "Usable delta-V is below the inertial impact-velocity lower bound.");

            return New(version, LandingGuidanceV2PreflightState.NeedsCompletePlan, localGravity, brakingDeltaVLowerBound,
                margin, "Snapshot lower bounds pass; use the landing plan below for strategic, trim, reserve, and terminal feasibility.");
        }

        private static LandingGuidanceV2PreflightAssessment New(long version, LandingGuidanceV2PreflightState state,
            double gravity, double brakingLowerBound, double margin, string reason) =>
            new LandingGuidanceV2PreflightAssessment(version, state, gravity, brakingLowerBound, margin, reason);

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
