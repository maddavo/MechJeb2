using System;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Computes the next intersection with the selected airless target's captured terrain radius.
    /// A new Orbit is reconstructed from the immutable snapshot for every call.  This
    /// prevents the result from inheriting mutable state from KSP's patched orbit or a
    /// previous prediction result.
    /// </summary>
    public static class AirlessImpactEstimator
    {
        public static LandingGuidanceV2Estimate Estimate(LandingGuidanceV2Snapshot snapshot)
        {
            if (snapshot == null || ReferenceEquals(snapshot.Body, null) || !IsFinite(snapshot.UT) ||
                !IsFinite(snapshot.Position) || !IsFinite(snapshot.Velocity) || snapshot.Position.sqrMagnitude <= 0)
            {
                return new LandingGuidanceV2Estimate(snapshot?.Version ?? -1,
                    LandingGuidanceV2EstimateOutcome.InvalidSnapshot, double.NaN, Vector3d.zero, Vector3d.zero,
                    double.NaN, "Snapshot did not contain a valid inertial state.");
            }

            if (snapshot.IsLandedOrSplashed)
            {
                return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NotFlight,
                    double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                    "Landed or splashed snapshots are non-flight data and are not ballistic estimates.");
            }

            if (snapshot.Body.atmosphere)
            {
                return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.AtmosphericBody,
                    double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                    "Atmospheric bodies require the separate V2 atmospheric estimator.");
            }

            try
            {
                var orbit = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT,
                    snapshot.Position, snapshot.Velocity);
                // The strategic planner and the estimator must describe the
                // same contact surface.  The target terrain elevation is captured
                // once with the immutable snapshot; querying whatever terrain lies
                // under a provisional miss makes repeated planning depend on live
                // KSP state and makes a selected elevated site impossible to hit.
                double terrainAltitude = AirlessTargetGeometry.TerrainAltitude(snapshot);
                double surfaceRadius = snapshot.Body.Radius + terrainAltitude;

                // A transfer constructed to meet the selected site's terrain
                // radius can differ by floating-point roundoff. Do not reject a
                // valid tangent/intersection before the crossing solver runs.
                if (!IsFinite(orbit.PeriapsisRadius) || orbit.PeriapsisRadius > surfaceRadius + 0.01)
                {
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The snapshot orbit does not intersect the selected target terrain radius.");
                }

                if (!orbit.TryNextRadiusCrossing(snapshot.UT, surfaceRadius, out double impactUT) ||
                    !IsFinite(impactUT) || impactUT < snapshot.UT)
                {
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The next selected-target terrain intersection time is not valid.");
                }

                if (!orbit.TryStateAt(impactUT, out Vector3d impactPosition, out Vector3d impactVelocity))
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The conic could not propagate to its surface intersection.");
                Vector3d targetPosition = TargetPositionAtUT(snapshot, impactUT);
                double targetError = Vector3d.Distance(impactPosition, targetPosition);

                return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.Impact, impactUT,
                    impactPosition, impactVelocity, targetError,
                    "Deterministic airless conic intersection at the captured selected-target terrain radius.",
                    terrainAltitude);
            }
            catch (Exception ex)
            {
                return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.InvalidSnapshot,
                    double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                    "Airless impact estimate failed: " + ex.GetType().Name);
            }
        }

        public static LandingGuidanceV2EstimatorValidation ValidateDeterminism(LandingGuidanceV2Snapshot snapshot,
            LandingGuidanceV2Estimate firstEstimate)
        {
            LandingGuidanceV2Estimate repeatedEstimate = Estimate(snapshot);
            bool deterministic = Equivalent(firstEstimate, repeatedEstimate);
            return new LandingGuidanceV2EstimatorValidation(repeatedEstimate, deterministic,
                deterministic
                    ? "Repeated evaluation of the same immutable snapshot matched."
                    : "Repeated evaluation of the same immutable snapshot differed.");
        }

        private static Vector3d TargetPositionAtUT(LandingGuidanceV2Snapshot snapshot, double ut)
        {
            return AirlessTargetGeometry.SurfacePositionAtUT(snapshot, ut);
        }

        internal static Vector3d RotateAroundAxis(Vector3d vector, Vector3d axis, double radians)
        {
            if (axis.sqrMagnitude < 1e-12) return vector;
            axis.Normalize();
            double cosine = Math.Cos(radians);
            double sine = Math.Sin(radians);
            return vector * cosine + Vector3d.Cross(axis, vector) * sine + axis * Vector3d.Dot(axis, vector) * (1.0 - cosine);
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private static bool IsFinite(Vector3d value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);

        private static bool Equivalent(LandingGuidanceV2Estimate first, LandingGuidanceV2Estimate second) =>
            first != null && second != null &&
            first.SnapshotVersion == second.SnapshotVersion &&
            first.Outcome == second.Outcome &&
            SameNumber(first.ImpactUT, second.ImpactUT) &&
            SameVector(first.ImpactPosition, second.ImpactPosition) &&
            SameVector(first.ImpactVelocity, second.ImpactVelocity) &&
            SameNumber(first.TargetError, second.TargetError) &&
            SameNumber(first.TerrainAltitude, second.TerrainAltitude);

        private static bool SameVector(Vector3d first, Vector3d second) =>
            SameNumber(first.x, second.x) && SameNumber(first.y, second.y) && SameNumber(first.z, second.z);

        private static bool SameNumber(double first, double second) =>
            first == second || (double.IsNaN(first) && double.IsNaN(second));
    }
}
