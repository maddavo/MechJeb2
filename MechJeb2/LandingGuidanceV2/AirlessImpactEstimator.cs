using System;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Computes the next sea-level intersection of an airless body's two-body orbit.
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
                double surfaceRadius = snapshot.Body.Radius;

                // A transfer that is constructed to meet the sea-level surface
                // can differ from that radius by floating-point roundoff.  Do
                // not reject a valid tangent/intersection before the actual
                // next-radius-crossing solver has evaluated it.
                if (!IsFinite(orbit.PeriapsisRadius) || orbit.PeriapsisRadius > surfaceRadius + 0.01)
                {
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The snapshot orbit does not intersect the body's sea-level surface.");
                }

                if (!orbit.TryNextRadiusCrossing(snapshot.UT, surfaceRadius, out double impactUT) ||
                    !IsFinite(impactUT) || impactUT < snapshot.UT)
                {
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The next surface-intersection time is not valid.");
                }

                // Terrain is refined from the deterministic conic solution in a
                // bounded fixed iteration count.  It improves the terminal
                // altitude without allowing a mutable terrain query to feed an
                // unbounded predictor loop.
                double terrainAltitude = 0;
                bool terrainRefined = false;
                for (int i = 0; i < 4; ++i)
                {
                    try
                    {
                        if (!orbit.TryStateAt(impactUT, out Vector3d trialPosition, out _)) break;
                        snapshot.Body.GetLatLngAltAtUT(impactUT, trialPosition, out double latitude, out double longitude, out _);
                        terrainAltitude = snapshot.Body.TerrainAltitude(latitude, longitude, true);
                        double terrainRadius = surfaceRadius + Math.Max(0, terrainAltitude);
                        if (!orbit.TryNextRadiusCrossing(snapshot.UT, terrainRadius, out double refinedUT) ||
                            !IsFinite(refinedUT) || refinedUT < snapshot.UT)
                            break;
                        terrainRefined = true;
                        if (Math.Abs(refinedUT - impactUT) < 0.01)
                        {
                            impactUT = refinedUT;
                            break;
                        }
                        impactUT = refinedUT;
                    }
                    catch (Exception)
                    {
                        terrainAltitude = double.NaN;
                        break;
                    }
                }

                if (!orbit.TryStateAt(impactUT, out Vector3d impactPosition, out Vector3d impactVelocity))
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The conic could not propagate to its surface intersection.");
                Vector3d targetPosition = TargetPositionAtUT(snapshot, impactUT);
                double targetError = Vector3d.Distance(impactPosition, targetPosition);

                return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.Impact, impactUT,
                    impactPosition, impactVelocity, targetError,
                    terrainRefined
                        ? "Deterministic airless conic intersection with bounded terrain refinement."
                        : "Deterministic airless conic intersection; terrain refinement was unavailable.", terrainAltitude);
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
            Vector3d target = snapshot.HasTargetReferencePosition ? snapshot.TargetReferencePosition :
                snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0) - snapshot.Body.position;
            // Planning states are propagated to future burns.  The Unity surface
            // query above is anchored to the immutable target reference time,
            // not to a future state-vector epoch.  Using snapshot.UT here made
            // every future candidate score its impact against a target displaced
            // by the coast to that candidate, which can reject an otherwise
            // reachable plane-alignment and deorbit sequence.
            double rotationRadians = 2.0 * Math.PI * (ut - snapshot.TargetReferenceUT) / snapshot.Body.rotationPeriod;
            return RotateAroundAxis(target, snapshot.Body.angularVelocity, rotationRadians);
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
