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
            if (snapshot == null || snapshot.Body == null || !IsFinite(snapshot.UT) ||
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
                var orbit = new Orbit();
                orbit.UpdateFromStateVectors(snapshot.Position, snapshot.Velocity, snapshot.Body, snapshot.UT);
                double surfaceRadius = snapshot.Body.Radius;

                if (!IsFinite(orbit.PeR) || orbit.PeR > surfaceRadius)
                {
                    return new LandingGuidanceV2Estimate(snapshot.Version, LandingGuidanceV2EstimateOutcome.NoImpact,
                        double.NaN, Vector3d.zero, Vector3d.zero, double.NaN,
                        "The snapshot orbit does not intersect the body's sea-level surface.");
                }

                double impactUT = orbit.NextTimeOfRadius(snapshot.UT, surfaceRadius);
                if (!IsFinite(impactUT) || impactUT < snapshot.UT)
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
                        Vector3d trialPosition = orbit.WorldBCIPositionAtUT(impactUT);
                        snapshot.Body.GetLatLngAltAtUT(impactUT, trialPosition, out double latitude, out double longitude, out _);
                        terrainAltitude = snapshot.Body.TerrainAltitude(latitude, longitude, true);
                        double terrainRadius = surfaceRadius + Math.Max(0, terrainAltitude);
                        double refinedUT = orbit.NextTimeOfRadius(snapshot.UT, terrainRadius);
                        if (!IsFinite(refinedUT) || refinedUT < snapshot.UT)
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

                Vector3d impactPosition = orbit.WorldBCIPositionAtUT(impactUT);
                Vector3d impactVelocity = orbit.WorldOrbitalVelocityAtUT(impactUT);
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
            double rotationDegrees = 360d * (ut - snapshot.TargetReferenceUT) / snapshot.Body.rotationPeriod;
            return Quaternion.AngleAxis((float)rotationDegrees, snapshot.Body.angularVelocity) * target;
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
