using System;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Keeps the selected target's terrain elevation fixed while its body-fixed
    /// reference vector is propagated to a future guidance epoch.
    /// </summary>
    internal static class AirlessTargetGeometry
    {
        // KSP's body-fixed longitude conversion (CelestialBodyExtensions) uses
        // this axis.  V2 must use that exact convention: a planner target that
        // cannot round-trip through GetLatLngAltAtUT is not the selected site.
        private static readonly Vector3d SurfaceFrameAxis = new Vector3d(0, -1, 0);

        internal static Vector3d ReferenceSurfacePosition(LandingGuidanceV2Snapshot snapshot)
        {
            Vector3d reference = snapshot.HasTargetReferencePosition
                ? snapshot.TargetReferencePosition
                : snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0) - snapshot.Body.position;
            if (reference.sqrMagnitude <= 1e-12) return reference;

            return reference.normalized * (snapshot.Body.Radius + TerrainAltitude(snapshot));
        }

        internal static Vector3d SurfacePositionAtUT(LandingGuidanceV2Snapshot snapshot, double ut)
        {
            Vector3d reference = ReferenceSurfacePosition(snapshot);
            if (reference.sqrMagnitude <= 1e-12 || ReferenceEquals(snapshot.Body, null) ||
                double.IsNaN(snapshot.Body.rotationPeriod) || double.IsInfinity(snapshot.Body.rotationPeriod) ||
                Math.Abs(snapshot.Body.rotationPeriod) <= 1e-12)
                return reference;

            double radians = 2.0 * Math.PI * (ut - snapshot.TargetReferenceUT) / snapshot.Body.rotationPeriod;
            return AirlessImpactEstimator.RotateAroundAxis(reference, SurfaceFrameAxis, radians);
        }

        // The inverse is deliberately exposed to the V2 harness. It represents
        // the same transformation used by GetCurrentSurfacePositionFromUT when
        // its current epoch is the captured reference epoch.
        internal static Vector3d SurfacePositionAtReferenceUT(LandingGuidanceV2Snapshot snapshot, Vector3d position,
            double positionUT) => AirlessImpactEstimator.RotateAroundAxis(position, SurfaceFrameAxis,
            -2.0 * Math.PI * (positionUT - snapshot.TargetReferenceUT) / snapshot.Body.rotationPeriod);

        internal static double TerrainAltitude(LandingGuidanceV2Snapshot snapshot)
        {
            if (!double.IsNaN(snapshot.TargetTerrainAltitude) && !double.IsInfinity(snapshot.TargetTerrainAltitude))
                return Math.Max(0, snapshot.TargetTerrainAltitude);
            try
            {
                double terrain = snapshot.Body.TerrainAltitude(snapshot.TargetLatitude, snapshot.TargetLongitude, true);
                return double.IsNaN(terrain) || double.IsInfinity(terrain) ? 0 : Math.Max(0, terrain);
            }
            catch (Exception)
            {
                // Pure recorded-snapshot tests may not have a PQS terrain
                // provider. Sea level remains the deterministic fallback.
                return 0;
            }
        }
    }
}
