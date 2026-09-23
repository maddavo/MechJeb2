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
        internal static Vector3d ReferenceSurfacePosition(LandingGuidanceV2Snapshot snapshot)
        {
            Vector3d reference = snapshot.HasTargetReferencePosition
                ? snapshot.TargetReferencePosition
                : snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0) - snapshot.Body.position;
            if (reference.sqrMagnitude <= 1e-12) return reference;

            return reference.normalized * (snapshot.Body.Radius + TerrainAltitude(snapshot));
        }

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
