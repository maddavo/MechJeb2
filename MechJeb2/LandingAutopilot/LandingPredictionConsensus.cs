using System;

namespace MuMech.Landing
{
    /// <summary>
    /// Keeps the V1 controller from reacting to two different impact branches.
    /// The simulator can occasionally return a geometrically nearby body-space
    /// endpoint while the terrain query or impact epoch has jumped to another
    /// surface feature.  That is not a control-quality result: V1 must wait
    /// until consecutive simulations agree on both the endpoint and arrival.
    /// </summary>
    public static class LandingPredictionConsensus
    {
        public static bool Agrees(ReentrySimulation.Result first, ReentrySimulation.Result second,
            double positionTolerance, double bodySpaceDistance, bool airlessTerrainProfile = false)
        {
            if (first == null || second == null || first.Body != second.Body || double.IsNaN(positionTolerance) || positionTolerance < 0)
                return false;

            if (double.IsNaN(bodySpaceDistance) || double.IsInfinity(bodySpaceDistance) ||
                bodySpaceDistance > positionTolerance)
                return false;

            // A terrain-profile endpoint can move along a local slope by one
            // trajectory sample even though the vacuum coast is continuous.
            // The profile branch still has to remain spatially local, but its
            // contact time and height have a wider, explicitly bounded window.
            // Atmospheric predictions retain the original strict rule.
            double inputDelta = Math.Abs(second.InputUT - first.InputUT);
            double arrivalTolerance = airlessTerrainProfile
                ? Math.Max(20.0, 4.0 * inputDelta + 0.01 * positionTolerance)
                : Math.Max(2.0, 4.0 * inputDelta + 0.01 * positionTolerance);
            if (Math.Abs(second.EndUT - first.EndUT) > arrivalTolerance)
                return false;

            // EndASL is obtained on the flight thread after the simulation.
            // A large disagreement here identifies exactly the flat-ground /
            // mountain branch flip observed on Minmus, even if a body-space
            // conversion reports the endpoints as deceptively close.
            if (IsFinite(first.EndASL) && IsFinite(second.EndASL))
            {
                double terrainTolerance = airlessTerrainProfile
                    ? Math.Max(250.0, 0.10 * positionTolerance)
                    : Math.Max(10.0, 0.10 * positionTolerance);
                if (Math.Abs(second.EndASL - first.EndASL) > terrainTolerance)
                    return false;
            }

            return true;
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}