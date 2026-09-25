using System;

namespace MuMech.Landing
{
    /// <summary>
    /// V1's re-entry simulator represents terrain as one spherical contact
    /// radius. On an airless body that radius must be a fixed point: the
    /// terrain height supplied to a simulation must match the terrain height
    /// at its simulated endpoint.  Publishing a result before that condition
    /// is met creates a flat-ground/mountain feedback loop.
    /// </summary>
    public static class LandingPredictionTerrainConvergence
    {
        // Terrain queries have metre-scale numerical noise. This is small
        // compared with the kilometre-scale branch flip seen on Minmus while
        // avoiding needless re-runs for an otherwise identical endpoint.
        public const double ToleranceMetres = 2.0;

        public static bool IsSelfConsistent(double simulatedTerrainAltitude, double endpointTerrainAltitude)
        {
            return Finite(simulatedTerrainAltitude) && Finite(endpointTerrainAltitude) &&
                Math.Abs(simulatedTerrainAltitude - endpointTerrainAltitude) <= ToleranceMetres;
        }

        public static double NextIterationTerrainAltitude(double observedTerrainAltitude,
            double fallbackTerrainAltitude)
        {
            return Finite(observedTerrainAltitude) ? observedTerrainAltitude : fallbackTerrainAltitude;
        }

        // A result may replace the published endpoint only when it agrees
        // with the immediately preceding self-consistent candidate. Keeping
        // this decision pure makes the A/B/A branch regression explicit.
        public static bool HasConsecutiveAgreement(bool hasCandidate, bool candidateAgreesWithCurrent) =>
            hasCandidate && candidateAgreesWithCurrent;

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
