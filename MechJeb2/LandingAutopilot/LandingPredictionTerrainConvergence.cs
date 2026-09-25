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

        // A result may replace the published endpoint only when it agrees
        // with the immediately preceding self-consistent candidate. Keeping
        // this decision pure makes the A/B/A branch regression explicit.
        public static bool HasConsecutiveAgreement(bool hasCandidate, bool candidateAgreesWithCurrent) =>
            hasCandidate && candidateAgreesWithCurrent;

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// Finds the terrain-height fixed point without following a two-cycle.
    /// Each simulation supplies f(h) = actualTerrainAtEndpoint(h) - h. Once
    /// samples of both signs exist, bisection gives a deterministic contact
    /// height whose simulated sphere reaches the real terrain surface.
    /// </summary>
    public sealed class AirlessTerrainHeightRootSolver
    {
        private bool _hasLower;
        private double _lowerHeight;
        private bool _hasUpper;
        private double _upperHeight;

        public void Reset()
        {
            _hasLower = false;
            _lowerHeight = double.NaN;
            _hasUpper = false;
            _upperHeight = double.NaN;
        }

        public AirlessTerrainHeightDecision Observe(double simulatedTerrainAltitude, double endpointTerrainAltitude)
        {
            if (!Finite(simulatedTerrainAltitude) || !Finite(endpointTerrainAltitude))
                return new AirlessTerrainHeightDecision(false, simulatedTerrainAltitude, "invalid terrain sample");

            double residual = endpointTerrainAltitude - simulatedTerrainAltitude;
            if (Math.Abs(residual) <= LandingPredictionTerrainConvergence.ToleranceMetres)
                return new AirlessTerrainHeightDecision(true, endpointTerrainAltitude, "terrain contact converged");

            if (residual > 0)
            {
                _hasLower = true;
                _lowerHeight = simulatedTerrainAltitude;
            }
            else
            {
                _hasUpper = true;
                _upperHeight = simulatedTerrainAltitude;
            }

            if (_hasLower && _hasUpper)
            {
                double low = Math.Min(_lowerHeight, _upperHeight);
                double high = Math.Max(_lowerHeight, _upperHeight);
                return new AirlessTerrainHeightDecision(false, (low + high) * 0.5,
                    "terrain bracket bisect");
            }

            return new AirlessTerrainHeightDecision(false, endpointTerrainAltitude,
                "terrain endpoint iteration");
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public sealed class AirlessTerrainHeightDecision
    {
        public readonly bool Converged;
        public readonly double NextTerrainAltitude;
        public readonly string Detail;

        public AirlessTerrainHeightDecision(bool converged, double nextTerrainAltitude, string detail)
        {
            Converged = converged;
            NextTerrainAltitude = nextTerrainAltitude;
            Detail = detail;
        }
    }
}
