using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    /// <summary>
    /// Selects the first sample on a descending vacuum trajectory that is at
    /// or below the real terrain surface.  Airless trajectories are
    /// independent of terrain until their first surface contact, so resolving
    /// contact from the recorded path avoids feeding a terrain height back
    /// into the simulator as a fictitious spherical body.
    /// </summary>
    public static class AirlessTerrainProfileContact
    {
        public static int FindFirstContactIndex(IList<double> altitudeASL, IList<double> terrainAltitude)
        {
            if (altitudeASL == null || terrainAltitude == null || altitudeASL.Count != terrainAltitude.Count)
                return -1;

            for (int i = 0; i < altitudeASL.Count; ++i)
            {
                if (!Finite(altitudeASL[i]) || !Finite(terrainAltitude[i]))
                    continue;

                if (altitudeASL[i] <= terrainAltitude[i])
                    return i;
            }

            return -1;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// A result may replace the published endpoint only when it agrees with
    /// the immediately preceding candidate. Keeping this decision pure makes
    /// branch-change regressions explicit.
    /// </summary>
    public static class LandingPredictionTerrainConvergence
    {
        public static bool HasConsecutiveAgreement(bool hasCandidate, bool candidateAgreesWithCurrent) =>
            hasCandidate && candidateAgreesWithCurrent;
    }
}