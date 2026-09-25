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
    /// A result may replace the published endpoint only when it remains
    /// consistent for long enough. A normal continuously moving endpoint
    /// needs two samples. A terrain branch that is materially displaced from
    /// the currently published endpoint needs three samples: a pair is not
    /// enough evidence to move V1's map marker or course-correction input to
    /// another mountain or valley.
    /// </summary>
    public static class LandingPredictionTerrainConvergence
    {
        public const int NormalRequiredSamples = 2;
        public const int DisplacedReplacementRequiredSamples = 3;

        public static int RequiredSamples(bool hasPublishedResult, bool materiallyDisplacedFromPublished) =>
            hasPublishedResult && materiallyDisplacedFromPublished
                ? DisplacedReplacementRequiredSamples
                : NormalRequiredSamples;

        public static int NextCompatibleSampleCount(int currentCount, bool candidateAgreesWithCurrent) =>
            candidateAgreesWithCurrent ? currentCount + 1 : 1;

        public static bool CanPublish(int compatibleSampleCount, int requiredSamples) =>
            compatibleSampleCount >= requiredSamples;
    }
}
