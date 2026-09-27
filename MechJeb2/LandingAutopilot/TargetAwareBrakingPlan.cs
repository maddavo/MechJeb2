using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    /// <summary>
    /// Selects a safe braking-start candidate by its signed downrange powered
    /// touchdown error. Negative is short; positive is long. The caller owns
    /// the trajectory simulations that produce the candidates.
    /// </summary>
    public static class TargetAwareBrakingPlan
    {
        public readonly struct Candidate
        {
            public readonly double StartUT;
            public readonly double DownrangeError;
            public readonly double CrossrangeError;
            public readonly bool Safe;

            public Candidate(double startUT, double downrangeError, double crossrangeError, bool safe)
            {
                StartUT = startUT;
                DownrangeError = downrangeError;
                CrossrangeError = crossrangeError;
                Safe = safe;
            }
        }

        public static bool TrySelect(IList<Candidate> candidates, out Candidate selected)
        {
            selected = default(Candidate);
            if (candidates == null || candidates.Count == 0)
                return false;

            bool haveSelection = false;
            double bestScore = double.PositiveInfinity;
            for (int i = 0; i < candidates.Count; ++i)
            {
                Candidate candidate = candidates[i];
                if (!candidate.Safe || !Finite(candidate.StartUT) || !Finite(candidate.DownrangeError) ||
                    !Finite(candidate.CrossrangeError))
                    continue;

                double score = candidate.DownrangeError * candidate.DownrangeError +
                    candidate.CrossrangeError * candidate.CrossrangeError;
                if (!haveSelection || score < bestScore ||
                    (score == bestScore && candidate.StartUT > selected.StartUT))
                {
                    selected = candidate;
                    bestScore = score;
                    haveSelection = true;
                }
            }

            return haveSelection;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
