using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    /// <summary>
    /// Evaluates braking-start candidates by signed downrange handoff error.
    /// Negative is short; positive is long. The caller owns
    /// the trajectory simulations that produce the candidates.
    /// </summary>
    public static class TargetAwareBrakingPlan
    {
        public readonly struct Bracket
        {
            public readonly Candidate Earlier;
            public readonly Candidate Later;

            public Bracket(Candidate earlier, Candidate later)
            {
                Earlier = earlier;
                Later = later;
            }
        }

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

        // Brake time controls signed downrange, not crossrange. Only two valid
        // adjacent time samples that straddle zero establish a refinement interval.
        // An unsafe or incomplete sample between them may indicate a terrain
        // branch discontinuity, so it must not be skipped to fabricate a bracket.
        // Candidate order is irrelevant to callers; sorting a copy leaves their
        // result ownership and original order unchanged.
        public static bool TryFindDownrangeBracket(IList<Candidate> candidates, out Bracket bracket)
        {
            bracket = default(Bracket);
            if (candidates == null || candidates.Count < 2)
                return false;

            var ordered = new List<Candidate>(candidates);
            ordered.Sort((left, right) => left.StartUT.CompareTo(right.StartUT));

            bool found = false;
            double bestResidual = double.PositiveInfinity;
            for (int i = 1; i < ordered.Count; ++i)
            {
                Candidate earlier = ordered[i - 1];
                Candidate later = ordered[i];
                if (!earlier.Safe || !later.Safe || !Finite(earlier.StartUT) || !Finite(later.StartUT) ||
                    !Finite(earlier.DownrangeError) || !Finite(later.DownrangeError) ||
                    !Finite(earlier.CrossrangeError) || !Finite(later.CrossrangeError) ||
                    earlier.StartUT >= later.StartUT ||
                    !((earlier.DownrangeError <= 0 && later.DownrangeError >= 0) ||
                      (earlier.DownrangeError >= 0 && later.DownrangeError <= 0)))
                    continue;

                double residual = Math.Max(Math.Abs(earlier.DownrangeError), Math.Abs(later.DownrangeError));
                if (!found || residual < bestResidual)
                {
                    bracket = new Bracket(earlier, later);
                    bestResidual = residual;
                    found = true;
                }
            }
            return found;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
