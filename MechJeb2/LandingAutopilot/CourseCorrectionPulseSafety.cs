using System;

namespace MuMech
{
    namespace Landing
    {
        /// <summary>
        /// Selects the largest usable part of a proposed correction pulse that
        /// retains a geometrically descending trajectory.  The caller supplies
        /// the body-specific impact check, keeping this policy deterministic
        /// and independently testable.
        /// </summary>
        public static class CourseCorrectionPulseSafety
        {
            private const int SearchIterations = 24;

            public static double LimitToImpactPreservingMagnitude(double requestedMagnitude,
                double minimumUsefulMagnitude, Func<double, bool> preservesImpact)
            {
                if (!IsFinite(requestedMagnitude) || !IsFinite(minimumUsefulMagnitude) ||
                    requestedMagnitude <= 0 || minimumUsefulMagnitude < 0 || preservesImpact == null)
                    return 0;

                // A correction may only be made from a trajectory that is
                // already known to reach the protected descent radius.
                if (!preservesImpact(0))
                    return 0;

                if (preservesImpact(requestedMagnitude))
                    return requestedMagnitude;

                double safeMagnitude = 0;
                double unsafeMagnitude = requestedMagnitude;
                for (int i = 0; i < SearchIterations; i++)
                {
                    double trialMagnitude = 0.5 * (safeMagnitude + unsafeMagnitude);
                    if (preservesImpact(trialMagnitude))
                        safeMagnitude = trialMagnitude;
                    else
                        unsafeMagnitude = trialMagnitude;
                }

                return safeMagnitude >= minimumUsefulMagnitude ? safeMagnitude : 0;
            }

            private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
