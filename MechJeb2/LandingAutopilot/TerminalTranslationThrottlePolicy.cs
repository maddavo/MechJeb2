using System;

namespace MuMech
{
    namespace Landing
    {
        /// <summary>
        /// Computes the throttle needed to hold the requested vertical speed
        /// while the terminal landing step removes lateral velocity. This is
        /// deliberately independent of the general purpose minimum-throttle
        /// limiter: on a low-gravity body a five-percent floor can produce a
        /// sustained climb rather than a hover.
        /// </summary>
        public static class TerminalTranslationThrottlePolicy
        {
            public static double ComputeThrottle(double currentVerticalSpeed, double targetVerticalSpeed,
                double localGravity, double maximumVerticalThrustAcceleration)
            {
                if (!Finite(currentVerticalSpeed) || !Finite(targetVerticalSpeed) ||
                    !Finite(localGravity) || !Finite(maximumVerticalThrustAcceleration) ||
                    localGravity < 0)
                    return 0;

                double minimumNetAcceleration = -localGravity;
                double maximumNetAcceleration = maximumVerticalThrustAcceleration - localGravity;
                if (maximumNetAcceleration <= minimumNetAcceleration)
                    return 0;

                double desiredNetAcceleration = targetVerticalSpeed - currentVerticalSpeed;
                return Clamp01((desiredNetAcceleration - minimumNetAcceleration) /
                    (maximumNetAcceleration - minimumNetAcceleration));
            }

            private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

            private static double Clamp01(double value) => Math.Max(0, Math.Min(1, value));
        }
    }
}
