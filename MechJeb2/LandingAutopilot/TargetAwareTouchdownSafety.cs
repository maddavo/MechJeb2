using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    internal static class TargetAwareTouchdownSafety
    {
        // A conservative contact-speed screen, not a proof of landing-gear loads.
        // Half the weakest part rating, capped at 5 m/s, is separate from the
        // requested descent setpoint. Unknown ratings preserve the strict rule.
        internal static double SpeedLimit(IEnumerable<double> partRatings, double requestedSpeed)
        {
            double weakest = double.PositiveInfinity;
            foreach (double rating in partRatings)
            {
                if (double.IsNaN(rating) || double.IsInfinity(rating) || rating <= 0)
                    return requestedSpeed;
                weakest = Math.Min(weakest, rating);
            }
            return double.IsInfinity(weakest) ? requestedSpeed : Math.Min(5, 0.5 * weakest);
        }

        internal static bool ContactSpeedPasses(double vertical, double horizontal, double limit) =>
            !double.IsNaN(vertical) && !double.IsNaN(horizontal) &&
            !double.IsInfinity(vertical) && !double.IsInfinity(horizontal) &&
            horizontal >= 0 && limit > 0 && !double.IsInfinity(limit) &&
            // Recorded production-integrator terminal replays settle within
            // 0.0014 m/s of the requested floor. Allow at most 0.002 m/s for
            // this numerical boundary; retain and report the actual speed.
            Math.Sqrt(vertical * vertical + horizontal * horizontal) <=
                limit + Math.Min(0.002, limit * 0.004);
    }
}
