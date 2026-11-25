using System;

namespace MuMech
{
    // Per-axis gain scheduling, not a reduction of actuator authority.
    public static class AttitudeRateResponse
    {
        public static double Gain(double positionError, double angularRate, double momentOfInertia,
            double availableTorque, double stoppingTime)
        {
            if (!Finite(positionError) || !Finite(angularRate) || !Finite(momentOfInertia) ||
                !Finite(availableTorque) || !Finite(stoppingTime) || momentOfInertia <= 0 ||
                availableTorque <= 0 || stoppingTime <= 0) return 0;

            double alpha = availableTorque / momentOfInertia;
            // Critical rate-loop response scaled by the time to accelerate through
            // one radian. The existing 4/s response remains the upper bound.
            // Do not make the inner rate loop slower than the existing stopping
            // time: the unchanged outer position PI still needs a responsive
            // inner loop, especially on a vessel with very low wheel authority.
            double holdGain = Math.Min(4, Math.Max(.5, Math.Max(1 / stoppingTime, 2 * Math.Sqrt(alpha))));
            // Existing stopping-time envelope defines when a real turn/spin needs
            // the original response. Squared blend avoids a gain jump near hold.
            double positionDemand = Math.Abs(positionError) / (alpha * stoppingTime * stoppingTime);
            double rateDemand = Math.Abs(angularRate) / (2 * alpha * stoppingTime);
            double demand = Math.Min(1, Math.Max(positionDemand, rateDemand));
            return holdGain + (4 - holdGain) * demand * demand;
        }

        private static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
    }
}
