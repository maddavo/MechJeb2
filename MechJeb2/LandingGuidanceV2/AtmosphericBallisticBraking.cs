using System;

namespace MuMech
{
    /// <summary>
    /// Continuous retrograde braking policy for the agreed V2 atmospheric
    /// baseline: a stable, ballistic capsule or bell-shaped craft. It does
    /// not command lift, bank, cross-range steering, or a V1 control path.
    /// </summary>
    public sealed class AtmosphericBallisticBrakingCommand
    {
        public readonly bool Valid;
        public readonly double RequestedThrottle;
        public readonly double DesiredDownSpeed;
        public readonly string RejectionReason;

        public AtmosphericBallisticBrakingCommand(bool valid, double requestedThrottle, double desiredDownSpeed,
            string rejectionReason)
        {
            Valid = valid;
            RequestedThrottle = requestedThrottle;
            DesiredDownSpeed = desiredDownSpeed;
            RejectionReason = rejectionReason;
        }
    }

    public static class AtmosphericBallisticBraking
    {
        public const double MinimumDesiredDownSpeed = 1.5;
        public const double EnvelopeReserveFactor = 0.01;

        public static AtmosphericBallisticBrakingCommand Calculate(double altitude, double downSpeed, double gravity,
            double minimumAcceleration, double maximumAcceleration, double availableMainThrottle = 1.0)
        {
            if (!Finite(altitude) || !Finite(downSpeed) || !Finite(gravity) || !Finite(minimumAcceleration) ||
                !Finite(maximumAcceleration) || maximumAcceleration < minimumAcceleration || gravity <= 0)
                return Invalid("V2 atmospheric braking requires finite local descent and engine measurements.");
            double throttleLimit = Clamp01(availableMainThrottle);
            double availableAcceleration = minimumAcceleration +
                (maximumAcceleration - minimumAcceleration) * throttleLimit;
            if (availableAcceleration <= gravity)
                return Invalid("V2 atmospheric braking requires available thrust acceleration greater than local gravity.");

            double netAcceleration = availableAcceleration - gravity;
            // This deliberately sits inside the two-stopping-distance energy
            // gate, so it commands a substantial speed reduction as soon as
            // powered braking begins and retains response/staging margin.
            double desiredDownSpeed = Math.Max(MinimumDesiredDownSpeed,
                Math.Sqrt(EnvelopeReserveFactor * netAcceleration * Math.Max(0, altitude)));
            double desiredAcceleration = gravity + Math.Max(0, downSpeed - desiredDownSpeed);
            double throttle = maximumAcceleration > minimumAcceleration
                ? Clamp01((Math.Min(availableAcceleration, desiredAcceleration) - minimumAcceleration) /
                    (maximumAcceleration - minimumAcceleration))
                : desiredAcceleration >= maximumAcceleration ? 1 : 0;
            return new AtmosphericBallisticBrakingCommand(true, throttle, desiredDownSpeed, null);
        }

        private static AtmosphericBallisticBrakingCommand Invalid(string reason) =>
            new AtmosphericBallisticBrakingCommand(false, 0, double.NaN, reason);
        private static double Clamp01(double value) => !Finite(value) ? 0 : Math.Max(0, Math.Min(1, value));
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
