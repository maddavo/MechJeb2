using System;

namespace MechJebLib.HoverslamSimulation
{
    /// <summary>
    /// The conservative delta-V allocation used to decide whether an airless
    /// strategic candidate leaves enough for terminal braking and protected
    /// local-divert reserve. Kept free of KSP/Unity state so it can be tested.
    /// </summary>
    public readonly struct AirlessLandingBudget
    {
        public readonly double Strategic;
        public readonly double Terminal;
        public readonly double Trim;
        public readonly double Reserve;
        public readonly double Contingency;
        public readonly double BrakingTime;
        public readonly double Total;

        private AirlessLandingBudget(double strategic, double terminal, double trim, double reserve, double contingency,
            double brakingTime)
        {
            Strategic = strategic;
            Terminal = terminal;
            Trim = trim;
            Reserve = reserve;
            Contingency = contingency;
            BrakingTime = brakingTime;
            Total = strategic + terminal + trim + reserve + contingency;
        }

        /// <summary>
        /// Builds an airless budget from the planned terminal state and the
        /// vehicle/body limits captured with that plan. Reserves are expressed
        /// as response time and local-divert capability, not a burn percentage.
        /// </summary>
        public static AirlessLandingBudget For(double strategic, double impactSpeed,
            double maximumAcceleration, double localGravity, double targetUncertainty,
            double corridorRadius)
        {
            if (maximumAcceleration <= localGravity || localGravity < 0 || double.IsNaN(impactSpeed))
                return new AirlessLandingBudget(strategic, double.PositiveInfinity,
                    double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);

            double netDeceleration = maximumAcceleration - localGravity;
            double brakingTime = Math.Max(0, impactSpeed) / netDeceleration;
            double terminal = Math.Max(0, impactSpeed) + localGravity * brakingTime;

            double trim = Math.Max(2.0, Math.Min(35.0, targetUncertainty / 40.0) + 2.0);

            double divertDistance = Math.Max(10.0, Math.Min(corridorRadius, targetUncertainty + 25.0));
            double responseSeconds = 2.0 + Math.Min(3.0, impactSpeed / Math.Max(1.0, maximumAcceleration));
            double lateralSpeed = Math.Sqrt(Math.Max(0, 2.0 * netDeceleration * divertDistance));
            double reserve = lateralSpeed + localGravity * responseSeconds;

            double thrustMargin = Math.Max(0.05, netDeceleration / maximumAcceleration);
            double contingency = localGravity * (1.0 + 1.0 / thrustMargin);
            return new AirlessLandingBudget(strategic, terminal, trim, reserve, contingency, brakingTime);
        }

        public bool Fits(double availableDeltaV) => availableDeltaV >= Total;
    }
}
