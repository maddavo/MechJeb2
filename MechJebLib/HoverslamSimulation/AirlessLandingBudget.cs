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
        public readonly double Total;

        private AirlessLandingBudget(double strategic, double terminal, double trim, double reserve, double contingency)
        {
            Strategic = strategic;
            Terminal = terminal;
            Trim = trim;
            Reserve = reserve;
            Contingency = contingency;
            Total = strategic + terminal + trim + reserve + contingency;
        }

        public static AirlessLandingBudget For(double strategic, double terminal)
        {
            double trim = Math.Max(5.0, strategic * 0.10);
            double reserve = Math.Max(20.0, terminal * 0.10);
            double contingency = Math.Max(10.0, strategic * 0.02);
            return new AirlessLandingBudget(strategic, terminal, trim, reserve, contingency);
        }

        public bool Fits(double availableDeltaV) => availableDeltaV >= Total;
    }
}
