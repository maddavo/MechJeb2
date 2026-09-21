using MechJebLib.HoverslamSimulation;
using Xunit;

namespace MechJebLibTest
{
    public class AirlessLandingBudgetTests
    {
        [Fact]
        public void FeasibleStrategicAndTerminalAllocationFits()
        {
            AirlessLandingBudget budget = AirlessLandingBudget.For(120, 520, 25, 1.63, 40, 400);

            Assert.True(budget.Fits(810));
            Assert.InRange(budget.Total, 700, 810);
            Assert.True(budget.BrakingTime > 0);
        }

        [Fact]
        public void HighEnergyStrategicCandidateCannotPassBudgetGate()
        {
            AirlessLandingBudget budget = AirlessLandingBudget.For(802.578702, 944.724828, 25, 1.63, 40, 400);

            Assert.False(budget.Fits(810.440617));
            Assert.True(budget.Total > 1700);
        }

        [Fact]
        public void LowTwrTerminalPlanFailsClosed()
        {
            AirlessLandingBudget budget = AirlessLandingBudget.For(120, 520, 1.5, 1.63, 40, 400);

            Assert.False(budget.Fits(100000));
            Assert.True(double.IsInfinity(budget.Terminal));
            Assert.True(double.IsInfinity(budget.BrakingTime));
        }

        [Fact]
        public void LargerUncertaintyConsumesMoreDivertReserve()
        {
            AirlessLandingBudget precise = AirlessLandingBudget.For(120, 520, 25, 1.63, 20, 400);
            AirlessLandingBudget uncertain = AirlessLandingBudget.For(120, 520, 25, 1.63, 300, 400);

            Assert.True(uncertain.Reserve > precise.Reserve);
        }

        [Fact]
        public void LowerThrustMarginRequiresMoreTerminalBraking()
        {
            AirlessLandingBudget highTwr = AirlessLandingBudget.For(120, 520, 25, 1.63, 40, 400);
            AirlessLandingBudget lowTwr = AirlessLandingBudget.For(120, 520, 5, 1.63, 40, 400);

            Assert.True(lowTwr.Terminal > highTwr.Terminal);
            Assert.True(lowTwr.BrakingTime > highTwr.BrakingTime);
            Assert.True(lowTwr.Contingency > highTwr.Contingency);
        }
    }
}
