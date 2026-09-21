using MechJebLib.HoverslamSimulation;
using Xunit;

namespace MechJebLibTest
{
    public class AirlessLandingBudgetTests
    {
        [Fact]
        public void FeasibleStrategicAndTerminalAllocationFits()
        {
            AirlessLandingBudget budget = AirlessLandingBudget.For(120, 520);

            Assert.True(budget.Fits(810));
            Assert.Equal(714, budget.Total, 8);
        }

        [Fact]
        public void HighEnergyStrategicCandidateCannotPassBudgetGate()
        {
            AirlessLandingBudget budget = AirlessLandingBudget.For(802.578702, 944.724828);

            Assert.False(budget.Fits(810.440617));
            Assert.True(budget.Total > 1700);
        }
    }
}
