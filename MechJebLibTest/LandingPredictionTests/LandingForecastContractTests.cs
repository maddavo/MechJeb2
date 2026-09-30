using MuMech;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class LandingForecastContractTests
    {
        [Fact]
        public void FreshImpactIsCorrectionReadyButNeverLandingReady()
        {
            var state = LandingForecastContract.FreshState(
                ReentrySimulation.LandingForecastKind.ImpactForecast,
                100, 105, 10, true);
            Assert.True(LandingForecastContract.CorrectionReady(state));
            Assert.False(LandingForecastContract.LandingReady(state));
        }

        [Fact]
        public void ExpiryAndPostPulseSettlingRemoveDecisionAuthority()
        {
            var landable = ReentrySimulation.LandingForecastKind.LandableForecast;
            Assert.Equal(ReentrySimulation.LandingForecastKind.NoForecast,
                LandingForecastContract.FreshState(landable, 100, 111, 10, true));
            Assert.Equal(ReentrySimulation.LandingForecastKind.NoForecast,
                LandingForecastContract.FreshState(landable, 100, 105, 10, false));
            Assert.Equal(ReentrySimulation.LandingForecastKind.NoForecast,
                LandingForecastContract.FreshState(landable, 106, 105, 10, true));
        }

        [Fact]
        public void PhysicalCorrectionDeadlineCannotWaitIndefinitely()
        {
            // One attitude turn, pulse, settling interval and two refreshes
            // take at least 14 seconds in this fixture.
            Assert.False(LandingForecastContract.CorrectionCycleImpossible(
                14.1, 2, 1.25, 0.75, 5, 0));
            Assert.True(LandingForecastContract.CorrectionCycleImpossible(
                14, 2, 1.25, 0.75, 5, 0));
            Assert.True(LandingForecastContract.CorrectionCycleImpossible(
                double.NaN, 2, 1.25, 0.75, 5, 0));
            Assert.True(LandingForecastContract.CorrectionCycleImpossible(
                100, double.PositiveInfinity, 1.25, 0.75, 5, 0));
        }
    }
}
