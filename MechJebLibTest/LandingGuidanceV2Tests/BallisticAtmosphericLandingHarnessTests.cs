using MuMech;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class BallisticAtmosphericLandingHarnessTests
    {
        [Theory]
        [InlineData(5000, 180, 9.81, 0, 30, true)]
        [InlineData(5000, 180, 9.81, 0, 9.81, false)]
        public void BallisticBrakingUsesContinuousThrottleAndRejectsInsufficientAvailableThrust(
            double altitude, double downSpeed, double gravity, double minimumAcceleration,
            double maximumAcceleration, bool expectedValid)
        {
            AtmosphericBallisticBrakingCommand command = AtmosphericBallisticBraking.Calculate(altitude,
                downSpeed, gravity, minimumAcceleration, maximumAcceleration, availableMainThrottle: 1.0);

            Assert.Equal(expectedValid, command.Valid);
            if (expectedValid)
            {
                Assert.InRange(command.RequestedThrottle, 0, 1);
                Assert.True(command.DesiredDownSpeed >= AtmosphericBallisticBraking.MinimumDesiredDownSpeed);
            }
        }

        [Fact]
        public void StableParachuteProfileReachesTheTouchdownBoundWithoutPoweredBraking()
        {
            BallisticAtmosphericLandingHarnessResult result = new BallisticAtmosphericLandingHarness().Execute(
                12000, 260, 9.81, 30, parachutesDeploy: true, injectStageFailure: false);

            Assert.True(result.Landed, result.RejectionReason);
            Assert.True(result.ParachutesDeployed);
            Assert.False(result.PoweredBrakingEntered);
            Assert.Equal(0, result.MaximumThrottle);
            Assert.InRange(result.TouchdownSpeed, 0, 8);
        }

        [Fact]
        public void PoweredBallisticProfileStagesAfterConfirmedLossAndStillLands()
        {
            BallisticAtmosphericLandingHarnessResult result = new BallisticAtmosphericLandingHarness().Execute(
                5000, 180, 9.81, 30, parachutesDeploy: false, injectStageFailure: true);

            Assert.True(result.PoweredBrakingEntered);
            Assert.True(result.StageLossObserved);
            Assert.True(result.StageCommanded);
            Assert.True(result.Landed, result.RejectionReason);
            Assert.InRange(result.TouchdownSpeed, 0, 8);
            Assert.InRange(result.MaximumThrottle, 0.90, 1.0);
            Assert.True(result.MinimumThrottleWhenPowered < 0.90);
        }

        [Fact]
        public void PoweredBallisticProfileLandsWithoutStagingWhenItsEngineRemainsHealthy()
        {
            BallisticAtmosphericLandingHarnessResult result = new BallisticAtmosphericLandingHarness().Execute(
                5000, 180, 9.81, 30, parachutesDeploy: false, injectStageFailure: false);

            Assert.True(result.Landed, result.RejectionReason);
            Assert.True(result.PoweredBrakingEntered);
            Assert.False(result.StageCommanded);
            Assert.InRange(result.TouchdownSpeed, 0, 8);
        }
    }
}
