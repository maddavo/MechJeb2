using MuMech;
using UnityEngine;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class BallisticAtmosphericLandingHarnessTests
    {
        [Fact]
        public void CompleteAtmosphericV2SequenceWarpsAlignsBurnsThenReachesBallisticTouchdown()
        {
            AtmosphericLandingPlan plan = Candidate(100, 1000, 30);
            AtmosphericEntryControllerHarnessResult entry = new AtmosphericEntryControllerHarness().Execute(plan,
                100, 0.65, 27.9, Candidate(102, 1001, 0));
            BallisticAtmosphericLandingHarnessResult descent = new BallisticAtmosphericLandingHarness().Execute(
                5000, 180, 9.81, 30, parachutesDeploy: false, injectStageFailure: true);

            Assert.True(entry.InitialWarpRequested);
            Assert.True(entry.FinalWarpRequested);
            Assert.True(entry.FreshBurnValidationRequired);
            Assert.True(entry.FiniteBurnCompleted);
            Assert.Equal(AtmosphericEntryPhase.Entry, entry.FinalPhase);
            Assert.True(descent.StageCommanded);
            Assert.True(descent.Landed, descent.RejectionReason);
        }
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

        [Theory]
        [InlineData(5000, 180, 9.81, 30)]
        [InlineData(5000, 180, 1.63, 12)]
        [InlineData(9000, 240, 9.81, 45)]
        public void PoweredBallisticProfileLandsAcrossGravityAndThrustRangesWhenItsEngineRemainsHealthy(
            double altitude, double downSpeed, double gravity, double maximumAcceleration)
        {
            BallisticAtmosphericLandingHarnessResult result = new BallisticAtmosphericLandingHarness().Execute(
                altitude, downSpeed, gravity, maximumAcceleration, parachutesDeploy: false, injectStageFailure: false);

            Assert.True(result.Landed, result.RejectionReason);
            Assert.True(result.PoweredBrakingEntered);
            Assert.False(result.StageCommanded);
            Assert.InRange(result.TouchdownSpeed, 0, 8);
        }

        private static AtmosphericLandingPlan Candidate(long version, double burnUT, double deltaV) =>
            new AtmosphericLandingPlan(version, AtmosphericLandingPlanState.Candidate, 100, 5000, 1000, 100, 100,
                "test", new Vector3d(deltaV, 0, 0), burnUT, burnUT + 200, 100);
    }
}
