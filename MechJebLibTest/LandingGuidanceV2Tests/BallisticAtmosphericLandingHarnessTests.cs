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

        [Fact]
        public void PostBurnEstimatorFailureRetainsV2EntryProfileAndReachesBallisticTouchdown()
        {
            AtmosphericEntryControllerHarnessResult entry = new AtmosphericEntryControllerHarness().Execute(
                Candidate(100, 1000, 30), 100, 0.65, 27.9, Rejected(102));
            BallisticAtmosphericLandingHarnessResult descent = new BallisticAtmosphericLandingHarness().Execute(
                5000, 180, 9.81, 30, parachutesDeploy: true, injectStageFailure: false);

            Assert.True(entry.FiniteBurnCompleted);
            Assert.True(entry.PostBurnValidationRequired);
            Assert.Equal(AtmosphericEntryPhase.Entry, entry.FinalPhase);
            Assert.Equal(AtmosphericEntryDirective.EnterAtmosphericEntry, entry.LastDirective);
            Assert.Contains("conservative", entry.LastReason);
            Assert.True(descent.ParachutesDeployed);
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
        [InlineData(5000, 180, 9.81, 30)] // Kerbin, TWR about 3.1
        [InlineData(50000, 180, 9.81, 12)] // Kerbin, low TWR about 1.2
        [InlineData(5000, 180, 2.94, 12)] // Duna, TWR about 4.1
        [InlineData(9000, 240, 2.94, 18)] // Duna, higher entry speed/TWR
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

        private static AtmosphericLandingPlan Rejected(long version) =>
            new AtmosphericLandingPlan(version, AtmosphericLandingPlanState.Rejected, double.NaN, double.NaN,
                double.NaN, double.NaN, double.NaN, "Harness deliberately rejected the post-burn estimate.");
    }
}
