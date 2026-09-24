using Xunit;
using MuMech;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class V2PoweredStagingGateTests
    {
        [Fact]
        public void StagesOnlyAfterSustainedFullThrottleThrustLossWithUsableNextStage()
        {
            var gate = new V2PoweredStagingGate();
            Assert.Equal(V2PoweredStageDirective.Hold,
                gate.Decide(10, true, false, 1, 0, 20, true).Directive);
            Assert.Equal(V2PoweredStageDirective.Hold,
                gate.Decide(10.74, true, false, 1, 0, 20, true).Directive);
            Assert.Equal(V2PoweredStageDirective.CommandStage,
                gate.Decide(10.75, true, false, 1, 0, 20, true).Directive);
        }

        [Theory]
        [InlineData(false, false, 1.0, 0.0, 20.0, true)]
        [InlineData(true, true, 1.0, 0.0, 20.0, true)]
        [InlineData(true, false, 0.5, 0.0, 20.0, true)]
        [InlineData(true, false, 1.0, 4.0, 20.0, true)]
        [InlineData(true, false, 1.0, 0.0, 20.0, false)]
        public void UnsafeOrHealthyConditionsNeverStage(bool poweredPhase, bool parachutesDeployed,
            double throttle, double measuredAcceleration, double maximumAcceleration, bool nextStage)
        {
            var gate = new V2PoweredStagingGate();
            Assert.Equal(V2PoweredStageDirective.Hold,
                gate.Decide(10, poweredPhase, parachutesDeployed, throttle, measuredAcceleration,
                    maximumAcceleration, nextStage).Directive);
            Assert.Equal(V2PoweredStageDirective.Hold,
                gate.Decide(20, poweredPhase, parachutesDeployed, throttle, measuredAcceleration,
                    maximumAcceleration, nextStage).Directive);
        }

        [Fact]
        public void AHealthyMeasurementResetsTheLossConfirmationWindow()
        {
            var gate = new V2PoweredStagingGate();
            gate.Decide(10, true, false, 1, 0, 20, true);
            gate.Decide(10.5, true, false, 1, 5, 20, true);
            Assert.Equal(V2PoweredStageDirective.Hold,
                gate.Decide(10.76, true, false, 1, 0, 20, true).Directive);
            Assert.Equal(V2PoweredStageDirective.CommandStage,
                gate.Decide(11.51, true, false, 1, 0, 20, true).Directive);
        }
    }
}
