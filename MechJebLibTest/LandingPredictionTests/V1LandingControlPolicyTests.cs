using MuMech.Landing;
using MuMech;
using System.Collections.Generic;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class V1LandingControlPolicyTests
    {
        [Fact]
        public void FiveSecondBrakingGuardAndThrottleGateRetainV1Boundaries()
        {
            Assert.True(V1LandingControlPolicy.ShouldCoastToBrake(150, 144.99, false));
            Assert.False(V1LandingControlPolicy.ShouldCoastToBrake(150, 145, false));
            Assert.False(V1LandingControlPolicy.ShouldCoastToBrake(150, 140, true));
            Assert.True(V1LandingControlPolicy.BrakeWarpReady(4.99, 0.0009));
            Assert.False(V1LandingControlPolicy.BrakeWarpReady(5, 0.0009));
            Assert.False(V1LandingControlPolicy.BrakeWarpReady(4, 0.001));
            Assert.True(V1LandingControlPolicy.BrakeAttitudeGate(0.01, 1));
            Assert.True(V1LandingControlPolicy.BrakeAttitudeGate(-1, 0.749));
            Assert.False(V1LandingControlPolicy.BrakeAttitudeGate(-1, 0.75));
        }

        [Fact]
        public void SharedThrottleCalculationsPreserveV1CommandValues()
        {
            Assert.Equal(1f, V1LandingControlPolicy.BrakingThrottle(
                -100, -50, -49, 0.02, -1, 9));
            Assert.Equal(0.1f, V1LandingControlPolicy.BrakingThrottle(
                -50, -50, -50, 0.02, -1, 9));
            Assert.Equal(0f, V1LandingControlPolicy.BrakingThrottle(
                -40, -50, -50, 0.02, -1, 9));
            Assert.Equal(0f, V1LandingControlPolicy.BrakingThrottle(
                -100, -50, -49, 0.02, 1, 1));
            Assert.Equal(0.45375f, V1LandingControlPolicy.HoverThrottle(
                -2, 1.63, 1, 8), 5);
            Assert.Equal(-6.52, V1LandingControlPolicy.FinalDescentSpeed(
                100, 10, 1.63, 0.5), 5);
        }

        [Fact]
        public void PulseAndHandoffThresholdsRemainUnchanged()
        {
            Assert.True(V1LandingControlPolicy.ShouldStartDeceleration(90.1, 100));
            Assert.False(V1LandingControlPolicy.ShouldStartDeceleration(90, 100));
            Assert.True(V1LandingControlPolicy.BelowTerminalHandoff(204.99, 200));
            Assert.False(V1LandingControlPolicy.BelowTerminalHandoff(205, 200));
            Assert.False(V1LandingControlPolicy.PostPulsePredictionSettled(11, 10,
                100.749, 100));
            Assert.True(V1LandingControlPolicy.PostPulsePredictionSettled(11, 10,
                100.75, 100));
            Assert.Equal(0.1, V1LandingControlPolicy.MaximumCourseCorrectionPulse(
                100, 200000, false, 0));
            Assert.Equal(0.25, V1LandingControlPolicy.MaximumCourseCorrectionPulse(
                3000, 200000, false, 0));
            Assert.Equal(0.1, V1LandingControlPolicy.MaximumCourseCorrectionPulse(
                10000, 200000, true, 100));
        }

        [Fact]
        public void ExplicitBrakeReferenceDoesNotMoveTheDisplayedPathStart()
        {
            var result = new ReentrySimulation.Result
            {
                Trajectory = new List<AbsoluteVector> { new AbsoluteVector { UT = 20 } }
            };
            Assert.Equal(20, result.BrakeReferenceUT(10));
            result.HasControllerBrakeReferenceUT = true;
            result.ControllerBrakeReferenceUT = 60;
            Assert.Equal(60, result.BrakeReferenceUT(10));
            Assert.Equal(20, result.Trajectory[0].UT);
        }
    }
}
