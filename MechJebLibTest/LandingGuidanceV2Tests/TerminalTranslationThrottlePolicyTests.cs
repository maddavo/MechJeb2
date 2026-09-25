using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class TerminalTranslationThrottlePolicyTests
    {
        [Fact]
        public void LowGravityHoverCanCommandBelowGeneralFivePercentFloor()
        {
            // At Minmus gravity with 20 m/s^2 available upward acceleration,
            // holding height requires 2.45%, not an imposed 5% climb command.
            double throttle = TerminalTranslationThrottlePolicy.ComputeThrottle(0, 0, 0.49, 20);

            Assert.InRange(throttle, 0.0244, 0.0246);
            Assert.True(throttle < 0.05);
        }

        [Fact]
        public void DescendingCraftRequestsEnoughAccelerationToStopDescent()
        {
            double throttle = TerminalTranslationThrottlePolicy.ComputeThrottle(-5, 0, 0.49, 20);

            Assert.InRange(throttle, 0.2744, 0.2746);
        }

        [Fact]
        public void NoUsableVerticalThrustCommandsZero()
        {
            Assert.Equal(0, TerminalTranslationThrottlePolicy.ComputeThrottle(0, 0, 0.49, 0));
        }
    }
}
