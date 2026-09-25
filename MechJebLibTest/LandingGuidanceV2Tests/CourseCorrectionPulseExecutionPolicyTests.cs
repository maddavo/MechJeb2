using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CourseCorrectionPulseExecutionPolicyTests
    {
        [Fact]
        public void EffectScaledTwoCentimetrePerSecondPulseIsNotDiscarded()
        {
            Assert.False(CourseCorrectionPulseExecutionPolicy.HasCompleted(0.020));
        }

        [Fact]
        public void CompletionToleranceIsFiveMillimetresPerSecond()
        {
            Assert.True(CourseCorrectionPulseExecutionPolicy.HasCompleted(0.005));
            Assert.True(CourseCorrectionPulseExecutionPolicy.HasCompleted(0.004));
        }
    }
}
