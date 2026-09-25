using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CourseCorrectionPulsePolicyTests
    {
        [Fact]
        public void AppliesHalfOfARemotePredictedImpactCorrection()
        {
            Assert.Equal(0.50, CourseCorrectionPulsePolicy.SelectEffectFraction(170683, 600, false), 6);
        }

        [Fact]
        public void ReducesTheImpactEffectAsTheTargetIsApproached()
        {
            Assert.Equal(0.25, CourseCorrectionPulsePolicy.SelectEffectFraction(5000, 600, false), 6);
            Assert.Equal(0.10, CourseCorrectionPulsePolicy.SelectEffectFraction(1500, 600, false), 6);
            Assert.Equal(0.05, CourseCorrectionPulsePolicy.SelectEffectFraction(500, 600, false), 6);
        }

        [Fact]
        public void DoesNotCapTheCorrectionByDeltaV()
        {
            // The policy has no delta-V input. A high-velocity or high-gravity
            // case receives the same bounded surface-effect fraction.
            Assert.Equal(0.50, CourseCorrectionPulsePolicy.SelectEffectFraction(170683, 600, false), 6);
        }

        [Fact]
        public void LimitsAReversalToFineSurfaceControl()
        {
            Assert.Equal(0.05, CourseCorrectionPulsePolicy.SelectEffectFraction(170683, 600, true), 6);
        }

        [Fact]
        public void RejectsInvalidInputs()
        {
            Assert.Equal(0, CourseCorrectionPulsePolicy.SelectEffectFraction(double.NaN, 600, false));
            Assert.Equal(0, CourseCorrectionPulsePolicy.SelectEffectFraction(100, 0, false));
        }
    }
}
