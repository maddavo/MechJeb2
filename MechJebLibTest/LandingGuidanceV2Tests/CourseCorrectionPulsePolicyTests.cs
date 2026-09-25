using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CourseCorrectionPulsePolicyTests
    {
        [Fact]
        public void BeginsWithHalfOfARemotePredictedImpactCorrection()
        {
            Assert.Equal(0.50, CourseCorrectionPulsePolicy.SelectEffectFraction(170683, 600, false,
                CourseCorrectionPulsePolicy.InitialRemoteEffectFraction), 6);
        }

        [Fact]
        public void RaisesRemoteGainOnlyAfterTwoMeasuredResponsesAgree()
        {
            int confirmations = 0;
            double gain = CourseCorrectionPulsePolicy.InitialRemoteEffectFraction;
            gain = CourseCorrectionPulsePolicy.UpdateRemoteEffectFraction(gain, 0.50, 0.50, ref confirmations);
            Assert.Equal(0.50, gain, 6);
            Assert.Equal(1, confirmations);

            gain = CourseCorrectionPulsePolicy.UpdateRemoteEffectFraction(gain, 0.50, 0.48, ref confirmations);
            Assert.Equal(0.60, gain, 6);
            Assert.Equal(0, confirmations);
        }

        [Fact]
        public void RemoteGainNeverExceedsEightyPercent()
        {
            int confirmations = 1;
            double gain = CourseCorrectionPulsePolicy.UpdateRemoteEffectFraction(0.80, 0.80, 0.80, ref confirmations);
            Assert.Equal(0.80, gain, 6);
        }

        [Fact]
        public void WeakOrAdverseResponseReducesRemoteGain()
        {
            int confirmations = 1;
            double gain = CourseCorrectionPulsePolicy.UpdateRemoteEffectFraction(0.60, 0.60, 0.05, ref confirmations);
            Assert.Equal(0.30, gain, 6);
            Assert.Equal(0, confirmations);
        }

        [Fact]
        public void ReducesTheImpactEffectAsTheTargetIsApproached()
        {
            Assert.Equal(0.25, CourseCorrectionPulsePolicy.SelectEffectFraction(5000, 600, false, 0.8), 6);
            Assert.Equal(0.10, CourseCorrectionPulsePolicy.SelectEffectFraction(1500, 600, false, 0.8), 6);
            Assert.Equal(0.05, CourseCorrectionPulsePolicy.SelectEffectFraction(500, 600, false, 0.8), 6);
        }

        [Fact]
        public void LimitsAReversalToFineSurfaceControl()
        {
            Assert.Equal(0.05, CourseCorrectionPulsePolicy.SelectEffectFraction(170683, 600, true, 0.8), 6);
        }
    }
}
