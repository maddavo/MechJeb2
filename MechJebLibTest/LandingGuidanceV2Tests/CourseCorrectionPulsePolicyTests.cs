using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CourseCorrectionPulsePolicyTests
    {
        [Fact]
        public void UsesALargerBoundedPulseForALargeRemoteCorrection()
        {
            Assert.Equal(5.0, CourseCorrectionPulsePolicy.SelectPulseDv(22.1, 170683, 600, false), 6);
        }

        [Fact]
        public void LimitsRemotePulseToOneQuarterOfTheRequestedCorrection()
        {
            Assert.Equal(2.5, CourseCorrectionPulsePolicy.SelectPulseDv(10, 12000, 600, false), 6);
        }

        [Fact]
        public void RetainsSmallPulsesNearTheTarget()
        {
            Assert.Equal(0.25, CourseCorrectionPulsePolicy.SelectPulseDv(5, 1500, 600, false), 6);
            Assert.Equal(0.1, CourseCorrectionPulsePolicy.SelectPulseDv(5, 500, 600, false), 6);
        }

        [Fact]
        public void LimitsAReversingPulseToFineControl()
        {
            Assert.Equal(0.1, CourseCorrectionPulsePolicy.SelectPulseDv(22.1, 170683, 600, true), 6);
        }

        [Fact]
        public void DoesNotCreateAPulseForInvalidOrSmallerRequests()
        {
            Assert.Equal(0, CourseCorrectionPulsePolicy.SelectPulseDv(double.NaN, 10000, 600, false));
            Assert.Equal(0.06, CourseCorrectionPulsePolicy.SelectPulseDv(0.06, 170683, 600, false), 6);
        }
    }
}
