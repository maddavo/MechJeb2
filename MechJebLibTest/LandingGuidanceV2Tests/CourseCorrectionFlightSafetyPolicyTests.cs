using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CourseCorrectionFlightSafetyPolicyTests
    {
        [Fact]
        public void RetainsAnImpactMarginBelowTheBodySurface()
        {
            Assert.True(CourseCorrectionFlightSafetyPolicy.HasImpactMargin(-1500, 1000));
            Assert.False(CourseCorrectionFlightSafetyPolicy.HasImpactMargin(-999, 1000));
            Assert.False(CourseCorrectionFlightSafetyPolicy.HasImpactMargin(1, 1000));
        }

        [Fact]
        public void RejectsMaterialPostBurnTargetRegression()
        {
            Assert.False(CourseCorrectionFlightSafetyPolicy.IsPostBurnEndpointAcceptable(147200, 148000));
        }

        [Fact]
        public void AcceptsAnImprovedEndpointAndPredictionScaleNoise()
        {
            Assert.True(CourseCorrectionFlightSafetyPolicy.IsPostBurnEndpointAcceptable(147200, 140000));
            Assert.True(CourseCorrectionFlightSafetyPolicy.IsPostBurnEndpointAcceptable(147200, 147600));
        }

        [Fact]
        public void RejectsInvalidPredictionErrors()
        {
            Assert.False(CourseCorrectionFlightSafetyPolicy.IsPostBurnEndpointAcceptable(double.NaN, 1));
            Assert.False(CourseCorrectionFlightSafetyPolicy.IsPostBurnEndpointAcceptable(1, double.NaN));
        }
    }
}