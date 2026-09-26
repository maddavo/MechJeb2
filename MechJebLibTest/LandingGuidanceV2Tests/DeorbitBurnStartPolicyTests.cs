using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class DeorbitBurnStartPolicyTests
    {
        [Fact]
        public void AlignedOrbitStillWaitsForTheTargetPhaseCorridor()
        {
            Assert.False(DeorbitBurnStartPolicy.IsInTargetingCorridor(0.1, 43.0, 1.0));
        }

        [Fact]
        public void AlignedOrbitCanStartInsideTheTargetPhaseCorridor()
        {
            Assert.True(DeorbitBurnStartPolicy.IsInTargetingCorridor(0.1, 75.0, 1.0));
        }

        [Fact]
        public void OffPlaneOrbitCannotBypassThePlaneChangeRequirement()
        {
            Assert.False(DeorbitBurnStartPolicy.IsInTargetingCorridor(11.0, 75.0, 1.0));
        }

        [Fact]
        public void InvalidGeometryCannotStartABurn()
        {
            Assert.False(DeorbitBurnStartPolicy.IsInTargetingCorridor(double.NaN, 75.0, 1.0));
        }
    }
}