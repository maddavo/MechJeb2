using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class DeorbitBurnStartPolicyTests
    {
        [Fact]
        public void WaitsOutsideTheTargetPhaseCorridor()
        {
            Assert.False(DeorbitBurnStartPolicy.IsInTargetingCorridor(90.0, 43.0, 1.0));
        }

        [Fact]
        public void EquatorialTargetCanStartInsideTheTargetPhaseCorridor()
        {
            // In an equatorial orbit, a target on the orbital plane has a
            // normal-angle near 90 degrees. That is diagnostic, not a veto.
            Assert.True(DeorbitBurnStartPolicy.IsInTargetingCorridor(90.0, 75.0, 1.0));
        }

        [Fact]
        public void InvalidInPlaneVelocityCannotStartABurn()
        {
            Assert.False(DeorbitBurnStartPolicy.IsInTargetingCorridor(90.0, 75.0, 91.0));
        }

        [Fact]
        public void InvalidGeometryCannotStartABurn()
        {
            Assert.False(DeorbitBurnStartPolicy.IsInTargetingCorridor(double.NaN, 75.0, 1.0));
        }

        [Fact]
        public void CandidateWithoutAnImpactCannotAuthorizeDeorbit()
        {
            Assert.False(DeorbitBurnStartPolicy.IsCandidateAcceptable(false, 0.0, 1000.0));
        }

        [Fact]
        public void CandidateOutsideItsEndpointCorridorCannotAuthorizeDeorbit()
        {
            Assert.False(DeorbitBurnStartPolicy.IsCandidateAcceptable(true, 185000.0, 1000.0));
        }

        [Fact]
        public void CandidateInsideItsEndpointCorridorCanAuthorizeDeorbit()
        {
            Assert.True(DeorbitBurnStartPolicy.IsCandidateAcceptable(true, 999.0, 1000.0));
        }
    }
}
