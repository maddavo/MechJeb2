using MuMech;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class LocalLandingSafetyTests
    {
        [Fact]
        public void LocalTargetMoveUsesOnlyDeltaVAboveTheProtectedReserve()
        {
            V2LocalDivertDecision accepted = V2LocalDivertGate.Decide(100, 40, 30, 10);
            Assert.True(accepted.Accepted);
            Assert.Equal(12, accepted.Cost, 6);

            V2LocalDivertDecision rejected = V2LocalDivertGate.Decide(51.9, 40, 30, 10);
            Assert.False(rejected.Accepted);
            Assert.Contains("protected terminal-divert reserve", rejected.Reason);
        }

        [Fact]
        public void LocalTargetMoveRejectsInvalidInputsBeforeItCanChangeRedTarget()
        {
            V2LocalDivertDecision decision = V2LocalDivertGate.Decide(double.NaN, 40, 30, 10);
            Assert.False(decision.Accepted);
            Assert.Contains("not physically valid", decision.Reason);
        }

        [Theory]
        [InlineData(true, 0, 1, 1, false)]
        [InlineData(false, 100, 15.01, 1, false)]
        [InlineData(false, 100, 1, 15.01, false)]
        [InlineData(false, 100, 15, 15, true)]
        public void LocalSiteAssessmentRejectsUnsafeTerrainWithoutRetargeting(bool ocean, double terrain,
            double slope, double roughness, bool expectedAccepted)
        {
            V2LocalSiteDecision decision = V2LocalSiteGate.Assess(ocean, terrain, slope, roughness);
            Assert.Equal(expectedAccepted, decision.Accepted);
            if (!expectedAccepted) Assert.NotEmpty(decision.Reason);
        }
    }
}
