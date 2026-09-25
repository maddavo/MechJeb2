using MuMech;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class FiniteBurnPrecisionTests
    {
        [Fact]
        public void DoesNotReleaseARecoveryTrimWithNineMillimetresPerSecondStillMissing()
        {
            var progress = new FiniteBurnProgress(1.09375);
            progress.Integrate(1, 1.08475);

            Assert.False(progress.IsComplete());
            Assert.True(progress.RemainingDeltaV > AirlessLandingPhaseManager.BurnCompleteDeltaV);
        }

        [Fact]
        public void ReleasesOnlyAfterTheOneMillimetrePerSecondDeliveryTolerance()
        {
            var progress = new FiniteBurnProgress(1.09375);
            progress.Integrate(1, 1.089);

            Assert.True(progress.IsComplete());
            Assert.InRange(progress.RemainingDeltaV, 0, AirlessLandingPhaseManager.BurnCompleteDeltaV);
        }
    }
}