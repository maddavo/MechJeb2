using MuMech;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class LandingPredictionConsensusTests
    {
        [Fact]
        public void RejectsFlatAndMountainBranchesWithDifferentArrivalTimes()
        {
            var flat = Result(1000, 0);
            var mountain = Result(1035, 1791);

            Assert.False(LandingPredictionConsensus.Agrees(flat, mountain, 200, 100));
        }

        [Fact]
        public void AcceptsNearbyPredictionsFromAContinuousCoast()
        {
            var first = Result(1000, 24);
            var second = Result(1001.4, 31);
            second.InputUT = first.InputUT + 0.25;

            Assert.True(LandingPredictionConsensus.Agrees(first, second, 200, 75));
        }

        [Fact]
        public void RejectsTerrainBranchEvenWhenBodySpaceDistanceAppearsSmall()
        {
            var flat = Result(1000, 0);
            var mountain = Result(1001, 650);
            mountain.InputUT = flat.InputUT + 0.1;

            Assert.False(LandingPredictionConsensus.Agrees(flat, mountain, 200, 20));
        }

        [Fact]
        public void AirlessTerrainProfileSelectsTheFirstRealSurfaceCrossing()
        {
            // The descending path crosses a 300 m ridge before reaching the
            // flat lowland at sea level. A scalar terrain-height iteration
            // may jump between both branches; profile contact must select the
            // first physical intersection on the path.
            Assert.Equal(2, AirlessTerrainProfileContact.FindFirstContactIndex(
                new[] { 1000d, 510d, 295d, 120d, -2d },
                new[] { 0d, 0d, 300d, 0d, 0d }));
        }

        [Fact]
        public void AirlessTerrainProfileIgnoresUnrelatedLaterTerrain()
        {
            Assert.Equal(3, AirlessTerrainProfileContact.FindFirstContactIndex(
                new[] { 600d, 420d, 250d, 80d, -1d },
                new[] { 0d, 0d, 0d, 100d, 400d }));
        }

        [Fact]
        public void AirlessTerrainProfileRequiresMatchingFiniteSamples()
        {
            Assert.Equal(-1, AirlessTerrainProfileContact.FindFirstContactIndex(
                new[] { 10d, double.NaN, -1d }, new[] { 0d, 0d }));
            Assert.Equal(-1, AirlessTerrainProfileContact.FindFirstContactIndex(
                new[] { 10d, double.NaN }, new[] { 0d, 0d }));
        }

        [Fact]
        public void DisplacedTerrainBranchRequiresThreeConsistentSamples()
        {
            int required = LandingPredictionTerrainConvergence.RequiredSamples(true, true);

            Assert.Equal(3, required);
            Assert.False(LandingPredictionTerrainConvergence.CanPublish(1, required));
            Assert.False(LandingPredictionTerrainConvergence.CanPublish(2, required));
            Assert.True(LandingPredictionTerrainConvergence.CanPublish(3, required));
        }

        [Fact]
        public void AlternatingDisplacedTerrainPairsCannotReplacePublishedEndpoint()
        {
            int required = LandingPredictionTerrainConvergence.RequiredSamples(true, true);
            int samples = 0;

            // A/A has already made the current published endpoint. B/B and
            // C/C are each internally consistent pairs, but neither has the
            // third observation needed to take over from A.
            samples = LandingPredictionTerrainConvergence.NextCompatibleSampleCount(samples, false); // B
            samples = LandingPredictionTerrainConvergence.NextCompatibleSampleCount(samples, true);  // B
            Assert.False(LandingPredictionTerrainConvergence.CanPublish(samples, required));

            samples = LandingPredictionTerrainConvergence.NextCompatibleSampleCount(samples, false); // C
            samples = LandingPredictionTerrainConvergence.NextCompatibleSampleCount(samples, true);  // C
            Assert.False(LandingPredictionTerrainConvergence.CanPublish(samples, required));
        }

        [Fact]
        public void NormalContinuousEndpointStillPublishesAfterTwoSamples()
        {
            int required = LandingPredictionTerrainConvergence.RequiredSamples(true, false);

            Assert.Equal(2, required);
            Assert.False(LandingPredictionTerrainConvergence.CanPublish(1, required));
            Assert.True(LandingPredictionTerrainConvergence.CanPublish(2, required));
        }

        private static ReentrySimulation.Result Result(double endUt, double endAsl)
        {
            return new ReentrySimulation.Result
            {
                InputUT = 100,
                EndUT = endUt,
                EndASL = endAsl
            };
        }
    }
}
