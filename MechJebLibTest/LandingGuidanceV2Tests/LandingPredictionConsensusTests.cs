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