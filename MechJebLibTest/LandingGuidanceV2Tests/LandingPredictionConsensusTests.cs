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
        public void AirlessTerrainIterationRequiresTheSimulatedContactRadiusToMatchTheEndpointTerrain()
        {
            Assert.False(LandingPredictionTerrainConvergence.IsSelfConsistent(0, 1791));
            Assert.True(LandingPredictionTerrainConvergence.IsSelfConsistent(1791.0, 1792.5));
            Assert.Equal(1791, LandingPredictionTerrainConvergence.NextIterationTerrainAltitude(1791, 0));
        }

        [Fact]
        public void AlternatingBranchesNeverSatisfyTheConsecutivePublicationGate()
        {
            // A arrives: hold it. B replaces A: hold it. A replaces B: hold it.
            // The former code compared the final A to the old published A and
            // republished it, which allowed an A/B visual and control loop.
            Assert.False(LandingPredictionTerrainConvergence.HasConsecutiveAgreement(false, false));
            Assert.False(LandingPredictionTerrainConvergence.HasConsecutiveAgreement(true, false));
            Assert.False(LandingPredictionTerrainConvergence.HasConsecutiveAgreement(true, false));
            Assert.True(LandingPredictionTerrainConvergence.HasConsecutiveAgreement(true, true));
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
