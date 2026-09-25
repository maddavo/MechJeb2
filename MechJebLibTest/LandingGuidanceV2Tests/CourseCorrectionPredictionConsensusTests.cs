using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CourseCorrectionPredictionConsensusTests
    {
        [Fact]
        public void NoisyValidPostBurnSolutionsProduceAMeanCorrection()
        {
            Vector3d first = new Vector3d(2, 0, 0);
            Vector3d mean = CourseCorrectionPredictionConsensus.AddSample(first, 1,
                new Vector3d(0, 2, 0));

            Assert.InRange(Vector3d.Distance(mean, new Vector3d(1, 1, 0)), 0, 1e-12);
            Assert.True(CourseCorrectionPredictionConsensus.IsUsable(mean, 0.05));
        }

        [Fact]
        public void ExactConflictingSolutionsCannotCommandABurn()
        {
            Vector3d mean = CourseCorrectionPredictionConsensus.AddSample(new Vector3d(2, 0, 0), 1,
                new Vector3d(-2, 0, 0));

            Assert.False(CourseCorrectionPredictionConsensus.IsUsable(mean, 0.05));
        }
    }
}
