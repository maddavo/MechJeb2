using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class CourseCorrectionPulseSafetyTests
    {
        [Fact]
        public void PreservesARequestedPulseThatKeepsTheImpactCorridor()
        {
            double result = CourseCorrectionPulseSafety.LimitToImpactPreservingMagnitude(1.0, 0.05, _ => true);
            Assert.Equal(1.0, result, 6);
        }

        [Fact]
        public void ReducesAPulseBeforeItWouldLoseTheImpactCorridor()
        {
            double result = CourseCorrectionPulseSafety.LimitToImpactPreservingMagnitude(1.0, 0.05, value => value <= 0.35);
            Assert.InRange(result, 0.349, 0.3501);
        }

        [Fact]
        public void RejectsAPulseWhenNoUsefulMagnitudePreservesImpact()
        {
            double result = CourseCorrectionPulseSafety.LimitToImpactPreservingMagnitude(1.0, 0.05, value => value <= 0.02);
            Assert.Equal(0, result);
        }

        [Fact]
        public void RejectsAPulseWhenTheStartingOrbitDoesNotReachDescent()
        {
            double result = CourseCorrectionPulseSafety.LimitToImpactPreservingMagnitude(1.0, 0.05, _ => false);
            Assert.Equal(0, result);
        }
    }
}
