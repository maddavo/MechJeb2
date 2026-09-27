using System.Collections.Generic;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwareBrakingPlanTests
    {
        [Fact]
        public void SelectsTheSafePoweredEndpointNearestTheTarget()
        {
            var candidates = new List<TargetAwareBrakingPlan.Candidate>
            {
                new TargetAwareBrakingPlan.Candidate(100, -800, 20, true),
                new TargetAwareBrakingPlan.Candidate(120, -15, 8, true),
                new TargetAwareBrakingPlan.Candidate(140, 250, 5, true)
            };

            Assert.True(TargetAwareBrakingPlan.TrySelect(candidates, out TargetAwareBrakingPlan.Candidate selected));
            Assert.Equal(120, selected.StartUT);
        }

        [Fact]
        public void RejectsUnsafeCandidatesEvenWhenTheyReachTheTarget()
        {
            var candidates = new List<TargetAwareBrakingPlan.Candidate>
            {
                new TargetAwareBrakingPlan.Candidate(100, 1, 1, false),
                new TargetAwareBrakingPlan.Candidate(120, 20, 0, true)
            };

            Assert.True(TargetAwareBrakingPlan.TrySelect(candidates, out TargetAwareBrakingPlan.Candidate selected));
            Assert.Equal(120, selected.StartUT);
        }

        [Fact]
        public void FailsWhenNoCandidateIsSafe()
        {
            var candidates = new List<TargetAwareBrakingPlan.Candidate>
            {
                new TargetAwareBrakingPlan.Candidate(100, 1, 1, false)
            };

            Assert.False(TargetAwareBrakingPlan.TrySelect(candidates, out _));
        }

    }
}
