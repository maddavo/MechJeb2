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

        [Fact]
        public void BracketsSignedDownrangeAcrossUnorderedSafeSamples()
        {
            var candidates = new List<TargetAwareBrakingPlan.Candidate>
            {
                new TargetAwareBrakingPlan.Candidate(140, 400, 900, true),
                new TargetAwareBrakingPlan.Candidate(100, -800, 20, true),
                new TargetAwareBrakingPlan.Candidate(120, -15, 700, true)
            };

            Assert.True(TargetAwareBrakingPlan.TryFindDownrangeBracket(candidates, out var bracket));
            Assert.Equal(120, bracket.Earlier.StartUT);
            Assert.Equal(140, bracket.Later.StartUT);
            Assert.Equal(700, bracket.Earlier.CrossrangeError);
        }

        [Fact]
        public void UnsafeIntermediateBranchCannotCreateBracket()
        {
            var candidates = new List<TargetAwareBrakingPlan.Candidate>
            {
                new TargetAwareBrakingPlan.Candidate(100, -100, 0, true),
                new TargetAwareBrakingPlan.Candidate(110, 0, 0, false),
                new TargetAwareBrakingPlan.Candidate(120, 100, 0, true)
            };

            Assert.False(TargetAwareBrakingPlan.TryFindDownrangeBracket(candidates, out _));
        }

    }
}
