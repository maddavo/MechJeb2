using MuMech;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwarePublicationGateTests
    {
        private readonly object _mun = new object();

        [Fact]
        public void NewerCompleteTargetAwareRefreshCanReplaceAcrossPhaseTransition()
        {
            var deorbit = Candidate(1, 100);
            var correction = Candidate(2, 103);
            Assert.Equal(TargetAwarePublicationDecision.Accept, Check(deorbit, null, 101));
            Assert.Equal(TargetAwarePublicationDecision.Accept, Check(correction, deorbit, 104));
        }

        [Fact]
        public void OrdinaryOrFailedRefreshCannotReplaceCommittedResult()
        {
            var committed = Candidate(1, 100);
            var ordinary = Candidate(2, 103, targetAware: false);
            var incomplete = Candidate(2, 103, complete: false);
            var unresolved = Candidate(2, 103, terrainResolved: false);
            var collision = Candidate(2, 103, clearPath: false);
            var terminal = Candidate(2, 103, terminalNecessaryBoundPasses: false);
            Assert.Equal(TargetAwarePublicationDecision.WrongModel, Check(ordinary, committed, 104));
            Assert.Equal(TargetAwarePublicationDecision.Incomplete, Check(incomplete, committed, 104));
            Assert.Equal(TargetAwarePublicationDecision.TerrainUnresolved, Check(unresolved, committed, 104));
            Assert.Equal(TargetAwarePublicationDecision.TerrainIntersection, Check(collision, committed, 104));
            Assert.Equal(TargetAwarePublicationDecision.TerminalNecessaryBoundFailed,
                Check(terminal, committed, 104));
        }

        [Fact]
        public void ResolvedImpactRemainsDiagnosticForBetaGuidance()
        {
            var first = new TargetAwareResultLineage(1, 1, _mun, 0.5, 23, 492,
                100, 100, true, true, true, true, false,
                ReentrySimulation.LandingForecastKind.ImpactForecast, true, false);
            var second = new TargetAwareResultLineage(1, 2, _mun, 0.5, 23, 492,
                103, 103, true, true, true, true, false,
                ReentrySimulation.LandingForecastKind.ImpactForecast, true, false);
            var noContact = new TargetAwareResultLineage(1, 3, _mun, 0.5, 23, 492,
                105, 105, true, true, true, true, false,
                ReentrySimulation.LandingForecastKind.ImpactForecast, false, false);
            Assert.Equal(TargetAwarePublicationDecision.ImpactOnly, Check(first, null, 101));
            Assert.Equal(TargetAwarePublicationDecision.ImpactOnly, Check(second, first, 104));
            Assert.Equal(TargetAwarePublicationDecision.ImpactOnly,
                Check(noContact, second, 106));
        }

        [Fact]
        public void HandoffOrUnvalidatedTerminalCannotClaimLandableTouchdown()
        {
            var committed = Candidate(1, 100);
            var handoff = new TargetAwareResultLineage(1, 2, _mun, 0.5, 23, 492,
                103, 103, true, true, true, true, true,
                ReentrySimulation.LandingForecastKind.LandableForecast, false, false);
            var contactMislabelledAsLanding = new TargetAwareResultLineage(1, 3, _mun,
                0.5, 23, 492, 104, 104, true, true, true, true, true,
                ReentrySimulation.LandingForecastKind.LandableForecast, true, true);
            Assert.Equal(TargetAwarePublicationDecision.Incomplete,
                Check(handoff, committed, 104));
            Assert.Equal(TargetAwarePublicationDecision.Incomplete,
                Check(contactMislabelledAsLanding, committed, 105));
            var unknown = new TargetAwareResultLineage(1, 4, _mun, 0.5, 23,
                492, 106, 106, true, true, true, true, true,
                (ReentrySimulation.LandingForecastKind)99, false, true);
            Assert.Equal(TargetAwarePublicationDecision.Incomplete,
                Check(unknown, committed, 107));
        }

        [Fact]
        public void RejectsOutOfOrderExpiredOrChangedTargetWork()
        {
            var committed = Candidate(2, 103);
            Assert.Equal(TargetAwarePublicationDecision.OutOfOrder,
                Check(Candidate(1, 100), committed, 104));
            Assert.Equal(TargetAwarePublicationDecision.ExpiredSnapshot,
                Check(Candidate(3, 105), committed, 120));
            Assert.Equal(TargetAwarePublicationDecision.ChangedBodyOrTarget,
                TargetAwarePublicationGate.Check(Candidate(3, 105), committed, 1, _mun,
                    0.5, 23.5, 492, 106, 10, 1));
            Assert.Equal(TargetAwarePublicationDecision.StaleGeneration,
                TargetAwarePublicationGate.Check(Candidate(3, 105), committed, 2, _mun,
                    0.5, 23.0, 492, 106, 10, 1));
        }

        [Fact]
        public void RefreshAndPhaseSequenceRetainsOnlyCompleteTargetAwareLineage()
        {
            // These are the same admission checks called at the predictor's
            // final active-slot write. Phase names are deliberately absent from
            // the gate: a DeorbitBurn result survives CourseCorrection and the
            // terminal phases until a valid newer transaction commits.
            TargetAwareResultLineage? committed = null;
            int version = 0;
            string phase = "DeorbitBurn";
            void Attempt(TargetAwareResultLineage candidate, double now,
                TargetAwarePublicationDecision expected, double terrain = 492,
                long generation = 1)
            {
                var decision = TargetAwarePublicationGate.Check(candidate, committed,
                    generation, _mun, 0.5, 23.0, terrain, now, 10, 0);
                Assert.Equal(expected, decision);
                if (decision == TargetAwarePublicationDecision.Accept)
                {
                    committed = candidate;
                    version++;
                }
            }

            Attempt(Candidate(1, 100), 101, TargetAwarePublicationDecision.Accept);
            phase = "CourseCorrection";
            Attempt(Candidate(2, 102, targetAware: false), 103,
                TargetAwarePublicationDecision.WrongModel);
            Attempt(Candidate(3, 102, terrainResolved: false), 103,
                TargetAwarePublicationDecision.TerrainUnresolved);
            Attempt(Candidate(4, 102, clearPath: false), 103,
                TargetAwarePublicationDecision.TerrainIntersection);
            Attempt(Candidate(5, 102, terminalNecessaryBoundPasses: false), 103,
                TargetAwarePublicationDecision.TerminalNecessaryBoundFailed);
            Assert.Equal(1, version);
            Assert.Equal(1, committed.GetValueOrDefault().Sequence);

            Attempt(Candidate(7, 105), 106, TargetAwarePublicationDecision.Accept);
            Attempt(Candidate(6, 104), 107, TargetAwarePublicationDecision.OutOfOrder);
            phase = "DecelerationBurn";
            Attempt(Candidate(8, 108), 109, TargetAwarePublicationDecision.Accept);
            phase = "KillHorizontalVelocity";
            Attempt(Candidate(9, 110), 111, TargetAwarePublicationDecision.Accept);
            phase = "FinalDescent";
            Attempt(Candidate(10, 112), 113, TargetAwarePublicationDecision.Accept);
            Assert.Equal(5, version);
            Assert.Equal(10, committed.GetValueOrDefault().Sequence);
            Assert.Equal("FinalDescent", phase);

            // Target terrain changed, then the landing generation was reset on
            // stop/abort. Neither pending completion can replace the result.
            Attempt(Candidate(11, 114), 115,
                TargetAwarePublicationDecision.ChangedBodyOrTarget, terrain: 493);
            Attempt(Candidate(12, 115), 116,
                TargetAwarePublicationDecision.StaleGeneration, generation: 2);
            Assert.Equal(5, version);
            Assert.Equal(10, committed.GetValueOrDefault().Sequence);
        }

        [Fact]
        public void ContinuitySeedRequiresFreshCompleteMatchingCommittedSearchForecast()
        {
            bool Eligible(TargetAwareResultLineage? prior, bool direct = false,
                long generation = 1, double now = 105, double lat = 0.5, double terrain = 492) =>
                TargetAwarePublicationGate.CanSeedSearch(prior, direct, generation, _mun,
                    lat, 23, terrain, now, 10);
            Assert.True(Eligible(Candidate(1, 100)));
            Assert.False(Eligible(null));
            Assert.False(Eligible(Candidate(1, 100), direct: true));
            Assert.False(Eligible(Candidate(1, 100), generation: 2));
            Assert.False(Eligible(Candidate(1, 100), now: 111));
            Assert.False(Eligible(Candidate(1, 100), now: 99));
            Assert.False(Eligible(Candidate(1, 100), lat: 0.6));
            Assert.False(Eligible(Candidate(1, 100), terrain: 493));
            Assert.False(Eligible(Candidate(1, 100, targetAware: false)));
            Assert.False(Eligible(Candidate(1, 100, complete: false)));
            Assert.False(Eligible(Candidate(1, 100, terrainResolved: false)));
            Assert.False(Eligible(Candidate(1, 100, clearPath: false)));
            Assert.False(Eligible(Candidate(1, 100, terminalNecessaryBoundPasses: false)));
        }

        private TargetAwareResultLineage Candidate(long sequence, double inputUT,
            bool targetAware = true, bool complete = true, bool terrainResolved = true,
            bool clearPath = true, bool terminalNecessaryBoundPasses = true) =>
            new TargetAwareResultLineage(1, sequence, _mun, 0.5, 23.0, 492, inputUT, inputUT,
                targetAware, complete, terrainResolved, clearPath,
                terminalNecessaryBoundPasses,
                ReentrySimulation.LandingForecastKind.LandableForecast, false, true);

        private TargetAwarePublicationDecision Check(TargetAwareResultLineage candidate,
            TargetAwareResultLineage? committed, double currentUT) =>
            TargetAwarePublicationGate.Check(candidate, committed, 1, _mun,
                0.5, 23.0, 492, currentUT, 10, 1);
    }
}
