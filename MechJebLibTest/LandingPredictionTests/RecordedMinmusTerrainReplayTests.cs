using System;
using System.Linq;
using System.Collections.Generic;
using MuMech.Landing;
using Xunit;
using Xunit.Abstractions;

namespace MechJebLibTest.LandingPredictionTests
{
    public class RecordedMinmusTerrainReplayTests
    {
        private readonly ITestOutputHelper _output;
        public RecordedMinmusTerrainReplayTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void RecordedApproachesHaveBoundedSearchAndNoHandoffOnlyLanding()
        {
            var terrain = new MinmusTerrain();
            var cache = new TargetAwareTerrainCache();
            var body = new object();
            int landable = 0;
            foreach (var row in RecordedMunTerrainReplayTests.ReadRows("Minmus-8ce76-inputs.csv")
                .Where(r => r["submissionId"] != 2976))
            {
                var snapshot = RecordedMunTerrainReplayTests.Snapshot(row);
                var planner = new TargetAwareAirlessPlanner(snapshot, 1,
                    (long)row["submissionId"], body, terrain.Height, 200, 512, 1536,
                    1000, terrainCache: cache);
                planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
                Drive(planner);
                _output.WriteLine($"Minmus {row["submissionId"]}: {planner.Stage} " +
                    $"landable={planner.TerminalTouchdownValidated} failure={planner.Failure} " +
                    $"last={planner.LastCandidateFailure} queries={planner.TerrainQueryCount} " +
                    $"policyRuns={planner.PolicySimulationCount} reuse={planner.PolicySimulationCacheHits} " +
                    $"coverage={planner.CoverageRefinements} terrainDistance={terrain.MaximumDistance:F2}m");
                Assert.InRange(planner.TerrainQueryCount, 0, 1536);
                Assert.InRange(planner.CoverageRefinements, 0, 1);
                Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
                Assert.True(planner.TerminalTouchdownValidated, planner.Failure ?? planner.LastCandidateFailure);
                landable++;
                Assert.True(planner.SelectedTerrain.ClearPath);
                Assert.True(planner.NominalTerminalTerrain.HasContact);
                Assert.True(planner.NominalTerminalTerrain.Contact.UT >=
                    planner.TerminalEnvelope.Nominal.HorizontalKillEndUT);
            }
            Assert.Equal(9, landable);
            Assert.InRange(terrain.MaximumDistance, 0, 500);
        }

        [Fact]
        public void LastMinmusBrakingInputRetainsThrustAndPredictsObservedTerminalTimescale()
        {
            var row = RecordedMunTerrainReplayTests.ReadRows("Minmus-8ce76-inputs.csv")
                .Single(r => r["submissionId"] == 2976);
            var source = RecordedMunTerrainReplayTests.Snapshot(row);
            var snapshot = new AirlessTargetAwareSnapshot(source, source.InitialMass,
                source.MaximumThrust, source.MinimumThrust, source.MaximumMassFlow,
                source.MinimumMassFlow, source.PolicyTerrainRadius, source.PolicyGravity,
                source.PolicyThrust, true, source.InitialForward,
                source.MinimumCommandThrottle, source.MaximumCommandThrottle,
                source.ThrottleSmoothingSeconds, source.InitialAppliedThrottle,
                source.TouchdownSpeed, source.BottomOffset, source.PreviousTransThrottle);
            var brake = AirlessTargetAwareSimulation.RunNominalV1(snapshot);
            Assert.True(brake.ReachedHandoff);
            Assert.True(brake.EndForward.sqrMagnitude > 0.9);
            Assert.True(brake.EndAppliedThrottle > 0);
            var terminal = AirlessTargetAwareSimulation.RunNominalTerminal(snapshot, brake, 0, 1);
            var ideal = AirlessTargetAwareSimulation.RunIdealHorizontalKillBound(snapshot, brake.End, brake.EndMass);
            double actualKillEnd = 24664542.873992812, actualContact = 24664586.973991826;
            _output.WriteLine($"Minmus terminal predicted kill duration={terminal.HorizontalKillEndUT - brake.End.UT:F3}s " +
                $"ideal={ideal.Trajectory.Last().UT - brake.End.UT:F3}s " +
                $"kill UT error={terminal.HorizontalKillEndUT - actualKillEnd:F3}s " +
                $"contact UT error={terminal.End.UT - actualContact:F3}s");
            Assert.True(terminal.FinalDescent.ReachedTerrain);
            Assert.InRange(Math.Abs(terminal.HorizontalKillEndUT - actualKillEnd), 0, 2);
            Assert.InRange(Math.Abs(terminal.End.UT - actualContact), 0, 12);
            Assert.True(terminal.HorizontalKillEndUT < ideal.Trajectory.Last().UT);
        }

        [Fact]
        public void FailedCoarseScreenChecksInterveningTimesBeforeReportingImpact()
        {
            var row = RecordedMunTerrainReplayTests.ReadRows("Minmus-8ce76-inputs.csv")
                .Single(r => r["submissionId"] == 2494);
            var snapshot = RecordedMunTerrainReplayTests.Snapshot(row);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (lat, lon) => 0, 200, 512, 1536, 1000);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            while (planner.Stage == TargetAwarePlannerStage.ResolveBallistic) planner.AdvanceTerrain();
            planner.BeginCoarseWorker();
            // Inject an exhausted coarse screen to exercise the search state
            // boundary. The intervening batch uses real V1-policy integration.
            planner.SetCoarseOutputs(planner.RunCoarse().Select(o => new AirlessTargetAwareOutput(
                o.BrakeUT, o.CoastSamples, o.Trajectory, false, o.EndSurfaceSpeed,
                o.VirtualDeltaV, o.Steps, true, o.EndMass, o.EndForward, o.EndAppliedThrottle)).ToList());
            Drive(planner);
            Assert.Equal(1, planner.CoverageRefinements);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.TerminalTouchdownValidated, planner.Failure);
            Assert.InRange(planner.TerrainQueryCount, 1, 1536);
        }

        [Fact]
        public void IdenticalPolicyWorkerInputReusesOnlyItsOwnTransaction()
        {
            var row = RecordedMunTerrainReplayTests.ReadRows("Minmus-8ce76-inputs.csv")
                .Single(r => r["submissionId"] == 2175);
            var snapshot = RecordedMunTerrainReplayTests.Snapshot(row);
            var terrain = new MinmusTerrain();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                terrain.Height, 200, 512, 1536, 1000);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            for (int i = 0; i < 1000 && planner.Stage != TargetAwarePlannerStage.ReadyPolicyEscalation; i++)
            {
                if (planner.Stage == TargetAwarePlannerStage.ReadyCoarse)
                { planner.BeginCoarseWorker(); planner.SetCoarseOutputs(planner.RunCoarse()); }
                else if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
                { planner.BeginRefinementWorker(); planner.SetRefinement(planner.RunRefinement()); }
                else planner.AdvanceTerrain();
            }
            Assert.Equal(TargetAwarePlannerStage.ReadyPolicyEscalation, planner.Stage);
            planner.BeginPolicyEscalation();
            var first = planner.RunPolicyEscalation();
            int simulations = planner.PolicySimulationCount;
            var again = planner.RunPolicyEscalation();
            Assert.Same(first, again);
            Assert.Equal(simulations, planner.PolicySimulationCount);
            Assert.True(planner.PolicySimulationCacheHits > 0);
            Assert.Equal(0, new TargetAwareAirlessPlanner(snapshot, 1, 2, planner.BodyIdentity,
                terrain.Height, 200, 512, 1536).PolicySimulationCacheHits);
        }

        internal static void Drive(TargetAwareAirlessPlanner planner)
        {
            for (int i = 0; i < 4000 && planner.Stage != TargetAwarePlannerStage.Complete &&
                planner.Stage != TargetAwarePlannerStage.Failed; i++)
                switch (planner.Stage)
                {
                    case TargetAwarePlannerStage.ReadyCoarse:
                        planner.BeginCoarseWorker(); planner.SetCoarseOutputs(planner.RunCoarse()); break;
                    case TargetAwarePlannerStage.ReadyRefinement:
                        planner.BeginRefinementWorker(); planner.SetRefinement(planner.RunRefinement()); break;
                    case TargetAwarePlannerStage.ReadyPolicyEscalation:
                        planner.BeginPolicyEscalation(); planner.SetPolicyEscalation(planner.RunPolicyEscalation()); break;
                    case TargetAwarePlannerStage.ReadyTerminal:
                        planner.BeginTerminalWorker(); planner.SetTerminalEnvelope(planner.RunTerminalEnvelope()); break;
                    default: planner.AdvanceTerrain(); break;
                }
        }

        internal sealed class MinmusTerrain
        {
            private readonly double _bodyRadius;
            private readonly Dictionary<(int, int), List<(double Lat, double Lon, double Height)>> _cells =
                new Dictionary<(int, int), List<(double, double, double)>>();
            internal double MaximumDistance { get; private set; }
            internal MinmusTerrain(string file = "Minmus-8ce76-demand-terrain.csv", double bodyRadius = 60000)
            {
                _bodyRadius = bodyRadius;
                foreach (var r in RecordedMunTerrainReplayTests.ReadRows(file))
                {
                    double lat = r["latitude"], lon = r["longitude"];
                    var key = ((int)Math.Floor(lat * 4), (int)Math.Floor(lon * 4));
                    if (!_cells.TryGetValue(key, out var cell))
                        _cells[key] = cell = new List<(double, double, double)>();
                    cell.Add((lat, lon, r["heightASL"]));
                }
            }
            internal double Height(double lat, double lon)
            {
                var nearest = new List<(double Distance, double Height)>();
                int y = (int)Math.Floor(lat * 4), x = (int)Math.Floor(lon * 4);
                double cos = Math.Cos(lat * Math.PI / 180);
                for (int j = y - 2; j <= y + 2; j++)
                    for (int i = x - 3; i <= x + 3; i++)
                        if (_cells.TryGetValue((j, (i + 720 + 1440) % 1440 - 720), out var cell))
                            foreach (var point in cell)
                            {
                                double dx = ((point.Lon - lon + 540) % 360 - 180) * cos;
                                double dy = point.Lat - lat;
                                double distance = Math.Sqrt(dx * dx + dy * dy) * _bodyRadius * Math.PI / 180;
                                if (nearest.Count < 4 || distance < nearest[nearest.Count - 1].Distance)
                                {
                                    nearest.Add((distance, point.Height));
                                    nearest.Sort((a, b) => a.Distance.CompareTo(b.Distance));
                                    if (nearest.Count > 4) nearest.RemoveAt(4);
                                }
                            }
                double closest = nearest.Count == 0 ? double.PositiveInfinity : nearest[0].Distance;
                MaximumDistance = Math.Max(MaximumDistance, closest);
                if (closest > 500) return double.NaN;
                double weight = 0, height = 0;
                foreach (var point in nearest)
                {
                    double w = 1 / Math.Max(1e-8, point.Distance * point.Distance);
                    weight += w; height += w * point.Height;
                }
                return height / weight;
            }
        }
    }
}
