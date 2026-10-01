using System;
using System.Linq;
using MuMech;
using MuMech.Landing;
using Xunit;
using Xunit.Abstractions;

namespace MechJebLibTest.LandingPredictionTests
{
    public class RecordedMunStallReplayTests
    {
        private readonly ITestOutputHelper _output;
        public RecordedMunStallReplayTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void CapturedLateDeorbitInputsRecoverWithoutTerrainBudgetExhaustion()
        {
            var terrain = new RecordedMinmusTerrainReplayTests.MinmusTerrain(
                "Mun-28a5-stall-terrain.csv", 200000);
            var cache = new TargetAwareTerrainCache();
            var body = new object();
            TargetAwareAirlessPlanner? prior = null;
            foreach (var row in RecordedMunTerrainReplayTests.ReadRows("Mun-28a5-stall-inputs.csv")
                .Where(r => r["submissionId"] >= 1973 && r["submissionId"] <= 1977))
            {
                var snapshot = RecordedMunTerrainReplayTests.Snapshot(row);
                bool fresh = prior != null && snapshot.InputUT - prior.Snapshot.InputUT <= 10;
                if (fresh) snapshot = snapshot.WithV1LandingTerrain(prior!.SelectedPolicyTerrainASL);
                var planner = new TargetAwareAirlessPlanner(snapshot, 2,
                    (long)row["submissionId"], body, terrain.Height, 200, 512, 1536,
                    1, terrainCache: cache, previousBrakeUT: fresh ? prior!.SelectedOutput.BrakeUT : row["previousBrakeUT"]);
                double latency = RecordedMunTerrainReplayTests.RunAtFlightCadence(planner,
                    snapshot.MinDt, MechJebModuleLandingPredictions.TargetAwareTerrainSamplesPerTick,
                    poweredDeorbit: true);
                _output.WriteLine($"stall id={row["submissionId"]} stage={planner.Stage} " +
                    $"valid={planner.TerminalTouchdownValidated} failure={planner.Failure} " +
                    $"queries={planner.TerrainQueryCount} latency={latency:F3}s " +
                    $"oracle={terrain.MaximumDistance:F2}m");
                Assert.InRange(planner.TerrainQueryCount, 1, 1535);
                if (row["submissionId"] == 1975 || row["submissionId"] == 1976)
                {
                    Assert.Equal("MovingDeorbitSnapshotRefresh", planner.Failure);
                    Assert.False(planner.TerminalTouchdownValidated);
                    Assert.Null(planner.SelectedOutput);
                    Assert.InRange(latency, 0, 1.0);
                    // Rejection does not alter the last committed seed/result.
                    Assert.Equal(1974, prior!.Sequence);
                }
                else
                {
                    Assert.True(planner.TerminalTouchdownValidated, planner.Failure ?? planner.LastCandidateFailure);
                    if (row["submissionId"] == 1974) Assert.InRange(latency, 0, 0.5);
                    prior = planner;
                }
            }
        }
    }
}
