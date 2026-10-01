using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using MuMech;
using MuMech.Landing;
using Xunit;
using Xunit.Abstractions;

namespace MechJebLibTest.LandingPredictionTests
{
    public class RecordedMunTerrainReplayTests
    {
        private readonly ITestOutputHelper _output;
        public RecordedMunTerrainReplayTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void FirstIntersectionAndAdvancingDeorbitInputsProducePoweredForecasts()
        {
            // Actual snapshots and PQS samples from d680. Interpolation supplies
            // an offline oracle, not a claim that this is the entire Mun map.
            var terrain = new RecordedTerrain();
            var cache = new TargetAwareTerrainCache();
            var body = new object();
            int count = 0;
            TargetAwareResultLineage? committed = null;
            foreach (var row in ReadRows("Mun-d680-deorbit-inputs.csv"))
            {
                var snapshot = Snapshot(row);
                var planner = new TargetAwareAirlessPlanner(snapshot, 1,
                    (long)row["submissionId"], body, terrain.Height, 200, 32, 1536,
                    1, false, cache);
                planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
                int updates = Run(planner);
                _output.WriteLine($"input={snapshot.InputUT:F6} stage={planner.Stage} " +
                    $"failure={planner.Failure} queries={planner.TerrainQueryCount} " +
                    $"hits={planner.TerrainCacheHits} updates={updates} " +
                    $"policy={planner.PolicyEscalations} brake={planner.SelectedOutput?.BrakeUT:F6} " +
                    $"last candidate={planner.LastCandidateFailure}");
                Assert.True(planner.Stage == TargetAwarePlannerStage.Complete &&
                    planner.TerminalTouchdownValidated, planner.Failure ?? planner.Stage.ToString());
                Assert.InRange(planner.TerrainQueryCount, 1, 1536);
                Assert.True(planner.TerrainCacheHits > 0);
                _output.WriteLine($"Beta trajectory reference={planner.SelectedOutput.Trajectory[0].UT:F6} " +
                    $"earliest release={planner.SelectedOutput.Trajectory[0].UT - 5:F6}");
                // The live gate's existing ten-second age limit, not a new
                // numerical targeting tolerance. Worker time is reported too.
                Assert.True(updates * snapshot.MinDt < 10);
                var lineage = new TargetAwareResultLineage(1, (long)row["submissionId"], body,
                    snapshot.TargetLatitude, snapshot.TargetLongitude, snapshot.TargetTerrainASL,
                    snapshot.InputUT, snapshot.InputUT, true, true, true, true,
                    planner.TerminalHandoff.NecessaryControlBoundPasses,
                    ReentrySimulation.LandingForecastKind.LandableForecast, false, true);
                Assert.Equal(TargetAwarePublicationDecision.Accept,
                    TargetAwarePublicationGate.Check(lineage, committed, 1, body,
                        snapshot.TargetLatitude, snapshot.TargetLongitude, snapshot.TargetTerrainASL,
                        snapshot.InputUT + updates * snapshot.MinDt, 10, 1));
                committed = lineage;
                var contact = planner.NominalTerminalTerrain.Contact;
                var location = AirlessTargetAwareSimulation.ToAbsolute(contact.Position, contact.UT, snapshot);
                _output.WriteLine($"touchdown lat={location.Latitude:F6} lon={location.Longitude:F6} " +
                    $"terrain={planner.NominalTerminalTerrain.ContactTerrainASL:F3} " +
                    $"handoff horizontal={planner.TerminalHandoff.EndHorizontalSpeed:F3} " +
                    $"stopping distance={planner.TerminalHandoff.IdealHoverHorizontalStoppingDistance:F3}");
                count++;
            }
            Assert.Equal(4, count);
            _output.WriteLine($"Offline oracle maximum sample distance={terrain.MaximumDistance:F2}m");
        }

        [Fact]
        public void GammaMovingDeorbitUsesFreshLocalSolutionsWithoutIncreasingReversals()
        {
            var terrain = new RecordedTerrain("Mun-gamma-demand-terrain.csv");
            var body = new object();
            var broadCache = new TargetAwareTerrainCache();
            var localCache = new TargetAwareTerrainCache();
            TargetAwareAirlessPlanner? prior = null;
            double lastBroadMiss = double.NaN, lastLocalMiss = double.NaN;
            int broadReversals = 0, localReversals = 0, broadValid = 0, localValid = 0,
                localSearches = 0, fallbacks = 0;
            double lastLocalMissToTarget = double.NaN;
            foreach (var row in ReadRows("Mun-gamma-deorbit-inputs.csv"))
            {
                var snapshot = Snapshot(row);
                var broad = new TargetAwareAirlessPlanner(snapshot, 1,
                    (long)row["submissionId"], body, terrain.Height, 200, 512, 1536,
                    1, false, broadCache);
                broad.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
                Run(broad, 512);
                bool fresh = prior != null && snapshot.InputUT >= prior.Snapshot.InputUT &&
                    snapshot.InputUT - prior.Snapshot.InputUT <=
                        MechJebModuleLandingPredictions.TargetAwareMaximumSnapshotAgeSeconds;
                var localSnapshot = fresh ? snapshot.WithV1LandingTerrain(prior!.SelectedPolicyTerrainASL) : snapshot;
                var local = new TargetAwareAirlessPlanner(localSnapshot, 1,
                    (long)row["submissionId"], body, terrain.Height, 200, 512, 1536,
                    1, false, localCache, fresh ? prior!.SelectedOutput.BrakeUT : double.NaN);
                local.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(localSnapshot));
                Run(local, 512);
                double Miss(TargetAwareAirlessPlanner p)
                {
                    var location = AirlessTargetAwareSimulation.ToAbsolute(
                        p.NominalTerminalTerrain.Contact.Position, p.NominalTerminalTerrain.Contact.UT, p.Snapshot);
                    double lat = location.Latitude * Math.PI / 180,
                        targetLat = snapshot.TargetLatitude * Math.PI / 180;
                    double h = Math.Pow(Math.Sin((lat - targetLat) / 2), 2) +
                        Math.Cos(lat) * Math.Cos(targetLat) *
                        Math.Pow(Math.Sin((location.Longitude - snapshot.TargetLongitude) * Math.PI / 360), 2);
                    return 2 * snapshot.BodyRadius * Math.Asin(Math.Min(1, Math.Sqrt(h)));
                }
                if (broad.TerminalTouchdownValidated)
                {
                    double miss = Miss(broad);
                    if (!double.IsNaN(lastBroadMiss) && miss > lastBroadMiss + 1) broadReversals++;
                    lastBroadMiss = miss; broadValid++;
                }
                if (local.UsedLocalSearch) localSearches++;
                if (local.BroadSearchFallback) fallbacks++;
                if (local.TerminalTouchdownValidated)
                {
                    Assert.True(local.SelectedTerrain.ClearPath);
                    Assert.True(local.TerminalHandoff.NecessaryControlBoundPasses);
                    Assert.InRange(local.TerrainQueryCount, 1, 1536);
                    double miss = Miss(local);
                    if (!double.IsNaN(lastLocalMiss) && miss > lastLocalMiss + 1) localReversals++;
                    lastLocalMiss = lastLocalMissToTarget = miss; localValid++;
                    prior = local;
                }
                _output.WriteLine($"gamma input={row["submissionId"]} broad={broad.Stage}/{broad.Failure} " +
                    $"local={local.Stage}/{local.Failure} seed={local.ContinuitySeedUT:F3} " +
                    $"localSearch={local.UsedLocalSearch} fallback={local.BroadSearchFallback} " +
                    $"brake={local.SelectedOutput?.BrakeUT:F3} miss={lastLocalMissToTarget:F2}");
            }
            _output.WriteLine($"Gamma replay broad/local valid={broadValid}/{localValid} " +
                $"reversals={broadReversals}/{localReversals} localSearches={localSearches} " +
                $"fallbacks={fallbacks} finalMiss={lastLocalMissToTarget:F2}m " +
                $"oracleMaxDistance={terrain.MaximumDistance:F2}m");
            Assert.True(localSearches > 0);
            Assert.True(localValid >= broadValid);
            // Both searches can now have zero reversals. Preserve the previous
            // accepted continuity baseline (137 valid inputs, three reversals).
            Assert.True(localReversals <= broadReversals);
            Assert.InRange(localReversals, 0, 3);
            Assert.True(localValid >= 137);
            Assert.InRange(lastLocalMissToTarget, 0, 200);
        }

        [Fact]
        public void ContinuitySeedIsResimulatedAndTerminalDirectForecastExcludesSearch()
        {
            var snapshot = Snapshot(ReadRows("Mun-gamma-deorbit-inputs.csv").Last());
            var terrain = new RecordedTerrain("Mun-gamma-demand-terrain.csv");
            var body = new object();
            var first = new TargetAwareAirlessPlanner(snapshot, 1, 1, body,
                terrain.Height, 200, 512, 1536);
            first.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            Run(first, 512);
            Assert.True(first.TerminalTouchdownValidated);
            var repeat = new TargetAwareAirlessPlanner(
                snapshot.WithV1LandingTerrain(first.SelectedPolicyTerrainASL), 1, 2, body,
                terrain.Height, 200, 512, 1536, previousBrakeUT: first.SelectedOutput.BrakeUT);
            repeat.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(repeat.Snapshot));
            Run(repeat, 512);
            Assert.True(repeat.TerminalTouchdownValidated);
            Assert.True(repeat.UsedLocalSearch);
            Assert.False(repeat.BroadSearchFallback);
            Assert.Equal(first.SelectedOutput.BrakeUT, repeat.SelectedOutput.BrakeUT);
            Assert.NotSame(first.SelectedOutput, repeat.SelectedOutput);
            Assert.True(repeat.SelectedTerrain.ClearPath);
            var direct = new TargetAwareAirlessPlanner(snapshot, 1, 3, body,
                terrain.Height, 200, 512, 1536, directForecast: true,
                previousBrakeUT: first.SelectedOutput.BrakeUT);
            Assert.True(double.IsNaN(direct.ContinuitySeedUT));
            Assert.False(direct.UsedLocalSearch);
        }

        [Fact]
        public void UnsafeLocalNeighborhoodFallsBackWithinOriginalTerrainBudget()
        {
            var snapshot = Snapshot(ReadRows("Mun-gamma-deorbit-inputs.csv").Last());
            var terrain = new RecordedTerrain("Mun-gamma-demand-terrain.csv");
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                terrain.Height, 200, 512, 1536, previousBrakeUT: snapshot.InputUT + 10000);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            Run(planner, 512);
            Assert.True(planner.UsedLocalSearch);
            Assert.True(planner.BroadSearchFallback);
            Assert.True(planner.TerminalTouchdownValidated, planner.Failure);
            Assert.True(planner.SelectedTerrain.ClearPath);
            Assert.InRange(planner.TerrainQueryCount, 1, 1536);
        }

        [Fact]
        public void ChangedUnresolvedTerrainCannotReusePreviouslyValidTrajectory()
        {
            var snapshot = Snapshot(ReadRows("Mun-gamma-deorbit-inputs.csv").Last());
            var terrain = new RecordedTerrain("Mun-gamma-demand-terrain.csv");
            var first = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                terrain.Height, 200, 512, 1536);
            first.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            Run(first, 512);
            Assert.True(first.TerminalTouchdownValidated);
            var changed = new TargetAwareAirlessPlanner(snapshot, 1, 2, first.BodyIdentity,
                (lat, lon) => double.NaN, 200, 512, 1536,
                previousBrakeUT: first.SelectedOutput.BrakeUT);
            changed.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            Run(changed, 512);
            Assert.Equal(TargetAwarePlannerStage.Failed, changed.Stage);
            Assert.False(changed.TerminalTouchdownValidated);
            Assert.Null(changed.SelectedOutput);
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void AdvancingCapturedDeorbitForecastsFitTwoHzDeliveryBudget(bool continuity)
        {
            var terrain = new RecordedTerrain();
            var cache = new TargetAwareTerrainCache();
            var body = new object();
            int count = 0;
            TargetAwareAirlessPlanner? prior = null;
            foreach (var row in ReadRows("Mun-d680-deorbit-inputs.csv"))
            {
                var snapshot = Snapshot(row);
                if (continuity && prior != null)
                    snapshot = snapshot.WithV1LandingTerrain(prior.SelectedPolicyTerrainASL);
                int samplesPerTick = MechJebModuleLandingPredictions.TargetAwareTerrainSamplesPerTick;
                var planner = new TargetAwareAirlessPlanner(snapshot, 1,
                    (long)row["submissionId"], body, terrain.Height, 200,
                    samplesPerTick, 1536, 1, false, cache,
                    continuity && prior != null ? prior.SelectedOutput.BrakeUT : double.NaN);
                double latency = RunAtFlightCadence(planner, snapshot.MinDt, samplesPerTick,
                    poweredDeorbit: continuity && count > 0);
                double publicationInterval = Math.Max(
                    MechJebModuleLandingPredictions.TargetAwareRefreshSeconds, latency);
                _output.WriteLine($"cadence input={snapshot.InputUT:F6} queries={planner.TerrainQueryCount} " +
                    $"samples={planner.TerrainSampleCount} latency={latency:F3}s " +
                    $"publicationInterval={publicationInterval:F3}s");
                Assert.True(planner.TerminalTouchdownValidated, planner.Failure ?? planner.Stage.ToString());
                Assert.InRange(planner.TerrainQueryCount, 1, 1536);
                // Startup can fill cold terrain coverage; subsequent moving
                // snapshots must fit the requested half-second delivery rate.
                if (count++ > 0) Assert.InRange(publicationInterval, 0, 0.5);
                prior = planner;
            }
            Assert.Equal(4, count);
        }

        internal static double RunAtFlightCadence(TargetAwareAirlessPlanner planner,
            double flightTickSeconds, int samplesPerTick, bool poweredDeorbit = false)
        {
            // Mirror production's order: consume completed worker, resolve
            // terrain, then queue the next worker in the SAME flight update.
            // Worker calculations run here solely to measure their duration;
            // their output is withheld until the simulated completion tick.
            double now = 0, completionTime = 0;
            Action? completion = null;
            void Queue(Func<Action> work)
            {
                var timer = Stopwatch.StartNew();
                completion = work();
                timer.Stop();
                completionTime = now + timer.Elapsed.TotalSeconds;
            }
            Queue(() => {
                var path = AirlessTargetAwareSimulation.BallisticTerrainPass(planner.Snapshot);
                return () => planner.SetBallisticSamples(path);
            });
            for (int tick = 0; tick < 3000; tick++)
            {
                now += flightTickSeconds;
                if (completion != null && now >= completionTime)
                {
                    var apply = completion;
                    completion = null;
                    apply();
                }
                int samples = planner.TerrainSampleCount;
                if (poweredDeorbit)
                {
                    // The same rejected captured path must remain intact while
                    // coasting, and before the moving-input refresh deadline.
                    Assert.False(planner.RefreshRejectedMovingSnapshot(now, false,
                        MechJebModuleLandingPredictions.TargetAwareRefreshSeconds));
                    Assert.False(planner.RefreshRejectedMovingSnapshot(Math.Min(now, 0.5), true,
                        MechJebModuleLandingPredictions.TargetAwareRefreshSeconds));
                }
                planner.RefreshRejectedMovingSnapshot(now, poweredDeorbit,
                    MechJebModuleLandingPredictions.TargetAwareRefreshSeconds);
                planner.AdvanceTerrain();
                Assert.InRange(planner.TerrainSampleCount - samples, 0, samplesPerTick);
                switch (planner.Stage)
                {
                    case TargetAwarePlannerStage.ReadyCoarse:
                        planner.BeginCoarseWorker();
                        Queue(() => {
                            var result = planner.RunCoarse();
                            return () => planner.SetCoarseOutputs(result);
                        }); break;
                    case TargetAwarePlannerStage.ReadyRefinement:
                        planner.BeginRefinementWorker();
                        Queue(() => {
                            var result = planner.RunRefinement();
                            return () => planner.SetRefinement(result);
                        }); break;
                    case TargetAwarePlannerStage.ReadyPolicyEscalation:
                        planner.BeginPolicyEscalation();
                        Queue(() => {
                            var result = planner.RunPolicyEscalation();
                            return () => planner.SetPolicyEscalation(result);
                        }); break;
                    case TargetAwarePlannerStage.ReadyTerminal:
                        planner.BeginTerminalWorker();
                        Queue(() => {
                            var result = planner.RunTerminalEnvelope();
                            return () => planner.SetTerminalEnvelope(result);
                        }); break;
                    case TargetAwarePlannerStage.Complete: return now;
                    case TargetAwarePlannerStage.Failed:
                        if (poweredDeorbit) return now;
                        Assert.True(false, planner.Failure); return now;
                }
            }
            throw new InvalidOperationException("Cadence replay did not finish");
        }

        [Fact]
        public void SpatialReusePreservesFirstPoweredEndpointAgainstUnquantizedReference()
        {
            var snapshot = Snapshot(ReadRows("Mun-d680-deorbit-inputs.csv").First());
            var terrain = new RecordedTerrain();
            TargetAwareAirlessPlanner Forecast(bool spatial)
            {
                // The larger offline-only reference budget measures the old
                // exact-coordinate cost; the live budget stays at 1,536.
                var p = new TargetAwareAirlessPlanner(snapshot, 1, 101, new object(),
                    terrain.Height, 200, 32, spatial ? 1536 : 20000, 1, false,
                    new TargetAwareTerrainCache(spatialReuseEnabled: spatial));
                p.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
                Run(p);
                Assert.True(p.TerminalTouchdownValidated, p.Failure ?? p.Stage.ToString());
                return p;
            }
            var spatial = Forecast(true);
            var exact = Forecast(false);
            double displacement = (spatial.NominalTerminalTerrain.Contact.Position -
                exact.NominalTerminalTerrain.Contact.Position).magnitude;
            _output.WriteLine($"spatial/exact displacement={displacement:F3}m " +
                $"brake delta={spatial.SelectedOutput.BrakeUT - exact.SelectedOutput.BrakeUT:F3}s " +
                $"queries={spatial.TerrainQueryCount}/{exact.TerrainQueryCount}");
            Assert.InRange(displacement, 0, 200); // existing targeting tolerance
            Assert.True(spatial.TerrainQueryCount < exact.TerrainQueryCount);
        }

        private int Run(TargetAwareAirlessPlanner planner, int samplesPerTick = 32)
        {
            int updates = 0;
            string? lastFailure = null;
            for (int i = 0; i < 3000 && planner.Stage != TargetAwarePlannerStage.Complete &&
                 planner.Stage != TargetAwarePlannerStage.Failed; i++)
            {
                if (lastFailure != planner.LastCandidateFailure)
                {
                    lastFailure = planner.LastCandidateFailure;
                    _output.WriteLine($"candidate rejected: {lastFailure}; terminal contact=" +
                        $"{planner.NominalTerminalTerrain.HasContact} " +
                        $"terrain={planner.NominalTerminalTerrain.ContactTerrainASL:F3}");
                }
                switch (planner.Stage)
                {
                    case TargetAwarePlannerStage.ReadyCoarse:
                        planner.BeginCoarseWorker(); planner.SetCoarseOutputs(planner.RunCoarse()); break;
                    case TargetAwarePlannerStage.ReadyRefinement:
                        planner.BeginRefinementWorker(); planner.SetRefinement(planner.RunRefinement()); break;
                    case TargetAwarePlannerStage.ReadyPolicyEscalation:
                        _output.WriteLine($"policy recheck {planner.SelectedPolicyTerrainASL:F1} " +
                            $"brake={planner.SelectedOutput?.BrakeUT:F2} " +
                            $"handoff={planner.SelectedOutput?.EndSurfaceSpeed:F2} " +
                            $"local={planner.SelectedTerrain?.LocalTerrainASL:F2}");
                        planner.BeginPolicyEscalation(); planner.SetPolicyEscalation(planner.RunPolicyEscalation()); break;
                    case TargetAwarePlannerStage.ReadyTerminal:
                        _output.WriteLine($"terminal policy={planner.SelectedPolicyTerrainASL:F1} " +
                            $"brake={planner.SelectedOutput?.BrakeUT:F2} " +
                            $"horizontal={planner.TerminalHandoff.EndHorizontalSpeed:F2}");
                        planner.BeginTerminalWorker();
                        var envelope = planner.RunTerminalEnvelope();
                        if (!envelope.Nominal.FinalDescent?.ReachedTerrain == true ||
                            !envelope.Delayed.FinalDescent?.ReachedTerrain == true)
                            _output.WriteLine($"incomplete terminal nominal=" +
                                $"{envelope.Nominal.FinalDescent?.ReachedTerrain} delayed=" +
                                $"{envelope.Delayed.FinalDescent?.ReachedTerrain} " +
                                $"nominal clearance={envelope.Nominal.End.Position.magnitude - planner.Snapshot.BodyRadius - envelope.Nominal.TerrainASL:F2} " +
                                $"delayed clearance={envelope.Delayed.End.Position.magnitude - planner.Snapshot.BodyRadius - envelope.Delayed.TerrainASL:F2}");
                        planner.SetTerminalEnvelope(envelope); break;
                    default:
                        int queries = planner.TerrainQueryCount, samples = planner.TerrainSampleCount;
                        planner.AdvanceTerrain(); updates++;
                        Assert.InRange(planner.TerrainQueryCount - queries, 0, samplesPerTick);
                        Assert.InRange(planner.TerrainSampleCount - samples, 0, samplesPerTick);
                        break;
                }
            }
            return updates;
        }

        internal static AirlessTargetAwareSnapshot Snapshot(Dictionary<string, double> r)
        {
            Vector3d V(string name) => new Vector3d(r[name + "0"], r[name + "1"], r[name + "2"]);
            var source = new AirlessTargetAwareSnapshot(r["inputUT"], r["captureEpochUT"],
                r["bodyRadius"], r["bodyMu"], r["bodyGeeASL"], r["rotationPeriod"],
                r["targetLatitude"], r["targetLongitude"], r["targetTerrainASL"], r["decelEndASL"],
                r["maxThrustAcceleration"], r["dt"], r["minDt"], r["maxOrbits"],
                r["minimumTerrainASL"], r["maximumTerrainASL"], V("positionBCI"),
                V("velocityBCI"), V("angularVelocity"), V("bodyAxis0"), V("bodyAxis90"), V("bodyAxisNorth"));
            return new AirlessTargetAwareSnapshot(source, r["controllerInitialMass"],
                r["controllerMaximumThrust"], r["controllerMinimumThrust"],
                r["controllerMaximumMassFlow"], r["controllerMinimumMassFlow"],
                r["controllerPolicyTerrainRadius"], r["controllerPolicyGravity"], r["controllerPolicyThrust"],
                false, V("forward"), r["controllerMinimumCommandThrottle"], r["controllerMaximumCommandThrottle"],
                r["controllerThrottleSmoothingSeconds"], r["controllerInitialAppliedThrottle"],
                r["controllerTouchdownSpeed"], r["controllerBottomOffset"], r["controllerPreviousTransThrottle"]);
        }

        internal static IEnumerable<Dictionary<string, double>> ReadRows(string file)
        {
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory,
                "LandingPredictionTests", "Fixtures", file);
            string[] lines = File.ReadAllLines(path), header = lines[0].Split(',');
            foreach (string line in lines.Skip(1))
            {
                string[] fields = line.Split(',');
                var row = new Dictionary<string, double>();
                for (int i = 0; i < header.Length; i++)
                    row[header[i]] = double.Parse(fields[i], CultureInfo.InvariantCulture);
                yield return row;
            }
        }

        internal sealed class RecordedTerrain
        {
            private readonly (double Lat, double Lon, double Height)[] _points;
            internal double MaximumDistance { get; private set; }
            internal RecordedTerrain(string file = "Mun-d680-demand-terrain.csv") => _points = ReadRows(file)
                .Select(r => (r["latitude"], r["longitude"], r["heightASL"]))
                .OrderBy(p => p.Item2).ToArray();

            internal double Height(double lat, double lon)
            {
                int low = 0, high = _points.Length;
                while (low < high)
                {
                    int mid = (low + high) / 2;
                    if (_points[mid].Lon < lon) low = mid + 1; else high = mid;
                }
                double weight = 0, height = 0, nearest = double.PositiveInfinity;
                // Nearby captured points define a local inverse-distance
                // estimate; missing coverage is explicitly unresolved.
                for (int i = Math.Max(0, low - 20); i < Math.Min(_points.Length, low + 20); i++)
                {
                    double x = (_points[i].Lon - lon) * Math.Cos(lat * Math.PI / 180);
                    double y = _points[i].Lat - lat;
                    double metres = Math.Sqrt(x * x + y * y) * 200000 * Math.PI / 180;
                    nearest = Math.Min(nearest, metres);
                    double w = 1 / Math.Max(1e-8, metres * metres);
                    weight += w; height += w * _points[i].Height;
                }
                MaximumDistance = Math.Max(MaximumDistance, nearest);
                return nearest <= 500 && weight > 0 ? height / weight : double.NaN;
            }
        }
    }
}
