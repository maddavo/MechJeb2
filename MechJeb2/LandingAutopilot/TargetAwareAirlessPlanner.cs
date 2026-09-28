using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace MuMech.Landing
{
    internal enum TargetAwarePlannerStage
    {
        AwaitBallistic, ResolveBallistic, ReadyCoarse, AwaitCoarse,
        ResolveCoarse, ReadyRefinement, AwaitRefinement, ResolveRefinement,
        ResolveDirectForecast, ResolveSelectedCoast, Complete, Failed
    }

    internal sealed class TargetAwareTerrainCandidate
    {
        internal readonly AirlessTargetAwareOutput Output;
        internal readonly List<AirlessTargetAwareState> QueryPath;
        internal int NextQuery;
        internal bool EarlyIntersection;
        internal bool HandoffBelowTerrain;
        internal bool Resolved;
        internal double MinimumSampledClearance = double.PositiveInfinity;
        internal double HandoffClearance = double.NaN;
        internal double LocalTerrainASL = double.NaN;

        internal TargetAwareTerrainCandidate(AirlessTargetAwareOutput output,
            AirlessTargetAwareSnapshot snapshot, bool endpointOnly = false)
        {
            Output = output;
            // The ballistic terrain pass already validates the shared coast
            // from this snapshot to first ground contact. Every brake UT is
            // before that contact, so re-querying each candidate's coast is
            // redundant and would multiply PQS work across the batch.
            QueryPath = new List<AirlessTargetAwareState>();
            double lastQueryUT = double.NegativeInfinity;
            foreach (AirlessTargetAwareState state in output.Trajectory)
                if (!endpointOnly &&
                    state.Position.magnitude - snapshot.BodyRadius <= snapshot.MaximumTerrainASL &&
                    (state.UT - lastQueryUT >= 1.0 ||
                     state.Position.magnitude - snapshot.BodyRadius <
                     snapshot.TargetTerrainASL + 500))
                {
                    QueryPath.Add(state);
                    lastQueryUT = state.UT;
                }
            if (output.Trajectory.Count > 0 && (QueryPath.Count == 0 ||
                QueryPath[QueryPath.Count - 1].UT != output.End.UT))
                QueryPath.Add(output.End);
        }

        internal bool ClearPath => Resolved && !EarlyIntersection && !HandoffBelowTerrain;
    }

    internal sealed class TargetAwareRefinementBatch
    {
        internal readonly List<AirlessTargetAwareOutput> Outputs;
        internal readonly AirlessTargetAwareOutput Selected;
        internal readonly AirlessTargetAwareOutput BracketEarlier, BracketLater;
        internal readonly bool Converged;
        internal readonly double TimingInterval, TimingDistanceEstimate;
        internal readonly double SignedDownrangeError, CrossrangeError;

        internal TargetAwareRefinementBatch(List<AirlessTargetAwareOutput> outputs,
            AirlessTargetAwareOutput selected, AirlessTargetAwareOutput bracketEarlier,
            AirlessTargetAwareOutput bracketLater, bool converged, double timingInterval,
            double timingDistanceEstimate, double downrange, double crossrange)
        {
            Outputs = outputs;
            Selected = selected;
            BracketEarlier = bracketEarlier;
            BracketLater = bracketLater;
            Converged = converged;
            TimingInterval = timingInterval;
            TimingDistanceEstimate = timingDistanceEstimate;
            SignedDownrangeError = downrange;
            CrossrangeError = crossrange;
        }
    }

    // This state machine is driven only on the flight thread. Worker methods
    // consume the copied snapshot and post immutable outputs back to it.
    internal sealed class TargetAwareAirlessPlanner
    {
        internal readonly AirlessTargetAwareSnapshot Snapshot;
        internal readonly long Generation, Sequence;
        internal readonly object BodyIdentity;
        internal bool IsDirectForecast { get; private set; }
        internal TargetAwarePlannerStage Stage { get; private set; } = TargetAwarePlannerStage.AwaitBallistic;
        internal string Failure { get; private set; }
        internal AirlessTargetAwareOutput SelectedOutput { get; private set; }
        internal TargetAwareTerrainCandidate SelectedTerrain { get; private set; }
        internal TargetAwareTerminalHandoff TerminalHandoff { get; private set; }
        internal double SignedDownrangeError { get; private set; }
        internal double CrossrangeError { get; private set; }
        internal double TimingInterval { get; private set; }
        internal double TimingDistanceEstimate { get; private set; }
        internal double BallisticContactUT { get; private set; }
        internal AbsoluteVector BallisticContact { get; private set; }
        internal double TerrainQueryMilliseconds => 1000d * _terrainTicks / Stopwatch.Frequency;
        internal int TerrainQueryCount { get; private set; }

        private readonly Func<double, double, double> _terrainAltitude;
        private readonly int _queriesPerTick, _maximumQueries;
        private readonly double _maximumTerrainMillisecondsPerTick;
        private readonly double _targetTolerance;
        private List<AirlessTargetAwareState> _ballisticSamples;
        private int _nextBallistic;
        private TargetAwareApproachFrame _approachFrame;
        private List<TargetAwareTerrainCandidate> _candidates;
        private List<TargetAwareTerrainCandidate> _coarseCandidates;
        private int _nextCandidate;
        private long _terrainTicks;
        private TargetAwareBrakingPlan.Bracket _bracket;
        private TargetAwareRefinementBatch _refinement;
        private bool _fallbackSelection;
        private List<TargetAwareTerrainCandidate> _fallbackOptions;
        private int _fallbackIndex;
        private int _nextSelectedCoast;

        internal TargetAwareAirlessPlanner(AirlessTargetAwareSnapshot snapshot, long generation,
            long sequence, object bodyIdentity, Func<double, double, double> terrainAltitude,
            double targetTolerance, int queriesPerTick, int maximumQueries,
            double maximumTerrainMillisecondsPerTick = 1,
            bool directForecast = false)
        {
            Snapshot = snapshot;
            Generation = generation;
            Sequence = sequence;
            BodyIdentity = bodyIdentity;
            IsDirectForecast = directForecast;
            _terrainAltitude = terrainAltitude ?? throw new ArgumentNullException(nameof(terrainAltitude));
            _targetTolerance = targetTolerance;
            _queriesPerTick = queriesPerTick;
            _maximumQueries = maximumQueries;
            _maximumTerrainMillisecondsPerTick = maximumTerrainMillisecondsPerTick;
            if (bodyIdentity == null || !Finite(targetTolerance) || targetTolerance <= 0 ||
                queriesPerTick <= 0 || maximumQueries < queriesPerTick ||
                !Finite(maximumTerrainMillisecondsPerTick) || maximumTerrainMillisecondsPerTick <= 0)
                throw new ArgumentException("Invalid target-aware planner inputs");
        }

        internal void SetBallisticSamples(List<AirlessTargetAwareState> samples)
        {
            if (Stage != TargetAwarePlannerStage.AwaitBallistic) throw new InvalidOperationException();
            _ballisticSamples = samples;
            _nextBallistic = 0;
            Stage = TargetAwarePlannerStage.ResolveBallistic;
        }

        internal void SetDirectOutput(AirlessTargetAwareOutput output)
        {
            if (Stage != TargetAwarePlannerStage.AwaitBallistic ||
                output == null || !output.UsesV1ControlModel)
                throw new InvalidOperationException("Incomplete live V1 forecast");
            IsDirectForecast = true;
            _fallbackSelection = true;
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(output, Snapshot) };
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveDirectForecast;
        }

        internal void SetCoarseOutputs(List<AirlessTargetAwareOutput> outputs)
        {
            if (Stage != TargetAwarePlannerStage.AwaitCoarse) throw new InvalidOperationException();
            if (outputs == null || outputs.Count != 9 || outputs.Exists(item => item == null))
            { Fail("IncompleteCoarseBatch"); return; }
            _candidates = new List<TargetAwareTerrainCandidate>(outputs.Count);
            foreach (AirlessTargetAwareOutput output in outputs)
                _candidates.Add(new TargetAwareTerrainCandidate(output, Snapshot, true));
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveCoarse;
        }

        internal void SetRefinement(TargetAwareRefinementBatch refinement)
        {
            if (Stage != TargetAwarePlannerStage.AwaitRefinement) throw new InvalidOperationException();
            if (refinement == null || !refinement.Converged || refinement.Selected == null ||
                refinement.BracketEarlier == null || refinement.BracketLater == null ||
                refinement.Outputs == null)
            {
                // A geometric sign change may straddle a branch that cannot
                // reach a safe handoff. Keep a real, validated miss available
                // for V1's post-burn CourseCorrection rather than freezing
                // the prior result.
                PrepareFallbackFromCoarse();
                return;
            }
            _refinement = refinement;
            _fallbackSelection = false;
            _coarseCandidates = _candidates;
            _candidates = new List<TargetAwareTerrainCandidate>(3);
            foreach (AirlessTargetAwareOutput output in new[]
                     { refinement.Selected, refinement.BracketEarlier, refinement.BracketLater })
                if (!_candidates.Exists(item => ReferenceEquals(item.Output, output)))
                    _candidates.Add(new TargetAwareTerrainCandidate(output, Snapshot));
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveRefinement;
        }

        internal void WorkerFailed(string reason) => Fail(reason);

        internal void AdvanceTerrain()
        {
            int remaining = _queriesPerTick;
            long tickStart = Stopwatch.GetTimestamp();
            try
            {
                if (Stage == TargetAwarePlannerStage.ResolveBallistic)
                {
                    if (_ballisticSamples == null || _ballisticSamples.Count == 0)
                    {
                        Fail("NoBallisticTerrainPass");
                        return;
                    }
                    while (remaining > 0 &&
                           (remaining == _queriesPerTick || WithinTickBudget(tickStart)) &&
                           _nextBallistic < _ballisticSamples.Count)
                    {
                        AirlessTargetAwareState state = _ballisticSamples[_nextBallistic++];
                        AbsoluteVector position = AirlessTargetAwareSimulation.ToAbsolute(state.Position,
                            state.UT, Snapshot);
                        double terrain = Query(position);
                        remaining--;
                        if (state.Position.magnitude - Snapshot.BodyRadius <= terrain)
                        {
                            BallisticContact = position;
                            BallisticContactUT = state.UT;
                            Vector3d target = AirlessTargetAwareSimulation.SurfaceDirection(
                                Snapshot.TargetLatitude, Snapshot.TargetLongitude, Snapshot);
                            Vector3d contact = AirlessTargetAwareSimulation.SurfaceDirection(
                                position.Latitude, position.Longitude, Snapshot);
                            if (!TargetAwareApproachFrame.TryCreate(target, contact,
                                Snapshot.BodyRadius, out _approachFrame))
                                Fail("InvalidBallisticApproachFrame");
                            else Stage = TargetAwarePlannerStage.ReadyCoarse;
                            return;
                        }
                    }
                    if (_nextBallistic == _ballisticSamples.Count) Fail("NoBallisticTerrainContact");
                }
                else if (Stage == TargetAwarePlannerStage.ResolveCoarse ||
                         Stage == TargetAwarePlannerStage.ResolveRefinement ||
                         Stage == TargetAwarePlannerStage.ResolveDirectForecast)
                {
                    while (remaining > 0 &&
                           (remaining == _queriesPerTick || WithinTickBudget(tickStart)) &&
                           _nextCandidate < _candidates.Count)
                    {
                        TargetAwareTerrainCandidate candidate = _candidates[_nextCandidate];
                        if (!candidate.Output.ReachedHandoff || candidate.QueryPath.Count == 0)
                        {
                            candidate.Resolved = true;
                            candidate.EarlyIntersection = true;
                            _nextCandidate++;
                            continue;
                        }
                        if (candidate.NextQuery == candidate.QueryPath.Count)
                        {
                            candidate.Resolved = true;
                            _nextCandidate++;
                            continue;
                        }
                        int index = candidate.NextQuery++;
                        AirlessTargetAwareState state = candidate.QueryPath[index];
                        AbsoluteVector position = AirlessTargetAwareSimulation.ToAbsolute(state.Position,
                            state.UT, Snapshot);
                        double terrain = Query(position);
                        remaining--;
                        double clearance = state.Position.magnitude - Snapshot.BodyRadius - terrain;
                        candidate.MinimumSampledClearance = Math.Min(candidate.MinimumSampledClearance, clearance);
                        bool endpoint = index == candidate.QueryPath.Count - 1;
                        if (endpoint)
                        {
                            candidate.HandoffClearance = clearance;
                            candidate.LocalTerrainASL = terrain;
                            candidate.HandoffBelowTerrain = clearance < 0;
                        }
                        else if (clearance <= 0)
                        {
                            candidate.EarlyIntersection = true;
                            candidate.LocalTerrainASL = terrain;
                            candidate.HandoffClearance = clearance;
                            candidate.Resolved = true;
                            _nextCandidate++;
                        }
                    }
                    if (_nextCandidate == _candidates.Count)
                    {
                        if (Stage == TargetAwarePlannerStage.ResolveCoarse) FinishCoarse();
                        else if (Stage == TargetAwarePlannerStage.ResolveRefinement) FinishRefinement();
                        else FinishDirectForecast();
                    }
                }
                else if (Stage == TargetAwarePlannerStage.ResolveSelectedCoast)
                {
                    while (remaining > 0 &&
                           (remaining == _queriesPerTick || WithinTickBudget(tickStart)) &&
                           _nextSelectedCoast < SelectedOutput.CoastSamples.Count)
                    {
                        AirlessTargetAwareState state =
                            SelectedOutput.CoastSamples[_nextSelectedCoast++];
                        if (state.Position.magnitude - Snapshot.BodyRadius >
                            Snapshot.MaximumTerrainASL)
                            continue;
                        AbsoluteVector position = AirlessTargetAwareSimulation.ToAbsolute(
                            state.Position, state.UT, Snapshot);
                        double terrain = Query(position);
                        remaining--;
                        if (state.Position.magnitude - Snapshot.BodyRadius <= terrain)
                        {
                            if (_fallbackSelection && !IsDirectForecast)
                            {
                                _fallbackIndex++;
                                PrepareFallbackCandidate();
                            }
                            else if (!IsDirectForecast && _coarseCandidates != null)
                                PrepareFallbackFromCoarse();
                            else Fail("SelectedCoastTerrainIntersection");
                            return;
                        }
                    }
                    if (_nextSelectedCoast == SelectedOutput.CoastSamples.Count)
                        Stage = TargetAwarePlannerStage.Complete;
                }
            }
            catch (Exception ex)
            {
                Fail("TerrainQueryFailed:" + ex.GetType().Name + ":" + ex.Message +
                     ":queries=" + TerrainQueryCount);
            }
        }

        internal void BeginCoarseWorker()
        {
            if (Stage != TargetAwarePlannerStage.ReadyCoarse) throw new InvalidOperationException();
            Stage = TargetAwarePlannerStage.AwaitCoarse;
        }

        internal void BeginRefinementWorker()
        {
            if (Stage != TargetAwarePlannerStage.ReadyRefinement) throw new InvalidOperationException();
            Stage = TargetAwarePlannerStage.AwaitRefinement;
        }

        internal List<AirlessTargetAwareOutput> RunCoarse()
        {
            var outputs = new List<AirlessTargetAwareOutput>(9);
            double earliest = Snapshot.InputUT +
                (Snapshot.HasV1ControlModel && !Snapshot.DecelerationAlreadyTriggered ? 15 : 0);
            double latest = BallisticContactUT - 2;
            if (latest <= earliest + 4) throw new InvalidOperationException("NoBrakingWindow");
            for (int i = 0; i < 9; ++i)
                outputs.Add(AirlessTargetAwareSimulation.Run(Snapshot,
                    earliest + i / 8d * (latest - earliest), true));
            return outputs;
        }

        internal TargetAwareRefinementBatch RunRefinement()
        {
            TargetAwareBrakingPlan.Candidate low = _bracket.Earlier;
            TargetAwareBrakingPlan.Candidate high = _bracket.Later;
            AirlessTargetAwareOutput lowOutput = _candidates.Find(item =>
                item.Output.BrakeUT == low.StartUT)?.Output;
            AirlessTargetAwareOutput highOutput = _candidates.Find(item =>
                item.Output.BrakeUT == high.StartUT)?.Output;
            if (lowOutput == null || highOutput == null)
                throw new InvalidOperationException("CoarseBracketOutputMissing");
            if (low.DownrangeError > 0)
            {
                TargetAwareBrakingPlan.Candidate temp = low;
                low = high;
                high = temp;
                AirlessTargetAwareOutput tempOutput = lowOutput;
                lowOutput = highOutput;
                highOutput = tempOutput;
            }
            AirlessTargetAwareOutput bestOutput = null;
            double bestDown = double.PositiveInfinity;
            double bestCross = double.NaN;
            var outputs = new List<AirlessTargetAwareOutput>(25);
            double maximumObservedSlope = Math.Abs((high.DownrangeError - low.DownrangeError) /
                (high.StartUT - low.StartUT));
            for (int i = 0; i < 24; ++i)
            {
                double interval = Math.Abs(high.StartUT - low.StartUT);
                if (i > 0 && Math.Max(Math.Abs(low.DownrangeError), Math.Abs(high.DownrangeError)) < _targetTolerance &&
                    interval * maximumObservedSlope < _targetTolerance)
                    break;
                double midpoint = (low.StartUT + high.StartUT) / 2;
                AirlessTargetAwareOutput output = AirlessTargetAwareSimulation.Run(Snapshot, midpoint, true);
                if (!output.ReachedHandoff)
                    return new TargetAwareRefinementBatch(outputs, bestOutput,
                        lowOutput, highOutput, false, interval,
                        double.NaN, bestDown, bestCross);
                outputs.Add(output);
                if (!TryProject(output, out double down, out double across))
                    return new TargetAwareRefinementBatch(outputs, bestOutput,
                        lowOutput, highOutput, false, interval,
                        double.NaN, bestDown, bestCross);
                if (Math.Abs(down) < Math.Abs(bestDown))
                {
                    bestOutput = output;
                    bestDown = down;
                    bestCross = across;
                }
                var point = new TargetAwareBrakingPlan.Candidate(midpoint, down, across, true);
                if (low.DownrangeError * down <= 0)
                {
                    maximumObservedSlope = Math.Max(maximumObservedSlope,
                        Math.Abs((high.DownrangeError - down) / (high.StartUT - midpoint)));
                    high = point;
                    highOutput = output;
                }
                else
                {
                    maximumObservedSlope = Math.Max(maximumObservedSlope,
                        Math.Abs((down - low.DownrangeError) / (midpoint - low.StartUT)));
                    low = point;
                    lowOutput = output;
                }
            }
            double finalInterval = Math.Abs(high.StartUT - low.StartUT);
            bool converged = bestOutput != null &&
                Math.Max(Math.Abs(low.DownrangeError), Math.Abs(high.DownrangeError)) < _targetTolerance &&
                finalInterval * maximumObservedSlope < _targetTolerance;
            return new TargetAwareRefinementBatch(outputs, bestOutput, lowOutput, highOutput,
                converged, finalInterval,
                finalInterval * maximumObservedSlope, bestDown, bestCross);
        }

        internal bool TryProject(AirlessTargetAwareOutput output, out double downrange, out double crossrange)
        {
            AbsoluteVector endpoint = AirlessTargetAwareSimulation.ToAbsolute(output.End.Position,
                output.End.UT, Snapshot);
            Vector3d direction = AirlessTargetAwareSimulation.SurfaceDirection(endpoint.Latitude,
                endpoint.Longitude, Snapshot);
            return _approachFrame.TryProject(direction, out downrange, out crossrange);
        }

        private double Query(AbsoluteVector position)
        {
            if (TerrainQueryCount >= _maximumQueries)
                throw new InvalidOperationException("TerrainQueryBudgetExceeded");
            long start = Stopwatch.GetTimestamp();
            double terrain;
            try { terrain = _terrainAltitude(position.Latitude, position.Longitude); }
            finally
            {
                _terrainTicks += Stopwatch.GetTimestamp() - start;
                TerrainQueryCount++;
            }
            if (!Finite(terrain)) throw new InvalidOperationException("TerrainUnresolved");
            return terrain;
        }

        private bool WithinTickBudget(long tickStart) =>
            1000d * (Stopwatch.GetTimestamp() - tickStart) / Stopwatch.Frequency <
            _maximumTerrainMillisecondsPerTick;

        private void FinishCoarse()
        {
            var points = new List<TargetAwareBrakingPlan.Candidate>(_candidates.Count);
            foreach (TargetAwareTerrainCandidate candidate in _candidates)
            {
                double down = double.NaN;
                double across = double.NaN;
                // A colliding far-downrange candidate can still establish a
                // geometric sign change. Only the *final* local bracket and
                // selected path must be terrain clear; an obstacle beyond the
                // root cannot erase a valid earlier braking solution.
                bool safe = candidate.Output.ReachedHandoff &&
                    TryProject(candidate.Output, out down, out across);
                points.Add(new TargetAwareBrakingPlan.Candidate(candidate.Output.BrakeUT,
                    safe ? down : double.NaN, safe ? across : double.NaN, safe));
            }
            if (TargetAwareBrakingPlan.TryFindDownrangeBracket(points, out _bracket))
            {
                Stage = TargetAwarePlannerStage.ReadyRefinement;
                return;
            }

            // V1 CourseCorrection needs an honest miss in order to change the
            // orbit. A brake-time root need not exist on the current orbit.
            // Publish the nearest terrain-clear controller forecast, not an
            // invented near-target endpoint or an old result from deorbit.
            PrepareFallbackFromCoarse();
        }

        private void PrepareFallbackFromCoarse()
        {
            List<TargetAwareTerrainCandidate> source = _coarseCandidates ?? _candidates;
            _fallbackOptions = new List<TargetAwareTerrainCandidate>();
            foreach (TargetAwareTerrainCandidate candidate in source)
            {
                if (!candidate.Resolved || !candidate.Output.ReachedHandoff ||
                    !candidate.Output.UsesV1ControlModel ||
                    !TryProject(candidate.Output, out double down, out _))
                    continue;
                _fallbackOptions.Add(candidate);
            }
            if (_fallbackOptions.Count == 0)
            {
                Fail("NoTerrainClearControllerForecast");
                return;
            }
            _fallbackOptions.Sort((a, b) =>
            {
                TryProject(a.Output, out double aDown, out _);
                TryProject(b.Output, out double bDown, out _);
                return Math.Abs(aDown).CompareTo(Math.Abs(bDown));
            });
            _fallbackSelection = true;
            _fallbackIndex = 0;
            TimingInterval = double.NaN;
            TimingDistanceEstimate = double.NaN;
            PrepareFallbackCandidate();
        }

        private void PrepareFallbackCandidate()
        {
            if (_fallbackIndex >= _fallbackOptions.Count)
            {
                Fail("NoTerrainClearControllerForecast");
                return;
            }
            AirlessTargetAwareOutput output = _fallbackOptions[_fallbackIndex].Output;
            TryProject(output, out double down, out double across);
            SignedDownrangeError = down;
            CrossrangeError = across;
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(output, Snapshot) };
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveRefinement;
        }

        private void FinishRefinement()
        {
            if (_fallbackSelection)
            {
                TargetAwareTerrainCandidate candidate = _candidates[0];
                if (!candidate.Resolved)
                {
                    _fallbackIndex++;
                    PrepareFallbackCandidate();
                    return;
                }
                SelectedOutput = candidate.Output;
                SelectedTerrain = candidate;
                CompleteOrRevalidateSelected();
                return;
            }
            SelectedTerrain = FindTerrain(_refinement.Selected);
            TargetAwareTerrainCandidate earlier = FindTerrain(_refinement.BracketEarlier);
            TargetAwareTerrainCandidate later = FindTerrain(_refinement.BracketLater);
            if (SelectedTerrain == null || earlier == null || later == null)
            {
                PrepareFallbackFromCoarse();
                return;
            }
            double firstUT = Math.Min(_refinement.BracketEarlier.BrakeUT,
                _refinement.BracketLater.BrakeUT);
            double lastUT = Math.Max(_refinement.BracketEarlier.BrakeUT,
                _refinement.BracketLater.BrakeUT);
            if (_refinement.Selected.BrakeUT < firstUT ||
                _refinement.Selected.BrakeUT > lastUT)
            {
                PrepareFallbackFromCoarse();
                return;
            }
            foreach (TargetAwareTerrainCandidate candidate in
                     new[] { SelectedTerrain, earlier, later })
            {
                if (!candidate.Resolved)
                {
                    PrepareFallbackFromCoarse();
                    return;
                }
                // Bracket paths establish only the timing sign. Local terrain
                // tests the selected path for clearance before publication.
            }
            SelectedOutput = _refinement.Selected;
            SignedDownrangeError = _refinement.SignedDownrangeError;
            CrossrangeError = _refinement.CrossrangeError;
            TimingInterval = _refinement.TimingInterval;
            TimingDistanceEstimate = _refinement.TimingDistanceEstimate;
            CompleteOrRevalidateSelected();
        }

        private void FinishDirectForecast()
        {
            TargetAwareTerrainCandidate candidate = _candidates[0];
            if (!candidate.Resolved)
            {
                Fail("DirectControllerPathTerrainUnresolved");
                return;
            }
            SelectedOutput = candidate.Output;
            SelectedTerrain = candidate;
            SignedDownrangeError = double.NaN;
            CrossrangeError = double.NaN;
            CompleteOrRevalidateSelected();
        }

        private void CompleteOrRevalidateSelected()
        {
            // Targeted airless V1 now keeps its speed-policy reference at the
            // selected target terrain. Local terrain is a clearance test; it
            // must not change the controller's policy or re-start the worker.
            if (!SelectedTerrain.ClearPath)
            {
                if (_fallbackSelection && !IsDirectForecast)
                {
                    _fallbackIndex++;
                    PrepareFallbackCandidate();
                }
                else if (SelectedOutput.UsesV1ControlModel && !IsDirectForecast &&
                         _coarseCandidates != null)
                    PrepareFallbackFromCoarse();
                else Fail("ControllerPathTerrainUnsafe");
                return;
            }
            var effectiveSnapshot = Snapshot;
            TerminalHandoff = TargetAwareTerminalHandoff.Assess(effectiveSnapshot,
                SelectedOutput, SelectedTerrain.LocalTerrainASL);
            if (!TerminalHandoff.NecessaryControlBoundPasses)
            {
                if (_fallbackSelection && !IsDirectForecast)
                {
                    _fallbackIndex++;
                    PrepareFallbackCandidate();
                }
                else if (SelectedOutput.UsesV1ControlModel && !IsDirectForecast &&
                         _coarseCandidates != null)
                    PrepareFallbackFromCoarse();
                else Fail("TerminalVerticalStoppingBoundFailed");
                return;
            }
            _nextSelectedCoast = 0;
            Stage = SelectedOutput.CoastSamples.Count == 0 ?
                TargetAwarePlannerStage.Complete : TargetAwarePlannerStage.ResolveSelectedCoast;
        }

        private TargetAwareTerrainCandidate FindTerrain(AirlessTargetAwareOutput output) =>
            _candidates.Find(item => ReferenceEquals(item.Output, output)) ??
            _coarseCandidates?.Find(item => ReferenceEquals(item.Output, output));

        private void Fail(string reason)
        {
            Failure = reason;
            SelectedOutput = null;
            SelectedTerrain = null;
            Stage = TargetAwarePlannerStage.Failed;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
