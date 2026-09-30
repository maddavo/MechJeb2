using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace MuMech.Landing
{
    internal enum TargetAwarePlannerStage
    {
        AwaitBallistic, ResolveBallistic, ReadyCoarse, AwaitCoarse,
        ResolveCoarse, ReadyRefinement, AwaitRefinement, ResolveRefinement,
        ReadyPolicyEscalation, AwaitPolicyEscalation, ResolvePolicyEscalation,
        ResolveDirectForecast, ResolveSelectedCoast, ReadyTerminal,
        AwaitTerminal, ResolveTerminalNominal, ResolveTerminalDelayed,
        Complete, Failed
    }

    internal enum TargetAwarePathEndpointKind
    {
        Unresolved, BallisticImpact, CoastImpact, BrakingImpact, BrakingHandoff
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
        internal bool HasFirstContact;
        internal AirlessTargetAwareState FirstContact;
        internal bool HasPreviousClearSample;
        internal AirlessTargetAwareState PreviousClearState;
        internal double PreviousClearance;
        internal double PreviousTerrainASL;

        internal TargetAwareTerrainCandidate(AirlessTargetAwareOutput output,
            AirlessTargetAwareSnapshot snapshot, bool endpointOnly = false,
            bool includeCoast = false)
        {
            Output = output;
            // The ballistic terrain pass already validates the shared coast
            // from this snapshot to first ground contact. Every brake UT is
            // before that contact, so re-querying each candidate's coast is
            // redundant and would multiply PQS work across the batch.
            QueryPath = new List<AirlessTargetAwareState>();
            double lastQueryUT = double.NegativeInfinity;
            if (includeCoast)
                foreach (AirlessTargetAwareState state in output.CoastSamples)
                    if (state.Position.magnitude - snapshot.BodyRadius <= snapshot.MaximumTerrainASL &&
                        state.UT - lastQueryUT >= 0.2)
                    {
                        QueryPath.Add(state);
                        lastQueryUT = state.UT;
                    }
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

    internal sealed class TargetAwareTerminalEnvelope
    {
        internal readonly AirlessNominalTerminalOutput Nominal, Delayed;

        internal TargetAwareTerminalEnvelope(AirlessNominalTerminalOutput nominal,
            AirlessNominalTerminalOutput delayed)
        {
            Nominal = nominal;
            Delayed = delayed;
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
        internal string LastCandidateFailure { get; private set; }
        internal AirlessTargetAwareOutput SelectedOutput { get; private set; }
        internal TargetAwareTerrainCandidate SelectedTerrain { get; private set; }
        internal TargetAwareTerminalHandoff TerminalHandoff { get; private set; }
        internal TargetAwareTerminalEnvelope TerminalEnvelope { get; private set; }
        internal TargetAwareTerminalTerrainProbeResult NominalTerminalTerrain { get; private set; }
        internal TargetAwareTerminalTerrainProbeResult DelayedTerminalTerrain { get; private set; }
        internal bool TerminalTouchdownValidated { get; private set; }
        internal bool TerminalImpactForecast { get; private set; }
        internal double TerminalEndpointSeparation { get; private set; } = double.NaN;
        internal TargetAwarePathEndpointKind PathEndpointKind
        {
            get
            {
                if (SelectedOutput == null || SelectedTerrain == null ||
                    !SelectedTerrain.Resolved) return TargetAwarePathEndpointKind.Unresolved;
                if (SelectedTerrain.HasFirstContact)
                {
                    if (double.IsNaN(SelectedOutput.BrakeUT))
                        return TargetAwarePathEndpointKind.BallisticImpact;
                    double burnStartUT = SelectedOutput.Trajectory[0].UT;
                    return SelectedTerrain.FirstContact.UT < burnStartUT ?
                        TargetAwarePathEndpointKind.CoastImpact :
                        TargetAwarePathEndpointKind.BrakingImpact;
                }
                return SelectedOutput.ReachedHandoff && SelectedTerrain.ClearPath ?
                    TargetAwarePathEndpointKind.BrakingHandoff :
                    TargetAwarePathEndpointKind.Unresolved;
            }
        }
        internal double SignedDownrangeError { get; private set; }
        internal double CrossrangeError { get; private set; }
        internal double TimingInterval { get; private set; }
        internal double TimingDistanceEstimate { get; private set; }
        internal double BallisticContactUT { get; private set; }
        internal AbsoluteVector BallisticContact { get; private set; }
        private AirlessTargetAwareState _ballisticContactState;
        internal double SelectedPolicyTerrainASL { get; private set; }
        internal int PolicyEscalations { get; private set; }
        internal double ContinuitySeedUT { get; }
        internal bool UsedLocalSearch { get; private set; }
        internal bool BroadSearchFallback { get; private set; }
        private bool _localSearchActive;
        private double _localAnchorUT;
        internal double TerrainQueryMilliseconds => 1000d * _terrainTicks / Stopwatch.Frequency;
        internal int TerrainQueryCount { get; private set; }
        internal int TerrainSampleCount { get; private set; }
        internal int TerrainCacheHits { get; private set; }
        internal double TerrainMaximumQueryOffsetMetres { get; private set; }
        internal const double CoarseTerrainResolutionMetres = 25;
        internal const double FineTerrainResolutionMetres = 1;

        private readonly Func<double, double, double> _terrainAltitude;
        private readonly TargetAwareTerrainCache _terrainCache;
        internal readonly Dictionary<(double Latitude, double Longitude), double> TerrainSamples =
            new Dictionary<(double Latitude, double Longitude), double>();
        private const int MaximumTerrainSamples = 32768;
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
        private double _safetyWindowEarlier, _safetyWindowLater;
        private bool _fallbackSelection;
        private List<TargetAwareTerrainCandidate> _fallbackOptions;
        private int _fallbackIndex;
        private int _nextSelectedCoast;
        private bool _selectedCoastFreshChecked;
        private double _pendingPolicyTerrainASL = double.NaN;
        private double _policyEscalationBrakeUT = double.NaN;
        private const int MaximumPolicyEscalations = 8;
        private TargetAwareTerminalTerrainResolver _terminalTerrainResolver;
        private double _terminalModelTerrainASL = double.NaN;
        private double _delayedTerminalModelTerrainASL = double.NaN;
        private int _terminalTerrainReplays;
        private bool _hasPriorTerminalContact;
        private AirlessTargetAwareState _priorTerminalContact;

        internal TargetAwareAirlessPlanner(AirlessTargetAwareSnapshot snapshot, long generation,
            long sequence, object bodyIdentity, Func<double, double, double> terrainAltitude,
            double targetTolerance, int queriesPerTick, int maximumQueries,
            double maximumTerrainMillisecondsPerTick = 1,
            bool directForecast = false, TargetAwareTerrainCache terrainCache = null,
            double previousBrakeUT = double.NaN)
        {
            Snapshot = snapshot;
            Generation = generation;
            Sequence = sequence;
            BodyIdentity = bodyIdentity;
            IsDirectForecast = directForecast;
            ContinuitySeedUT = !directForecast && snapshot.HasV1ControlModel &&
                Finite(previousBrakeUT) ? previousBrakeUT : double.NaN;
            _localSearchActive = Finite(ContinuitySeedUT);
            _terrainAltitude = terrainAltitude ?? throw new ArgumentNullException(nameof(terrainAltitude));
            _terrainCache = terrainCache ?? new TargetAwareTerrainCache();
            _targetTolerance = targetTolerance;
            _queriesPerTick = queriesPerTick;
            _maximumQueries = maximumQueries;
            _maximumTerrainMillisecondsPerTick = maximumTerrainMillisecondsPerTick;
            TimingInterval = double.NaN;
            TimingDistanceEstimate = double.NaN;
            SignedDownrangeError = double.NaN;
            CrossrangeError = double.NaN;
            SelectedPolicyTerrainASL = snapshot.PolicyTerrainRadius - snapshot.BodyRadius - 200;
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
                output == null || !output.UsesV1ControlModel ||
                output.Trajectory == null || output.Trajectory.Count == 0)
                throw new InvalidOperationException("Incomplete live V1 forecast");
            IsDirectForecast = true;
            _fallbackSelection = true;
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(output, Snapshot, includeCoast: true) };
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
            if (Snapshot.HasV1ControlModel)
            {
                if (refinement?.Outputs == null || refinement.Outputs.Count != 17 ||
                    refinement.Outputs.Exists(item => item == null || !item.UsesV1ControlModel))
                { Fail("IncompleteSafetyTimingBatch"); return; }
                _coarseCandidates = _candidates;
                _refinement = refinement;
                _candidates = new List<TargetAwareTerrainCandidate>(17);
                foreach (AirlessTargetAwareOutput output in refinement.Outputs)
                    _candidates.Add(new TargetAwareTerrainCandidate(output, Snapshot, true));
                _nextCandidate = 0;
                Stage = TargetAwarePlannerStage.ResolveRefinement;
                return;
            }
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

        internal void BeginTerminalWorker()
        {
            if (Stage != TargetAwarePlannerStage.ReadyTerminal)
                throw new InvalidOperationException();
            Stage = TargetAwarePlannerStage.AwaitTerminal;
        }

        internal TargetAwareTerminalEnvelope RunTerminalEnvelope()
        {
            if (SelectedOutput == null || SelectedTerrain == null ||
                !SelectedTerrain.ClearPath)
                throw new InvalidOperationException("Incomplete terminal handoff");
            var nominal = AirlessTargetAwareSimulation.RunNominalTerminal(Snapshot,
                SelectedOutput, _terminalModelTerrainASL, FineTerrainResolutionMetres);
            // The old Mun trace measured 5.1 s to settle a 62 degree KHV
            // turn. This no-thrust sensitivity path is a recorded uncertainty
            // bound for the diagnostic vessel, not a new V1 command law.
            var delayedKill = AirlessTargetAwareSimulation.RunDelayedHorizontalKillBound(
                Snapshot, SelectedOutput.End, SelectedOutput.EndMass, 5.1);
            double delayedTerrain = Finite(_delayedTerminalModelTerrainASL) ?
                _delayedTerminalModelTerrainASL : _terminalModelTerrainASL;
            var delayedFinal = delayedKill.ReachedFinalDescent ?
                AirlessTargetAwareSimulation.RunIdealFinalDescentBound(Snapshot,
                    delayedKill, delayedTerrain,
                    Snapshot.TouchdownSpeed, Snapshot.PreviousTransThrottle,
                    Snapshot.BottomOffset, FineTerrainResolutionMetres) : null;
            return new TargetAwareTerminalEnvelope(nominal,
                new AirlessNominalTerminalOutput(delayedKill, delayedFinal,
                    delayedTerrain));
        }

        internal void SetTerminalEnvelope(TargetAwareTerminalEnvelope envelope)
        {
            if (Stage != TargetAwarePlannerStage.AwaitTerminal || envelope == null ||
                envelope.Nominal == null || envelope.Delayed == null)
                throw new InvalidOperationException("Incomplete terminal worker result");
            TerminalEnvelope = envelope;
            if (!envelope.Nominal.HorizontalKill.ReachedFinalDescent ||
                envelope.Nominal.FinalDescent == null ||
                !envelope.Nominal.FinalDescent.ReachedTerrain ||
                !envelope.Delayed.HorizontalKill.ReachedFinalDescent ||
                envelope.Delayed.FinalDescent == null ||
                !envelope.Delayed.FinalDescent.ReachedTerrain)
            {
                TryNextCandidate("TerminalControllerContinuationIncomplete");
                return;
            }
            _terminalTerrainResolver = new TargetAwareTerminalTerrainResolver(Snapshot,
                envelope.Nominal.Trajectory, Query, MaximumTerrainSamples);
            Stage = TargetAwarePlannerStage.ResolveTerminalNominal;
        }

        internal void BeginPolicyEscalation()
        {
            if (Stage != TargetAwarePlannerStage.ReadyPolicyEscalation)
                throw new InvalidOperationException();
            Stage = TargetAwarePlannerStage.AwaitPolicyEscalation;
        }

        internal AirlessTargetAwareOutput RunPolicyEscalation() =>
            AirlessTargetAwareSimulation.Run(
                Snapshot.WithV1LandingTerrain(_pendingPolicyTerrainASL),
                _policyEscalationBrakeUT, true);

        internal void SetPolicyEscalation(AirlessTargetAwareOutput output)
        {
            if (Stage != TargetAwarePlannerStage.AwaitPolicyEscalation ||
                output == null || !output.UsesV1ControlModel)
                throw new InvalidOperationException("Incomplete policy escalation");
            if (!output.ReachedHandoff)
            {
                TryNextCandidate("EscalatedControllerPathIncomplete");
                return;
            }
            SelectedPolicyTerrainASL = _pendingPolicyTerrainASL;
            PolicyEscalations++;
            TerminalEnvelope = null;
            NominalTerminalTerrain = default(TargetAwareTerminalTerrainProbeResult);
            DelayedTerminalTerrain = default(TargetAwareTerminalTerrainProbeResult);
            TerminalTouchdownValidated = false;
            TerminalImpactForecast = false;
            _terminalModelTerrainASL = double.NaN;
            _delayedTerminalModelTerrainASL = double.NaN;
            _terminalTerrainReplays = 0;
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(output, Snapshot) };
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolvePolicyEscalation;
        }

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
                            _ballisticContactState = state;
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
                         Stage == TargetAwarePlannerStage.ResolvePolicyEscalation ||
                         Stage == TargetAwarePlannerStage.ResolveDirectForecast)
                {
                    while (remaining > 0 &&
                           (remaining == _queriesPerTick || WithinTickBudget(tickStart)) &&
                           _nextCandidate < _candidates.Count)
                    {
                        TargetAwareTerrainCandidate candidate = _candidates[_nextCandidate];
                        if ((!IsDirectForecast && !candidate.Output.ReachedHandoff) ||
                            candidate.QueryPath.Count == 0)
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
                        if (IsDirectForecast && clearance <= 0)
                        {
                            candidate.HasFirstContact = true;
                            double fraction = candidate.HasPreviousClearSample ?
                                Math.Max(0, Math.Min(1, candidate.PreviousClearance /
                                    (candidate.PreviousClearance - clearance))) : 1;
                            candidate.FirstContact = candidate.HasPreviousClearSample ?
                                new AirlessTargetAwareState(
                                    candidate.PreviousClearState.Position + fraction *
                                        (state.Position - candidate.PreviousClearState.Position),
                                    candidate.PreviousClearState.Velocity + fraction *
                                        (state.Velocity - candidate.PreviousClearState.Velocity),
                                    candidate.PreviousClearState.UT + fraction *
                                        (state.UT - candidate.PreviousClearState.UT)) : state;
                            candidate.LocalTerrainASL = candidate.HasPreviousClearSample ?
                                candidate.PreviousTerrainASL + fraction *
                                    (terrain - candidate.PreviousTerrainASL) : terrain;
                            candidate.HandoffClearance = candidate.FirstContact.Position.magnitude -
                                Snapshot.BodyRadius - candidate.LocalTerrainASL;
                            candidate.Resolved = true;
                            _nextCandidate++;
                        }
                        else if (IsDirectForecast)
                        {
                            candidate.HasPreviousClearSample = true;
                            candidate.PreviousClearState = state;
                            candidate.PreviousClearance = clearance;
                            candidate.PreviousTerrainASL = terrain;
                            if (endpoint)
                            {
                                candidate.HandoffClearance = clearance;
                                candidate.LocalTerrainASL = terrain;
                            }
                        }
                        else if (endpoint)
                        {
                            candidate.HandoffClearance = clearance;
                            candidate.LocalTerrainASL = terrain;
                            candidate.HandoffBelowTerrain = clearance < 0;
                        }
                        else if (clearance <= 0)
                        {
                            candidate.EarlyIntersection = true;
                            candidate.HasFirstContact = true;
                            candidate.FirstContact = state;
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
                        else if (Stage == TargetAwarePlannerStage.ResolvePolicyEscalation)
                            FinishPolicyEscalation();
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
                        // Reuse the snapshot's checked coast. Rechecking every
                        // point against PQS on every policy replay defeated the
                        // cache. Critical selected endpoint checks remain fresh.
                        bool refresh = !_selectedCoastFreshChecked;
                        if (refresh && Snapshot.HasV1ControlModel && _terrainCache.SpatialReuseEnabled)
                            TargetAwareTerrainCache.QueryLocation(position.Latitude, position.Longitude,
                                Snapshot.BodyRadius, FineTerrainResolutionMetres,
                                out position.Latitude, out position.Longitude);
                        double terrain = Query(position.Latitude, position.Longitude, refresh);
                        _selectedCoastFreshChecked = true;
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
                        Stage = Snapshot.HasV1ControlModel ?
                            TargetAwarePlannerStage.ReadyTerminal :
                            TargetAwarePlannerStage.Complete;
                }
                else if (Stage == TargetAwarePlannerStage.ResolveTerminalNominal ||
                         Stage == TargetAwarePlannerStage.ResolveTerminalDelayed)
                {
                    _terminalTerrainResolver.Advance(remaining,
                        _maximumTerrainMillisecondsPerTick);
                    if (_terminalTerrainResolver.Done)
                    {
                        TargetAwareTerminalTerrainProbeResult terrain =
                            _terminalTerrainResolver.Result;
                        if (!terrain.Resolved)
                        {
                            Fail("TerminalTerrainUnresolved:resolved=" + terrain.Resolved +
                                ":contact=" + terrain.HasContact +
                                ":queries=" + TerrainQueryCount);
                            return;
                        }
                        if (!terrain.HasContact)
                        {
                            // The worker stops at its provisional ground height.
                            // Lower PQS ground is a request to continue V1 final
                            // descent, not proof that no landing exists.
                            AirlessTargetAwareState end = Stage ==
                                TargetAwarePlannerStage.ResolveTerminalNominal ?
                                TerminalEnvelope.Nominal.Trajectory[
                                    TerminalEnvelope.Nominal.Trajectory.Count - 1] :
                                TerminalEnvelope.Delayed.Trajectory[
                                    TerminalEnvelope.Delayed.Trajectory.Count - 1];
                            double actualTerrain = end.Position.magnitude -
                                Snapshot.BodyRadius - Snapshot.BottomOffset -
                                terrain.EndClearance;
                            double currentTerrain = Stage == TargetAwarePlannerStage.ResolveTerminalNominal ||
                                !Finite(_delayedTerminalModelTerrainASL) ?
                                _terminalModelTerrainASL : _delayedTerminalModelTerrainASL;
                            if (!Finite(actualTerrain) ||
                                actualTerrain >= currentTerrain - 0.01 ||
                                ++_terminalTerrainReplays > 3)
                            {
                                TryNextCandidate("TerminalContinuationCannotReachTerrain");
                                return;
                            }
                            if (Stage == TargetAwarePlannerStage.ResolveTerminalNominal)
                                _terminalModelTerrainASL = actualTerrain;
                            else _delayedTerminalModelTerrainASL = actualTerrain;
                            Stage = TargetAwarePlannerStage.ReadyTerminal;
                            return;
                        }
                        if (Stage == TargetAwarePlannerStage.ResolveTerminalNominal)
                        {
                            NominalTerminalTerrain = terrain;
                            _terminalTerrainResolver = new TargetAwareTerminalTerrainResolver(
                                Snapshot, TerminalEnvelope.Delayed.Trajectory,
                                Query, MaximumTerrainSamples);
                            Stage = TargetAwarePlannerStage.ResolveTerminalDelayed;
                        }
                        else
                        {
                            DelayedTerminalTerrain = terrain;
                            FinishTerminalEnvelope();
                        }
                    }
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
            if (_localSearchActive)
            {
                // Resolve the prior absolute search time against this NEW
                // physical snapshot. Use the existing broad/refinement grid
                // spacing for the neighbourhood, not a targeting tolerance.
                double anchor = Math.Max(earliest, Math.Min(latest, ContinuitySeedUT));
                double step = Math.Max(Snapshot.Dt, (latest - earliest) / (8 * 18));
                for (int i = -4; i <= 4; i++)
                    outputs.Add(AirlessTargetAwareSimulation.Run(Snapshot,
                        Math.Max(earliest, Math.Min(latest, anchor + i * step)), true));
                return outputs;
            }
            for (int i = 0; i < 9; ++i)
                outputs.Add(AirlessTargetAwareSimulation.Run(Snapshot,
                    earliest + i / 8d * (latest - earliest), true));
            return outputs;
        }

        internal TargetAwareRefinementBatch RunRefinement()
        {
            if (Snapshot.HasV1ControlModel)
            {
                var safetyOutputs = new List<AirlessTargetAwareOutput>(17);
                for (int i = 0; i < 17; i++)
                    safetyOutputs.Add(AirlessTargetAwareSimulation.Run(Snapshot,
                        _safetyWindowEarlier + (i + 1) / 18d *
                        (_safetyWindowLater - _safetyWindowEarlier), true));
                double interval = (_safetyWindowLater - _safetyWindowEarlier) / 18d;
                return new TargetAwareRefinementBatch(safetyOutputs, null, null, null, true,
                    interval, double.NaN, double.NaN, double.NaN);
            }
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
            => Query(position.Latitude, position.Longitude);

        private double Query(double latitude, double longitude) => Query(latitude, longitude, false);

        private double Query(double latitude, double longitude, bool refresh)
        {
            if (!refresh && Snapshot.HasV1ControlModel && _terrainCache.SpatialReuseEnabled)
            {
                bool coarse = Stage == TargetAwarePlannerStage.ResolveBallistic ||
                    Stage == TargetAwarePlannerStage.ResolveCoarse ||
                    (Stage == TargetAwarePlannerStage.ResolveRefinement && !_fallbackSelection);
                double originalLatitude = latitude, originalLongitude = longitude;
                TargetAwareTerrainCache.QueryLocation(latitude, longitude, Snapshot.BodyRadius,
                    coarse ? CoarseTerrainResolutionMetres : FineTerrainResolutionMetres,
                    out latitude, out longitude);
                double longitudeOffset = ((longitude - originalLongitude + 180) % 360 + 360) % 360 - 180;
                double latitudeOffset = latitude - originalLatitude;
                double offset = Math.Sqrt(latitudeOffset * latitudeOffset +
                    Math.Pow(longitudeOffset * Math.Cos(originalLatitude * Math.PI / 180), 2)) *
                    Snapshot.BodyRadius * Math.PI / 180;
                TerrainMaximumQueryOffsetMetres = Math.Max(TerrainMaximumQueryOffsetMetres, offset);
            }
            if (TerrainSampleCount >= MaximumTerrainSamples)
                throw new InvalidOperationException("TerrainSampleBudgetExceeded");
            TerrainSampleCount++;
            bool hasCached = _terrainCache.TryGet(latitude, longitude, out double cached);
            if (!refresh && hasCached)
            {
                TerrainCacheHits++;
                TerrainSamples[(latitude, longitude)] = cached;
                return cached;
            }
            if (TerrainQueryCount >= _maximumQueries)
                throw new InvalidOperationException("TerrainQueryBudgetExceeded");
            long start = Stopwatch.GetTimestamp();
            double terrain;
            try { terrain = _terrainAltitude(latitude, longitude); }
            finally
            {
                _terrainTicks += Stopwatch.GetTimestamp() - start;
                TerrainQueryCount++;
            }
            if (!Finite(terrain)) throw new InvalidOperationException("TerrainUnresolved");
            if (hasCached && cached != terrain)
            {
                _terrainCache.Clear();
                throw new InvalidOperationException("TerrainChangedDuringTransaction");
            }
            _terrainCache.Store(latitude, longitude, terrain);
            TerrainSamples[(latitude, longitude)] = terrain;
            return terrain;
        }

        private bool WithinTickBudget(long tickStart) =>
            1000d * (Stopwatch.GetTimestamp() - tickStart) / Stopwatch.Frequency <
            _maximumTerrainMillisecondsPerTick;

        private void FinishCoarse()
        {
            if (_localSearchActive)
            {
                UsedLocalSearch = true;
                double earliest = Snapshot.InputUT +
                    (Snapshot.DecelerationAlreadyTriggered ? 0 : 15);
                _localAnchorUT = Math.Max(earliest,
                    Math.Min(BallisticContactUT - 2, ContinuitySeedUT));
                // Each candidate still goes through the full selected coast,
                // braking, terminal, terrain and thrust-margin validation.
                PrepareFallbackFromCoarse();
                return;
            }
            if (Snapshot.HasV1ControlModel)
            {
                TargetAwareTerrainCandidate latestSafe = null;
                TargetAwareTerrainCandidate nextLater = null;
                foreach (TargetAwareTerrainCandidate candidate in _candidates)
                {
                    if (latestSafe != null && nextLater == null)
                        nextLater = candidate;
                    if (SafetyMarginPasses(candidate))
                    {
                        latestSafe = candidate;
                        nextLater = null;
                    }
                }
                if (latestSafe == null)
                {
                    PrepareFallbackFromCoarse();
                    return;
                }
                if (nextLater == null)
                {
                    PrepareFallbackFromCoarse();
                    return;
                }
                _safetyWindowEarlier = latestSafe.Output.BrakeUT;
                _safetyWindowLater = nextLater.Output.BrakeUT;
                Stage = TargetAwarePlannerStage.ReadyRefinement;
                return;
            }
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
                if (Snapshot.HasV1ControlModel)
                {
                    if (SafetyMarginPasses(candidate) ||
                        CanRevalidateWithEndpointTerrain(candidate))
                        _fallbackOptions.Add(candidate);
                    continue;
                }
                if (!candidate.Resolved || !candidate.Output.ReachedHandoff ||
                    !candidate.Output.UsesV1ControlModel ||
                    !TryProject(candidate.Output, out double down, out _))
                    continue;
                // The last coarse brake can reach the target only by arriving
                // at hundreds of metres per second. Do not spend terrain work
                // on a path that fails even the optimistic terminal bound.
                if (!TargetAwareTerminalHandoff.Assess(Snapshot, candidate.Output,
                        Snapshot.TargetTerrainASL).NecessaryControlBoundPasses)
                    continue;
                _fallbackOptions.Add(candidate);
            }
            if (_fallbackOptions.Count == 0)
            {
                if (_localSearchActive) { BeginBroadFallback(); return; }
                if (Snapshot.HasV1ControlModel)
                    PrepareBallisticImpact();
                else Fail("NoTerrainClearControllerForecast");
                return;
            }
            _fallbackOptions.Sort((a, b) =>
            {
                if (Snapshot.HasV1ControlModel)
                {
                    if (Finite(ContinuitySeedUT))
                    {
                        double anchor = _localSearchActive ? _localAnchorUT : ContinuitySeedUT;
                        int proximity = Math.Abs(a.Output.BrakeUT - anchor).CompareTo(
                            Math.Abs(b.Output.BrakeUT - anchor));
                        // Stable nearest-neighbour order, with an earlier
                        // braking tie-break. Provisional terrain classification
                        // cannot displace the seed before full validation.
                        return proximity != 0 ? proximity :
                            a.Output.BrakeUT.CompareTo(b.Output.BrakeUT);
                    }
                    bool aSafe = SafetyMarginPasses(a);
                    bool bSafe = SafetyMarginPasses(b);
                    if (aSafe != bSafe) return aSafe ? -1 : 1;
                    // For paths hitting terrain above the provisional policy,
                    // try the earlier braking reference first. The final
                    // endpoint and policy must still be revalidated.
                    return aSafe ? b.Output.BrakeUT.CompareTo(a.Output.BrakeUT) :
                        a.Output.BrakeUT.CompareTo(b.Output.BrakeUT);
                }
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
                if (_localSearchActive) { BeginBroadFallback(); return; }
                if (Snapshot.HasV1ControlModel)
                    PrepareBallisticImpact();
                else Fail("NoTerrainClearControllerForecast");
                return;
            }
            AirlessTargetAwareOutput output = _fallbackOptions[_fallbackIndex].Output;
            SelectedPolicyTerrainASL = Snapshot.PolicyTerrainRadius - Snapshot.BodyRadius - 200;
            PolicyEscalations = 0;
            _hasPriorTerminalContact = false;
            _terminalModelTerrainASL = double.NaN;
            _delayedTerminalModelTerrainASL = double.NaN;
            _terminalTerrainReplays = 0;
            TryProject(output, out double down, out double across);
            SignedDownrangeError = down;
            CrossrangeError = across;
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(output, Snapshot) };
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveRefinement;
        }

        private void BeginBroadFallback()
        {
            // Exhausting a neighbourhood is not proof that no solution exists.
            // Keep this snapshot and the shared transaction budgets, then run
            // the original broad search. Never restore an old trajectory.
            _localSearchActive = false;
            BroadSearchFallback = true;
            _coarseCandidates = null;
            _fallbackSelection = false;
            _fallbackOptions = null;
            SelectedOutput = null;
            SelectedTerrain = null;
            TerminalEnvelope = null;
            NominalTerminalTerrain = default(TargetAwareTerminalTerrainProbeResult);
            DelayedTerminalTerrain = default(TargetAwareTerminalTerrainProbeResult);
            TerminalTouchdownValidated = false;
            TerminalImpactForecast = false;
            Stage = TargetAwarePlannerStage.ReadyCoarse;
        }

        private void PrepareBallisticImpact()
        {
            // No candidate reaches the terminal handoff with V1's speed and
            // thrust margin. The resolved unpowered first contact remains a
            // useful correction datum, but can never be labelled a landing.
            IsDirectForecast = true;
            var impact = new AirlessTargetAwareOutput(double.NaN,
                new List<AirlessTargetAwareState>(),
                new List<AirlessTargetAwareState> { _ballisticContactState },
                false, double.NaN, 0, 0, true, Snapshot.InitialMass);
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(impact, Snapshot) };
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveDirectForecast;
        }

        private void FinishRefinement()
        {
            if (Snapshot.HasV1ControlModel && !_fallbackSelection)
            {
                _coarseCandidates.AddRange(_candidates);
                PrepareFallbackFromCoarse();
                return;
            }
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
            SelectedPolicyTerrainASL = Snapshot.PolicyTerrainRadius - Snapshot.BodyRadius - 200;
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
            SelectedPolicyTerrainASL = Snapshot.PolicyTerrainRadius - Snapshot.BodyRadius - 200;
            SignedDownrangeError = double.NaN;
            CrossrangeError = double.NaN;
            if (candidate.HasFirstContact)
            {
                Stage = TargetAwarePlannerStage.Complete;
                return;
            }
            if (!candidate.Output.ReachedHandoff)
            {
                Fail("BallisticContactTerrainChanged");
                return;
            }
            CompleteOrRevalidateSelected();
        }

        private void FinishPolicyEscalation()
        {
            TargetAwareTerrainCandidate candidate = _candidates[0];
            if (!candidate.Resolved)
            {
                TryNextCandidate("EscalatedControllerPathTerrainUnresolved");
                return;
            }
            SelectedOutput = candidate.Output;
            SelectedTerrain = candidate;
            if (!TryProject(SelectedOutput, out double down, out double across))
            {
                TryNextCandidate("EscalatedControllerEndpointInvalid");
                return;
            }
            SignedDownrangeError = down;
            CrossrangeError = across;
            // Raising the real V1 speed-policy height changes the endpoint.
            // The old brake-time bracket no longer proves target accuracy.
            TimingInterval = double.NaN;
            TimingDistanceEstimate = double.NaN;
            CompleteOrRevalidateSelected();
        }

        private bool TryEscalatePolicy()
        {
            // Before the first publication, V1's live policy uses its old
            // endpoint height. Publication replaces that input with this
            // forecast's terrain. Re-run the same brake time with the higher
            // prospective endpoint terrain, then check the resulting endpoint
            // again. An impact during the ballistic coast cannot be repaired
            // by changing the later braking policy.
            if (!CanRevalidateWithEndpointTerrain(SelectedTerrain) ||
                PolicyEscalations >= MaximumPolicyEscalations)
                return false;
            _pendingPolicyTerrainASL = SelectedTerrain.LocalTerrainASL;
            _policyEscalationBrakeUT = SelectedOutput.BrakeUT;
            Stage = TargetAwarePlannerStage.ReadyPolicyEscalation;
            return true;
        }

        private bool CanRevalidateWithEndpointTerrain(TargetAwareTerrainCandidate candidate)
        {
            if (candidate == null || !candidate.Resolved ||
                !candidate.Output.ReachedHandoff ||
                !candidate.Output.UsesV1ControlModel ||
                !Finite(candidate.LocalTerrainASL) ||
                candidate.LocalTerrainASL <= SelectedPolicyTerrainASL + 1 ||
                !(candidate.HandoffBelowTerrain || candidate.EarlyIntersection))
                return false;
            return !candidate.EarlyIntersection ||
                (candidate.HasFirstContact &&
                 candidate.FirstContact.UT >= candidate.Output.Trajectory[0].UT);
        }

        private bool SafetyMarginPasses(TargetAwareTerrainCandidate candidate)
        {
            if (!candidate.Resolved || !candidate.ClearPath ||
                !candidate.Output.ReachedHandoff ||
                !candidate.Output.UsesV1ControlModel) return false;
            AirlessTargetAwareSnapshot effectiveSnapshot = Snapshot.WithV1LandingTerrain(
                SelectedPolicyTerrainASL);
            TargetAwareTerminalHandoff handoff = TargetAwareTerminalHandoff.Assess(
                effectiveSnapshot, candidate.Output, candidate.LocalTerrainASL);
            if (!handoff.NecessaryControlBoundPasses) return false;
            double allowedHandoffSpeed = V1LandingControlPolicy.SafeDescentMaximumSpeed(
                candidate.Output.End.Position.magnitude, effectiveSnapshot.PolicyTerrainRadius,
                effectiveSnapshot.PolicyGravity,
                effectiveSnapshot.PolicyThrust * Snapshot.InitialMass /
                    candidate.Output.EndMass);
            // Reaching the handoff radius without having reduced the actual
            // surface speed to V1's envelope is an impact path, even if its
            // vertical component alone has a short stopping distance.
            if (double.IsNaN(allowedHandoffSpeed) ||
                candidate.Output.EndSurfaceSpeed > 1.1 * allowedHandoffSpeed)
                return false;
            // Reserve at least one fifth of the current engine acceleration
            // above the ideal vertical stop requirement. This is a screening
            // bound; the selected path still needs full terminal validation.
            double radius = candidate.Output.End.Position.magnitude;
            double gravity = Snapshot.BodyMu / (radius * radius);
            double maxAcceleration = Snapshot.MaximumThrust / Snapshot.InitialMass;
            double required = gravity +
                Math.Pow(Math.Max(0, -handoff.EndVerticalSpeed), 2) /
                (2 * Math.Max(1, handoff.RemainingClearance));
            return required <= 0.8 * maxAcceleration;
        }

        private void TryNextCandidate(string reason)
        {
            LastCandidateFailure = reason;
            if (_fallbackSelection && !IsDirectForecast)
            {
                _fallbackIndex++;
                PrepareFallbackCandidate();
            }
            else if (!IsDirectForecast && _coarseCandidates != null)
                PrepareFallbackFromCoarse();
            else Fail(reason);
        }

        private void CompleteOrRevalidateSelected()
        {
            // A terrain-clear handoff is only an intermediate event. The
            // direct baseline records it even if a necessary terminal bound
            // fails; neither case is a publishable landing prediction.
            if (!SelectedTerrain.ClearPath)
            {
                if (!TryEscalatePolicy())
                    TryNextCandidate("ControllerPathTerrainUnsafe");
                return;
            }
            // Beta uses the eventual endpoint's terrain for its braking
            // handoff. A lower local surface can leave a provisional handoff
            // above the final 300 m branch. Correct that prospective policy
            // input before terminal replay instead of declaring no forecast.
            if (Snapshot.HasV1ControlModel && !IsDirectForecast &&
                SelectedTerrain.HandoffClearance - Snapshot.BottomOffset > 300 &&
                Math.Abs(SelectedTerrain.LocalTerrainASL - SelectedPolicyTerrainASL) > 1)
            {
                if (PolicyEscalations >= MaximumPolicyEscalations)
                {
                    TryNextCandidate("V1EndpointTerrainPolicyDidNotConverge");
                    return;
                }
                _pendingPolicyTerrainASL = SelectedTerrain.LocalTerrainASL;
                _policyEscalationBrakeUT = SelectedOutput.BrakeUT;
                Stage = TargetAwarePlannerStage.ReadyPolicyEscalation;
                return;
            }
            var effectiveSnapshot = Snapshot.HasV1ControlModel ?
                Snapshot.WithV1LandingTerrain(SelectedPolicyTerrainASL) : Snapshot;
            TerminalHandoff = TargetAwareTerminalHandoff.Assess(effectiveSnapshot,
                SelectedOutput, SelectedTerrain.LocalTerrainASL);
            if (Snapshot.HasV1ControlModel && !IsDirectForecast &&
                !SafetyMarginPasses(SelectedTerrain))
            {
                TryNextCandidate("ControllerThrottleMarginFailed");
                return;
            }
            if (!TerminalHandoff.NecessaryControlBoundPasses && !IsDirectForecast)
            {
                TryNextCandidate("TerminalVerticalStoppingBoundFailed");
                return;
            }
            _nextSelectedCoast = 0;
            _selectedCoastFreshChecked = false;
            if (!Finite(_terminalModelTerrainASL))
                _terminalModelTerrainASL = SelectedTerrain.LocalTerrainASL;
            Stage = SelectedOutput.CoastSamples.Count == 0 ?
                (Snapshot.HasV1ControlModel ? TargetAwarePlannerStage.ReadyTerminal :
                    TargetAwarePlannerStage.Complete) :
                TargetAwarePlannerStage.ResolveSelectedCoast;
        }

        private void FinishTerminalEnvelope()
        {
            double endpointTerrain = NominalTerminalTerrain.ContactTerrainASL;
            AirlessTargetAwareState nominal = NominalTerminalTerrain.Contact;
            AirlessTargetAwareState delayed = DelayedTerminalTerrain.Contact;
            double nominalVertical, nominalHorizontal, delayedVertical, delayedHorizontal;
            SurfaceSpeeds(nominal, out nominalVertical, out nominalHorizontal);
            SurfaceSpeeds(delayed, out delayedVertical, out delayedHorizontal);
            TerminalEndpointSeparation = (nominal.Position - delayed.Position).magnitude;
            bool contactsInFinalDescent = nominal.UT >= TerminalEnvelope.Nominal.HorizontalKillEndUT &&
                delayed.UT >= TerminalEnvelope.Delayed.HorizontalKillEndUT;
            double delayedModel = Finite(_delayedTerminalModelTerrainASL) ?
                _delayedTerminalModelTerrainASL : _terminalModelTerrainASL;
            // A higher local ground contact is terrain feedback for V1's
            // final-descent altitude input, not an irreparable speed failure.
            // Re-run the same controller equations with the resolved height.
            if (contactsInFinalDescent &&
                (Math.Abs(nominalVertical) > Snapshot.TouchdownSpeed ||
                 Math.Abs(delayedVertical) > Snapshot.TouchdownSpeed) &&
                (Math.Abs(endpointTerrain - _terminalModelTerrainASL) > 0.01 ||
                 Math.Abs(DelayedTerminalTerrain.ContactTerrainASL - delayedModel) > 0.01) &&
                _terminalTerrainReplays < 3)
            {
                _terminalTerrainReplays++;
                _terminalModelTerrainASL = endpointTerrain;
                _delayedTerminalModelTerrainASL = DelayedTerminalTerrain.ContactTerrainASL;
                Stage = TargetAwarePlannerStage.ReadyTerminal;
                return;
            }
            TerminalTouchdownValidated = contactsInFinalDescent &&
                Math.Abs(nominalVertical) <= Snapshot.TouchdownSpeed &&
                Math.Abs(delayedVertical) <= Snapshot.TouchdownSpeed &&
                nominalHorizontal <= 1 && delayedHorizontal <= 1 &&
                TerminalEndpointSeparation <= _targetTolerance;
            TerminalImpactForecast = !TerminalTouchdownValidated &&
                (!contactsInFinalDescent ||
                 Math.Abs(nominalVertical) > Snapshot.TouchdownSpeed ||
                 nominalHorizontal > 1);
            if (!TerminalTouchdownValidated && !IsDirectForecast)
            {
                TryNextCandidate("TerminalTouchdownUnsafe");
                return;
            }
            // Beta derives its next speed-policy height from the published
            // endpoint. A one-metre equality requirement caused the captured
            // Mun transaction to repropagate eight times and publish nothing.
            // Compare consecutive, fully terrain-validated landing positions:
            // when both policies produce the same site within the targeting
            // tolerance, their remaining terrain-height feedback cannot move
            // V1's landing site materially. A moving endpoint remains unsafe.
            if (!IsDirectForecast &&
                Math.Abs(endpointTerrain - SelectedPolicyTerrainASL) > 1.0)
            {
                if (!Finite(endpointTerrain))
                {
                    TryNextCandidate("V1EndpointTerrainInvalid");
                    return;
                }
                if (_hasPriorTerminalContact &&
                    (nominal.Position - _priorTerminalContact.Position).magnitude <=
                    _targetTolerance)
                {
                    Stage = TargetAwarePlannerStage.Complete;
                    return;
                }
                if (PolicyEscalations >= MaximumPolicyEscalations)
                {
                    TryNextCandidate("V1EndpointTerrainPolicyDidNotConverge");
                    return;
                }
                _priorTerminalContact = nominal;
                _hasPriorTerminalContact = true;
                _pendingPolicyTerrainASL = endpointTerrain;
                _policyEscalationBrakeUT = SelectedOutput.BrakeUT;
                Stage = TargetAwarePlannerStage.ReadyPolicyEscalation;
                return;
            }
            Stage = TargetAwarePlannerStage.Complete;
        }

        private void SurfaceSpeeds(AirlessTargetAwareState state,
            out double vertical, out double horizontal)
        {
            Vector3d up = state.Position.normalized;
            Vector3d velocity = state.Velocity -
                Vector3d.Cross(Snapshot.AngularVelocity, state.Position);
            vertical = Vector3d.Dot(velocity, up);
            horizontal = Vector3d.Exclude(up, velocity).magnitude;
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
