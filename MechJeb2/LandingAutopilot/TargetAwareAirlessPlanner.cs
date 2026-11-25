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
        Complete, Failed, Uncertain, ResolveBrakingResponse
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
        internal readonly BrakingEndpointResponse BrakingResponse;

        internal TargetAwareTerminalEnvelope(AirlessNominalTerminalOutput nominal,
            AirlessNominalTerminalOutput delayed, BrakingEndpointResponse brakingResponse = null)
        {
            Nominal = nominal;
            Delayed = delayed;
            BrakingResponse=brakingResponse;
        }
    }

    // This state machine is driven only on the flight thread. Worker methods
    // consume the copied snapshot and post immutable outputs back to it.
    internal sealed class TargetAwareAirlessPlanner
    {
        internal readonly AirlessTargetAwareSnapshot Snapshot;
        internal AirlessTargetAwareSnapshot EvaluatedSnapshot { get; private set; }
        internal readonly long Generation, Sequence;
        internal readonly object BodyIdentity;
        internal bool IsDirectForecast { get; private set; }
        // A deorbit delivery recheck owns one selected reference, but must
        // converge the same prospective terrain policy as candidate selection.
        // Observed coast/braking/terminal forecasts keep their direct semantics.
        internal bool IsDeliveryValidation { get; }
        internal bool InheritedBallisticContactWindow { get; }
        private bool UsesCandidateValidation => !IsDirectForecast || IsDeliveryValidation;
        internal TargetAwarePlannerStage Stage { get; private set; } = TargetAwarePlannerStage.AwaitBallistic;
        internal string Failure { get; private set; }
        internal string LastCandidateFailure { get; private set; }
        internal AirlessTargetAwareOutput SelectedOutput { get; private set; }
        internal TargetAwareTerrainCandidate SelectedTerrain { get; private set; }
        internal TargetAwareTerminalHandoff TerminalHandoff { get; private set; }
        internal TargetAwareTerminalEnvelope TerminalEnvelope { get; private set; }
        internal BrakingEndpointResponse BrakingResponse { get; private set; }
        internal string BrakingResponseFailure { get; private set; }
        private int _brakingResponseIndex,_brakingResponseTicks;
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
        internal bool BallisticContactFound { get; private set; }
        internal double BrakeSearchWindowEndUT { get; private set; }
        internal double BallisticReferenceUT { get; private set; } = double.NaN;
        internal AbsoluteVector BallisticContact { get; private set; }
        private AirlessTargetAwareState _ballisticContactState;
        private AirlessTargetAwareState _ballisticReferenceState;
        private double _ballisticReferenceTargetDot = double.NegativeInfinity;
        internal double SelectedPolicyTerrainASL { get; private set; }
        internal int PolicyEscalations { get; private set; }
        internal double ContinuitySeedUT { get; private set; }
        internal bool ContinuitySeedOutsideApproach { get; private set; }
        internal bool UsedLocalSearch { get; private set; }
        internal double FixedBrakeReferenceUT { get; }
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
        private readonly Func<double, double, double, double> _terminalSurface;
        internal readonly Dictionary<(double Latitude, double Longitude), double> TerminalSurfaceSamples =
            new Dictionary<(double Latitude, double Longitude), double>();
        private readonly TargetAwareTerrainCache _terrainCache;
        internal readonly Dictionary<(double Latitude, double Longitude), double> TerrainSamples =
            new Dictionary<(double Latitude, double Longitude), double>();
        private const int MaximumTerrainSamples = 32768;
        private const int MaximumBallisticSamplesPerTick = 512;
        private readonly int _queriesPerTick, _maximumQueries;
        private readonly double _maximumTerrainMillisecondsPerTick;
        private readonly double _targetTolerance;
        private List<AirlessTargetAwareState> _ballisticSamples;
        private int _nextBallistic;
        private bool _approachReferenceLocked;
        private double _priorBallisticTargetDot = double.NaN;
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
        private readonly Dictionary<(double BrakeUT, double Terrain), AirlessTargetAwareOutput> _policyOutputs =
            new Dictionary<(double, double), AirlessTargetAwareOutput>();
        private readonly List<(double Terrain, double Residual)> _policyResiduals =
            new List<(double, double)>();
        private bool _coverageRefinement;
        private double _searchPolicyTerrainASL, _ballisticTerrainASL;
        private bool _ballisticPolicySeedUsed;
        private readonly HashSet<double> _rejectedBrakeReferences = new HashSet<double>();
        internal int CoverageRefinements { get; private set; }
        internal int PolicySimulationCount { get; private set; }
        internal int PolicySimulationCacheHits { get; private set; }
        private bool _terminalCandidateRejected;
        internal int ContinuityGapRefinements { get; private set; }
        private bool _refiningContinuityGap;
        private volatile bool _workersCancelled;
        private readonly Dictionary<(double, double), AirlessTargetAwareOutput> _candidateOutputs =
            new Dictionary<(double, double), AirlessTargetAwareOutput>();
        private readonly Dictionary<(double, double), AirlessTargetAwareOutput> _reserveOutputs =
            new Dictionary<(double, double), AirlessTargetAwareOutput>();
        internal const double BrakingAvailableThrustFraction = 0.8;
        internal int BrakingReserveSimulationCount { get; private set; }
        internal int CandidateSimulationCount { get; private set; }
        internal int CandidateSimulationCacheHits { get; private set; }
        internal int DeliveryAlternatives { get; private set; }
        private readonly HashSet<double> _deliveryTriedReferences = new HashSet<double>();

        private double DeliveryReferenceAnchor()
        {
            double stateUT = Snapshot.InputUT + Snapshot.DeliveryLeadSeconds;
            return FixedBrakeReferenceUT < stateUT + 5 ?
                EarliestExecutableBrakeReference(Snapshot, stateUT + 19) : FixedBrakeReferenceUT;
        }

        internal void CancelWorkers() => _workersCancelled = true;

        private void CheckWorkerCancellation()
        {
            if (_workersCancelled) throw new OperationCanceledException("Predictor transaction cancelled");
        }

        private AirlessTargetAwareOutput RunCandidate(AirlessTargetAwareSnapshot snapshot, double brakeUT)
        {
            CheckWorkerCancellation();
            EvaluatedSnapshot = AirlessTargetAwareSimulation.AtDeliveryState(snapshot);
            var key = (brakeUT, _searchPolicyTerrainASL);
            if (_candidateOutputs.TryGetValue(key, out var cached))
            {
                CandidateSimulationCacheHits++;
                return cached;
            }
            CandidateSimulationCount++;
            var output = AirlessTargetAwareSimulation.Run(snapshot, brakeUT, true);
            EvaluateBrakingReserve(snapshot, output.BrakeUT);
            output = BoundCandidateToApproach(output);
            _candidateOutputs.Add(key, output);
            return output;
        }

        private void EvaluateBrakingReserve(AirlessTargetAwareSnapshot snapshot, double brakeUT,
            double terrainASL = double.NaN)
        {
            if (!snapshot.HasV1ControlModel || snapshot.DecelerationAlreadyTriggered ||
                snapshot.InitialAppliedThrottle > 0 ||
                snapshot.TerminalPhase != 0) return;
            CheckWorkerCancellation();
            var key = (brakeUT, Finite(terrainASL) ? terrainASL : _searchPolicyTerrainASL);
            lock (_reserveOutputs)
                if (_reserveOutputs.ContainsKey(key)) return;
            // The existing one-fifth acceleration reserve must cover the burn,
            // not merely the final ideal vertical stopping-distance check.
            var lower = snapshot.WithAvailableThrustFraction(BrakingAvailableThrustFraction);
            BrakingReserveSimulationCount++;
            var output = AirlessTargetAwareSimulation.Run(lower, brakeUT, true);
            lock (_reserveOutputs) _reserveOutputs[key] = output;
        }

        private AirlessTargetAwareOutput BoundCandidateToApproach(AirlessTargetAwareOutput output)
        {
            if (output.AttitudeUncertain || !Snapshot.HasV1ControlModel || BallisticContactFound ||
                IsDeliveryValidation && InheritedBallisticContactWindow || IsDirectForecast && !IsDeliveryValidation ||
                output.Trajectory[0].UT <= BrakeSearchWindowEndUT &&
                output.FirstRequiredThrust.HasValue && output.FirstRequiredThrust.Value.UT <= BrakeSearchWindowEndUT)
                return output;
            // A bounded reference cannot hide a policy wait into a later pass.
            // Preserve the diagnostic trajectory, but it is not a handoff.
            return new AirlessTargetAwareOutput(output.BrakeUT, output.CoastSamples, output.Trajectory,
                false, output.EndSurfaceSpeed, output.VirtualDeltaV, output.Steps,
                output.UsesV1ControlModel, output.EndMass, output.EndForward, output.EndAppliedThrottle);
        }


        internal TargetAwareAirlessPlanner(AirlessTargetAwareSnapshot snapshot, long generation,
            long sequence, object bodyIdentity, Func<double, double, double> terrainAltitude,
            double targetTolerance, int queriesPerTick, int maximumQueries,
            double maximumTerrainMillisecondsPerTick = 1,
            bool directForecast = false, TargetAwareTerrainCache terrainCache = null,
            double previousBrakeUT = double.NaN, double fixedBrakeReferenceUT = double.NaN,
            bool deliveryValidation = false, double deliveryWindowEndUT = double.NaN,
            bool deliveryHasBallisticContact = false,
            Func<double, double, double, double> terminalSurface = null)
        {
            Snapshot = snapshot;
            EvaluatedSnapshot = snapshot;
            Generation = generation;
            Sequence = sequence;
            BodyIdentity = bodyIdentity;
            IsDirectForecast = directForecast;
            IsDeliveryValidation = deliveryValidation && directForecast && snapshot.TerminalPhase == 0;
            if (IsDeliveryValidation)
            {
                BrakeSearchWindowEndUT = deliveryWindowEndUT;
                InheritedBallisticContactWindow = deliveryHasBallisticContact;
            }
            FixedBrakeReferenceUT = fixedBrakeReferenceUT;
            ContinuitySeedUT = !directForecast && snapshot.HasV1ControlModel &&
                Finite(previousBrakeUT) ? previousBrakeUT : double.NaN;
            _localSearchActive = Finite(ContinuitySeedUT);
            _terrainAltitude = terrainAltitude ?? throw new ArgumentNullException(nameof(terrainAltitude));
            _terminalSurface = terminalSurface;
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
            _searchPolicyTerrainASL = SelectedPolicyTerrainASL;
            if (bodyIdentity == null || !Finite(targetTolerance) || targetTolerance <= 0 ||
                queriesPerTick <= 0 || maximumQueries < queriesPerTick ||
                !Finite(maximumTerrainMillisecondsPerTick) || maximumTerrainMillisecondsPerTick <= 0)
                throw new ArgumentException("Invalid target-aware planner inputs");
        }

        internal AirlessTargetAwareOutput RunDirectForecast()
        {
            CheckWorkerCancellation();
            EvaluatedSnapshot = AirlessTargetAwareSimulation.AtDeliveryState(Snapshot);
            double reference = IsDeliveryValidation ? DeliveryReferenceAnchor() : FixedBrakeReferenceUT;
            var output = Snapshot.TerminalPhase != 0 ?
                AirlessTargetAwareSimulation.RunObservedTerminal(Snapshot, FixedBrakeReferenceUT) :
                !Snapshot.DecelerationAlreadyTriggered && Finite(reference) ?
                    AirlessTargetAwareSimulation.RunCoastForecast(Snapshot, reference) :
                    AirlessTargetAwareSimulation.RunNominalV1(Snapshot);
            if (IsDeliveryValidation && Finite(output.BrakeUT)) _deliveryTriedReferences.Add(output.BrakeUT);
            return BoundCandidateToApproach(output);
        }

        internal void SetBallisticSamples(List<AirlessTargetAwareState> samples)
        {
            if (Stage != TargetAwarePlannerStage.AwaitBallistic) throw new InvalidOperationException();
            _ballisticSamples = samples;
            _nextBallistic = 0;
            if (samples != null && samples.Count > 0)
                BrakeSearchWindowEndUT = samples[samples.Count - 1].UT;
            Stage = TargetAwarePlannerStage.ResolveBallistic;
        }

        internal void SetDirectOutput(AirlessTargetAwareOutput output)
        {
            if (Stage != TargetAwarePlannerStage.AwaitBallistic ||
                output == null || !output.UsesV1ControlModel ||
                output.Trajectory == null || output.Trajectory.Count == 0)
                throw new InvalidOperationException("Incomplete live V1 forecast");
            IsDirectForecast = true;
            if (IsDeliveryValidation && !output.ReachedHandoff && !output.AttitudeUncertain)
            {
                Fail("DeliveryReferenceOutsideCurrentApproach");
                return;
            }
            if (output.AttitudeUncertain)
            {
                SelectedOutput = output;
                Stage = TargetAwarePlannerStage.Uncertain;
                return;
            }
            _fallbackSelection = true;
            _candidates = new List<TargetAwareTerrainCandidate> {
                new TargetAwareTerrainCandidate(output, Snapshot, includeCoast: true) };
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveDirectForecast;
        }

        internal void SetCoarseOutputs(List<AirlessTargetAwareOutput> outputs)
        {
            if (Stage != TargetAwarePlannerStage.AwaitCoarse) throw new InvalidOperationException();
            if (outputs != null && outputs.Count == 0)
            {
                if (BallisticContactFound) PrepareBallisticImpact();
                else Fail("NoExecutableBrakingInterval");
                return;
            }
            if (outputs == null || outputs.Count > 9 || outputs.Exists(item => item == null))
            { Fail("IncompleteCoarseBatch"); return; }
            if (Snapshot.HasV1ControlModel && !_ballisticPolicySeedUsed &&
                !outputs.Exists(item => item.ReachedHandoff) &&
                _ballisticTerrainASL > _searchPolicyTerrainASL + 1)
            {
                // The initial sea-level policy may never trigger above an
                // elevated surface intersected by this orbit. Seed from that
                // measured contact height, then repeat the powered search.
                // This is a policy input guess, never a published endpoint.
                SeedSearchTerrain(_ballisticTerrainASL);
                return;
            }
            // Every coarse batch uses the search policy, including a broad
            // fallback after a different local candidate height was tried.
            SelectedPolicyTerrainASL = _searchPolicyTerrainASL;
            _candidates = new List<TargetAwareTerrainCandidate>(outputs.Count);
            foreach (AirlessTargetAwareOutput output in outputs)
            {
                _candidates.Add(new TargetAwareTerrainCandidate(output, Snapshot, true));
                _policyOutputs[(output.BrakeUT, SelectedPolicyTerrainASL)] = output;
            }
            _nextCandidate = 0;
            Stage = TargetAwarePlannerStage.ResolveCoarse;
        }

        internal void SetRefinement(TargetAwareRefinementBatch refinement)
        {
            if (Stage != TargetAwarePlannerStage.AwaitRefinement) throw new InvalidOperationException();
            if (Snapshot.HasV1ControlModel)
            {
                int expected = _refiningContinuityGap ? 1 : 17;
                if (refinement?.Outputs == null || refinement.Outputs.Count != expected ||
                    refinement.Outputs.Exists(item => item == null || !item.UsesV1ControlModel))
                { Fail("IncompleteSafetyTimingBatch"); return; }
                _refiningContinuityGap = false;
                if (!_coverageRefinement) _coarseCandidates = _candidates;
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
                refinement.Outputs == null ||
                (Snapshot.HasV1ControlModel && refinement.Outputs.Exists(item =>
                    item == null || !item.UsesV1ControlModel)))
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

        // An unsuccessful search cannot spend seconds following a snapshot
        // whose orbit is being changed by a live deorbit burn. Cold coverage
        // and a still-validating first terminal candidate keep the normal age
        // limit. Cancellation never modifies the committed prediction.
        internal bool RefreshRejectedMovingSnapshot(double age, bool poweredDeorbit,
            double submissionInterval)
        {
            if (!poweredDeorbit || IsDirectForecast || !_terminalCandidateRejected ||
                Stage == TargetAwarePlannerStage.Complete || Stage == TargetAwarePlannerStage.Failed ||
                Stage == TargetAwarePlannerStage.AwaitBallistic || Stage == TargetAwarePlannerStage.AwaitCoarse ||
                Stage == TargetAwarePlannerStage.AwaitRefinement || Stage == TargetAwarePlannerStage.AwaitPolicyEscalation ||
                Stage == TargetAwarePlannerStage.AwaitTerminal ||
                age <= 2 * submissionInterval)
                return false;
            Fail("MovingDeorbitSnapshotRefresh");
            return true;
        }

        internal void BeginTerminalWorker()
        {
            if (Stage != TargetAwarePlannerStage.ReadyTerminal)
                throw new InvalidOperationException();
            Stage = TargetAwarePlannerStage.AwaitTerminal;
        }

        internal TargetAwareTerminalEnvelope RunTerminalEnvelope()
        {
            CheckWorkerCancellation();
            if (_impactResponse)
            {
                BrakingEndpointResponse impactResponse = null;
                try { impactResponse = BrakingEndpointResponse.RunImpact(Snapshot,
                    SelectedTerrain.FirstContact, CheckWorkerCancellation); }
                catch (OperationCanceledException) { throw; }
                catch (Exception) { /* An optional correction cannot suppress an impact forecast. */ }
                return new TargetAwareTerminalEnvelope(null, null, impactResponse);
            }
            if (SelectedOutput == null || SelectedTerrain == null ||
                !SelectedTerrain.ClearPath)
                throw new InvalidOperationException("Incomplete terminal handoff");
            var nominal = AirlessTargetAwareSimulation.RunNominalTerminal(Snapshot,
                SelectedOutput, _terminalModelTerrainASL, FineTerrainResolutionMetres);
            CheckWorkerCancellation();
            double delayedTerrain = Finite(_delayedTerminalModelTerrainASL) ?
                _delayedTerminalModelTerrainASL : _terminalModelTerrainASL;
            var delayed = AirlessTargetAwareSimulation.RunNominalTerminal(Snapshot,
                    SelectedOutput, delayedTerrain, FineTerrainResolutionMetres,
                    AirlessTargetAwareSimulation.SlowTerminalTurnRate);
            BrakingEndpointResponse response=null;
            if (IsDirectForecast && !IsDeliveryValidation)
            {
                try { response=BrakingEndpointResponse.Run(Snapshot,SelectedOutput,nominal,
                    _terminalModelTerrainASL,FineTerrainResolutionMetres,CheckWorkerCancellation); }
                catch (OperationCanceledException) { throw; }
                catch (Exception) { /* Optional response cannot reject the baseline. */ }
            }
            if(response==null && Snapshot.DecelerationAlreadyTriggered && Snapshot.TerminalPhase==0 &&
                Snapshot.InitialAppliedThrottle>0) BrakingResponseFailure="ResponseModelUnavailable";
            return new TargetAwareTerminalEnvelope(nominal,delayed,response);
        }

        internal void SetTerminalEnvelope(TargetAwareTerminalEnvelope envelope)
        {
            if (_impactResponse && Stage == TargetAwarePlannerStage.AwaitTerminal && envelope != null)
            {
                BrakingResponse = envelope.BrakingResponse;
                if (BrakingResponse == null) FinishWithoutBrakingResponse("ImpactCorrectionUnavailable");
                else BeginBrakingResponseTerrain();
                return;
            }
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
                envelope.Nominal.Trajectory, QueryTerminalSurface, MaximumTerrainSamples);
            Stage = TargetAwarePlannerStage.ResolveTerminalNominal;
        }

        internal void BeginPolicyEscalation()
        {
            if (Stage != TargetAwarePlannerStage.ReadyPolicyEscalation)
                throw new InvalidOperationException();
            Stage = TargetAwarePlannerStage.AwaitPolicyEscalation;
        }

        internal AirlessTargetAwareOutput RunPolicyEscalation()
        {
            CheckWorkerCancellation();
            if (_policyOutputs.TryGetValue((_policyEscalationBrakeUT, _pendingPolicyTerrainASL), out var cached))
            {
                PolicySimulationCacheHits++;
                return cached;
            }
            PolicySimulationCount++;
            var output = AirlessTargetAwareSimulation.Run(
                Snapshot.WithV1LandingTerrain(_pendingPolicyTerrainASL),
                _policyEscalationBrakeUT, true);
            EvaluateBrakingReserve(Snapshot.WithV1LandingTerrain(_pendingPolicyTerrainASL),
                output.BrakeUT, _pendingPolicyTerrainASL);
            output = BoundCandidateToApproach(output);
            _policyOutputs[(_policyEscalationBrakeUT, _pendingPolicyTerrainASL)] = output;
            return output;
        }

        internal void SetPolicyEscalation(AirlessTargetAwareOutput output)
        {
            if (Stage != TargetAwarePlannerStage.AwaitPolicyEscalation ||
                output == null || !output.UsesV1ControlModel)
                throw new InvalidOperationException("Incomplete policy escalation");
            _policyOutputs[(_policyEscalationBrakeUT, _pendingPolicyTerrainASL)] = output;
            if (!output.ReachedHandoff)
            {
                TryNextCandidate("EscalatedControllerPathIncomplete");
                return;
            }
            SelectedPolicyTerrainASL = _pendingPolicyTerrainASL;
            PolicyEscalations++;
            TerminalEnvelope = null;
            BrakingResponse=null;BrakingResponseFailure=null;
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
            _terrainTickQueriesAtStart = TerrainQueryCount;
            long tickStart = Stopwatch.GetTimestamp();
            try
            {
                if (Stage == TargetAwarePlannerStage.ResolveBrakingResponse)
                {
                    if (++_brakingResponseTicks * Snapshot.MinDt >
                        MechJebModuleLandingPredictions.TargetAwareRefreshSeconds)
                    { FinishWithoutBrakingResponse("ResponseTerrainRefreshBudget"); return; }
                    _terminalTerrainResolver.Advance(_terminalSurface == null ? remaining : Math.Max(1, remaining / 2),
                        _maximumTerrainMillisecondsPerTick);
                    if (!_terminalTerrainResolver.Done) return;
                    var contact=_terminalTerrainResolver.Result;
                    if (!contact.Resolved || !contact.HasContact ||
                        !BrakingResponse.ImpactBaseline &&
                        contact.Contact.UT < BrakingResponse.FinalDescentUTs[_brakingResponseIndex])
                    { FinishWithoutBrakingResponse("ResponseTerrainIncomplete"); return; }
                    SurfaceSpeeds(contact.Contact,out double vertical,out double horizontal);
                    if (!BrakingResponse.ImpactBaseline &&
                        !TargetAwareTouchdownSafety.ContactSpeedPasses(vertical,horizontal,Snapshot.SafeTouchdownSpeed))
                    { FinishWithoutBrakingResponse("ResponseContactUnsafe"); return; }
                    BrakingResponse.Contacts[_brakingResponseIndex++]=contact.Contact;
                    if (_brakingResponseIndex==BrakingResponse.Paths.Length)
                    {
                        if (!BrakingResponse.Resolve(Snapshot,BrakingResponse.ImpactBaseline ?
                            SelectedTerrain.FirstContact : NominalTerminalTerrain.Contact))
                            FinishWithoutBrakingResponse(BrakingResponse.ImpactBaseline ?
                                "ImpactCorrectionNotImproving" : "ResponseAsymmetricOrIllConditioned");
                        else Stage=TargetAwarePlannerStage.Complete;
                    }
                    else _terminalTerrainResolver=new TargetAwareTerminalTerrainResolver(Snapshot,
                        BrakingResponse.Paths[_brakingResponseIndex],QueryTerminalSurface,MaximumTerrainSamples,
                        _terminalSurface == null ? Snapshot.MaximumTerrainASL : double.NaN,
                        _terminalSurface == null ? QueryBrakingResponseSurface :
                            (Func<double, double, double, double>)null, ResponseTerrainBudgetAvailable);
                    return;
                }
                if (Stage == TargetAwarePlannerStage.ResolveBallistic)
                {
                    if (_ballisticSamples == null || _ballisticSamples.Count == 0)
                    {
                        Fail("NoBallisticTerrainPass");
                        return;
                    }
                    int processedBallisticSamples = 0;
                    while (remaining > 0 &&
                           processedBallisticSamples < Math.Min(_queriesPerTick,
                               MaximumBallisticSamplesPerTick) &&
                           WithinTickBudget(tickStart) &&
                           _nextBallistic < _ballisticSamples.Count)
                    {
                        AirlessTargetAwareState state = _ballisticSamples[_nextBallistic++];
                        processedBallisticSamples++;
                        AbsoluteVector position = AirlessTargetAwareSimulation.ToAbsolute(state.Position,
                            state.UT, Snapshot);
                        Vector3d target = AirlessTargetAwareSimulation.SurfaceDirection(
                            Snapshot.TargetLatitude, Snapshot.TargetLongitude, Snapshot);
                        Vector3d direction = AirlessTargetAwareSimulation.SurfaceDirection(
                            position.Latitude, position.Longitude, Snapshot);
                        double targetDot = Vector3d.Dot(target, direction);
                        // First pass only. A following-orbit closest point is
                        // not the current approach, even on an unpowered orbit.
                        if (!_approachReferenceLocked && targetDot > _ballisticReferenceTargetDot)
                        {
                            _ballisticReferenceTargetDot = targetDot;
                            _ballisticReferenceState = state;
                            BallisticReferenceUT = state.UT;
                        }
                        if (Finite(_priorBallisticTargetDot) &&
                            targetDot < _priorBallisticTargetDot - 1e-10)
                            _approachReferenceLocked = true;
                        _priorBallisticTargetDot = targetDot;

                        // High orbital corridor samples establish an approach
                        // frame and brake-time window only. PQS is queried only
                        // inside the sampled terrain envelope.
                        if (state.Position.magnitude - Snapshot.BodyRadius >
                            Snapshot.MaximumTerrainASL)
                            continue;

                        double terrain = Query(position);
                        remaining--;
                        if (state.Position.magnitude - Snapshot.BodyRadius <= terrain)
                        {
                            _ballisticContactState = state;
                            _ballisticTerrainASL = terrain;
                            BallisticContact = position;
                            BallisticContactUT = state.UT;
                            BallisticContactFound = true;
                            BrakeSearchWindowEndUT = state.UT;
                            Vector3d contact = AirlessTargetAwareSimulation.SurfaceDirection(
                                position.Latitude, position.Longitude, Snapshot);
                            if (!TargetAwareApproachFrame.TryCreate(target, contact,
                                Snapshot.BodyRadius, out _approachFrame))
                                Fail("InvalidBallisticApproachFrame");
                            else Stage = TargetAwarePlannerStage.ReadyCoarse;
                            return;
                        }
                    }
                    if (_nextBallistic == _ballisticSamples.Count)
                    {
                        if (BallisticContactFound) return;
                        if (Snapshot.HasV1ControlModel)
                            BrakeSearchWindowEndUT = CurrentApproachWindowEnd(Snapshot,
                                BrakeSearchWindowEndUT, BallisticReferenceUT);
                        Vector3d target = AirlessTargetAwareSimulation.SurfaceDirection(
                            Snapshot.TargetLatitude, Snapshot.TargetLongitude, Snapshot);
                        if (!Finite(BrakeSearchWindowEndUT) ||
                            !TargetAwareApproachFrame.TryCreateFromTrack(target,
                                _ballisticReferenceState.Position, _ballisticReferenceState.Velocity,
                                Snapshot.AngularVelocity, Snapshot.BodyRadius, out _approachFrame))
                        {
                            Fail("NoBallisticContactOrUsableApproachTrack");
                            return;
                        }
                        // This is the end of the bounded orbital candidate
                        // window, not a terrain contact or predicted landing.
                        Stage = TargetAwarePlannerStage.ReadyCoarse;
                    }
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
                            candidate.EarlyIntersection = !endpoint;
                            candidate.HandoffBelowTerrain = endpoint;
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
                            if (Snapshot.HasV1ControlModel && !IsDirectForecast &&
                                !_ballisticPolicySeedUsed && terrain > _searchPolicyTerrainASL + 1 &&
                                terrain > SelectedPolicyTerrainASL + 1 &&
                                CoastBlocksRemainingCandidates(state.UT))
                            {
                                // Every later brake shares this obstructed
                                // ballistic coast. Retry the batch once with
                                // the observed ground height; never accept the
                                // intersecting path or increase its budgets.
                                SeedSearchTerrain(terrain);
                                LastCandidateFailure = "SelectedCoastTerrainPolicyRetry";
                                return;
                            }
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
                    _terminalTerrainResolver.Advance(_terminalSurface == null ? remaining : Math.Max(1, remaining / 2),
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
                                Snapshot.BodyRadius - Snapshot.TerminalBottomOffset -
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
                                QueryTerminalSurface, MaximumTerrainSamples);
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
                if (Stage==TargetAwarePlannerStage.ResolveBrakingResponse)
                { FinishWithoutBrakingResponse("ResponseTerrainFailed:"+ex.GetType().Name); return; }
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
            var searchSnapshot = Snapshot.HasV1ControlModel ?
                Snapshot.WithV1LandingTerrain(_searchPolicyTerrainASL) : Snapshot;
            double latest = BrakeSearchWindowEndUT - 2;
            double earliest = EarliestExecutableBrakeReference(Snapshot, latest);
            if (latest < earliest) return outputs;
            if (_localSearchActive && ContinuitySeedUT > latest)
            {
                // Deorbit may shorten the real contact window while an older
                // forecast is still fresh by age. That reference is no longer
                // an executable continuity constraint. Search this approach
                // afresh rather than clamp the seed and refine back toward an
                // impossible time after finding a complete safe candidate.
                // A past reference is different: V1 may already have released
                // its guard but still be coasting to the speed trigger. Retain
                // that continuity and the existing executable-floor handling.
                ContinuitySeedOutsideApproach = true;
                ContinuitySeedUT = double.NaN;
                _localSearchActive = false;
                BroadSearchFallback = true;
            }
            if (_localSearchActive)
            {
                // Resolve the prior absolute search time against this NEW
                // physical snapshot. Check integration-scale neighbours as
                // well as the existing broader search neighbourhood.
                double anchor = Math.Max(earliest, Math.Min(latest, ContinuitySeedUT));
                double step = Math.Max(Snapshot.Dt, (latest - earliest) / (8 * 18));
                foreach (double offset in new[] { -4 * step, -step, -step / 4,
                    -Snapshot.Dt, 0, Snapshot.Dt, step / 4, step, 4 * step })
                    outputs.Add(RunCandidate(searchSnapshot,
                        Math.Max(earliest, Math.Min(latest, anchor + offset))));
                return outputs;
            }
            int count = latest == earliest ? 1 : Math.Min(9,
                Math.Max(2, 1 + (int)Math.Ceiling((latest - earliest) / Math.Max(Snapshot.Dt, Snapshot.MinDt))));
            for (int i = 0; i < count; ++i)
                outputs.Add(RunCandidate(searchSnapshot,
                    earliest + (count == 1 ? 0 : i / (double)(count - 1)) * (latest - earliest)));
            return outputs;
        }

        internal static double CurrentApproachWindowEnd(AirlessTargetAwareSnapshot snapshot,
            double orbitalWindowEnd, double firstApproachUT)
        {
            Vector3d surface = snapshot.Velocity - Vector3d.Cross(snapshot.AngularVelocity, snapshot.Position);
            // The existing safety screen reserves 20% thrust. One speed-removal
            // interval past the first pass bounds this approach without aiming
            // the powered endpoint at the target or admitting a later orbit.
            double net = 0.8 * snapshot.MaximumThrustAcceleration -
                snapshot.BodyMu / snapshot.Position.sqrMagnitude;
            double removalSeconds = net > 0 ? surface.magnitude / net : 0;
            return Math.Min(orbitalWindowEnd, firstApproachUT + removalSeconds);
        }

        internal static double EarliestExecutableBrakeReference(AirlessTargetAwareSnapshot snapshot,
            double latestReferenceUT = double.PositiveInfinity)
        {
            double stateUT = snapshot.InputUT + snapshot.DeliveryLeadSeconds;
            if (!snapshot.HasV1ControlModel || snapshot.DecelerationAlreadyTriggered) return stateUT;
            Vector3d surface = snapshot.Velocity - Vector3d.Cross(snapshot.AngularVelocity, snapshot.Position);
            bool ready = snapshot.InitialForward.sqrMagnitude > 0 && surface.sqrMagnitude > 0 &&
                !V1LandingControlPolicy.BrakeAttitudeGate(Vector3d.Dot(surface, snapshot.Position.normalized),
                    Vector3d.Dot(snapshot.InitialForward.normalized, -surface.normalized));
            // Reference minus five is release, not reference itself. A vessel
            // already passing the real thrust gate can release immediately.
            // Preserve the existing preparation interval when it is not ready.
            return stateUT + (ready && latestReferenceUT <= stateUT + 19 ? 5 : 15);
        }

        internal TargetAwareRefinementBatch RunRefinement()
        {
            if (Snapshot.HasV1ControlModel)
            {
                if (_refiningContinuityGap)
                {
                    var midpoint = RunCandidate(Snapshot.WithV1LandingTerrain(_searchPolicyTerrainASL),
                        (_safetyWindowEarlier + _safetyWindowLater) / 2);
                    return new TargetAwareRefinementBatch(new List<AirlessTargetAwareOutput> { midpoint },
                        null, null, null, true, (_safetyWindowLater - _safetyWindowEarlier) / 2,
                        double.NaN, double.NaN, double.NaN);
                }
                var safetyOutputs = new List<AirlessTargetAwareOutput>(17);
                for (int i = 0; i < 17; ++i)
                    safetyOutputs.Add(RunCandidate(
                        Snapshot.WithV1LandingTerrain(_searchPolicyTerrainASL),
                        _safetyWindowEarlier + (i + 1) / 18d *
                        (_safetyWindowLater - _safetyWindowEarlier)));
                double interval = (_safetyWindowLater - _safetyWindowEarlier) / 18d;
                return new TargetAwareRefinementBatch(safetyOutputs, null, null, null,
                    true, interval, double.NaN, double.NaN, double.NaN);
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
                AirlessTargetAwareSnapshot searchSnapshot = Snapshot.HasV1ControlModel ?
                    Snapshot.WithV1LandingTerrain(_searchPolicyTerrainASL) : Snapshot;
                AirlessTargetAwareOutput output = AirlessTargetAwareSimulation.Run(searchSnapshot,
                    midpoint, true);
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

        // Surface geometry is never stored in the shared PQS cache or used to
        // raise the braking policy. Called only by flight-thread path resolution.
        internal double QueryTerminalSurface(double latitude, double longitude)
        {
            double ground = Query(latitude, longitude);
            if (_terminalSurface == null) return ground;
            if (TerrainQueryCount >= _maximumQueries)
                throw new InvalidOperationException("TerrainQueryBudgetExceeded");
            long start = Stopwatch.GetTimestamp();
            try
            {
                double surface = _terminalSurface(latitude, longitude, ground);
                if (!Finite(surface) || surface < ground)
                    throw new InvalidOperationException("TerminalSurfaceUnresolved");
                var key = (latitude, longitude);
                if (TerminalSurfaceSamples.TryGetValue(key, out double previous) && Math.Abs(previous - surface) > 0.01)
                    throw new InvalidOperationException("TerminalSurfaceChangedDuringTransaction");
                TerminalSurfaceSamples[key] = surface;
                return surface;
            }
            finally
            {
                _terrainTicks += Stopwatch.GetTimestamp() - start;
                TerrainQueryCount++;
            }
        }

        internal bool TerminalSurfaceStillValid()
        {
            if (_terminalSurface == null) return true;
            foreach (var contact in new[] { NominalTerminalTerrain, DelayedTerminalTerrain })
            {
                if (!contact.HasContact) continue;
                var location = AirlessTargetAwareSimulation.ToAbsolute(contact.Contact.Position, contact.Contact.UT, Snapshot);
                if (Math.Abs(QueryTerminalSurface(location.Latitude, location.Longitude) - contact.ContactTerrainASL) >
                    FineTerrainResolutionMetres) return false;
            }
            return true;
        }

        private double QueryBrakingResponseSurface(double latitude, double longitude, double radius)
        {
            double coarse = Query(latitude, longitude, false, true);
            // Reuse the existing 25 m corridor cells while clear. The existing
            // 200 m terminal margin switches back to the 1 m query grid before
            // measuring contact; coarse values are never touchdown heights.
            return radius - Snapshot.BodyRadius - Snapshot.TerminalBottomOffset - coarse > 200 ?
                coarse : Query(latitude, longitude, false);
        }

        private double Query(double latitude, double longitude, bool refresh, bool coarseResponse = false)
        {
            if (!refresh && Snapshot.HasV1ControlModel && _terrainCache.SpatialReuseEnabled)
            {
                bool coarse = coarseResponse || Stage == TargetAwarePlannerStage.ResolveBallistic ||
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
                double earliest = EarliestExecutableBrakeReference(Snapshot, BrakeSearchWindowEndUT - 2);
                _localAnchorUT = Math.Max(earliest,
                    Math.Min(BrakeSearchWindowEndUT - 2, ContinuitySeedUT));
                // Preserve the proven continuity path: revalidate the prior
                // absolute brake reference against this fresh snapshot before
                // entering a broad timing search.
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
                if (latestSafe == null || nextLater == null)
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
            SelectedPolicyTerrainASL = _searchPolicyTerrainASL;
            var source = new List<TargetAwareTerrainCandidate>(_coarseCandidates ?? _candidates);
            _fallbackOptions = new List<TargetAwareTerrainCandidate>();
            foreach (TargetAwareTerrainCandidate candidate in source)
            {
                if (Snapshot.HasV1ControlModel)
                {
                    if (_rejectedBrakeReferences.Contains(candidate.Output.BrakeUT) ||
                        _fallbackOptions.Exists(c => c.Output.BrakeUT == candidate.Output.BrakeUT)) continue;
                    // Coarse endpoint heights are only a screen. A provisional
                    // margin failure can disappear after the endpoint's actual
                    // terrain supplies V1's policy height. Validate it fully.
                    if (candidate.Resolved && candidate.Output.ReachedHandoff &&
                        candidate.Output.UsesV1ControlModel)
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
                    RefineCoverageOrImpact();
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
                    RefineCoverageOrImpact();
                else Fail("NoTerrainClearControllerForecast");
                return;
            }
            AirlessTargetAwareOutput output = _fallbackOptions[_fallbackIndex].Output;
            SelectedPolicyTerrainASL = OutputTerrainPolicy(output);
            PolicyEscalations = 0;
            _policyResiduals.Clear();
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
            if (Snapshot.HasV1ControlModel &&
                Math.Abs(SelectedPolicyTerrainASL - _searchPolicyTerrainASL) > 0.01)
            {
                // A nearby failed candidate can supply a better numerical
                // terrain seed. Repropagate this candidate on the worker;
                // never label an old trajectory with the new policy height.
                SelectedOutput = output;
                RequestTerrainPolicy(_searchPolicyTerrainASL);
                return;
            }
            TargetAwareTerrainCandidate screen = _fallbackOptions[_fallbackIndex];
            if (Snapshot.HasV1ControlModel && Finite(screen.LocalTerrainASL) &&
                Math.Abs(screen.LocalTerrainASL - SelectedPolicyTerrainASL) > 1 &&
                (screen.HandoffBelowTerrain || screen.HandoffClearance - Snapshot.TerminalBottomOffset > 300))
            {
                // The coarse endpoint already supplies a numerical height seed.
                // Re-simulate it before spending queries on the known wrong
                // provisional path. The new full path is still validated.
                SelectedOutput = output;
                RequestTerrainPolicy(screen.LocalTerrainASL);
            }
        }

        private void BeginBroadFallback()
        {
            // Exhausting a neighbourhood is not proof that no solution exists.
            // Keep this snapshot and the shared transaction budgets, then run
            // the original broad search. Never restore an old trajectory.
            _localSearchActive = false;
            _coverageRefinement = false;
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
            if (!BallisticContactFound || !TargetAwareTerrainContext.ContactValid(
                    Snapshot, _ballisticContactState, _ballisticTerrainASL))
            {
                Fail("NoResolvedBallisticContact");
                return;
            }
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
            SelectedPolicyTerrainASL = Snapshot.HasV1ControlModel ? _searchPolicyTerrainASL :
                Snapshot.PolicyTerrainRadius - Snapshot.BodyRadius - 200;
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
                if (IsDeliveryValidation && candidate.Output.ReachedHandoff)
                {
                    if (!TryEscalatePolicy()) TryNextCandidate("DeliveryControllerPathTerrainUnsafe");
                    return;
                }
                if (Snapshot.DecelerationAlreadyTriggered && Snapshot.TerminalPhase == 0 &&
                    Snapshot.MinimumCommandThrottle == 0)
                {
                    _impactResponse = true;
                    Stage = TargetAwarePlannerStage.ReadyTerminal;
                    return;
                }
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
            double down = double.NaN, across = double.NaN;
            if (!IsDirectForecast && !TryProject(SelectedOutput, out down, out across))
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
            // again. An intersecting coast itself cannot be accepted. Batch
            // terrain reseeding may instead change the actual speed trigger;
            // that new coast requires its own complete validation.
            if (!CanRevalidateWithEndpointTerrain(SelectedTerrain) ||
                PolicyEscalations >= MaximumPolicyEscalations)
                return false;
            return RequestTerrainPolicy(SelectedTerrain.LocalTerrainASL);
        }

        private bool RequestTerrainPolicy(double terrain)
        {
            if (!Finite(terrain) || PolicyEscalations >= MaximumPolicyEscalations)
                return false;
            _policyResiduals.Add((SelectedPolicyTerrainASL, terrain - SelectedPolicyTerrainASL));
            // Terrain feedback can alternate between plateau and flat. Once
            // opposite residuals bracket a policy height, bisect that interval
            // instead of jumping between the same two policies indefinitely.
            double width = double.PositiveInfinity, next = terrain;
            foreach (var a in _policyResiduals)
                foreach (var b in _policyResiduals)
                    if (a.Residual > 0 && b.Residual < 0 && a.Terrain < b.Terrain &&
                        b.Terrain - a.Terrain < width)
                    {
                        width = b.Terrain - a.Terrain;
                        next = (a.Terrain + b.Terrain) / 2;
                    }
            if (Math.Abs(next - SelectedPolicyTerrainASL) < 0.01)
                return false;
            _pendingPolicyTerrainASL = next;
            _policyEscalationBrakeUT = SelectedOutput.BrakeUT;
            Stage = TargetAwarePlannerStage.ReadyPolicyEscalation;
            return true;
        }

        private void RefineCoverageOrImpact()
        {
            if (Snapshot.HasV1ControlModel && !IsDirectForecast &&
                !_ballisticPolicySeedUsed && _ballisticTerrainASL > _searchPolicyTerrainASL + 1)
            {
                // Reaching a numerical handoff below the real ground does
                // not make the original terrain policy adequate. Only after
                // exhausting that search, retry from the measured ballistic
                // contact height, including candidates previously rejected
                // under the incompatible provisional height.
                SeedSearchTerrain(_ballisticTerrainASL);
                return;
            }
            if (Snapshot.HasV1ControlModel && !IsDirectForecast && CoverageRefinements == 0)
            {
                // A failed nine-point screen is not evidence that the entire
                // braking window is infeasible. Test the intervening times on
                // this same snapshot, using the existing worker and budgets.
                _safetyWindowEarlier = EarliestExecutableBrakeReference(Snapshot, BrakeSearchWindowEndUT - 2);
                _safetyWindowLater = BrakeSearchWindowEndUT - 2;
                if (_safetyWindowLater > _safetyWindowEarlier)
                {
                    CoverageRefinements++;
                    _coverageRefinement = true;
                    _coarseCandidates = _fallbackOptions ?? _candidates;
                    _fallbackSelection = false;
                    Stage = TargetAwarePlannerStage.ReadyRefinement;
                    return;
                }
            }
            PrepareBallisticImpact();
        }

        private void SeedSearchTerrain(double terrainASL)
        {
            _ballisticPolicySeedUsed = true;
            _searchPolicyTerrainASL = terrainASL;
            SelectedPolicyTerrainASL = terrainASL;
            _coarseCandidates = null;
            _fallbackOptions = null;
            _fallbackSelection = false;
            _coverageRefinement = false;
            _rejectedBrakeReferences.Clear();
            Stage = TargetAwarePlannerStage.ReadyCoarse;
        }

        private bool CoastBlocksRemainingCandidates(double contactUT)
        {
            if (!_fallbackSelection || _fallbackOptions == null) return false;
            for (int i = _fallbackIndex + 1; i < _fallbackOptions.Count; ++i)
            {
                var path = _fallbackOptions[i].Output.Trajectory;
                // An earlier powered start might avoid this obstruction.
                // Preserve its original validation attempt before reseeding.
                if (path.Count > 0 && path[0].UT <= contactUT) return false;
            }
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
            if (!Snapshot.DecelerationAlreadyTriggered && Snapshot.InitialAppliedThrottle == 0 && !IsDeliveryValidation &&
                (!TryReserveOutput(candidate.Output.BrakeUT, SelectedPolicyTerrainASL, out var reserve) ||
                 !BrakingReservePasses(effectiveSnapshot, reserve, candidate.LocalTerrainASL))) return false;
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
            return required <= BrakingAvailableThrustFraction * maxAcceleration;
        }

        internal static bool BrakingReservePasses(AirlessTargetAwareSnapshot nominal,
            AirlessTargetAwareOutput reserve, double terrainASL)
        {
            if (reserve == null || !reserve.ReachedHandoff || reserve.AttitudeUncertain) return false;
            var lower = nominal.WithAvailableThrustFraction(BrakingAvailableThrustFraction);
            var handoff = TargetAwareTerminalHandoff.Assess(lower, reserve, terrainASL);
            double allowed = V1LandingControlPolicy.SafeDescentMaximumSpeed(reserve.End.Position.magnitude,
                lower.PolicyTerrainRadius, lower.PolicyGravity,
                lower.PolicyThrust * lower.InitialMass / reserve.EndMass);
            return handoff.NecessaryControlBoundPasses && Finite(allowed) &&
                reserve.EndSurfaceSpeed <= 1.1 * allowed;
        }

        internal bool SelectedBrakingReservePasses => SelectedOutput != null &&
            TryReserveOutput(SelectedOutput.BrakeUT, SelectedPolicyTerrainASL, out var reserve) &&
            BrakingReservePasses(Snapshot.WithV1LandingTerrain(SelectedPolicyTerrainASL), reserve,
                SelectedTerrain.LocalTerrainASL);
        internal bool SelectedBrakingReserveChecked => SelectedOutput != null &&
            TryReserveOutput(SelectedOutput.BrakeUT, SelectedPolicyTerrainASL, out _);

        private bool TryReserveOutput(double referenceUT, double terrainASL, out AirlessTargetAwareOutput output)
        {
            lock (_reserveOutputs) return _reserveOutputs.TryGetValue((referenceUT, terrainASL), out output);
        }

        private void TryNextCandidate(string reason)
        {
            LastCandidateFailure = reason;
            if (IsDeliveryValidation && DeliveryAlternatives < 4 && SelectedOutput != null &&
                Finite(FixedBrakeReferenceUT) && Finite(SelectedPolicyTerrainASL))
            {
                // A changed burning state may invalidate this exact reference.
                // Check only integration-scale neighbours, within this same
                // owned transaction and terrain budget; do not restart an orbit
                // search or recycle the same failing fixed time indefinitely.
                double[] offsets = { -Snapshot.Dt, Snapshot.Dt, -2 * Snapshot.Dt, 2 * Snapshot.Dt };
                double anchor = DeliveryReferenceAnchor();
                double floor = Math.Min(anchor, EarliestExecutableBrakeReference(Snapshot,
                    Snapshot.InputUT + Snapshot.DeliveryLeadSeconds + 19));
                while (DeliveryAlternatives < offsets.Length)
                {
                    double next = Math.Max(floor, anchor + offsets[DeliveryAlternatives++]);
                    if (!_deliveryTriedReferences.Add(next)) continue;
                    _policyEscalationBrakeUT = next;
                    _pendingPolicyTerrainASL = SelectedPolicyTerrainASL;
                    PolicyEscalations = 0;
                    _policyResiduals.Clear();
                    _hasPriorTerminalContact = false;
                    _terminalTerrainReplays = 0;
                    Stage = TargetAwarePlannerStage.ReadyPolicyEscalation;
                    return;
                }
            }
            if (!IsDirectForecast && SelectedTerrain != null && SelectedTerrain.Resolved &&
                Finite(SelectedPolicyTerrainASL))
                _searchPolicyTerrainASL = SelectedPolicyTerrainASL;
            if (_localSearchActive && reason == "ControllerThrottleMarginFailed")
            {
                // The old reference has failed FULL terrain/policy validation.
                // Bracket broadly now; the subsequent safety-gap refinement
                // still tests nearby times. Do not spend the query budget
                // repeating the same terrain root at eight adjacent times.
                _rejectedBrakeReferences.Add(SelectedOutput.BrakeUT);
                BeginBroadFallback();
                return;
            }
            if (reason.StartsWith("Terminal", StringComparison.Ordinal))
                _terminalCandidateRejected = true;
            if (_fallbackSelection && !IsDirectForecast)
            {
                _rejectedBrakeReferences.Add(_fallbackOptions[_fallbackIndex].Output.BrakeUT);
                _fallbackIndex++;
                PrepareFallbackCandidate();
            }
            else if (!IsDirectForecast && _coarseCandidates != null)
                PrepareFallbackFromCoarse();
            else Fail(reason);
        }

        private double OutputTerrainPolicy(AirlessTargetAwareOutput output)
        {
            foreach (var entry in _policyOutputs)
                if (ReferenceEquals(entry.Value, output)) return entry.Key.Terrain;
            foreach (var entry in _candidateOutputs)
                if (ReferenceEquals(entry.Value, output)) return entry.Key.Item2;
            return _searchPolicyTerrainASL;
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
            if (Snapshot.HasV1ControlModel && UsesCandidateValidation &&
                SelectedTerrain.HandoffClearance - Snapshot.TerminalBottomOffset > 300 &&
                Math.Abs(SelectedTerrain.LocalTerrainASL - SelectedPolicyTerrainASL) > 1)
            {
                if (PolicyEscalations >= MaximumPolicyEscalations)
                {
                    TryNextCandidate("V1EndpointTerrainPolicyDidNotConverge");
                    return;
                }
                if (!RequestTerrainPolicy(SelectedTerrain.LocalTerrainASL))
                    TryNextCandidate("V1EndpointTerrainPolicyDidNotConverge");
                return;
            }
            var effectiveSnapshot = Snapshot.HasV1ControlModel ?
                Snapshot.WithV1LandingTerrain(SelectedPolicyTerrainASL) : Snapshot;
            TerminalHandoff = TargetAwareTerminalHandoff.Assess(effectiveSnapshot,
                SelectedOutput, SelectedTerrain.LocalTerrainASL);
            if (Snapshot.HasV1ControlModel && UsesCandidateValidation &&
                !SafetyMarginPasses(SelectedTerrain))
            {
                TryNextCandidate("ControllerThrottleMarginFailed");
                return;
            }
            if (!TerminalHandoff.NecessaryControlBoundPasses && UsesCandidateValidation)
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

        internal double NominalTouchdownVerticalSpeed
        {
            get
            {
                if (!NominalTerminalTerrain.HasContact) return double.NaN;
                SurfaceSpeeds(NominalTerminalTerrain.Contact, out double vertical, out _);
                return vertical;
            }
        }
        internal double DelayedTouchdownVerticalSpeed
        {
            get
            {
                if (!DelayedTerminalTerrain.HasContact) return double.NaN;
                SurfaceSpeeds(DelayedTerminalTerrain.Contact, out double vertical, out _);
                return vertical;
            }
        }
        internal bool RequestedTouchdownSpeedMet => TerminalTouchdownValidated &&
            Math.Abs(NominalTouchdownVerticalSpeed) <= Snapshot.TouchdownSpeed &&
            Math.Abs(DelayedTouchdownVerticalSpeed) <= Snapshot.TouchdownSpeed;

        private void FinishTerminalEnvelope()
        {
            double endpointTerrain = NominalTerminalTerrain.ContactTerrainASL;
            AirlessTargetAwareState nominal = NominalTerminalTerrain.Contact;
            AirlessTargetAwareState delayed = DelayedTerminalTerrain.Contact;
            // Terminal support may be a raised deck. The unchanged braking
            // height continues to use natural ground, independently of support.
            double brakingGround = _terminalSurface == null ? endpointTerrain :
                Query(AirlessTargetAwareSimulation.ToAbsolute(nominal.Position, nominal.UT, Snapshot));
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
                (!TargetAwareTouchdownSafety.ContactSpeedPasses(nominalVertical, nominalHorizontal, Snapshot.TouchdownSpeed) ||
                 !TargetAwareTouchdownSafety.ContactSpeedPasses(delayedVertical, delayedHorizontal, Snapshot.TouchdownSpeed)) &&
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
                TargetAwareTouchdownSafety.ContactSpeedPasses(nominalVertical, nominalHorizontal, Snapshot.SafeTouchdownSpeed) &&
                TargetAwareTouchdownSafety.ContactSpeedPasses(delayedVertical, delayedHorizontal, Snapshot.SafeTouchdownSpeed) &&
                nominalHorizontal <= 1 && delayedHorizontal <= 1 &&
                TerminalEndpointSeparation <= _targetTolerance;
            TerminalImpactForecast = !TerminalTouchdownValidated &&
                (!contactsInFinalDescent ||
                 !TargetAwareTouchdownSafety.ContactSpeedPasses(nominalVertical, nominalHorizontal, Snapshot.SafeTouchdownSpeed) ||
                 nominalHorizontal > 1);
            if (!TerminalTouchdownValidated && UsesCandidateValidation)
            {
                // A hill first encountered during terminal continuation can
                // require an earlier/higher V1 handoff. Re-run the braking
                // policy using this resolved height before discarding the
                // time. Never accept the intersecting path itself.
                if (brakingGround > SelectedPolicyTerrainASL + 1 &&
                    RequestTerrainPolicy(brakingGround)) return;
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
            if (UsesCandidateValidation &&
                Math.Abs(brakingGround - SelectedPolicyTerrainASL) > 1.0)
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
                    FinishValidatedForecast();
                    return;
                }
                if (PolicyEscalations >= MaximumPolicyEscalations)
                {
                    TryNextCandidate("V1EndpointTerrainPolicyDidNotConverge");
                    return;
                }
                _priorTerminalContact = nominal;
                _hasPriorTerminalContact = true;
                if (!RequestTerrainPolicy(brakingGround))
                    TryNextCandidate("V1EndpointTerrainPolicyDidNotConverge");
                return;
            }
            FinishValidatedForecast();
        }

        private void FinishValidatedForecast()
        {
            // A coarse fallback is a safe sample, not proof that the whole
            // interval to the rejected old reference is unsafe. Refine that
            // interval before a jump becomes V1's apparent target crossing.
            if (Snapshot.HasV1ControlModel && !IsDirectForecast && BroadSearchFallback &&
                Finite(ContinuitySeedUT) && SelectedOutput.BrakeUT < ContinuitySeedUT)
            {
                double later = double.PositiveInfinity;
                foreach (double rejected in _rejectedBrakeReferences)
                    if (rejected > SelectedOutput.BrakeUT && rejected <= ContinuitySeedUT)
                        later = Math.Min(later, rejected);
                double surfaceSpeed = (Snapshot.Velocity -
                    Vector3d.Cross(Snapshot.AngularVelocity, Snapshot.Position)).magnitude;
                if (Finite(later) && (later - SelectedOutput.BrakeUT) * surfaceSpeed > _targetTolerance)
                {
                    _safetyWindowEarlier = SelectedOutput.BrakeUT;
                    _safetyWindowLater = later;
                    _searchPolicyTerrainASL = SelectedPolicyTerrainASL;
                    _candidates = new List<TargetAwareTerrainCandidate> { SelectedTerrain };
                    _fallbackSelection = false;
                    _localSearchActive = false;
                    _refiningContinuityGap = true;
                    ContinuityGapRefinements++;
                    Stage = TargetAwarePlannerStage.ReadyRefinement;
                    return;
                }
            }
            BrakingResponse=TerminalEnvelope?.BrakingResponse;
            if (BrakingResponse!=null)
            {
                BeginBrakingResponseTerrain();
            }
            else Stage = TargetAwarePlannerStage.Complete;
        }

        private bool _impactResponse;
        private int _terrainTickQueriesAtStart;

        private bool ResponseTerrainBudgetAvailable() =>
            TerrainQueryCount - _terrainTickQueriesAtStart <= _queriesPerTick - 2;

        private void BeginBrakingResponseTerrain()
        {
            _brakingResponseIndex = _brakingResponseTicks = 0;
            _terminalTerrainResolver = new TargetAwareTerminalTerrainResolver(Snapshot,
                BrakingResponse.Paths[0], QueryTerminalSurface, MaximumTerrainSamples,
                _terminalSurface == null ? Snapshot.MaximumTerrainASL : double.NaN,
                _terminalSurface == null ? QueryBrakingResponseSurface :
                    (Func<double, double, double, double>)null, ResponseTerrainBudgetAvailable);
            Stage = TargetAwarePlannerStage.ResolveBrakingResponse;
        }

        private void FinishWithoutBrakingResponse(string reason)
        {
            BrakingResponse=null;
            BrakingResponseFailure=reason;
            Stage=TargetAwarePlannerStage.Complete;
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
