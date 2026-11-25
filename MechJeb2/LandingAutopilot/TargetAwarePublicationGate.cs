using System;

namespace MuMech.Landing
{
    // End-to-end UT timing includes queued workers, flight-thread terrain and
    // terminal continuation. Search and fixed-reference validation have very
    // different costs. The upper quartile of the last eight complete attempts
    // covers their captured variation without retaining a cold search forever.
    internal sealed class TargetAwareDeliveryLatency
    {
        private readonly System.Collections.Generic.Queue<double> _search = new System.Collections.Generic.Queue<double>();
        private readonly System.Collections.Generic.Queue<double> _validation = new System.Collections.Generic.Queue<double>();
        private readonly System.Collections.Generic.Queue<double> _localSearch = new System.Collections.Generic.Queue<double>();
        private object _body;
        private object _phase;
        private long _generation;
        private double _latitude, _longitude, _lastUT = double.NaN;

        internal void Reset()
        {
            _search.Clear(); _validation.Clear(); _localSearch.Clear(); _body = _phase = null; _lastUT = double.NaN;
        }

        internal void Configure(long generation, object body, object phase, double latitude, double longitude, double now)
        {
            if (generation != _generation || !ReferenceEquals(body, _body) || !Equals(phase, _phase) ||
                latitude != _latitude || longitude != _longitude || (!double.IsNaN(_lastUT) && now < _lastUT))
                Reset();
            _generation = generation; _body = body; _phase = phase;
            _latitude = latitude; _longitude = longitude; _lastUT = now;
        }

        internal bool ObserveCompleted(long generation, object body, object phase, double latitude,
            double longitude, double inputUT, double completionUT, bool validation, bool localSearch = false)
        {
            double latency = completionUT - inputUT;
            if (generation != _generation || !ReferenceEquals(body, _body) || !Equals(phase, _phase) ||
                latitude != _latitude || longitude != _longitude || double.IsNaN(latency) ||
                double.IsInfinity(latency) || latency < 0 || completionUT < _lastUT)
                return false;
            var samples = validation ? _validation : localSearch ? _localSearch : _search;
            samples.Enqueue(latency);
            if (samples.Count > 8) samples.Dequeue();
            _lastUT = completionUT;
            return true;
        }

        internal double Estimate(bool validation, bool localSearch = false)
        {
            var samples = validation && _validation.Count > 0 ? _validation :
                localSearch && _localSearch.Count > 0 ? _localSearch : _search;
            if (samples.Count == 0) return 0;
            double[] sorted = samples.ToArray(); Array.Sort(sorted);
            return sorted[(int)Math.Ceiling(.75 * sorted.Length) - 1];
        }
    }

    // Immutable lineage accompanies, but never changes, the V1-compatible
    // ReentrySimulation.Result committed to the predictor's public slot.
    internal readonly struct TargetAwareResultLineage
    {
        internal readonly long Generation;
        internal readonly long Sequence;
        internal readonly object Body;
        internal readonly double TargetLatitude;
        internal readonly double TargetLongitude;
        internal readonly double TargetTerrainASL;
        internal readonly double CaptureUT;
        internal readonly double InputUT;
        internal readonly bool TargetAwareModel;
        internal readonly bool Complete;
        internal readonly bool TerrainResolved;
        internal readonly bool ClearPath;
        internal readonly bool TerminalNecessaryBoundPasses;
        internal readonly bool TerminalTouchdownValidated;
        internal readonly ReentrySimulation.LandingForecastKind ForecastKind;
        internal readonly bool HasFirstContact;

        internal TargetAwareResultLineage(long generation, long sequence, object body,
            double targetLatitude, double targetLongitude, double targetTerrainASL,
            double captureUT, double inputUT,
            bool targetAwareModel, bool complete, bool terrainResolved, bool clearPath,
            bool terminalNecessaryBoundPasses,
            ReentrySimulation.LandingForecastKind forecastKind,
            bool hasFirstContact,
            bool terminalTouchdownValidated)
        {
            Generation = generation;
            Sequence = sequence;
            Body = body;
            TargetLatitude = targetLatitude;
            TargetLongitude = targetLongitude;
            TargetTerrainASL = targetTerrainASL;
            CaptureUT = captureUT;
            InputUT = inputUT;
            TargetAwareModel = targetAwareModel;
            Complete = complete;
            TerrainResolved = terrainResolved;
            ClearPath = clearPath;
            TerminalNecessaryBoundPasses = terminalNecessaryBoundPasses;
            TerminalTouchdownValidated = terminalTouchdownValidated;
            ForecastKind = forecastKind;
            HasFirstContact = hasFirstContact;
        }
    }

    internal enum TargetAwarePublicationDecision
    {
        Accept,
        WrongModel,
        Incomplete,
        TerrainUnresolved,
        TerrainIntersection,
        TerminalNecessaryBoundFailed,
        ImpactOnly,
        StaleGeneration,
        ChangedBodyOrTarget,
        ExpiredSnapshot,
        OutOfOrder
    }

    internal static class TargetAwarePublicationGate
    {
        internal static bool RequiresDeliveryReplay(double age, double modelLead,
            double physicalTick, bool powered) => powered &&
            Finite(age) && Finite(modelLead) && Finite(physicalTick) &&
            Math.Abs(age - modelLead) > physicalTick;

        // Compare the worker's actual evaluation epoch to the observed vessel
        // at delivery, in either direction. This is a short state residual,
        // not a new trajectory or permission to change the observation UT.
        internal static bool DeliveryResidualWithinBudget(AirlessTargetAwareSnapshot evaluated,
            double now, Vector3d position, Vector3d velocity, double mass, double referenceUT,
            double targetingBudget, out double displacementBound)
        {
            displacementBound = double.PositiveInfinity;
            double seconds = now - evaluated.StateUT;
            if (!Finite(seconds) || Math.Abs(seconds) > .5 || !Finite(referenceUT) ||
                !Finite(mass) || mass <= 0 || !Finite(targetingBudget) || targetingBudget <= 0 ||
                evaluated.Position.sqrMagnitude <= 0 || evaluated.InitialMass <= 0)
                return false;
            Vector3d gravity = -evaluated.BodyMu / Math.Pow(evaluated.Position.magnitude, 3) * evaluated.Position;
            Vector3d expectedPosition = evaluated.Position + seconds * evaluated.Velocity +
                .5 * seconds * seconds * gravity;
            Vector3d expectedVelocity = evaluated.Velocity + seconds * gravity;
            double positionResidual = (position - expectedPosition).magnitude;
            double velocityResidual = (velocity - expectedVelocity).magnitude;
            Vector3d surface = velocity - Vector3d.Cross(evaluated.AngularVelocity, position);
            double usableThrust = evaluated.MinimumThrust + evaluated.MaximumCommandThrottle *
                (evaluated.MaximumThrust - evaluated.MinimumThrust);
            double net = .8 * usableThrust / mass - evaluated.BodyMu / position.sqrMagnitude;
            if (!Finite(net) || net <= 0) return false;
            // Use the same reserved-thrust bound as candidate safety, plus
            // coast until reference release. A mass mismatch contributes its
            // fractional speed-removal error; it cannot be silently ignored.
            double responseSeconds = Math.Max(0, referenceUT - 5 - now) + surface.magnitude / net;
            velocityResidual += surface.magnitude * Math.Abs(mass - evaluated.InitialMass) / mass;
            displacementBound = positionResidual + velocityResidual * responseSeconds;
            return Finite(displacementBound) && displacementBound <= targetingBudget;
        }
        internal static bool CanSeedSearch(TargetAwareResultLineage? committed,
            bool directForecast, long currentGeneration, object currentBody,
            double targetLatitude, double targetLongitude, double targetTerrainASL,
            double currentUT, double maximumSnapshotAge) =>
            !directForecast && committed.HasValue &&
            Check(committed.Value, null, currentGeneration, currentBody,
                targetLatitude, targetLongitude, targetTerrainASL,
                currentUT, maximumSnapshotAge, 0) == TargetAwarePublicationDecision.Accept;

        internal static TargetAwarePublicationDecision Check(TargetAwareResultLineage candidate,
            TargetAwareResultLineage? committed, long currentGeneration, object currentBody,
            double currentTargetLatitude, double currentTargetLongitude, double currentTargetTerrainASL,
            double currentUT, double maximumSnapshotAge, double maximumTerrainChange)
        {
            if (!candidate.TargetAwareModel) return TargetAwarePublicationDecision.WrongModel;
            // V1 Beta's sole readiness gate accepts Outcome.LANDED. An impact
            // may be captured for diagnosis but cannot own its active slot.
            if (candidate.ForecastKind == ReentrySimulation.LandingForecastKind.ImpactForecast)
                return TargetAwarePublicationDecision.ImpactOnly;
            return CheckObservation(candidate, committed, currentGeneration, currentBody,
                currentTargetLatitude, currentTargetLongitude, currentTargetTerrainASL,
                currentUT, maximumSnapshotAge, maximumTerrainChange);
        }

        // A fresh complete impact is evidence contradicting an older landing,
        // not permission to label that impact LANDED. Validate its lineage just
        // as strictly before allowing it to suppress the older landing claim.
        internal static TargetAwarePublicationDecision CheckObservation(TargetAwareResultLineage candidate,
            TargetAwareResultLineage? previous, long currentGeneration, object currentBody,
            double currentTargetLatitude, double currentTargetLongitude, double currentTargetTerrainASL,
            double currentUT, double maximumSnapshotAge, double maximumTerrainChange)
        {
            if (!candidate.TargetAwareModel) return TargetAwarePublicationDecision.WrongModel;
            if ((candidate.ForecastKind != ReentrySimulation.LandingForecastKind.ImpactForecast &&
                 candidate.ForecastKind != ReentrySimulation.LandingForecastKind.LandableForecast) ||
                (candidate.ForecastKind == ReentrySimulation.LandingForecastKind.ImpactForecast &&
                 (!candidate.HasFirstContact || candidate.TerminalTouchdownValidated)) ||
                (candidate.ForecastKind == ReentrySimulation.LandingForecastKind.LandableForecast &&
                 (!candidate.TerminalTouchdownValidated || candidate.HasFirstContact)))
                return TargetAwarePublicationDecision.Incomplete;
            if (!candidate.Complete) return TargetAwarePublicationDecision.Incomplete;
            if (!candidate.TerrainResolved) return TargetAwarePublicationDecision.TerrainUnresolved;
            if (!candidate.ClearPath) return TargetAwarePublicationDecision.TerrainIntersection;
            if (candidate.ForecastKind ==
                ReentrySimulation.LandingForecastKind.LandableForecast &&
                !candidate.TerminalNecessaryBoundPasses)
                return TargetAwarePublicationDecision.TerminalNecessaryBoundFailed;
            if (candidate.Generation != currentGeneration)
                return TargetAwarePublicationDecision.StaleGeneration;
            if (candidate.Body == null || !ReferenceEquals(candidate.Body, currentBody) ||
                !Finite(candidate.TargetLatitude) || !Finite(candidate.TargetLongitude) ||
                !Finite(candidate.TargetTerrainASL) || !Finite(currentTargetTerrainASL) ||
                candidate.TargetLatitude != currentTargetLatitude ||
                candidate.TargetLongitude != currentTargetLongitude ||
                !Finite(maximumTerrainChange) || maximumTerrainChange < 0 ||
                Math.Abs(candidate.TargetTerrainASL - currentTargetTerrainASL) > maximumTerrainChange)
                return TargetAwarePublicationDecision.ChangedBodyOrTarget;
            if (!Finite(currentUT) || !Finite(candidate.CaptureUT) || !Finite(candidate.InputUT) ||
                !Finite(maximumSnapshotAge) || maximumSnapshotAge < 0 ||
                candidate.CaptureUT > currentUT || currentUT - candidate.CaptureUT > maximumSnapshotAge)
                return TargetAwarePublicationDecision.ExpiredSnapshot;
            if (candidate.Sequence <= 0 || (previous.HasValue &&
                (candidate.Sequence <= previous.Value.Sequence ||
                 candidate.CaptureUT <= previous.Value.CaptureUT)))
                return TargetAwarePublicationDecision.OutOfOrder;
            return TargetAwarePublicationDecision.Accept;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    internal static class TargetAwareForecastPresentation
    {
        internal static TargetAwareResultLineage? Latest(TargetAwareResultLineage? landing,
            TargetAwareResultLineage? impact) => impact.HasValue &&
            (!landing.HasValue || impact.Value.Sequence > landing.Value.Sequence) ? impact : landing;

        internal static ReentrySimulation.LandingForecastKind AvailableKind(
            TargetAwareResultLineage? latest, long generation, object body,
            double latitude, double longitude, double terrainASL, double now, double maximumAge) =>
            latest.HasValue && TargetAwarePublicationGate.CheckObservation(latest.Value, null,
                generation, body, latitude, longitude, terrainASL, now, maximumAge, 0) ==
                TargetAwarePublicationDecision.Accept ? latest.Value.ForecastKind :
                ReentrySimulation.LandingForecastKind.NoForecast;

        internal static bool ConditionalBeforeDeorbit(bool targetAware, bool deorbitPhase,
            bool observedDeorbitThrust) => targetAware && deorbitPhase && !observedDeorbitThrust;

        // An expired or contradicted forecast cannot remain a safe landing.
        // DecelerationBurn unconditionally reads its executed timing reference;
        // retain that reference in a non-ready, trajectory-free view. This is
        // not a new forecast and never claims a landing or an impact location.
        internal static ReentrySimulation.Result UnavailableTimingView(ReentrySimulation.Result history) =>
            new ReentrySimulation.Result
            {
                Outcome = ReentrySimulation.Outcome.TIMED_OUT,
                ForecastKind = ReentrySimulation.LandingForecastKind.NoForecast,
                Body = history.Body, InputUT = history.InputUT,
                CaptureSubmissionId = history.CaptureSubmissionId,
                Trajectory = new System.Collections.Generic.List<AbsoluteVector>(),
                HasControllerBrakeReferenceUT = history.HasControllerBrakeReferenceUT,
                OwnedBrakingPlanId = history.OwnedBrakingPlanId,
                ControllerBrakeReferenceUT = history.ControllerBrakeReferenceUT,
                ControllerBrakeReferencePosition = history.ControllerBrakeReferencePosition,
                SimulatedBrakingStartUT = history.SimulatedBrakingStartUT
            };
    }
}
