using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Player-facing V2 target semantics. Original remains a reference,
    /// active is always the requested red target, and predicted is the
    /// independent blue endpoint from the current immutable estimate.
    /// </summary>
    public sealed class LandingGuidanceV2TargetState
    {
        public readonly double OriginalLatitude;
        public readonly double OriginalLongitude;
        public readonly double ActiveLatitude;
        public readonly double ActiveLongitude;
        public readonly double PredictedLatitude;
        public readonly double PredictedLongitude;
        public readonly long PredictionSnapshotVersion;
        public readonly bool VisualRebaseDone;

        public LandingGuidanceV2TargetState(double originalLatitude, double originalLongitude,
            double activeLatitude, double activeLongitude, double predictedLatitude, double predictedLongitude,
            long predictionSnapshotVersion, bool visualRebaseDone)
        {
            OriginalLatitude = originalLatitude;
            OriginalLongitude = originalLongitude;
            ActiveLatitude = activeLatitude;
            ActiveLongitude = activeLongitude;
            PredictedLatitude = predictedLatitude;
            PredictedLongitude = predictedLongitude;
            PredictionSnapshotVersion = predictionSnapshotVersion;
            VisualRebaseDone = visualRebaseDone;
        }
    }

    /// <summary>
    /// Immutable data captured on the Unity thread for one V2 planning pass.
    /// The estimator must use this value object rather than live Vessel, Orbit, or
    /// predictor state so that a result can always identify the state that produced it.
    /// </summary>
    public sealed class LandingGuidanceV2Snapshot
    {
        public readonly long Version;
        public readonly double UT;
        public readonly CelestialBody Body;
        public readonly Vector3d Position;
        public readonly Vector3d Velocity;
        public readonly double Mass;
        public readonly double AvailableDeltaV;
        public readonly double MaximumAcceleration;
        public readonly double MinimumAcceleration;
        public readonly double TargetLatitude;
        public readonly double TargetLongitude;
        // The body-fixed target is sampled from Unity at this UT. Planning
        // snapshots may be propagated to a future burn, but must retain this
        // inertial reference instead of treating a future burn as "now".
        public readonly double TargetReferenceUT;
        public readonly Vector3d TargetReferencePosition;
        public readonly bool HasTargetReferencePosition;
        public readonly bool IsLandedOrSplashed;

        public LandingGuidanceV2Snapshot(long version, double ut, CelestialBody body, Vector3d position,
            Vector3d velocity, double mass, double availableDeltaV, double maximumAcceleration, double minimumAcceleration,
            double targetLatitude, double targetLongitude, bool isLandedOrSplashed, double targetReferenceUT = double.NaN,
            Vector3d targetReferencePosition = default(Vector3d), bool hasTargetReferencePosition = false)
        {
            Version = version;
            UT = ut;
            Body = body;
            Position = position;
            Velocity = velocity;
            Mass = mass;
            AvailableDeltaV = availableDeltaV;
            MaximumAcceleration = maximumAcceleration;
            MinimumAcceleration = minimumAcceleration;
            TargetLatitude = targetLatitude;
            TargetLongitude = targetLongitude;
            TargetReferenceUT = double.IsNaN(targetReferenceUT) ? ut : targetReferenceUT;
            TargetReferencePosition = targetReferencePosition;
            HasTargetReferencePosition = hasTargetReferencePosition;
            IsLandedOrSplashed = isLandedOrSplashed;
        }

        // Compatibility with the integration branch's older V2 capture call.
        // Current production captures provide the minimum acceleration and
        // immutable target reference explicitly.
        public LandingGuidanceV2Snapshot(long version, double ut, CelestialBody body, Vector3d position,
            Vector3d velocity, double mass, double availableDeltaV, double maximumAcceleration,
            double targetLatitude, double targetLongitude, bool isLandedOrSplashed)
            : this(version, ut, body, position, velocity, mass, availableDeltaV, maximumAcceleration, 0,
                targetLatitude, targetLongitude, isLandedOrSplashed)
        {
        }
    }

    public enum LandingGuidanceV2EstimateOutcome
    {
        NotAvailable,
        AtmosphericBody,
        NotFlight,
        NoImpact,
        Impact,
        InvalidSnapshot
    }

    /// <summary>
    /// A deterministic, terrain-free airless-body estimate.  Terrain refinement and
    /// atmospheric estimation are separate future responsibilities; this result must
    /// never imply that a sea-level intersection certifies a safe landing site.
    /// </summary>
    public sealed class LandingGuidanceV2Estimate
    {
        public readonly long SnapshotVersion;
        public readonly LandingGuidanceV2EstimateOutcome Outcome;
        public readonly double ImpactUT;
        public readonly Vector3d ImpactPosition;
        public readonly Vector3d ImpactVelocity;
        public readonly double TargetError;
        public readonly double TerrainAltitude;
        public readonly string Detail;

        public bool HasImpact => Outcome == LandingGuidanceV2EstimateOutcome.Impact;

        public LandingGuidanceV2Estimate(long snapshotVersion, LandingGuidanceV2EstimateOutcome outcome,
            double impactUT, Vector3d impactPosition, Vector3d impactVelocity, double targetError, string detail,
            double terrainAltitude = double.NaN)
        {
            SnapshotVersion = snapshotVersion;
            Outcome = outcome;
            ImpactUT = impactUT;
            ImpactPosition = impactPosition;
            ImpactVelocity = impactVelocity;
            TargetError = targetError;
            Detail = detail;
            TerrainAltitude = terrainAltitude;
        }
    }

    /// <summary>
    /// Records whether two independent estimator evaluations of the same immutable
    /// snapshot produced the same result. This is diagnostic evidence only.
    /// </summary>
    public sealed class LandingGuidanceV2EstimatorValidation
    {
        public readonly LandingGuidanceV2Estimate RepeatedEstimate;
        public readonly bool IsDeterministic;
        public readonly string Detail;

        public LandingGuidanceV2EstimatorValidation(LandingGuidanceV2Estimate repeatedEstimate, bool isDeterministic,
            string detail)
        {
            RepeatedEstimate = repeatedEstimate;
            IsDeterministic = isDeterministic;
            Detail = detail;
        }
    }

    public enum LandingGuidanceV2PreflightState
    {
        NotFlight,
        UnsupportedBody,
        WaitingForImpactTrajectory,
        Rejected,
        NeedsCompletePlan
    }

    /// <summary>
    /// A passive screen for physical lower bounds. It never represents a complete
    /// landing plan and never authorizes a V2 command.
    /// </summary>
    public sealed class LandingGuidanceV2PreflightAssessment
    {
        public readonly long SnapshotVersion;
        public readonly LandingGuidanceV2PreflightState State;
        public readonly double LocalGravity;
        public readonly double BrakingDeltaVLowerBound;
        public readonly double DeltaVAboveLowerBound;
        public readonly string Reason;

        public bool CommandAuthorized => false;

        public LandingGuidanceV2PreflightAssessment(long snapshotVersion, LandingGuidanceV2PreflightState state,
            double localGravity, double brakingDeltaVLowerBound, double deltaVAboveLowerBound, string reason)
        {
            SnapshotVersion = snapshotVersion;
            State = state;
            LocalGravity = localGravity;
            BrakingDeltaVLowerBound = brakingDeltaVLowerBound;
            DeltaVAboveLowerBound = deltaVAboveLowerBound;
            Reason = reason;
        }
    }

    public enum AirlessLandingPlanState
    {
        NotApplicable,
        Rejected,
        Candidate
    }

    public enum AtmosphericLandingPlanState
    {
        NotApplicable,
        WaitingForEstimate,
        Rejected,
        Candidate
    }

    /// <summary>
    /// A robust atmospheric entry plan. The endpoint is deliberately a corridor
    /// assessment, not an exact-touchdown promise: atmosphere, lift, parachutes
    /// and powered terminal response retain explicit uncertainty.
    /// </summary>
    public sealed class AtmosphericLandingPlan
    {
        public readonly long SnapshotVersion;
        public readonly AtmosphericLandingPlanState State;
        public readonly double PredictedTargetError;
        public readonly double EntryCorridorRadius;
        public readonly double EndpointUncertainty;
        public readonly double TerminalReserve;
        public readonly double LandingMargin;
        public readonly Vector3d StrategicEntryDeltaV;
        public readonly double StrategicEntryBurnUT;
        public readonly double EntryUT;
        public readonly double EntryTargetError;
        public readonly string Reason;

        public AtmosphericLandingPlan(long snapshotVersion, AtmosphericLandingPlanState state,
            double predictedTargetError, double entryCorridorRadius, double endpointUncertainty,
            double terminalReserve, double landingMargin, string reason,
            Vector3d strategicEntryDeltaV = default(Vector3d), double strategicEntryBurnUT = double.NaN,
            double entryUT = double.NaN, double entryTargetError = double.NaN)
        {
            SnapshotVersion = snapshotVersion;
            State = state;
            PredictedTargetError = predictedTargetError;
            EntryCorridorRadius = entryCorridorRadius;
            EndpointUncertainty = endpointUncertainty;
            TerminalReserve = terminalReserve;
            LandingMargin = landingMargin;
            StrategicEntryDeltaV = strategicEntryDeltaV;
            StrategicEntryBurnUT = strategicEntryBurnUT;
            EntryUT = entryUT;
            EntryTargetError = entryTargetError;
            Reason = reason;
        }
    }

    /// <summary>
    /// Immutable strategic-deorbit candidate. It identifies the exact snapshot
    /// that authorized planning; the phase manager must still take a fresh
    /// snapshot at each command and warp boundary before using it.
    /// </summary>
    public sealed class AirlessLandingPlan
    {
        public readonly long SnapshotVersion;
        public readonly AirlessLandingPlanState State;
        public readonly Vector3d PlaneAlignmentDeltaV;
        public readonly double PlaneAlignmentDeltaVMagnitude;
        public readonly double PlaneAlignmentBurnUT;
        public readonly Vector3d StrategicDeorbitDeltaV;
        public readonly double StrategicDeorbitDeltaVMagnitude;
        public readonly double StrategicBurnUT;
        public readonly double BrakingEntryUT;
        public readonly double TerminalBrakingLowerBound;
        public readonly double TrimBudget;
        public readonly double TerminalDivertReserve;
        public readonly double Contingency;
        public readonly double TotalLowerBound;
        public readonly double LowerBoundMargin;
        public readonly double SignedDownrange;
        public readonly double CrossRange;
        public readonly double CorridorLimit;
        public readonly LandingGuidanceV2Estimate CandidateEstimate;
        public readonly string Reason;

        public bool CommandAuthorized => State == AirlessLandingPlanState.Candidate;

        public AirlessLandingPlan(long snapshotVersion, AirlessLandingPlanState state, Vector3d strategicDeorbitDeltaV,
            double terminalBrakingLowerBound, double signedDownrange, double crossRange, double corridorLimit,
            LandingGuidanceV2Estimate candidateEstimate, double availableDeltaV, string reason, double strategicBurnUT = double.NaN,
            double trimBudget = 0, double terminalDivertReserve = 0, double contingency = 0,
            Vector3d planeAlignmentDeltaV = default(Vector3d), double planeAlignmentBurnUT = double.NaN,
            double brakingEntryUT = double.NaN)
        {
            SnapshotVersion = snapshotVersion;
            State = state;
            PlaneAlignmentDeltaV = planeAlignmentDeltaV;
            PlaneAlignmentDeltaVMagnitude = planeAlignmentDeltaV.magnitude;
            PlaneAlignmentBurnUT = planeAlignmentBurnUT;
            StrategicDeorbitDeltaV = strategicDeorbitDeltaV;
            StrategicDeorbitDeltaVMagnitude = strategicDeorbitDeltaV.magnitude;
            StrategicBurnUT = strategicBurnUT;
            BrakingEntryUT = brakingEntryUT;
            TerminalBrakingLowerBound = terminalBrakingLowerBound;
            TrimBudget = trimBudget;
            TerminalDivertReserve = terminalDivertReserve;
            Contingency = contingency;
            TotalLowerBound = PlaneAlignmentDeltaVMagnitude + StrategicDeorbitDeltaVMagnitude + terminalBrakingLowerBound + trimBudget + terminalDivertReserve + contingency;
            LowerBoundMargin = availableDeltaV - TotalLowerBound;
            SignedDownrange = signedDownrange;
            CrossRange = crossRange;
            CorridorLimit = corridorLimit;
            CandidateEstimate = candidateEstimate;
            Reason = reason;
        }

    }

    /// <summary>
    /// Preview-only assessment.  It intentionally exposes a lower bound instead of a
    /// false feasibility verdict until V2 owns a complete deorbit, braking, and reserve plan.
    /// </summary>
    public sealed class LandingGuidanceV2Preflight
    {
        public readonly LandingGuidanceV2Snapshot Snapshot;
        public readonly LandingGuidanceV2Estimate Estimate;
        public readonly LandingGuidanceV2EstimatorValidation EstimatorValidation;
        public readonly LandingGuidanceV2PreflightAssessment Assessment;
        public readonly AirlessLandingPlan AirlessPlan;
        public readonly AtmosphericLandingPlan AtmosphericPlan;
        public readonly double BrakingDeltaVLowerBound;
        public readonly double DeltaVAboveLowerBound;

        public bool CanAssess => Snapshot != null && Estimate != null;

        public LandingGuidanceV2Preflight(LandingGuidanceV2Snapshot snapshot, LandingGuidanceV2Estimate estimate,
            LandingGuidanceV2EstimatorValidation estimatorValidation, LandingGuidanceV2PreflightAssessment assessment,
            AirlessLandingPlan airlessPlan, double brakingDeltaVLowerBound, AtmosphericLandingPlan atmosphericPlan = null)
        {
            Snapshot = snapshot;
            Estimate = estimate;
            EstimatorValidation = estimatorValidation;
            Assessment = assessment;
            AirlessPlan = airlessPlan;
            AtmosphericPlan = atmosphericPlan;
            BrakingDeltaVLowerBound = brakingDeltaVLowerBound;
            DeltaVAboveLowerBound = snapshot == null ? double.NaN : snapshot.AvailableDeltaV - brakingDeltaVLowerBound;
        }
    }
}
