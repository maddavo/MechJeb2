using UnityEngine;

namespace MuMech
{
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
        public readonly double TargetLatitude;
        public readonly double TargetLongitude;
        public readonly bool IsLandedOrSplashed;

        public LandingGuidanceV2Snapshot(long version, double ut, CelestialBody body, Vector3d position,
            Vector3d velocity, double mass, double availableDeltaV, double maximumAcceleration,
            double targetLatitude, double targetLongitude, bool isLandedOrSplashed)
        {
            Version = version;
            UT = ut;
            Body = body;
            Position = position;
            Velocity = velocity;
            Mass = mass;
            AvailableDeltaV = availableDeltaV;
            MaximumAcceleration = maximumAcceleration;
            TargetLatitude = targetLatitude;
            TargetLongitude = targetLongitude;
            IsLandedOrSplashed = isLandedOrSplashed;
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
        public readonly string Detail;

        public bool HasImpact => Outcome == LandingGuidanceV2EstimateOutcome.Impact;

        public LandingGuidanceV2Estimate(long snapshotVersion, LandingGuidanceV2EstimateOutcome outcome,
            double impactUT, Vector3d impactPosition, Vector3d impactVelocity, double targetError, string detail)
        {
            SnapshotVersion = snapshotVersion;
            Outcome = outcome;
            ImpactUT = impactUT;
            ImpactPosition = impactPosition;
            ImpactVelocity = impactVelocity;
            TargetError = targetError;
            Detail = detail;
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

    /// <summary>
    /// Immutable strategic-deorbit candidate. It is data only: V2 does not yet
    /// hand this vector to attitude, throttle, RCS, staging, target, or warp.
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

        public bool CommandAuthorized => false;

        public AirlessLandingPlan(long snapshotVersion, AirlessLandingPlanState state, Vector3d strategicDeorbitDeltaV,
            double terminalBrakingLowerBound, double signedDownrange, double crossRange, double corridorLimit,
            LandingGuidanceV2Estimate candidateEstimate, double availableDeltaV, string reason, double strategicBurnUT = double.NaN,
            double trimBudget = 0, double terminalDivertReserve = 0, double contingency = 0,
            Vector3d planeAlignmentDeltaV = default(Vector3d), double planeAlignmentBurnUT = double.NaN)
        {
            SnapshotVersion = snapshotVersion;
            State = state;
            PlaneAlignmentDeltaV = planeAlignmentDeltaV;
            PlaneAlignmentDeltaVMagnitude = planeAlignmentDeltaV.magnitude;
            PlaneAlignmentBurnUT = planeAlignmentBurnUT;
            StrategicDeorbitDeltaV = strategicDeorbitDeltaV;
            StrategicDeorbitDeltaVMagnitude = strategicDeorbitDeltaV.magnitude;
            StrategicBurnUT = strategicBurnUT;
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
        public readonly double BrakingDeltaVLowerBound;
        public readonly double DeltaVAboveLowerBound;

        public bool CanAssess => Snapshot != null && Estimate != null;

        public LandingGuidanceV2Preflight(LandingGuidanceV2Snapshot snapshot, LandingGuidanceV2Estimate estimate,
            LandingGuidanceV2EstimatorValidation estimatorValidation, LandingGuidanceV2PreflightAssessment assessment,
            AirlessLandingPlan airlessPlan, double brakingDeltaVLowerBound)
        {
            Snapshot = snapshot;
            Estimate = estimate;
            EstimatorValidation = estimatorValidation;
            Assessment = assessment;
            AirlessPlan = airlessPlan;
            BrakingDeltaVLowerBound = brakingDeltaVLowerBound;
            DeltaVAboveLowerBound = snapshot == null ? double.NaN : snapshot.AvailableDeltaV - brakingDeltaVLowerBound;
        }
    }
}
