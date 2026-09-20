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

    /// <summary>
    /// Preview-only assessment.  It intentionally exposes a lower bound instead of a
    /// false feasibility verdict until V2 owns a complete deorbit, braking, and reserve plan.
    /// </summary>
    public sealed class LandingGuidanceV2Preflight
    {
        public readonly LandingGuidanceV2Snapshot Snapshot;
        public readonly LandingGuidanceV2Estimate Estimate;
        public readonly LandingGuidanceV2EstimatorValidation EstimatorValidation;
        public readonly double BrakingDeltaVLowerBound;
        public readonly double DeltaVAboveLowerBound;

        public bool CanAssess => Snapshot != null && Estimate != null;

        public LandingGuidanceV2Preflight(LandingGuidanceV2Snapshot snapshot, LandingGuidanceV2Estimate estimate,
            LandingGuidanceV2EstimatorValidation estimatorValidation, double brakingDeltaVLowerBound)
        {
            Snapshot = snapshot;
            Estimate = estimate;
            EstimatorValidation = estimatorValidation;
            BrakingDeltaVLowerBound = brakingDeltaVLowerBound;
            DeltaVAboveLowerBound = snapshot == null ? double.NaN : snapshot.AvailableDeltaV - brakingDeltaVLowerBound;
        }
    }
}
