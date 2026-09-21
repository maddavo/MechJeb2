using System;
using UnityEngine;

namespace MuMech
{
    public static class AirlessLandingPlanner
    {
        private const int CoarseSamples = 180;
        private const int RefinementSamples = 48;

        public static AirlessLandingPlan Plan(LandingGuidanceV2Snapshot snapshot)
        {
            if (snapshot == null || snapshot.Body == null || snapshot.IsLandedOrSplashed || snapshot.Body.atmosphere)
                return Reject(snapshot, "An airborne airless snapshot is required.");
            try
            {
                var coast = new Orbit();
                coast.UpdateFromStateVectors(snapshot.Position, snapshot.Velocity, snapshot.Body, snapshot.UT);
                if (!Finite(coast.period) || coast.eccentricity >= 1.0)
                    return Reject(snapshot, "V2 requires a bound airless coast orbit before strategic-deorbit planning.");

                Candidate best = default(Candidate);
                double bestMiss = double.PositiveInfinity;
                for (int i = 0; i <= CoarseSamples; ++i)
                    Consider(Evaluate(snapshot, coast, snapshot.UT + 10.0 + coast.period * i / CoarseSamples), ref best, ref bestMiss);

                if (best.Valid)
                {
                    double span = coast.period / CoarseSamples;
                    for (int i = 0; i <= RefinementSamples; ++i)
                    {
                        double ut = best.BurnUT - span + 2.0 * span * i / RefinementSamples;
                        if (ut >= snapshot.UT + 2.0) Consider(Evaluate(snapshot, coast, ut), ref best, ref bestMiss);
                    }
                }

                if (!best.Valid || !best.WithinCorridor)
                    return Reject(snapshot, "No future strategic-deorbit burn reaches the required long-side target corridor.");
                double terminal = best.Estimate.ImpactVelocity.magnitude;
                double trim = Math.Max(5.0, best.Burn.magnitude * 0.10);
                double reserve = Math.Max(20.0, terminal * 0.10);
                double contingency = Math.Max(10.0, best.Burn.magnitude * 0.02);
                double plannedDeltaV = best.Burn.magnitude + terminal + trim + reserve + contingency;
                if (snapshot.AvailableDeltaV < plannedDeltaV)
                    return Reject(snapshot, best.Burn, best.Estimate, best.Downrange, best.CrossRange, best.Corridor,
                        "Usable delta-V is below the V2 strategic, trim, terminal-reserve, and contingency budget.");
                return new AirlessLandingPlan(snapshot.Version, AirlessLandingPlanState.Candidate, best.Burn,
                    best.Estimate.ImpactVelocity.magnitude, best.Downrange, best.CrossRange, best.Corridor, best.Estimate,
                    snapshot.AvailableDeltaV, "Future strategic-deorbit candidate satisfies the impact, corridor, and budget constraints.", best.BurnUT,
                    trim, reserve, contingency);
            }
            catch (Exception ex) { return Reject(snapshot, "Airless strategic-deorbit planning failed: " + ex.GetType().Name); }
        }

        private static void Consider(Candidate candidate, ref Candidate best, ref double bestMiss)
        {
            if (!candidate.Valid) return;
            double miss = candidate.Downrange + candidate.CrossRange;
            if (miss < bestMiss) { best = candidate; bestMiss = miss; }
        }

        private static Candidate Evaluate(LandingGuidanceV2Snapshot source, Orbit coast, double burnUT)
        {
            Vector3d position = coast.WorldBCIPositionAtUT(burnUT);
            Vector3d velocity = coast.WorldOrbitalVelocityAtUT(burnUT);
            var burnOrbit = new Orbit(); burnOrbit.UpdateFromStateVectors(position, velocity, source.Body, burnUT);
            Vector3d baselineBurn = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(burnOrbit, burnUT, source.Body.Radius * 0.9);
            Vector3d normal = Vector3d.Cross(position, velocity).normalized;
            double normalStep = Math.Max(5.0, Math.Min(100.0, baselineBurn.magnitude));
            Candidate best = default(Candidate);
            for (int i = -4; i <= 4; ++i)
            {
                Vector3d burn = baselineBurn + i * normalStep * normal;
                var state = new LandingGuidanceV2Snapshot(source.Version, burnUT, source.Body, position, velocity + burn,
                    source.Mass, source.AvailableDeltaV, source.MaximumAcceleration, source.TargetLatitude, source.TargetLongitude, false);
                LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(state);
                if (!estimate.HasImpact) continue;
                Vector3d error = estimate.ImpactPosition - TargetAt(source, estimate.ImpactUT);
                Vector3d direction = Vector3d.Exclude(estimate.ImpactPosition.normalized, estimate.ImpactVelocity);
                if (direction.sqrMagnitude < 1e-9) continue;
                direction.Normalize();
                double downrange = Vector3d.Dot(error, direction);
                double crossRange = Math.Sqrt(Math.Max(0, error.sqrMagnitude - downrange * downrange));
                double corridor = Math.Max(100.0, source.Body.Radius * 0.002);
                if (downrange < 0) continue;
                Candidate candidate = new Candidate(burnUT, burn, estimate, downrange, crossRange, corridor);
                if (!best.Valid || candidate.Burn.magnitude < best.Burn.magnitude) best = candidate;
            }
            return best;
        }

        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot s, string reason) => Reject(s, Vector3d.zero, null, double.NaN, double.NaN, double.NaN, reason);
        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot s, Vector3d burn, LandingGuidanceV2Estimate estimate, double downrange, double crossRange, double corridor, string reason) =>
            new AirlessLandingPlan(s?.Version ?? -1, AirlessLandingPlanState.Rejected, burn, estimate?.ImpactVelocity.magnitude ?? double.NaN, downrange, crossRange, corridor, estimate, s?.AvailableDeltaV ?? double.NaN, reason);
        private static Vector3d TargetAt(LandingGuidanceV2Snapshot s, double ut)
        {
            Vector3d target = s.Body.GetWorldSurfacePosition(s.TargetLatitude, s.TargetLongitude, 0) - s.Body.position;
            return Quaternion.AngleAxis((float)(360d * (ut - s.UT) / s.Body.rotationPeriod), s.Body.angularVelocity) * target;
        }
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private struct Candidate
        {
            public readonly double BurnUT; public readonly Vector3d Burn; public readonly LandingGuidanceV2Estimate Estimate;
            public readonly double Downrange; public readonly double CrossRange; public readonly double Corridor;
            public bool Valid => Estimate != null;
            public bool WithinCorridor => Valid && Downrange <= Corridor && CrossRange <= Corridor;
            public Candidate(double burnUT, Vector3d burn, LandingGuidanceV2Estimate estimate, double downrange, double crossRange, double corridor)
            { BurnUT = burnUT; Burn = burn; Estimate = estimate; Downrange = downrange; CrossRange = crossRange; Corridor = corridor; }
        }
    }
}
