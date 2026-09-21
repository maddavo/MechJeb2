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
                double plannedDeltaV = best.PlaneAlignmentBurn.magnitude + best.Burn.magnitude + terminal + trim + reserve + contingency;
                if (snapshot.AvailableDeltaV < plannedDeltaV)
                    return Reject(snapshot, best.Burn, best.Estimate, best.Downrange, best.CrossRange, best.Corridor,
                        "Usable delta-V is below the V2 strategic, trim, terminal-reserve, and contingency budget.");
                return new AirlessLandingPlan(snapshot.Version, AirlessLandingPlanState.Candidate, best.Burn,
                    best.Estimate.ImpactVelocity.magnitude, best.Downrange, best.CrossRange, best.Corridor, best.Estimate,
                    snapshot.AvailableDeltaV, "Future strategic-deorbit candidate satisfies the impact, corridor, and budget constraints.", best.BurnUT,
                    trim, reserve, contingency, best.PlaneAlignmentBurn);
            }
            catch (Exception ex) { return Reject(snapshot, "Airless strategic-deorbit planning failed: " + ex.GetType().Name); }
        }

        public static bool TryPlanBoundedTrim(LandingGuidanceV2Snapshot snapshot, double trimBudget,
            out Vector3d correction, out LandingGuidanceV2Estimate improvedEstimate)
        {
            correction = Vector3d.zero;
            improvedEstimate = null;
            if (snapshot == null || trimBudget <= 0) return false;
            LandingGuidanceV2Estimate current = AirlessImpactEstimator.Estimate(snapshot);
            if (!current.HasImpact) return false;
            Vector3d direction = Vector3d.Exclude(snapshot.Position.normalized,
                TargetAt(snapshot, current.ImpactUT) - snapshot.Position);
            if (direction.sqrMagnitude < 1e-9) return false;
            direction.Normalize();
            double bestError = current.TargetError;
            for (int i = 1; i <= 8; ++i)
            {
                double magnitude = trimBudget * i / 8.0;
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    Vector3d candidateBurn = sign * magnitude * direction;
                    var candidateSnapshot = new LandingGuidanceV2Snapshot(snapshot.Version, snapshot.UT, snapshot.Body,
                        snapshot.Position, snapshot.Velocity + candidateBurn, snapshot.Mass, snapshot.AvailableDeltaV,
                        snapshot.MaximumAcceleration, snapshot.TargetLatitude, snapshot.TargetLongitude, false);
                    LandingGuidanceV2Estimate candidate = AirlessImpactEstimator.Estimate(candidateSnapshot);
                    if (!candidate.HasImpact || candidate.TargetError >= bestError) continue;
                    correction = candidateBurn;
                    improvedEstimate = candidate;
                    bestError = candidate.TargetError;
                }
            }
            return improvedEstimate != null;
        }

        private static void Consider(Candidate candidate, ref Candidate best, ref double bestMiss)
        {
            if (!candidate.Valid) return;
            // Keep the nearest solution even when the coarse pass is on the
            // short side.  The former positive-downrange filter could discard
            // every coarse sample, leaving no point to refine and reporting a
            // false "no candidate" result for an otherwise reachable target.
            double miss = Math.Abs(candidate.Downrange) + candidate.CrossRange;
            if (miss < bestMiss) { best = candidate; bestMiss = miss; }
        }

        private static Candidate Evaluate(LandingGuidanceV2Snapshot source, Orbit coast, double burnUT)
        {
            Vector3d position = coast.WorldBCIPositionAtUT(burnUT);
            Vector3d velocity = coast.WorldOrbitalVelocityAtUT(burnUT);
            var burnOrbit = new Orbit(); burnOrbit.UpdateFromStateVectors(position, velocity, source.Body, burnUT);
            Vector3d baselineBurn = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(burnOrbit, burnUT, source.Body.Radius * 0.9);
            Orbit baselineOrbit = burnOrbit.PerturbedOrbit(burnUT, baselineBurn);
            double baselineImpactUT = baselineOrbit.NextTimeOfRadius(burnUT, source.Body.Radius);
            if (!Finite(baselineImpactUT)) return default(Candidate);
            Vector3d up = position.normalized;
            Vector3d horizontalVelocity = Vector3d.Exclude(up, velocity);
            Vector3d targetRadial = TargetAt(source, baselineImpactUT).normalized;
            Vector3d targetPlaneNormal = Vector3d.Cross(position, targetRadial);
            if (horizontalVelocity.sqrMagnitude < 1e-9 || targetPlaneNormal.sqrMagnitude < 1e-9) return default(Candidate);
            targetPlaneNormal.Normalize();
            Vector3d desiredHorizontalVelocity = Vector3d.Cross(targetPlaneNormal, up).normalized * horizontalVelocity.magnitude;
            if (Vector3d.Dot(desiredHorizontalVelocity, horizontalVelocity) < 0) desiredHorizontalVelocity = -desiredHorizontalVelocity;
            Vector3d planeChangeBurn = desiredHorizontalVelocity - horizontalVelocity;
            var planeAlignedOrbit = new Orbit();
            planeAlignedOrbit.UpdateFromStateVectors(position, velocity + planeChangeBurn, source.Body, burnUT);
            Vector3d alignedDeorbitBurn = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(planeAlignedOrbit, burnUT, source.Body.Radius * 0.9);
            Candidate best = default(Candidate);
            for (int i = 0; i <= 16; ++i)
            {
                // Keep the plane-change and deorbit components explicit.  The
                // interpolation searches only the deorbit depth after the target
                // plane is aligned, and every resulting impact is re-estimated.
                Vector3d deorbitBurn = Vector3d.Lerp(baselineBurn, alignedDeorbitBurn, i / 16.0);
                Vector3d burn = planeChangeBurn + deorbitBurn;
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
                Candidate candidate = new Candidate(burnUT, planeChangeBurn, deorbitBurn, estimate, downrange, crossRange, corridor);
                if (!best.Valid || Math.Abs(candidate.Downrange) + candidate.CrossRange < Math.Abs(best.Downrange) + best.CrossRange) best = candidate;
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
            public readonly double BurnUT; public readonly Vector3d PlaneAlignmentBurn; public readonly Vector3d Burn; public readonly LandingGuidanceV2Estimate Estimate;
            public readonly double Downrange; public readonly double CrossRange; public readonly double Corridor;
            public bool Valid => Estimate != null;
            public bool WithinCorridor => Valid && Downrange <= Corridor && CrossRange <= Corridor;
            public Candidate(double burnUT, Vector3d planeAlignmentBurn, Vector3d burn, LandingGuidanceV2Estimate estimate, double downrange, double crossRange, double corridor)
            { BurnUT = burnUT; PlaneAlignmentBurn = planeAlignmentBurn; Burn = burn; Estimate = estimate; Downrange = downrange; CrossRange = crossRange; Corridor = corridor; }
        }
    }
}
