using System;
using MechJebLib.HoverslamSimulation;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Searches a future burn epoch and the complete strategic burn vector for
    /// a target-relative, long-side impact corridor on an airless body.
    /// </summary>
    public static class AirlessLandingPlanner
    {
        private const int CoarseSamples = 72;
        // A low Mun orbit moves several hundred metres over one second.  This
        // final temporal resolution keeps the long-side corridor meaningful,
        // instead of letting epoch quantisation alone exceed its width.
        private const int RefinementSamples = 360;
        private const double DesiredLongSideFraction = 0.50;

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
                double bestScore = double.PositiveInfinity;
                for (int i = 0; i <= CoarseSamples; ++i)
                    Consider(SolveStrategicVector(snapshot, coast, snapshot.UT + 10.0 + coast.period * i / CoarseSamples), snapshot.AvailableDeltaV, ref best, ref bestScore);

                if (best.Valid)
                {
                    double span = coast.period / CoarseSamples;
                    for (int i = 0; i <= RefinementSamples; ++i)
                    {
                        double ut = best.BurnUT - span + 2.0 * span * i / RefinementSamples;
                        if (ut >= snapshot.UT + 2.0)
                            Consider(SolveStrategicVector(snapshot, coast, ut), snapshot.AvailableDeltaV, ref best, ref bestScore);
                    }
                }

                if (!best.Valid || !best.WithinCorridor)
                    return Reject(snapshot, best.Burn, best.Estimate, best.Downrange, best.CrossRange, best.Corridor,
                        best.Valid
                            ? "The closest strategic vector is outside the long-side corridor; no engine command was authorized."
                            : "No finite airless strategic-deorbit vector produced a valid impact trajectory.");

                AirlessLandingBudget budget = CandidateBudget(best);
                if (!budget.Fits(snapshot.AvailableDeltaV))
                    return Reject(snapshot, best.Burn, best.Estimate, best.Downrange, best.CrossRange, best.Corridor, budget,
                        "Usable delta-V is below the V2 strategic, trim, terminal-reserve, and contingency budget.");

                return new AirlessLandingPlan(snapshot.Version, AirlessLandingPlanState.Candidate, best.Burn,
                    budget.Terminal, best.Downrange, best.CrossRange, best.Corridor, best.Estimate, snapshot.AvailableDeltaV,
                    "Future strategic vector satisfies the impact, long-side corridor, and budget constraints.", best.BurnUT,
                    budget.Trim, budget.Reserve, budget.Contingency, Vector3d.zero);
            }
            catch (Exception ex)
            {
                return Reject(snapshot, "Airless strategic-deorbit planning failed: " + ex.GetType().Name);
            }
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

        private static void Consider(Candidate candidate, double availableDeltaV, ref Candidate best, ref double bestScore)
        {
            if (!candidate.Valid) return;
            double score = CandidateScore(candidate, availableDeltaV);
            if (score < bestScore)
            {
                best = candidate;
                bestScore = score;
            }
        }

        private static Candidate SolveStrategicVector(LandingGuidanceV2Snapshot source, Orbit coast, double burnUT)
        {
            Vector3d position = coast.WorldBCIPositionAtUT(burnUT);
            Vector3d velocity = coast.WorldOrbitalVelocityAtUT(burnUT);
            var burnOrbit = new Orbit();
            burnOrbit.UpdateFromStateVectors(position, velocity, source.Body, burnUT);
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
            Vector3d planeSeed = desiredHorizontalVelocity - horizontalVelocity;
            var alignedOrbit = new Orbit();
            alignedOrbit.UpdateFromStateVectors(position, velocity + planeSeed, source.Body, burnUT);
            Vector3d deorbitSeed = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(alignedOrbit, burnUT, source.Body.Radius * 0.9);

            // Start from both the inexpensive conventional deorbit and the
            // geometry-derived plane seed. The previous single high-energy
            // seed could trap coordinate descent near an unusable solution.
            Vector3d[] seeds = { baselineBurn, 1.5 * baselineBurn, planeSeed + deorbitSeed };
            Candidate best = default(Candidate);
            foreach (Vector3d seed in seeds)
            {
                Candidate candidate = RefineStrategicVector(source, burnUT, position, velocity, seed,
                    horizontalVelocity.normalized, targetPlaneNormal, up);
                if (candidate.Valid && (!best.Valid || CandidateScore(candidate, source.AvailableDeltaV) < CandidateScore(best, source.AvailableDeltaV)))
                    best = candidate;
            }
            return best;
        }

        private static Candidate RefineStrategicVector(LandingGuidanceV2Snapshot source, double burnUT,
            Vector3d position, Vector3d velocity, Vector3d seed, Vector3d alongTrack, Vector3d planeNormal, Vector3d radial)
        {
            Candidate best = EvaluateVector(source, burnUT, position, velocity, seed);
            Vector3d[] axes = { alongTrack, planeNormal, radial };
            double[] steps = { 80.0, 40.0, 20.0, 10.0, 5.0, 2.0, 0.5 };
            foreach (double step in steps)
            {
                for (int pass = 0; pass < 8; ++pass)
                {
                    bool improved = false;
                    foreach (Vector3d axis in axes)
                    {
                        for (int sign = -1; sign <= 1; sign += 2)
                        {
                            Vector3d burn = best.Valid
                                ? best.Burn + sign * step * axis
                                : seed + sign * step * axis;
                            Candidate candidate = EvaluateVector(source, burnUT, position, velocity, burn);
                            if (!candidate.Valid || best.Valid && CandidateScore(candidate, source.AvailableDeltaV) >= CandidateScore(best, source.AvailableDeltaV)) continue;
                            best = candidate;
                            improved = true;
                        }
                    }
                    if (!improved) break;
                }
            }
            return best;
        }

        private static Candidate EvaluateVector(LandingGuidanceV2Snapshot source, double burnUT, Vector3d position,
            Vector3d velocity, Vector3d burn)
        {
            if (burn.magnitude > source.AvailableDeltaV) return default(Candidate);
            var state = new LandingGuidanceV2Snapshot(source.Version, burnUT, source.Body, position, velocity + burn,
                source.Mass, source.AvailableDeltaV, source.MaximumAcceleration, source.TargetLatitude, source.TargetLongitude, false);
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(state);
            if (!estimate.HasImpact) return default(Candidate);
            Vector3d error = estimate.ImpactPosition - TargetAt(source, estimate.ImpactUT);
            Vector3d direction = Vector3d.Exclude(estimate.ImpactPosition.normalized, estimate.ImpactVelocity);
            if (direction.sqrMagnitude < 1e-9) return default(Candidate);
            direction.Normalize();
            double downrange = Vector3d.Dot(error, direction);
            double crossRange = Math.Sqrt(Math.Max(0, error.sqrMagnitude - downrange * downrange));
            double corridor = Math.Max(100.0, source.Body.Radius * 0.002);
            return new Candidate(burnUT, Vector3d.zero, burn, estimate, downrange, crossRange, corridor, source.AvailableDeltaV);
        }

        private static double CandidateScore(Candidate candidate, double availableDeltaV)
        {
            double desiredDownrange = candidate.Corridor * DesiredLongSideFraction;
            double targetError = Math.Abs(candidate.Downrange - desiredDownrange) + 1.5 * candidate.CrossRange;
            AirlessLandingBudget budget = CandidateBudget(candidate);
            // Feasibility is a hard planning criterion. The previous score had
            // a tiny burn penalty, so it preferred an accurately aimed but
            // unusable high-energy impact over an affordable landing vector.
            if (!budget.Fits(availableDeltaV))
                return 1000000000.0 + 10000.0 * (budget.Total - availableDeltaV) + targetError;
            return 10.0 * targetError + budget.Total;
        }

        private static AirlessLandingBudget CandidateBudget(Candidate candidate) =>
            AirlessLandingBudget.For(candidate.Burn.magnitude, candidate.Estimate.ImpactVelocity.magnitude);

        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot snapshot, string reason) =>
            Reject(snapshot, Vector3d.zero, null, double.NaN, double.NaN, double.NaN, reason);

        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot snapshot, Vector3d burn,
            LandingGuidanceV2Estimate estimate, double downrange, double crossRange, double corridor, string reason) =>
            new AirlessLandingPlan(snapshot?.Version ?? -1, AirlessLandingPlanState.Rejected, burn,
                estimate?.ImpactVelocity.magnitude ?? double.NaN, downrange, crossRange, corridor, estimate,
                snapshot?.AvailableDeltaV ?? double.NaN, reason);

        private static AirlessLandingPlan Reject(LandingGuidanceV2Snapshot snapshot, Vector3d burn,
            LandingGuidanceV2Estimate estimate, double downrange, double crossRange, double corridor,
            AirlessLandingBudget budget, string reason) =>
            new AirlessLandingPlan(snapshot?.Version ?? -1, AirlessLandingPlanState.Rejected, burn,
                budget.Terminal, downrange, crossRange, corridor, estimate, snapshot?.AvailableDeltaV ?? double.NaN,
                reason, double.NaN, budget.Trim, budget.Reserve, budget.Contingency, Vector3d.zero);

        private static Vector3d TargetAt(LandingGuidanceV2Snapshot snapshot, double ut)
        {
            Vector3d target = snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0) - snapshot.Body.position;
            return Quaternion.AngleAxis((float)(360d * (ut - snapshot.UT) / snapshot.Body.rotationPeriod), snapshot.Body.angularVelocity) * target;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private struct Candidate
        {
            public readonly double BurnUT;
            public readonly Vector3d PlaneAlignmentBurn;
            public readonly Vector3d Burn;
            public readonly LandingGuidanceV2Estimate Estimate;
            public readonly double Downrange;
            public readonly double CrossRange;
            public readonly double Corridor;
            public readonly double AvailableDeltaV;
            public bool Valid => Estimate != null;
            public bool WithinCorridor => Valid && Downrange >= 0 && Downrange <= Corridor && CrossRange <= Corridor;

            public Candidate(double burnUT, Vector3d planeAlignmentBurn, Vector3d burn, LandingGuidanceV2Estimate estimate,
                double downrange, double crossRange, double corridor, double availableDeltaV = double.NaN)
            {
                BurnUT = burnUT;
                PlaneAlignmentBurn = planeAlignmentBurn;
                Burn = burn;
                Estimate = estimate;
                Downrange = downrange;
                CrossRange = crossRange;
                Corridor = corridor;
                AvailableDeltaV = availableDeltaV;
            }
        }
    }
}
