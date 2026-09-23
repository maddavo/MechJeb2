using System;
using MechJebLib.HoverslamSimulation;
using MechJebLib.Lambert;
using MechJebLib.Primitives;
using MechJebLibBindings;
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

        public static AirlessLandingPlan Planning(LandingGuidanceV2Snapshot snapshot) =>
            new AirlessLandingPlan(snapshot?.Version ?? -1, AirlessLandingPlanState.Planning, Vector3d.zero,
                double.NaN, double.NaN, double.NaN, double.NaN, null,
                snapshot?.AvailableDeltaV ?? double.NaN,
                "V2 is searching the immutable airless target plan on a background worker.");

        public static AirlessLandingPlan Plan(LandingGuidanceV2Snapshot snapshot)
        {
            if (snapshot == null || ReferenceEquals(snapshot.Body, null) || snapshot.IsLandedOrSplashed || snapshot.Body.atmosphere)
                return Reject(snapshot, "An airborne airless snapshot is required.");
            try
            {
                var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT,
                    snapshot.Position, snapshot.Velocity);
                if (!coast.IsBound)
                    return Reject(snapshot, "V2 requires a bound airless coast orbit before strategic-deorbit planning.");

                // The former local deorbit perturbation search could improve an
                // existing impact but could not discover the narrow targeting
                // solution in the recorded Mun case.  Solve the complete
                // two-body transfer to the rotating target at each coast epoch,
                // then independently propagate and budget that proposed burn.
                Candidate direct = SolveTargetedTransfers(snapshot, coast);
                if (direct.Valid && direct.WithinCorridor)
                {
                    AirlessLandingBudget directBudget = CandidateBudget(direct);
                    if (directBudget.Fits(snapshot.AvailableDeltaV))
                        return new AirlessLandingPlan(snapshot.Version, AirlessLandingPlanState.Candidate, direct.Burn,
                            directBudget.Terminal, direct.Downrange, direct.CrossRange, direct.Corridor, direct.Estimate,
                            snapshot.AvailableDeltaV,
                            "Direct rotating-target transfer satisfies the impact, long-side corridor, and budget constraints.",
                            direct.BurnUT, directBudget.Trim, directBudget.Reserve, directBudget.Contingency,
                            Vector3d.zero, double.NaN, direct.Estimate.ImpactUT - directBudget.BrakingTime);
                }

                Candidate best = direct;
                double bestScore = direct.Valid ? CandidateScore(direct, snapshot.AvailableDeltaV) : double.PositiveInfinity;
                for (int i = 0; i <= CoarseSamples; ++i)
                    Consider(SolveStrategicVector(snapshot, coast, snapshot.UT + 10.0 + coast.Period * i / CoarseSamples), snapshot.AvailableDeltaV, ref best, ref bestScore);

                if (best.Valid)
                {
                    double span = coast.Period / CoarseSamples;
                    for (int i = 0; i <= RefinementSamples; ++i)
                    {
                        double ut = best.BurnUT - span + 2.0 * span * i / RefinementSamples;
                        if (ut >= snapshot.UT + 2.0)
                            Consider(SolveStrategicVector(snapshot, coast, ut), snapshot.AvailableDeltaV, ref best, ref bestScore);
                    }
                }

                // A target outside the current orbital plane must be reached by
                // a distinct alignment burn.  Do not hide that cost inside a
                // giant deorbit vector: it is a separate phase with its own
                // epoch, attitude gate and counted delta-V.
                if (!best.Valid || !best.WithinCorridor)
                {
                    Candidate aligned = SolveWithPlaneAlignment(snapshot, coast);
                    aligned = RefineAlignedStrategicVector(snapshot, coast, aligned);
                    if (aligned.Valid && (!best.Valid || CandidateScore(aligned, snapshot.AvailableDeltaV) < CandidateScore(best, snapshot.AvailableDeltaV)))
                        best = aligned;
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
                    budget.Trim, budget.Reserve, budget.Contingency, best.PlaneAlignmentBurn, best.PlaneAlignmentBurnUT,
                    best.Estimate.ImpactUT - budget.BrakingTime);
            }
            catch (Exception ex)
            {
                return Reject(snapshot, "Airless strategic-deorbit planning failed: " + ex.GetType().Name);
            }
        }

        /// <summary>
        /// Revalidates the strategic vector which was accepted before warp at
        /// the fresh burn-gate snapshot.  The phase manager must not replace a
        /// committed finite burn with a newly searched, later burn every time
        /// it exits warp: that creates an endless moving ignition gate.
        /// </summary>
        public static bool TryValidateCommittedStrategicBurn(LandingGuidanceV2Snapshot snapshot,
            AirlessLandingPlan committedPlan, out AirlessLandingPlan validatedPlan, out string reason)
        {
            validatedPlan = null;
            reason = null;
            if (snapshot == null || committedPlan == null ||
                committedPlan.State != AirlessLandingPlanState.Candidate)
            {
                reason = "V2 has no committed strategic plan to validate at the burn gate.";
                return false;
            }
            if (committedPlan.PlaneAlignmentDeltaVMagnitude > 0.5)
            {
                reason = "V2 cannot validate a combined plane-alignment and strategic vector at one burn gate.";
                return false;
            }

            // The plan's epoch represents the midpoint of a finite burn.  The
            // controller takes its fresh snapshot at ignition, then propagates
            // that unburned state to the committed midpoint before checking the
            // impulse-equivalent vector.  Evaluating the vector at ignition
            // treats a correctly timed finite burn as an early impulse and
            // falsely rejects it.
            if (snapshot.UT > committedPlan.StrategicBurnUT + 0.25)
            {
                reason = "V2 reached the strategic validation gate after the committed burn midpoint.";
                return false;
            }
            Vector3d position = snapshot.Position;
            Vector3d velocity = snapshot.Velocity;
            if (snapshot.UT < committedPlan.StrategicBurnUT)
            {
                var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT, position, velocity);
                if (!coast.IsBound || !coast.TryStateAt(committedPlan.StrategicBurnUT, out position, out velocity))
                {
                    reason = "V2 could not propagate the fresh ignition snapshot to the committed burn midpoint.";
                    return false;
                }
            }
            Candidate candidate = EvaluateVector(snapshot, committedPlan.StrategicBurnUT, position, velocity,
                committedPlan.StrategicDeorbitDeltaV);
            if (!candidate.Valid || !candidate.WithinCorridor)
            {
                reason = "The committed V2 strategic vector no longer reaches the required long-side target corridor.";
                return false;
            }

            AirlessLandingBudget budget = CandidateBudget(candidate);
            if (!budget.Fits(snapshot.AvailableDeltaV))
            {
                reason = "The committed V2 strategic vector no longer preserves the terminal reserve and contingency budget.";
                return false;
            }

            validatedPlan = CandidatePlan(snapshot, candidate, budget,
                "Committed strategic vector passed fresh burn-gate impact, corridor, and budget validation.");
            return true;
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
                        snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude, snapshot.TargetLongitude, false, snapshot.TargetReferenceUT, snapshot.TargetReferencePosition, snapshot.HasTargetReferencePosition, snapshot.TargetTerrainAltitude);
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

        private static Candidate SolveTargetedTransfers(LandingGuidanceV2Snapshot source, AirlessConicTrajectory coast)
        {
            Candidate best = default(Candidate);
            double bestScore = double.PositiveInfinity;
            const int epochSamples = 48;
            const int flightSamples = 36;
            const double minimumFlightTime = 45.0;

            // Sample every possible strategic burn epoch over one coast orbit.
            // For each epoch, solve the target's rotated surface position at a
            // finite future impact time.  This explores the complete burn vector
            // instead of nudging a retrograde seed in only two dimensions.
            for (int epochIndex = 0; epochIndex <= epochSamples; ++epochIndex)
            {
                double burnUT = source.UT + 2.0 + coast.Period * epochIndex / epochSamples;
                if (!coast.TryStateAt(burnUT, out Vector3d position, out Vector3d velocity)) continue;
                V3 positionV3 = position.ToV3();
                V3 velocityV3 = velocity.ToV3();
                V3 angularMomentum = V3.Cross(positionV3, velocityV3);
                if (angularMomentum.sqrMagnitude <= 1e-12) continue;

                for (int flightIndex = 1; flightIndex <= flightSamples; ++flightIndex)
                {
                    double flightTime = minimumFlightTime + coast.Period * flightIndex / flightSamples;
                    double impactUT = burnUT + flightTime;
                    Vector3d target = TargetAt(source, impactUT);
                    if (!Finite(target.x) || !Finite(target.y) || !Finite(target.z) || target.sqrMagnitude <= 0) continue;
                    try
                    {
                        (V3 initialTransferVelocity, _) = Gooding.Solve(source.Body.gravParameter, positionV3,
                            target.ToV3(), flightTime, TransferGeometry.Prograde, 0, angularMomentum);
                        Vector3d burn = initialTransferVelocity.ToVector3d() - velocity;
                        Candidate candidate = EvaluateVector(source, burnUT, position, velocity, burn);
                        if (!candidate.Valid || !candidate.WithinCorridor) continue;

                        double score = CandidateScore(candidate, source.AvailableDeltaV);
                        if (score < bestScore)
                        {
                            best = candidate;
                            bestScore = score;
                        }
                    }
                    catch (Exception)
                    {
                        // A singular Lambert arc leaves all other sampled arcs eligible.
                    }
                }
            }
            return best;
        }

        private static Candidate SolveStrategicVector(LandingGuidanceV2Snapshot source, AirlessConicTrajectory coast, double burnUT)
        {
            if (!coast.TryStateAt(burnUT, out Vector3d position, out Vector3d velocity)) return default(Candidate);
            Vector3d baselineBurn = BaselineDeorbitBurn(source, burnUT, position, velocity);
            var baselineOrbit = new AirlessConicTrajectory(source.Body.gravParameter, burnUT, position, velocity + baselineBurn);
            if (!baselineOrbit.TryNextRadiusCrossing(burnUT, source.Body.Radius, out double baselineImpactUT) || !Finite(baselineImpactUT))
                return default(Candidate);

            Vector3d up = position.normalized;
            Vector3d horizontalVelocity = Vector3d.Exclude(up, velocity);
            if (horizontalVelocity.sqrMagnitude < 1e-9) return default(Candidate);

            // This routine solves deorbit only. Plane matching is deliberately
            // performed by SolveWithPlaneAlignment so the burn budget and the
            // phase manager cannot mistake a plane change for deorbit energy.
            Vector3d[] seeds = { baselineBurn, 1.5 * baselineBurn };
            Candidate best = default(Candidate);
            foreach (Vector3d seed in seeds)
            {
                Candidate candidate = RefineStrategicVector(source, burnUT, position, velocity, seed,
                    horizontalVelocity.normalized, up);
                if (candidate.Valid && (!best.Valid || CandidateScore(candidate, source.AvailableDeltaV) < CandidateScore(best, source.AvailableDeltaV)))
                    best = candidate;
            }
            return best;
        }

        private static Candidate SolveWithPlaneAlignment(LandingGuidanceV2Snapshot source, AirlessConicTrajectory coast)
        {
            Candidate best = default(Candidate);
            double bestScore = double.PositiveInfinity;
            const int alignmentSamples = 16;
            const int deorbitSamples = 24;
            for (int i = 1; i <= alignmentSamples; ++i)
            {
                double alignmentUT = source.UT + coast.Period * i / alignmentSamples;
                if (!coast.TryStateAt(alignmentUT, out Vector3d position, out Vector3d velocity)) continue;
                Vector3d up = position.normalized;
                Vector3d horizontal = Vector3d.Exclude(up, velocity);
                for (int j = 1; j <= deorbitSamples; ++j)
                {
                    double deorbitUT = alignmentUT + 5.0 + coast.Period * j / deorbitSamples;
                    // First find the flight time of a deorbit candidate on the
                    // unrotated coast.  That establishes the epoch at which the
                    // target must lie in the new orbital plane.  Aligning to a
                    // target one arbitrary orbit after the plane burn omitted
                    // the body's rotation during coast-to-impact and left the
                    // Mun test case kilometres crossrange.
                    Candidate timing = SolveStrategicVector(source, coast, deorbitUT);
                    if (!timing.Valid || !Finite(timing.Estimate.ImpactUT)) continue;
                    if (!TryPlaneAlignmentBurn(source, position, horizontal, timing.Estimate.ImpactUT, out Vector3d planeBurn)) continue;
                    if (planeBurn.magnitude > source.AvailableDeltaV) continue;

                    var alignedCoast = new AirlessConicTrajectory(source.Body.gravParameter, alignmentUT,
                        position, velocity + planeBurn);
                    if (!alignedCoast.IsBound) continue;
                    var alignedSnapshot = new LandingGuidanceV2Snapshot(source.Version, alignmentUT, source.Body,
                        position, velocity + planeBurn, source.Mass, source.AvailableDeltaV - planeBurn.magnitude,
                    source.MaximumAcceleration, source.MinimumAcceleration, source.TargetLatitude, source.TargetLongitude, false, source.TargetReferenceUT, source.TargetReferencePosition, source.HasTargetReferencePosition, source.TargetTerrainAltitude);
                    Candidate candidate = SolveStrategicVector(alignedSnapshot, alignedCoast, deorbitUT);
                    if (!candidate.Valid) continue;
                    // The first estimate was made on the unaligned coast.  Feed
                    // the resulting impact epoch back into the plane geometry so
                    // the target and the new orbital plane converge together.
                    for (int iteration = 0; iteration < 3; ++iteration)
                    {
                        if (!TryPlaneAlignmentBurn(source, position, horizontal, candidate.Estimate.ImpactUT,
                            out Vector3d refinedPlaneBurn)) break;
                        if ((refinedPlaneBurn - planeBurn).magnitude < 0.01) break;
                        planeBurn = refinedPlaneBurn;
                        if (planeBurn.magnitude > source.AvailableDeltaV) break;
                        alignedCoast = new AirlessConicTrajectory(source.Body.gravParameter, alignmentUT,
                            position, velocity + planeBurn);
                        if (!alignedCoast.IsBound) break;
                        alignedSnapshot = new LandingGuidanceV2Snapshot(source.Version, alignmentUT, source.Body,
                            position, velocity + planeBurn, source.Mass, source.AvailableDeltaV - planeBurn.magnitude,
                            source.MaximumAcceleration, source.MinimumAcceleration, source.TargetLatitude, source.TargetLongitude,
                            false, source.TargetReferenceUT, source.TargetReferencePosition, source.HasTargetReferencePosition, source.TargetTerrainAltitude);
                        candidate = SolveStrategicVector(alignedSnapshot, alignedCoast, deorbitUT);
                        if (!candidate.Valid) break;
                    }
                    if (!candidate.Valid) continue;
                    candidate = candidate.WithPlaneAlignment(planeBurn, alignmentUT, source);
                    double score = CandidateScore(candidate, source.AvailableDeltaV);
                    if (score < bestScore)
                    {
                        best = candidate;
                        bestScore = score;
                    }
                }
            }
            return best;
        }

        private static Candidate RefineAlignedStrategicVector(LandingGuidanceV2Snapshot source, AirlessConicTrajectory originalCoast,
            Candidate coarse)
        {
            if (!coarse.Valid || coarse.PlaneAlignmentBurn.sqrMagnitude < 1e-9 || !Finite(coarse.PlaneAlignmentBurnUT))
                return coarse;

            if (!originalCoast.TryStateAt(coarse.PlaneAlignmentBurnUT, out Vector3d position, out Vector3d velocity))
                return coarse;
            var alignedCoast = new AirlessConicTrajectory(source.Body.gravParameter, coarse.PlaneAlignmentBurnUT,
                position, velocity + coarse.PlaneAlignmentBurn);
            if (!alignedCoast.IsBound)
                return coarse;

            var alignedSnapshot = new LandingGuidanceV2Snapshot(source.Version, coarse.PlaneAlignmentBurnUT, source.Body,
                position, velocity + coarse.PlaneAlignmentBurn, source.Mass,
                source.AvailableDeltaV - coarse.PlaneAlignmentBurn.magnitude, source.MaximumAcceleration,
                source.MinimumAcceleration, source.TargetLatitude, source.TargetLongitude, false, source.TargetReferenceUT, source.TargetReferencePosition, source.HasTargetReferencePosition, source.TargetTerrainAltitude);
            Candidate best = coarse;
            double bestScore = CandidateScore(best, source.AvailableDeltaV);
            double span = alignedCoast.Period / 24.0;
            for (int i = 0; i <= RefinementSamples; ++i)
            {
                double burnUT = coarse.BurnUT - span + 2.0 * span * i / RefinementSamples;
                if (burnUT < coarse.PlaneAlignmentBurnUT + 2.0) continue;
                Candidate candidate = SolveStrategicVector(alignedSnapshot, alignedCoast, burnUT);
                if (!candidate.Valid) continue;
                candidate = candidate.WithPlaneAlignment(coarse.PlaneAlignmentBurn, coarse.PlaneAlignmentBurnUT, source);
                double score = CandidateScore(candidate, source.AvailableDeltaV);
                if (score < bestScore)
                {
                    best = candidate;
                    bestScore = score;
                }
            }
            return best;
        }

        private static Candidate RefineStrategicVector(LandingGuidanceV2Snapshot source, double burnUT,
            Vector3d position, Vector3d velocity, Vector3d seed, Vector3d alongTrack, Vector3d radial)
        {
            Candidate best = EvaluateVector(source, burnUT, position, velocity, seed);
            // Plane matching belongs exclusively to SolveWithPlaneAlignment.
            // Allowing this optimiser to refine along the orbit normal silently
            // folds a plane change into a supposed deorbit burn.
            Vector3d[] axes = { alongTrack, radial };
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
                source.Mass, source.AvailableDeltaV, source.MaximumAcceleration, source.MinimumAcceleration,
                source.TargetLatitude, source.TargetLongitude, false, source.TargetReferenceUT, source.TargetReferencePosition, source.HasTargetReferencePosition, source.TargetTerrainAltitude);
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(state);
            if (!estimate.HasImpact) return default(Candidate);
            Vector3d error = estimate.ImpactPosition - TargetAt(source, estimate.ImpactUT);
            Vector3d surfaceVelocity = SurfaceRelativeImpactVelocity(source, estimate);
            Vector3d direction = Vector3d.Exclude(estimate.ImpactPosition.normalized, surfaceVelocity);
            if (direction.sqrMagnitude < 1e-9) return default(Candidate);
            direction.Normalize();
            double downrange = Vector3d.Dot(error, direction);
            double crossRange = Math.Sqrt(Math.Max(0, error.sqrMagnitude - downrange * downrange));
            // A Lambert transfer that terminates on the selected surface point
            // is numerically centred on the corridor.  Preserve that physical
            // result rather than letting sub-millimetre rounding turn it into a
            // false "short side" rejection.
            if (Math.Abs(downrange) < 0.001) downrange = 0;
            if (crossRange < 0.001) crossRange = 0;
            double corridor = Math.Max(100.0, source.Body.Radius * 0.002);
            return new Candidate(source, burnUT, Vector3d.zero, burn, estimate, downrange, crossRange, corridor, source.AvailableDeltaV);
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

        private static AirlessLandingBudget CandidateBudget(Candidate candidate)
        {
            double radius = candidate.Estimate.ImpactPosition.magnitude;
            double gravity = candidate.Estimate == null || radius <= 0
                ? double.NaN
                : candidate.Source.Body.gravParameter / (radius * radius);
            double uncertainty = Math.Sqrt(candidate.Downrange * candidate.Downrange + candidate.CrossRange * candidate.CrossRange);
            return AirlessLandingBudget.For(candidate.PlaneAlignmentBurn.magnitude + candidate.Burn.magnitude,
                SurfaceRelativeImpactVelocity(candidate.Source, candidate.Estimate).magnitude,
                candidate.Source.MaximumAcceleration, gravity, uncertainty, candidate.Corridor,
                candidate.Source.MinimumAcceleration);
        }

        private static Vector3d SurfaceRelativeImpactVelocity(LandingGuidanceV2Snapshot snapshot,
            LandingGuidanceV2Estimate estimate) =>
            estimate.ImpactVelocity - Vector3d.Cross(snapshot.Body.angularVelocity, estimate.ImpactPosition);

        private static AirlessLandingPlan CandidatePlan(LandingGuidanceV2Snapshot snapshot, Candidate candidate,
            AirlessLandingBudget budget, string reason) =>
            new AirlessLandingPlan(snapshot.Version, AirlessLandingPlanState.Candidate, candidate.Burn,
                budget.Terminal, candidate.Downrange, candidate.CrossRange, candidate.Corridor, candidate.Estimate,
                snapshot.AvailableDeltaV, reason, candidate.BurnUT, budget.Trim, budget.Reserve, budget.Contingency,
                candidate.PlaneAlignmentBurn, candidate.PlaneAlignmentBurnUT,
                candidate.Estimate.ImpactUT - budget.BrakingTime);

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
            Vector3d target = AirlessTargetGeometry.ReferenceSurfacePosition(snapshot);
            double rotationRadians = 2.0 * Math.PI * (ut - snapshot.TargetReferenceUT) / snapshot.Body.rotationPeriod;
            return AirlessImpactEstimator.RotateAroundAxis(target, snapshot.Body.angularVelocity, rotationRadians);
        }

        private static Vector3d BaselineDeorbitBurn(LandingGuidanceV2Snapshot source, double epoch, Vector3d position,
            Vector3d velocity)
        {
            Vector3d up = position.normalized;
            Vector3d horizontal = Vector3d.Exclude(up, velocity);
            if (horizontal.sqrMagnitude < 1e-9) return Vector3d.zero;
            Vector3d direction = -horizontal.normalized;
            double targetPeriapsis = source.Body.Radius * 0.9;
            double low = 0;
            double high = Math.Min(source.AvailableDeltaV, Math.Max(10.0, horizontal.magnitude));
            while (high < source.AvailableDeltaV)
            {
                var trial = new AirlessConicTrajectory(source.Body.gravParameter, epoch, position,
                    velocity + high * direction);
                if (Finite(trial.PeriapsisRadius) && trial.PeriapsisRadius <= targetPeriapsis) break;
                high = Math.Min(source.AvailableDeltaV, high * 2.0);
            }

            var maximum = new AirlessConicTrajectory(source.Body.gravParameter, epoch, position,
                velocity + high * direction);
            if (!Finite(maximum.PeriapsisRadius) || maximum.PeriapsisRadius > targetPeriapsis)
                return high * direction;

            for (int i = 0; i < 32; ++i)
            {
                double middle = (low + high) / 2.0;
                var trial = new AirlessConicTrajectory(source.Body.gravParameter, epoch, position,
                    velocity + middle * direction);
                if (Finite(trial.PeriapsisRadius) && trial.PeriapsisRadius <= targetPeriapsis) high = middle;
                else low = middle;
            }
            return high * direction;
        }

        private static bool TryPlaneAlignmentBurn(LandingGuidanceV2Snapshot source, Vector3d position,
            Vector3d horizontal, double impactUT, out Vector3d planeBurn)
        {
            planeBurn = Vector3d.zero;
            Vector3d targetRadial = TargetAt(source, impactUT).normalized;
            Vector3d planeNormal = Vector3d.Cross(position, targetRadial);
            if (horizontal.sqrMagnitude < 1e-9 || planeNormal.sqrMagnitude < 1e-9) return false;
            planeNormal.Normalize();
            Vector3d desiredHorizontal = Vector3d.Cross(planeNormal, position.normalized).normalized * horizontal.magnitude;
            if (Vector3d.Dot(desiredHorizontal, horizontal) < 0) desiredHorizontal = -desiredHorizontal;
            planeBurn = desiredHorizontal - horizontal;
            return Finite(planeBurn.x) && Finite(planeBurn.y) && Finite(planeBurn.z);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private struct Candidate
        {
            public readonly LandingGuidanceV2Snapshot Source;
            public readonly double BurnUT;
            public readonly double PlaneAlignmentBurnUT;
            public readonly Vector3d PlaneAlignmentBurn;
            public readonly Vector3d Burn;
            public readonly LandingGuidanceV2Estimate Estimate;
            public readonly double Downrange;
            public readonly double CrossRange;
            public readonly double Corridor;
            public readonly double AvailableDeltaV;
            public bool Valid => Estimate != null;
            public bool WithinCorridor => Valid && Downrange >= 0 && Downrange <= Corridor && CrossRange <= Corridor;

            public Candidate(LandingGuidanceV2Snapshot source, double burnUT, Vector3d planeAlignmentBurn, Vector3d burn, LandingGuidanceV2Estimate estimate,
                double downrange, double crossRange, double corridor, double availableDeltaV = double.NaN, double planeAlignmentBurnUT = double.NaN)
            {
                Source = source;
                BurnUT = burnUT;
                PlaneAlignmentBurnUT = planeAlignmentBurnUT;
                PlaneAlignmentBurn = planeAlignmentBurn;
                Burn = burn;
                Estimate = estimate;
                Downrange = downrange;
                CrossRange = crossRange;
                Corridor = corridor;
                AvailableDeltaV = availableDeltaV;
            }

            public Candidate WithPlaneAlignment(Vector3d planeBurn, double planeBurnUT, LandingGuidanceV2Snapshot source) =>
                new Candidate(source, BurnUT, planeBurn, Burn, Estimate, Downrange, CrossRange, Corridor, source.AvailableDeltaV, planeBurnUT);
        }
    }
}
