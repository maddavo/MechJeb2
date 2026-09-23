using System;
using MechJebLib.HoverslamSimulation;
using MechJebLib.Lambert;
using MechJebLib.Primitives;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech
{
    public enum AirlessPostBurnAction { Coast, RecoveryTrim, Replan }

    /// <summary>
    /// A deterministic post-burn decision. Once a finite airless deorbit burn
    /// has begun, no result in this decision authorizes an uncontrolled exit.
    /// </summary>
    public sealed class AirlessPostBurnDecision
    {
        public readonly AirlessPostBurnAction Action;
        public readonly Vector3d Correction;
        public readonly LandingGuidanceV2Estimate Estimate;
        public readonly double RecoveryBudget;
        public readonly string Reason;

        public AirlessPostBurnDecision(AirlessPostBurnAction action, Vector3d correction,
            LandingGuidanceV2Estimate estimate, double recoveryBudget, string reason)
        { Action = action; Correction = correction; Estimate = estimate; RecoveryBudget = recoveryBudget; Reason = reason; }
    }

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

                // A target-transfer burn cannot be scheduled after the
                // unburned vehicle has already reached the surface.  Rails
                // warp previously accepted this stale epoch and carried V2
                // past impact while it remained in WarpToStrategic.
                LandingGuidanceV2Estimate currentImpact = AirlessImpactEstimator.Estimate(snapshot);

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
                        return CandidatePlan(snapshot, direct, directBudget,
                            "Direct rotating-target transfer satisfies the impact, long-side corridor, and budget constraints.");
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

                if (currentImpact.HasImpact && best.BurnUT >= currentImpact.ImpactUT - 0.25)
                    return Reject(snapshot, best.Burn, best.Estimate, best.Downrange, best.CrossRange, best.Corridor,
                        "V2 rejected the strategic burn because the current unburned trajectory impacts before its planned burn epoch.");

                AirlessLandingBudget budget = CandidateBudget(best);
                if (!budget.Fits(snapshot.AvailableDeltaV))
                    return Reject(snapshot, best.Burn, best.Estimate, best.Downrange, best.CrossRange, best.Corridor, budget,
                        "Usable delta-V is below the V2 strategic, trim, terminal-reserve, and contingency budget.");

                return CandidatePlan(snapshot, best, budget,
                    "Future strategic vector satisfies the impact, long-side corridor, and budget constraints.");
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
            if (!committedPlan.HasTargetReferencePosition || !snapshot.HasTargetReferencePosition ||
                Math.Abs(committedPlan.TargetReferenceUT - snapshot.TargetReferenceUT) > 0.001 ||
                (committedPlan.TargetReferencePosition - snapshot.TargetReferencePosition).sqrMagnitude > 1e-4 ||
                Math.Abs(committedPlan.TargetTerrainAltitude - snapshot.TargetTerrainAltitude) > 0.001)
            {
                reason = "V2 ignition snapshot does not retain the plan's immutable target reference and terrain epoch.";
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
                // Rails and KSP's patched-conic state can differ slightly from
                // the ideal coast used at plan time. Retarget at the existing
                // finite-burn midpoint from this fresh state; do not search a
                // later epoch, which would restart auto-warp indefinitely.
                candidate = SolveTargetedTransferAtEpoch(snapshot, committedPlan.StrategicBurnUT);
                if (!candidate.Valid || !candidate.WithinCorridor)
                {
                    reason = "The committed V2 strategic vector no longer reaches the required long-side target corridor.";
                    return false;
                }
            }

            AirlessLandingBudget budget = CandidateBudget(candidate);
            if (!budget.Fits(snapshot.AvailableDeltaV))
            {
                reason = "The committed V2 strategic vector no longer preserves the terminal reserve and contingency budget.";
                return false;
            }

            validatedPlan = CandidatePlan(snapshot, candidate, budget,
                "Fresh ignition snapshot passed impact, corridor, and budget validation at the committed burn midpoint.");
            return true;
        }

        /// <summary>
        /// Performs the cheap, deterministic safety validation required after
        /// rails warp and before the separately planned plane-alignment burn.
        /// A full strategic plan is intentionally rebuilt after this burn; at
        /// this gate V2 verifies that the committed burn epoch, target lineage,
        /// coast state, and complete budget are still usable.
        /// </summary>
        public static bool TryValidateCommittedPlaneAlignmentBurn(LandingGuidanceV2Snapshot snapshot,
            AirlessLandingPlan committedPlan, out string reason)
        {
            reason = null;
            if (snapshot == null || committedPlan == null ||
                committedPlan.State != AirlessLandingPlanState.Candidate ||
                committedPlan.PlaneAlignmentDeltaVMagnitude <= AirlessLandingPhaseManager.BurnCompleteDeltaV)
            {
                reason = "V2 has no committed plane-alignment burn to validate at the burn gate.";
                return false;
            }
            if (!committedPlan.HasTargetReferencePosition || !snapshot.HasTargetReferencePosition ||
                Math.Abs(committedPlan.TargetReferenceUT - snapshot.TargetReferenceUT) > 0.001 ||
                (committedPlan.TargetReferencePosition - snapshot.TargetReferencePosition).sqrMagnitude > 1e-4 ||
                Math.Abs(committedPlan.TargetTerrainAltitude - snapshot.TargetTerrainAltitude) > 0.001)
            {
                reason = "V2 plane-alignment ignition snapshot does not retain the immutable target reference and terrain epoch.";
                return false;
            }
            if (snapshot.UT > committedPlan.PlaneAlignmentBurnUT + 0.25)
            {
                reason = "V2 reached the plane-alignment validation gate after the committed burn midpoint.";
                return false;
            }
            if (snapshot.AvailableDeltaV + 1e-6 < committedPlan.TotalLowerBound)
            {
                reason = "V2 plane-alignment ignition snapshot no longer preserves the complete terminal reserve and contingency budget.";
                return false;
            }
            if (snapshot.UT < committedPlan.PlaneAlignmentBurnUT)
            {
                var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT,
                    snapshot.Position, snapshot.Velocity);
                if (!coast.IsBound || !coast.TryStateAt(committedPlan.PlaneAlignmentBurnUT, out _, out _))
                {
                    reason = "V2 could not propagate the fresh plane-alignment snapshot to the committed burn midpoint.";
                    return false;
                }
            }
            return true;
        }

        public static AirlessPostBurnDecision DecidePostBurn(LandingGuidanceV2Snapshot snapshot,
            AirlessLandingPlan committedPlan, bool allowRecoveryTrim)
        {
            LandingGuidanceV2Estimate estimate = snapshot == null ? null : AirlessImpactEstimator.Estimate(snapshot);
            if (estimate != null && estimate.HasImpact && estimate.TargetError <= committedPlan.CorridorLimit)
                return new AirlessPostBurnDecision(AirlessPostBurnAction.Coast, Vector3d.zero, estimate, 0,
                    "Strategic deorbit is inside the accepted target corridor; coasting to V2 braking approach.");

            // A finite-burn result outside the corridor must be corrected by a
            // new complete rotating-target transfer, not merely an improvement
            // in the scalar miss distance. A bounded trim may be used only
            // when its own propagated endpoint is inside that same corridor.
            if (allowRecoveryTrim && estimate != null && estimate.HasImpact &&
                TryPlanRecoveryTrim(snapshot, committedPlan, out Vector3d correction,
                    out LandingGuidanceV2Estimate recovered, out double recoveryBudget) &&
                recovered.TargetError <= committedPlan.CorridorLimit)
                return new AirlessPostBurnDecision(AirlessPostBurnAction.RecoveryTrim, correction, recovered,
                    recoveryBudget, "V2 found a reserve-protected recovery trim whose propagated endpoint is inside the target corridor.");

            return new AirlessPostBurnDecision(AirlessPostBurnAction.Replan, Vector3d.zero, estimate, 0,
                estimate == null || !estimate.HasImpact
                    ? "V2 lost the predicted impact after its finite burn; acquiring a fresh complete target-transfer plan."
                    : "V2 finite-burn endpoint is outside the target corridor; acquiring a fresh complete target-transfer plan.");
        }

        /// <summary>
        /// Searches a bounded post-deorbit correction in the full local orbital
        /// frame.  A long-range error cannot be corrected reliably by a single
        /// along-track nudge: radial, normal, and combined directions change
        /// the next surface crossing in different ways.
        /// </summary>
        public static bool TryPlanBoundedTrim(LandingGuidanceV2Snapshot snapshot, double trimBudget,
            out Vector3d correction, out LandingGuidanceV2Estimate improvedEstimate)
        {
            correction = Vector3d.zero;
            improvedEstimate = null;
            if (snapshot == null || trimBudget <= 0) return false;
            LandingGuidanceV2Estimate current = AirlessImpactEstimator.Estimate(snapshot);
            if (!current.HasImpact) return false;

            Vector3d radial = snapshot.Position.normalized;
            Vector3d prograde = Vector3d.Exclude(radial, snapshot.Velocity);
            if (prograde.sqrMagnitude < 1e-9) return false;
            prograde.Normalize();
            Vector3d normal = Vector3d.Cross(radial, prograde);
            if (normal.sqrMagnitude < 1e-9) return false;
            normal.Normalize();

            double bestError = current.TargetError;
            // The full signed cube supplies the six axes, twelve planar
            // diagonals, and eight three-axis directions.  It is small enough
            // for a physics tick and deterministic for trace replay.
            for (int radialSign = -1; radialSign <= 1; ++radialSign)
            for (int progradeSign = -1; progradeSign <= 1; ++progradeSign)
            for (int normalSign = -1; normalSign <= 1; ++normalSign)
            {
                if (radialSign == 0 && progradeSign == 0 && normalSign == 0) continue;
                Vector3d direction = radialSign * radial + progradeSign * prograde + normalSign * normal;
                direction.Normalize();
                for (int i = 1; i <= 32; ++i)
                {
                    double magnitude = trimBudget * i / 32.0;
                    Vector3d candidateBurn = magnitude * direction;
                    var candidateSnapshot = new LandingGuidanceV2Snapshot(snapshot.Version, snapshot.UT, snapshot.Body,
                        snapshot.Position, snapshot.Velocity + candidateBurn, snapshot.Mass, snapshot.AvailableDeltaV,
                        snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude, snapshot.TargetLongitude, false,
                        snapshot.TargetReferenceUT, snapshot.TargetReferencePosition, snapshot.HasTargetReferencePosition,
                        snapshot.TargetTerrainAltitude);
                    LandingGuidanceV2Estimate candidate = AirlessImpactEstimator.Estimate(candidateSnapshot);
                    if (!candidate.HasImpact || candidate.TargetError >= bestError) continue;
                    correction = candidateBurn;
                    improvedEstimate = candidate;
                    bestError = candidate.TargetError;
                }
            }
            return improvedEstimate != null;
        }

        /// <summary>
        /// Returns a recovery trim that can use only delta-v left after the
        /// current terminal braking, local-divert reserve, and contingency have
        /// been protected.  This is used after an executed finite strategic
        /// burn, when a rail/finite-burn mismatch leaves a larger residual than
        /// the original two metre-per-second trim allocation.
        /// </summary>
        public static bool TryPlanRecoveryTrim(LandingGuidanceV2Snapshot snapshot, AirlessLandingPlan committedPlan,
            out Vector3d correction, out LandingGuidanceV2Estimate improvedEstimate, out double recoveryBudget)
        {
            correction = Vector3d.zero;
            improvedEstimate = null;
            recoveryBudget = 0;
            if (snapshot == null || committedPlan == null) return false;
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(snapshot);
            if (!estimate.HasImpact) return false;

            double radius = estimate.ImpactPosition.magnitude;
            double gravity = radius > 0 ? snapshot.Body.gravParameter / (radius * radius) : double.NaN;
            double impactSpeed = SurfaceRelativeImpactVelocity(snapshot, estimate).magnitude;
            AirlessLandingBudget terminalBudget = AirlessLandingBudget.For(0, impactSpeed, snapshot.MaximumAcceleration,
                gravity, estimate.TargetError, committedPlan.CorridorLimit, snapshot.MinimumAcceleration);
            if (double.IsInfinity(terminalBudget.Total) || double.IsNaN(terminalBudget.Total)) return false;

            // Retain the whole protected terminal allocation.  The cap avoids
            // turning a recovery trim into a second strategic burn.
            double protectedTerminal = terminalBudget.Terminal + terminalBudget.Reserve + terminalBudget.Contingency;
            recoveryBudget = Math.Max(0, Math.Min(35.0, snapshot.AvailableDeltaV - protectedTerminal));
            if (recoveryBudget < Math.Max(0.25, committedPlan.TrimBudget)) return false;
            return TryPlanBoundedTrim(snapshot, recoveryBudget, out correction, out improvedEstimate);
        }

        /// <summary>
        /// Checks whether an off-corridor but valid post-burn impact may remain
        /// under V2 terminal control.  It deliberately protects braking,
        /// divert, and contingency allocations; it never authorizes a release
        /// of guidance merely because the strategic corridor was missed.
        /// </summary>
        public static bool CanContinueToTerminal(LandingGuidanceV2Snapshot snapshot, AirlessLandingPlan committedPlan,
            out string reason)
        {
            reason = null;
            if (snapshot == null || committedPlan == null)
            {
                reason = "V2 has no committed airless plan for terminal continuation.";
                return false;
            }
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(snapshot);
            if (!estimate.HasImpact)
            {
                reason = "V2 has no valid post-burn impact trajectory for terminal continuation.";
                return false;
            }
            double radius = estimate.ImpactPosition.magnitude;
            double gravity = radius > 0 ? snapshot.Body.gravParameter / (radius * radius) : double.NaN;
            AirlessLandingBudget terminalBudget = AirlessLandingBudget.For(0,
                SurfaceRelativeImpactVelocity(snapshot, estimate).magnitude, snapshot.MaximumAcceleration, gravity,
                estimate.TargetError, committedPlan.CorridorLimit, snapshot.MinimumAcceleration);
            if (!terminalBudget.Fits(snapshot.AvailableDeltaV))
            {
                reason = "V2 cannot protect the current terminal braking and divert reserve.";
                return false;
            }
            reason = "V2 retained terminal braking, divert, and contingency reserve after the strategic correction.";
            return true;
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

        private static Candidate SolveTargetedTransferAtEpoch(LandingGuidanceV2Snapshot source, double burnUT)
        {
            if (burnUT < source.UT - 0.25) return default(Candidate);
            var coast = new AirlessConicTrajectory(source.Body.gravParameter, source.UT, source.Position, source.Velocity);
            if (!coast.IsBound || !coast.TryStateAt(burnUT, out Vector3d position, out Vector3d velocity))
                return default(Candidate);
            V3 positionV3 = position.ToV3();
            V3 velocityV3 = velocity.ToV3();
            V3 angularMomentum = V3.Cross(positionV3, velocityV3);
            if (angularMomentum.sqrMagnitude <= 1e-12) return default(Candidate);

            Candidate best = default(Candidate);
            double bestScore = double.PositiveInfinity;
            const int flightSamples = 72;
            const double minimumFlightTime = 45.0;
            for (int i = 1; i <= flightSamples; ++i)
            {
                double flightTime = minimumFlightTime + coast.Period * i / flightSamples;
                try
                {
                    Vector3d target = TargetAt(source, burnUT + flightTime);
                    (V3 initialVelocity, _) = Gooding.Solve(source.Body.gravParameter, positionV3, target.ToV3(),
                        flightTime, TransferGeometry.Prograde, 0, angularMomentum);
                    Candidate trial = EvaluateVector(source, burnUT, position, velocity, initialVelocity.ToVector3d() - velocity);
                    if (!trial.Valid || !trial.WithinCorridor) continue;
                    double score = CandidateScore(trial, source.AvailableDeltaV);
                    if (score < bestScore) { best = trial; bestScore = score; }
                }
                catch (Exception)
                {
                    // A singular Lambert arc leaves other transfer times eligible.
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
                candidate.Estimate.ImpactUT - budget.BrakingTime, snapshot.TargetReferenceUT,
                snapshot.TargetReferencePosition, snapshot.HasTargetReferencePosition, snapshot.TargetTerrainAltitude);

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
