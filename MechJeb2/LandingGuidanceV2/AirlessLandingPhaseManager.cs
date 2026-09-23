using System;

namespace MuMech
{
    /// <summary>
    /// Deterministic authority gate for V2's finite airless burns.  It contains
    /// no Unity or vessel access: the flight module supplies observations and
    /// performs the requested command, while tests can exercise every phase
    /// boundary without launching KSP.
    /// </summary>
    public sealed class AirlessLandingPhaseManager
    {
        public const double WarpSettleMargin = 20.0;
        public const double AttitudeReadyDegrees = 2.0;
        // Completion must be materially tighter than the landing corridor.
        // The thrust controller ramps down for the final portion; this is only
        // the residual which permits releasing finite-burn authority.
        public const double BurnCompleteDeltaV = 0.01;

        private AirlessLandingPlan _plan;
        private bool _strategicGateValidated;
        private bool _requiresStrategicReplan;
        private double _finiteBurnLeadSeconds;

        public AirlessLandingPhaseManagerPhase Phase { get; private set; } = AirlessLandingPhaseManagerPhase.Idle;
        public AirlessLandingPlan Plan => _plan;
        public int LastWorkUnits { get; private set; }

        public AirlessLandingPhaseDecision Start(AirlessLandingPlan plan, double finiteBurnLeadSeconds = 0)
        {
            _plan = plan;
            _finiteBurnLeadSeconds = Finite(finiteBurnLeadSeconds) ? Math.Max(0, finiteBurnLeadSeconds) : 0;
            _strategicGateValidated = false;
            _requiresStrategicReplan = false;
            if (!IsExecutable(plan, out string reason))
            {
                Phase = AirlessLandingPhaseManagerPhase.Rejected;
                return Reject(reason);
            }
            // Acquiring burn attitude is a prerequisite for warp.  Rails warp
            // cannot be the mechanism that creates the time needed to turn a
            // vessel, so establish the attitude at 1x first and retain the
            // normal post-warp attitude/validation gate as a second check.
            Phase = plan.PlaneAlignmentDeltaVMagnitude > BurnCompleteDeltaV
                ? AirlessLandingPhaseManagerPhase.PreparePlaneAlignmentWarp
                : AirlessLandingPhaseManagerPhase.PrepareStrategicWarp;
            return Decision(AirlessLandingPhaseDirective.None);
        }

        public AirlessLandingPhaseDecision Tick(double ut, bool autoWarp, double attitudeErrorDegrees,
            double remainingBurnDeltaV)
        {
            if (_plan == null) return Reject("V2 phase manager has no active airless plan.");
            switch (Phase)
            {
                case AirlessLandingPhaseManagerPhase.PreparePlaneAlignmentWarp:
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                    Phase = AirlessLandingPhaseManagerPhase.WarpToPlaneAlignment;
                    return Decision(AirlessLandingPhaseDirective.WarpAuthorized);
                case AirlessLandingPhaseManagerPhase.PrepareStrategicWarp:
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                    Phase = AirlessLandingPhaseManagerPhase.WarpToStrategicBurn;
                    return Decision(AirlessLandingPhaseDirective.WarpAuthorized);
                case AirlessLandingPhaseManagerPhase.WarpToPlaneAlignment:
                    return TickWarp(ut, autoWarp, _plan.PlaneAlignmentBurnUT - PlaneBurnLeadSeconds,
                        AirlessLandingPhaseManagerPhase.AlignPlaneAlignment);
                case AirlessLandingPhaseManagerPhase.WarpToStrategicBurn:
                    return TickWarp(ut, autoWarp, StrategicIgnitionUT,
                        AirlessLandingPhaseManagerPhase.AlignStrategicBurn);
                case AirlessLandingPhaseManagerPhase.AlignPlaneAlignment:
                    if (ut < _plan.PlaneAlignmentBurnUT - PlaneBurnLeadSeconds)
                        return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                    Phase = AirlessLandingPhaseManagerPhase.PlaneAlignmentBurn;
                    return Decision(AirlessLandingPhaseDirective.BeginFiniteBurn);
                case AirlessLandingPhaseManagerPhase.PlaneAlignmentBurn:
                    if (remainingBurnDeltaV > BurnCompleteDeltaV)
                    {
                        if (attitudeErrorDegrees > AttitudeReadyDegrees)
                            return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                        return Decision(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle);
                    }
                    Phase = AirlessLandingPhaseManagerPhase.AwaitStrategicReplan;
                    _requiresStrategicReplan = true;
                    return Decision(AirlessLandingPhaseDirective.FiniteBurnComplete);
                case AirlessLandingPhaseManagerPhase.AwaitStrategicReplan:
                    return Decision(AirlessLandingPhaseDirective.RequireStrategicReplan);
                case AirlessLandingPhaseManagerPhase.AlignStrategicBurn:
                    if (ut < StrategicIgnitionUT)
                        return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                    if (!_strategicGateValidated)
                        return Decision(AirlessLandingPhaseDirective.RequireFreshStrategicValidation);
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                    Phase = AirlessLandingPhaseManagerPhase.StrategicBurn;
                    return Decision(AirlessLandingPhaseDirective.BeginFiniteBurn);
                case AirlessLandingPhaseManagerPhase.StrategicBurn:
                    if (remainingBurnDeltaV > BurnCompleteDeltaV)
                    {
                        if (attitudeErrorDegrees > AttitudeReadyDegrees)
                            return Decision(AirlessLandingPhaseDirective.RequestAttitude);
                        return Decision(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle);
                    }
                    Phase = AirlessLandingPhaseManagerPhase.Coast;
                    return Decision(AirlessLandingPhaseDirective.FiniteBurnComplete);
                case AirlessLandingPhaseManagerPhase.Coast:
                    return Decision(AirlessLandingPhaseDirective.None);
                case AirlessLandingPhaseManagerPhase.Rejected:
                    return Decision(AirlessLandingPhaseDirective.None);
                default:
                    return Reject("V2 phase manager entered an unknown airless phase.");
            }
        }

        public AirlessLandingPhaseDecision AcceptStrategicValidation(long snapshotVersion, double snapshotUT,
            bool valid, string rejectionReason = null)
        {
            if (Phase != AirlessLandingPhaseManagerPhase.AlignStrategicBurn)
                return Reject("A strategic validation was supplied outside the strategic alignment phase.");
            if (snapshotUT < StrategicIgnitionUT)
                return Reject("V2 strategic validation arrived before the finite-burn ignition gate.");
            if (snapshotUT > _plan.StrategicBurnUT + 0.25)
                return Reject("V2 strategic validation missed the planned finite-burn midpoint.");
            if (snapshotVersion <= _plan.SnapshotVersion)
                return Reject("V2 strategic validation did not use a fresh post-warp snapshot.");
            if (!valid)
                return Reject(string.IsNullOrEmpty(rejectionReason)
                    ? "V2 strategic validation rejected the committed burn."
                    : rejectionReason);
            _strategicGateValidated = true;
            return Decision(AirlessLandingPhaseDirective.RequestAttitude);
        }

        public AirlessLandingPhaseDecision AdoptStrategicReplan(AirlessLandingPlan plan)
        {
            if (Phase != AirlessLandingPhaseManagerPhase.AwaitStrategicReplan || !_requiresStrategicReplan)
                return Reject("V2 received a strategic replan outside the plane-alignment replan gate.");
            if (!IsExecutable(plan, out string reason)) return Reject(reason);
            if (plan.PlaneAlignmentDeltaVMagnitude > BurnCompleteDeltaV)
                return Reject("V2 plane-alignment replan still requires another plane burn.");
            _plan = plan;
            _requiresStrategicReplan = false;
            _strategicGateValidated = false;
            Phase = AirlessLandingPhaseManagerPhase.PrepareStrategicWarp;
            return Decision(AirlessLandingPhaseDirective.None);
        }

        private double StrategicIgnitionUT => _plan.StrategicBurnUT - _finiteBurnLeadSeconds;
        private double PlaneBurnLeadSeconds => _plan.PlaneAlignmentDeltaVMagnitude > BurnCompleteDeltaV ? _finiteBurnLeadSeconds : 0;

        private AirlessLandingPhaseDecision TickWarp(double ut, bool autoWarp, double burnUT,
            AirlessLandingPhaseManagerPhase next)
        {
            if (!Finite(burnUT) || burnUT <= 0) return Reject("V2 plan has no finite burn epoch.");
            if (autoWarp && ut < burnUT - WarpSettleMargin)
                return Decision(AirlessLandingPhaseDirective.RequestWarp, burnUT - WarpSettleMargin);
            Phase = next;
            _strategicGateValidated = false;
            return Decision(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude);
        }

        private static bool IsExecutable(AirlessLandingPlan plan, out string reason)
        {
            if (plan == null || !plan.CommandAuthorized) { reason = "V2 requires a command-authorized airless plan."; return false; }
            if (!Finite(plan.StrategicBurnUT) || plan.StrategicBurnUT <= 0) { reason = "V2 plan has no finite strategic ignition epoch."; return false; }
            if (plan.StrategicDeorbitDeltaVMagnitude <= 0) { reason = "V2 plan has no finite strategic burn magnitude."; return false; }
            if (!Finite(plan.LowerBoundMargin) || plan.LowerBoundMargin < 0) { reason = "V2 plan does not preserve a positive landing margin."; return false; }
            if (!Finite(plan.SignedDownrange) || !Finite(plan.CrossRange) || !Finite(plan.CorridorLimit) ||
                plan.SignedDownrange < 0 || plan.SignedDownrange > plan.CorridorLimit || plan.CrossRange > plan.CorridorLimit)
            { reason = "V2 plan does not satisfy the long-side/cross-range corridor."; return false; }
            if (plan.PlaneAlignmentDeltaVMagnitude > BurnCompleteDeltaV &&
                (!Finite(plan.PlaneAlignmentBurnUT) || plan.PlaneAlignmentBurnUT <= 0 || plan.PlaneAlignmentBurnUT >= plan.StrategicBurnUT))
            { reason = "V2 plane-alignment event is not chronologically valid."; return false; }
            reason = null;
            return true;
        }

        private AirlessLandingPhaseDecision Reject(string reason)
        {
            Phase = AirlessLandingPhaseManagerPhase.Rejected;
            LastWorkUnits = 1;
            return new AirlessLandingPhaseDecision(Phase, AirlessLandingPhaseDirective.Reject, double.NaN, reason, LastWorkUnits);
        }

        private AirlessLandingPhaseDecision Decision(AirlessLandingPhaseDirective directive, double warpUT = double.NaN)
        {
            LastWorkUnits = 1;
            return new AirlessLandingPhaseDecision(Phase, directive, warpUT, null, LastWorkUnits);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// Tracks a finite powered burn from measured thrust acceleration rather
    /// than orbital velocity. Gravity changes orbital velocity during an
    /// otherwise correctly aimed burn, so it must never be used as a burn
    /// completion meter.
    /// </summary>
    public sealed class FiniteBurnProgress
    {
        public const double FineControlStartDeltaV = 0.50;
        public const float FineControlThrottleCap = 0.02f;
        public readonly double PlannedDeltaV;
        public double DeliveredDeltaV { get; private set; }
        public double RemainingDeltaV => Math.Max(0, PlannedDeltaV - DeliveredDeltaV);
        public bool IsComplete(double tolerance = AirlessLandingPhaseManager.BurnCompleteDeltaV) =>
            RemainingDeltaV <= Math.Max(0, tolerance);

        public FiniteBurnProgress(double plannedDeltaV)
        {
            PlannedDeltaV = Math.Max(0, plannedDeltaV);
        }

        public void Integrate(double deltaTime, double actualThrustAcceleration)
        {
            if (double.IsNaN(deltaTime) || double.IsInfinity(deltaTime) ||
                double.IsNaN(actualThrustAcceleration) || double.IsInfinity(actualThrustAcceleration)) return;
            DeliveredDeltaV += Math.Max(0, deltaTime) * Math.Max(0, actualThrustAcceleration);
        }

        // V2's finite-burn authority has its own final-delta-v throttle cap.
        // It does not change the vessel's persistent throttle limiter, which
        // belongs to the player and other MechJeb controllers.
        public static float LimitThrottleForFineControl(double remainingDeltaV, float requestedThrottle)
        {
            if (double.IsNaN(remainingDeltaV) || double.IsInfinity(remainingDeltaV)) return 0;
            return remainingDeltaV <= FineControlStartDeltaV
                ? Math.Min(requestedThrottle, FineControlThrottleCap)
                : requestedThrottle;
        }
    }

    public enum AirlessLandingPhaseManagerPhase
    {
        Idle, PreparePlaneAlignmentWarp, WarpToPlaneAlignment, AlignPlaneAlignment, PlaneAlignmentBurn, AwaitStrategicReplan,
        PrepareStrategicWarp, WarpToStrategicBurn, AlignStrategicBurn, StrategicBurn, Coast, Rejected
    }

    public enum AirlessLandingPhaseDirective
    {
        None, WarpAuthorized, RequestWarp, ExitWarpAndRequestAttitude, RequestAttitude, RequireFreshStrategicValidation,
        BeginFiniteBurn, RequestFiniteBurnThrottle, FiniteBurnComplete, RequireStrategicReplan, Reject
    }

    public sealed class AirlessLandingPhaseDecision
    {
        public readonly AirlessLandingPhaseManagerPhase Phase;
        public readonly AirlessLandingPhaseDirective Directive;
        public readonly double WarpUT;
        public readonly string Reason;
        // Structural bound: a tick contains one branch and emits one directive.
        public readonly int WorkUnits;
        public AirlessLandingPhaseDecision(AirlessLandingPhaseManagerPhase phase, AirlessLandingPhaseDirective directive,
            double warpUT, string reason, int workUnits)
        { Phase = phase; Directive = directive; WarpUT = warpUT; Reason = reason; WorkUnits = workUnits; }
    }
}
