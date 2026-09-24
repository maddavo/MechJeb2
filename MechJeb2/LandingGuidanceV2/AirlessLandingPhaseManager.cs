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
        // Mirror the proven node-executor sequence: leave coarse rails ten
        // minutes before ignition, acquire and settle burn attitude at 1x,
        // then permit the final warp to the short ignition margin.
        public const double InitialWarpLeadSeconds = 600.0;
        public const double AttitudeReadyDegrees = 1.0;
        // Completion must be materially tighter than the landing corridor.
        // The thrust controller ramps down for the final portion; this is only
        // the residual which permits releasing finite-burn authority.
        public const double BurnCompleteDeltaV = 0.01;

        private AirlessLandingPlan _plan;
        private bool _planeGateValidated;
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
            _planeGateValidated = false;
            _strategicGateValidated = false;
            _requiresStrategicReplan = false;
            if (!IsExecutable(plan, out string reason))
            {
                Phase = AirlessLandingPhaseManagerPhase.Rejected;
                return Reject(reason);
            }
            // Coarse warp ends 600 seconds before the finite event. Alignment
            // then occurs at 1x before the final warp and again after it.
            Phase = plan.PlaneAlignmentDeltaVMagnitude > BurnCompleteDeltaV
                ? AirlessLandingPhaseManagerPhase.InitialWarpToPlaneAlignment
                : AirlessLandingPhaseManagerPhase.InitialWarpToStrategicBurn;
            return Decision(AirlessLandingPhaseDirective.None);
        }

        public AirlessLandingPhaseDecision Tick(double ut, bool autoWarp, double attitudeErrorDegrees,
            double remainingBurnDeltaV)
        {
            if (_plan == null) return Reject("V2 phase manager has no active airless plan.");
            switch (Phase)
            {
                case AirlessLandingPhaseManagerPhase.InitialWarpToPlaneAlignment:
                    return TickInitialWarp(ut, autoWarp, _plan.PlaneAlignmentBurnUT - PlaneBurnLeadSeconds,
                        AirlessLandingPhaseManagerPhase.PreparePlaneAlignmentWarp);
                case AirlessLandingPhaseManagerPhase.InitialWarpToStrategicBurn:
                    return TickInitialWarp(ut, autoWarp, StrategicIgnitionUT,
                        AirlessLandingPhaseManagerPhase.PrepareStrategicWarp);
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
                    if (!_planeGateValidated)
                        return Decision(AirlessLandingPhaseDirective.RequireFreshPlaneValidation);
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

        public AirlessLandingPhaseDecision AcceptPlaneAlignmentValidation(long snapshotVersion, double snapshotUT,
            bool valid, string rejectionReason = null)
        {
            if (Phase != AirlessLandingPhaseManagerPhase.AlignPlaneAlignment)
                return Reject("A plane-alignment validation was supplied outside the plane alignment phase.");
            double ignitionUT = _plan.PlaneAlignmentBurnUT - PlaneBurnLeadSeconds;
            if (snapshotUT < ignitionUT)
                return Reject("V2 plane-alignment validation arrived before the finite-burn ignition gate.");
            if (snapshotUT > _plan.PlaneAlignmentBurnUT + 0.25)
                return Reject("V2 plane-alignment validation missed the planned finite-burn midpoint.");
            if (snapshotVersion <= _plan.SnapshotVersion)
                return Reject("V2 plane-alignment validation did not use a fresh post-warp snapshot.");
            if (!valid)
                return Reject(string.IsNullOrEmpty(rejectionReason)
                    ? "V2 plane-alignment validation rejected the committed burn."
                    : rejectionReason);
            _planeGateValidated = true;
            return Decision(AirlessLandingPhaseDirective.RequestAttitude);
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
            _planeGateValidated = false;
            _strategicGateValidated = false;
            Phase = AirlessLandingPhaseManagerPhase.InitialWarpToStrategicBurn;
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
            if (next == AirlessLandingPhaseManagerPhase.AlignPlaneAlignment)
                _planeGateValidated = false;
            _strategicGateValidated = false;
            return Decision(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude);
        }

        private AirlessLandingPhaseDecision TickInitialWarp(double ut, bool autoWarp, double burnUT,
            AirlessLandingPhaseManagerPhase next)
        {
            if (!Finite(burnUT) || burnUT <= 0) return Reject("V2 plan has no finite burn epoch.");
            if (autoWarp && ut < burnUT - InitialWarpLeadSeconds)
                return Decision(AirlessLandingPhaseDirective.RequestInitialWarp, burnUT - InitialWarpLeadSeconds);
            Phase = next;
            return Decision(AirlessLandingPhaseDirective.RequestAttitude);
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
        public const double FineControlThrottleCap = 0.02;
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

    }

    /// <summary>
    /// Converts the final portion of a finite burn into a command that uses
    /// KSP's per-engine thrust limiter.  This is deliberately distinct from
    /// MechJeb's ThrottleLimit, which is only a cap on FlightCtrlState's main
    /// throttle.  The flight module saves and restores each engine's original
    /// thrustPercentage while this command is active.
    /// </summary>
    public sealed class AirlessFineThrustCommand
    {
        public readonly bool UseEngineThrustLimiter;
        public readonly double RequestedThrottle;
        public readonly double RelativeEngineThrustLimit;
        public readonly double ExpectedAcceleration;

        public AirlessFineThrustCommand(bool useEngineThrustLimiter, double requestedThrottle,
            double relativeEngineThrustLimit, double expectedAcceleration)
        {
            UseEngineThrustLimiter = useEngineThrustLimiter;
            RequestedThrottle = requestedThrottle;
            RelativeEngineThrustLimit = relativeEngineThrustLimit;
            ExpectedAcceleration = expectedAcceleration;
        }
    }

    public static class AirlessFineThrustControl
    {
        // A final burn is remapped into a band with twice the requested
        // acceleration.  With a zero-minimum-thrust engine, a 2% requested
        // final throttle therefore becomes a 4% engine limit and 50% main
        // throttle: the same thrust, with fifty times the throttle resolution.
        public const double FineControlHeadroom = 2.0;

        public static AirlessFineThrustCommand Calculate(double remainingDeltaV, double requestedThrottle,
            double minimumAcceleration, double maximumAcceleration,
            double fineControlStartDeltaV = FiniteBurnProgress.FineControlStartDeltaV,
            double fineControlThrottleCeiling = FiniteBurnProgress.FineControlThrottleCap,
            double availableMainThrottle = 1.0)
        {
            double throttle = Clamp01(requestedThrottle);
            if (!Finite(remainingDeltaV) || remainingDeltaV > Math.Max(0, fineControlStartDeltaV) ||
                throttle <= 0 || !Finite(minimumAcceleration) || !Finite(maximumAcceleration) ||
                maximumAcceleration <= minimumAcceleration)
                return new AirlessFineThrustCommand(false, throttle, 1, Math.Max(0, maximumAcceleration) * throttle);

            double limitedThrottle = Math.Min(throttle, Clamp01(fineControlThrottleCeiling));
            if (limitedThrottle <= 0)
                return new AirlessFineThrustCommand(false, 0, 1, 0);

            double mainThrottleLimit = Clamp01(availableMainThrottle);
            if (mainThrottleLimit <= 0 || limitedThrottle > mainThrottleLimit)
                return new AirlessFineThrustCommand(false, limitedThrottle, 1,
                    minimumAcceleration + (maximumAcceleration - minimumAcceleration) * limitedThrottle);
            // Retain the requested physical acceleration after MechJeb's
            // separate main-throttle cap is applied. This is the same engine
            // range calculation used by terminal descent.
            double relativeLimit = Math.Min(1, Math.Max(limitedThrottle * FineControlHeadroom,
                limitedThrottle / mainThrottleLimit));
            return RemapThrottle(limitedThrottle, minimumAcceleration, maximumAcceleration, relativeLimit);
        }

        // Terminal velocity-null and divert commands use the same physical
        // engine limiter mechanism as a finite burn, but their low-throttle
        // range is wider.  Preserve the requested acceleration; remap only
        // its available full-throttle range so the main throttle has useful
        // continuous resolution near touchdown.
        public static AirlessFineThrustCommand CalculateTerminal(double requestedThrottle,
            double minimumAcceleration, double maximumAcceleration, double terminalFineThrottleCeiling = 0.10,
            double availableMainThrottle = 1.0)
        {
            double throttle = Clamp01(requestedThrottle);
            double mainThrottleLimit = Clamp01(availableMainThrottle);
            if (throttle <= 0 || throttle > Clamp01(terminalFineThrottleCeiling) ||
                !Finite(minimumAcceleration) || !Finite(maximumAcceleration) || maximumAcceleration <= minimumAcceleration)
                return new AirlessFineThrustCommand(false, throttle, 1, Math.Max(0, maximumAcceleration) * throttle);
            // A MechJeb safety limiter is a cap on main throttle.  Select an
            // engine range that leaves the remapped main throttle at or below
            // that cap; otherwise the thrust controller would silently clip
            // a terminal command after V2 had planned for its full value.
            if (mainThrottleLimit <= 0 || throttle > mainThrottleLimit)
                return new AirlessFineThrustCommand(false, throttle, 1,
                    minimumAcceleration + (maximumAcceleration - minimumAcceleration) * throttle);
            double relativeLimit = Math.Min(1, Math.Max(throttle * FineControlHeadroom,
                throttle / mainThrottleLimit));
            return RemapThrottle(throttle, minimumAcceleration, maximumAcceleration, relativeLimit);
        }

        private static AirlessFineThrustCommand RemapThrottle(double throttle, double minimumAcceleration,
            double maximumAcceleration, double relativeLimit)
        {
            double expectedAcceleration = minimumAcceleration + (maximumAcceleration - minimumAcceleration) * throttle;
            double limitedMaximumAcceleration = minimumAcceleration +
                (maximumAcceleration - minimumAcceleration) * relativeLimit;
            double remappedThrottle = (expectedAcceleration - minimumAcceleration) /
                (limitedMaximumAcceleration - minimumAcceleration);
            return new AirlessFineThrustCommand(true, Clamp01(remappedThrottle), relativeLimit, expectedAcceleration);
        }

        private static double Clamp01(double value) => !Finite(value) ? 0 : Math.Max(0, Math.Min(1, value));
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public enum VisualTargetAction { Rebase, RetainOriginalAndReportFailure }

    /// <summary>
    /// The visual gate is allowed to rebase red to blue only when the current
    /// independent impact estimate has already reached the requested target
    /// corridor.  A late rebase must never hide a kilometre-scale targeting
    /// error by redefining it as the new requested site.
    /// </summary>
    public static class VisualRebaseGate
    {
        public static VisualTargetAction Decide(double targetError, double maximumRebaseError) =>
            Finite(targetError) && targetError >= 0 && Finite(maximumRebaseError) && maximumRebaseError >= 0 &&
            targetError <= maximumRebaseError
                ? VisualTargetAction.Rebase
                : VisualTargetAction.RetainOriginalAndReportFailure;

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// KSP-independent terminal command policy shared by the V2 flight module
    /// and the deterministic controller validation.  Given the current local
    /// state it produces a thrust direction and acceleration request which
    /// removes horizontal target error while preserving a velocity-null
    /// touchdown profile.  It deliberately owns no vessel authority.
    /// </summary>
    public sealed class AirlessTerminalGuidanceCommand
    {
        public readonly bool Valid;
        public readonly bool TouchdownReady;
        public readonly double DesiredAcceleration;
        public readonly double RequestedThrottle;
        public readonly Vector3d ThrustDirection;
        public readonly Vector3d DesiredVelocity;
        public readonly Vector3d VelocityError;
        public readonly string RejectionReason;

        public AirlessTerminalGuidanceCommand(bool valid, bool touchdownReady,
            double desiredAcceleration, double requestedThrottle, Vector3d thrustDirection,
            Vector3d desiredVelocity, Vector3d velocityError, string rejectionReason)
        {
            Valid = valid;
            TouchdownReady = touchdownReady;
            DesiredAcceleration = desiredAcceleration;
            RequestedThrottle = requestedThrottle;
            ThrustDirection = thrustDirection;
            DesiredVelocity = desiredVelocity;
            VelocityError = velocityError;
            RejectionReason = rejectionReason;
        }
    }

    public static class AirlessTerminalGuidance
    {
        public const double TouchdownHorizontalTolerance = 5.0;
        public const double TouchdownAltitude = 0.5;
        public const double TouchdownSpeed = 0.5;

        public static AirlessTerminalGuidanceCommand Calculate(Vector3d positionError,
            Vector3d surfaceVelocity, Vector3d up, double altitude, double gravity,
            double minimumAcceleration, double maximumAcceleration, double finalDescentSpeed,
            double availableMainThrottle = 1.0)
        {
            double mainThrottleLimit = Clamp01(availableMainThrottle);
            double availableMaximumAcceleration = minimumAcceleration +
                (maximumAcceleration - minimumAcceleration) * mainThrottleLimit;
            if (up.sqrMagnitude < 1e-12 || !Finite(altitude) || !Finite(gravity) ||
                !Finite(maximumAcceleration) || maximumAcceleration <= minimumAcceleration ||
                !Finite(minimumAcceleration) || availableMaximumAcceleration <= gravity)
                return Invalid("V2 terminal guidance requires thrust acceleration greater than local gravity.");

            Vector3d localUp = up.normalized;
            double height = Math.Max(0.05, altitude);
            double descentSpeed = Math.Max(0, -Vector3d.Dot(surfaceVelocity, localUp));
            double requestedTouchdownSpeed = Math.Max(0.1, finalDescentSpeed);
            double maximumNetBrakingAcceleration = availableMaximumAcceleration - gravity;
            double minimumBrakingDistance = descentSpeed <= requestedTouchdownSpeed
                ? 0
                : (descentSpeed * descentSpeed - requestedTouchdownSpeed * requestedTouchdownSpeed) /
                    (2.0 * maximumNetBrakingAcceleration);
            // A throttle cap can leave enough thrust to hover yet still make
            // the current descent physically unrecoverable.  Reject before
            // sending a burn command in that case; engine limiting cannot add
            // braking authority beyond the selected main-throttle cap.
            // Inside the final contact-height tolerance the next physics tick
            // can register touchdown before a continuous-speed calculation
            // reaches its mathematical target. Keep maximum braking authority
            // there; outside it, an insufficient stopping distance is a real
            // unrecoverable condition.
            if (height > TouchdownAltitude && height + 1e-6 < minimumBrakingDistance)
                return Invalid("V2 terminal guidance cannot stop the current descent within the available altitude under the active main-throttle limit.");
            Vector3d horizontalError = Vector3d.Exclude(localUp, positionError);
            double verticalSpeedDown = -Vector3d.Dot(surfaceVelocity, localUp);
            double timeToGround = Math.Max(3.0, height / Math.Max(0.5, verticalSpeedDown));
            Vector3d desiredHorizontalVelocity = horizontalError.sqrMagnitude < 1.0
                ? Vector3d.zero
                // Terminal divert must be able to retire a material residual
                // before touchdown. The former 12 m/s cap left a 600 m
                // target miss essentially untouched during a low-Mun descent.
                // This remains bounded and is scaled from the current time to
                // ground rather than a body-specific correction impulse.
                : horizontalError.normalized * Math.Min(30.0, horizontalError.magnitude / timeToGround);
            Vector3d desiredVelocity = desiredHorizontalVelocity - requestedTouchdownSpeed * localUp;
            Vector3d velocityError = surfaceVelocity - desiredVelocity;

            // The commanded acceleration is sized from the velocity that must
            // be removed before the remaining altitude is exhausted. Gravity
            // is included explicitly so the vector represents engine thrust,
            // not just a steering correction.
            double closingSpeed = Math.Max(0, Vector3d.Dot(velocityError, -localUp));
            double verticalDemand = gravity + closingSpeed * closingSpeed / (2.0 * height);
            // The desired horizontal velocity is derived from remaining range
            // and time to terrain.  Damp the measured error on a one-second
            // response while reserving enough engine authority for vertical
            // braking.
            Vector3d horizontalVelocityError = Vector3d.Exclude(localUp, velocityError);
            Vector3d horizontalCorrection = -horizontalVelocityError;
            double horizontalDemand = Math.Min(Math.Max(0, availableMaximumAcceleration - verticalDemand), horizontalCorrection.magnitude);
            Vector3d thrustVector = horizontalDemand <= 1e-12
                ? localUp * verticalDemand
                : horizontalCorrection.normalized * horizontalDemand + localUp * verticalDemand;
            if (thrustVector.sqrMagnitude < 1e-12) thrustVector = localUp;
            // This is an acceleration request, not a promise that every
            // engine can continuously deliver below its physical minimum
            // thrust.  The flight layer accounts for that engine constraint;
            // V2 never uses pulse-width modulation to hide it.
            double desiredAcceleration = Math.Min(availableMaximumAcceleration, Math.Max(0, thrustVector.magnitude));
            if (minimumAcceleration > desiredAcceleration + 1e-6)
                return Invalid("V2 terminal guidance cannot meet the requested descent profile because minimum continuous thrust exceeds the required acceleration.");
            // Convert requested acceleration through the engine's actual
            // continuous throttle range.  Minimum thrust is a floor in KSP;
            // dividing by maximum acceleration alone would command the wrong
            // physical acceleration whenever that floor is nonzero.
            double throttle = maximumAcceleration > minimumAcceleration
                ? Math.Max(0, Math.Min(1, (desiredAcceleration - minimumAcceleration) /
                    (maximumAcceleration - minimumAcceleration)))
                : desiredAcceleration >= maximumAcceleration ? 1 : 0;
            bool touchdownReady = altitude <= TouchdownAltitude && horizontalError.magnitude <= TouchdownHorizontalTolerance &&
                surfaceVelocity.magnitude <= TouchdownSpeed;
            return new AirlessTerminalGuidanceCommand(true, touchdownReady, desiredAcceleration, throttle,
                thrustVector.normalized, desiredVelocity, velocityError, null);
        }

        private static AirlessTerminalGuidanceCommand Invalid(string reason) =>
            new AirlessTerminalGuidanceCommand(false, false, 0, 0, Vector3d.zero, Vector3d.zero, Vector3d.zero, reason);
        private static double Clamp01(double value) => !Finite(value) ? 0 : Math.Max(0, Math.Min(1, value));
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public enum AirlessCoastSafetyAction { Continue, Replan, EmergencyBrake }

    /// <summary>
    /// A finite airless deorbit burn commits the vessel to a surface-intercept
    /// trajectory.  A later endpoint disagreement must never restart the
    /// strategic sequence, because that sequence is permitted to request warp.
    /// Keep the vessel at 1x in controlled coast until the terminal burn gate.
    /// </summary>
    public enum CommittedAirlessDescentRecoveryAction { StrategicReplan, ControlledCoast }

    public static class CommittedAirlessDescentRecoveryGate
    {
        public static CommittedAirlessDescentRecoveryAction Decide(bool descentCommitted) =>
            descentCommitted ? CommittedAirlessDescentRecoveryAction.ControlledCoast :
                CommittedAirlessDescentRecoveryAction.StrategicReplan;

        public static bool RequiresTerminalAlignmentReset(bool descentCommitted) => !descentCommitted;
    }

    /// <summary>
    /// Hoverslam solutions are produced asynchronously.  Before rails warp,
    /// V2 needs one inertial burn attitude that SAS can settle on; following
    /// each newly completed simulation makes the vessel chase a moving target.
    /// The terminal solution is rebuilt after warp has ended, before throttle
    /// is allowed.
    /// </summary>
    public sealed class AirlessTerminalAttitudeLatch
    {
        public bool IsLatched { get; private set; }
        public Vector3d Attitude { get; private set; } = Vector3d.zero;

        public bool TryLatch(Vector3d candidate)
        {
            if (IsLatched) return true;
            double magnitude = candidate.magnitude;
            if (!Finite(candidate.x) || !Finite(candidate.y) || !Finite(candidate.z) || magnitude <= 1e-9)
                return false;
            Attitude = candidate / magnitude;
            IsLatched = true;
            return true;
        }

        public void Reset()
        {
            IsLatched = false;
            Attitude = Vector3d.zero;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// A plan authorizes commands only while the live vessel still has the
    /// minimum powered authority needed to arrest an airless descent.  It is
    /// intentionally independent of the planner: a plan from an earlier
    /// snapshot cannot keep requesting warp after an engine, propellant, or
    /// vessel-loss event has removed that authority.
    /// </summary>
    public enum AirlessAuthorityAction { Continue, Abort }

    /// <summary>
    /// Shared policy for a planned atmospheric entry burn.  The atmospheric
    /// controller owns the candidate simulation, but it must obey the same
    /// safety rule as an airless burn: rails warp is not allowed until the
    /// measured burn attitude is already settled.  This deliberately has no
    /// Unity or vessel dependency so the command gate is regression-testable.
    /// </summary>
    public static class AtmosphericBurnWarpGate
    {
        public static bool CanRequestWarp(double currentUT, double burnUT, double leadSeconds,
            bool autoWarpEnabled, bool attitudeAligned)
        {
            return autoWarpEnabled && attitudeAligned && Finite(currentUT) && Finite(burnUT) &&
                Finite(leadSeconds) && leadSeconds >= 0 && currentUT < burnUT - leadSeconds;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public static class AirlessAuthorityGate
    {
        public static AirlessAuthorityAction Decide(double maximumAcceleration, double localGravity, out string reason)
        {
            reason = null;
            if (!Finite(localGravity) || localGravity < 0 || !Finite(maximumAcceleration) ||
                maximumAcceleration <= localGravity)
            {
                reason = "V2 lost thrust authority: available acceleration no longer exceeds local gravity.";
                return AirlessAuthorityAction.Abort;
            }
            return AirlessAuthorityAction.Continue;
        }

        public static AirlessAuthorityAction Decide(LandingGuidanceV2Snapshot snapshot, double localGravity,
            out string reason)
        {
            reason = null;
            if (snapshot == null)
            {
                reason = "V2 lost the live airless vessel snapshot.";
                return AirlessAuthorityAction.Abort;
            }
            if (Decide(snapshot.MaximumAcceleration, localGravity, out reason) == AirlessAuthorityAction.Abort)
                return AirlessAuthorityAction.Abort;
            if (!Finite(snapshot.AvailableDeltaV) || snapshot.AvailableDeltaV <= 0)
            {
                reason = "V2 lost propulsion authority: no usable delta-V remains.";
                return AirlessAuthorityAction.Abort;
            }
            return AirlessAuthorityAction.Continue;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// Keeps a committed airless descent from silently coasting on an invalid
    /// or out-of-corridor endpoint. The emergency lead is derived from the
    /// current vehicle acceleration and the plan's terminal braking lower
    /// bound, rather than a body-specific time constant.
    /// </summary>
    public static class AirlessCoastSafetyGate
    {
        public static AirlessCoastSafetyAction Decide(LandingGuidanceV2Snapshot snapshot,
            LandingGuidanceV2Estimate estimate, double corridorLimit, double terminalBrakingDeltaV,
            bool terminalIgnitionAvailable)
        {
            if (snapshot == null || estimate == null || !estimate.HasImpact ||
                double.IsNaN(corridorLimit) || double.IsInfinity(corridorLimit) ||
                double.IsNaN(estimate.TargetError) || double.IsInfinity(estimate.TargetError))
                return AirlessCoastSafetyAction.Replan;

            double acceleration = snapshot.MaximumAcceleration;
            double timeToImpact = estimate.ImpactUT - snapshot.UT;
            double emergencyLead = acceleration > 0 && terminalBrakingDeltaV > 0
                ? terminalBrakingDeltaV / acceleration + 2.0
                : double.PositiveInfinity;
            if (!terminalIgnitionAvailable && timeToImpact <= emergencyLead)
                return AirlessCoastSafetyAction.EmergencyBrake;
            return estimate.TargetError <= corridorLimit
                ? AirlessCoastSafetyAction.Continue
                : timeToImpact <= emergencyLead
                    ? AirlessCoastSafetyAction.EmergencyBrake
                    : AirlessCoastSafetyAction.Replan;
        }
    }

    /// <summary>
    /// Authorizes a terminal coast warp only after terminal-braking attitude
    /// has remained inside the authority gate at 1x for a sustained interval.
    /// A single stale attitude sample must never start rails warp.
    /// </summary>
    public sealed class AirlessTerminalWarpGate
    {
        public const double AttitudeHoldSeconds = 2.0;
        private double _readySinceUT = double.NaN;

        public bool ObserveAttitude(double ut, double attitudeErrorDegrees)
        {
            if (double.IsNaN(ut) || double.IsInfinity(ut) ||
                double.IsNaN(attitudeErrorDegrees) || double.IsInfinity(attitudeErrorDegrees) ||
                attitudeErrorDegrees > AirlessLandingPhaseManager.AttitudeReadyDegrees)
            {
                _readySinceUT = double.NaN;
                return false;
            }
            if (double.IsNaN(_readySinceUT)) _readySinceUT = ut;
            return ut - _readySinceUT >= AttitudeHoldSeconds;
        }

        public void Reset() => _readySinceUT = double.NaN;

        public static bool EndpointIsCurrentAndWithinCorridor(LandingGuidanceV2Estimate estimate, double corridorLimit) =>
            estimate != null && estimate.HasImpact && !double.IsNaN(estimate.TargetError) &&
            !double.IsInfinity(estimate.TargetError) && !double.IsNaN(corridorLimit) &&
            !double.IsInfinity(corridorLimit) && estimate.TargetError <= corridorLimit;
    }

    /// <summary>
    /// A finite burn requires agreement between the attitude controller and
    /// the vessel's physical thrust-vector observation. Neither signal alone
    /// is sufficient authority for warp or throttle.
    /// </summary>
    public static class AirlessBurnAlignmentGate
    {
        public const double SettledAngularVelocityRadiansPerSecond = 0.001;

        public static bool IsReady(double controllerErrorDegrees, double thrustVectorErrorDegrees)
        {
            return Finite(controllerErrorDegrees) && Finite(thrustVectorErrorDegrees) &&
                controllerErrorDegrees <= AirlessLandingPhaseManager.AttitudeReadyDegrees &&
                thrustVectorErrorDegrees <= AirlessLandingPhaseManager.AttitudeReadyDegrees;
        }

        public static bool IsReady(double controllerErrorDegrees, double thrustVectorErrorDegrees,
            double angularVelocityRadiansPerSecond) =>
            IsReady(controllerErrorDegrees, thrustVectorErrorDegrees) &&
            Finite(angularVelocityRadiansPerSecond) &&
            angularVelocityRadiansPerSecond <= SettledAngularVelocityRadiansPerSecond;

        public static double CombinedError(double controllerErrorDegrees, double thrustVectorErrorDegrees)
        {
            if (!Finite(controllerErrorDegrees) || !Finite(thrustVectorErrorDegrees)) return double.PositiveInfinity;
            return Math.Max(controllerErrorDegrees, thrustVectorErrorDegrees);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// Deterministic, KSP-free execution harness for the airless strategic
    /// sequence. It drives the same phase manager used in flight and records
    /// every authority directive, including staged warp, fresh validation, and
    /// measured finite-burn completion.
    /// </summary>
    public sealed class AirlessLandingControllerHarness
    {
        public AirlessLandingControllerHarnessResult Execute(AirlessLandingPlan plan, double startUT,
            double finiteBurnLeadSeconds, double maximumAcceleration, double physicsStepSeconds = 0.02,
            bool freshValidationIsValid = true, double controllerErrorDegrees = 0,
            double thrustVectorErrorDegrees = 0, double angularVelocityRadiansPerSecond = 0,
            AirlessLandingPlan strategicReplan = null, bool planeValidationIsValid = true, bool autoWarp = true)
        {
            var result = new AirlessLandingControllerHarnessResult();
            var manager = new AirlessLandingPhaseManager();
            result.Add(manager.Start(plan, finiteBurnLeadSeconds));
            if (result.LastDirective == AirlessLandingPhaseDirective.Reject) return result;

            double alignmentError = AirlessBurnAlignmentGate.IsReady(controllerErrorDegrees,
                thrustVectorErrorDegrees, angularVelocityRadiansPerSecond)
                ? AirlessBurnAlignmentGate.CombinedError(controllerErrorDegrees, thrustVectorErrorDegrees)
                : double.PositiveInfinity;
            double step = Math.Max(0.001, physicsStepSeconds);
            double acceleration = Math.Max(0.001, maximumAcceleration);
            double ut = startUT;

            if (plan.PlaneAlignmentDeltaVMagnitude > AirlessLandingPhaseManager.BurnCompleteDeltaV)
            {
                if (!AdvanceToBurnGate(manager, result, ref ut, alignmentError, step, autoWarp)) return result;
                ut = plan.PlaneAlignmentBurnUT - finiteBurnLeadSeconds;
                result.Add(manager.Tick(ut, false, alignmentError, double.NaN));
                if (result.LastDirective != AirlessLandingPhaseDirective.RequireFreshPlaneValidation) return result;
                result.FreshPlaneValidationRequired = true;
                result.Add(manager.AcceptPlaneAlignmentValidation(plan.SnapshotVersion + 1, ut, planeValidationIsValid,
                    planeValidationIsValid ? null : "Harness deliberately rejected the fresh plane-alignment snapshot."));
                if (result.LastDirective == AirlessLandingPhaseDirective.Reject) return result;
                result.Add(manager.Tick(ut, false, alignmentError, plan.PlaneAlignmentDeltaVMagnitude));
                if (result.LastDirective != AirlessLandingPhaseDirective.BeginFiniteBurn) return result;
                result.PlaneBurnStarted = true;
                result.PlaneDeliveredDeltaV = ExecuteFiniteBurn(manager, result, ref ut, plan.PlaneAlignmentDeltaVMagnitude,
                    acceleration, step, alignmentError);
                if (!result.PlaneBurnCompleted) return result;
                result.Add(manager.Tick(ut, false, alignmentError, 0));
                if (result.LastDirective != AirlessLandingPhaseDirective.RequireStrategicReplan || strategicReplan == null)
                    return result;
                result.Add(manager.AdoptStrategicReplan(strategicReplan));
                if (result.LastDirective == AirlessLandingPhaseDirective.Reject) return result;
                plan = strategicReplan;
            }

            if (!AdvanceToBurnGate(manager, result, ref ut, alignmentError, step, autoWarp)) return result;
            double ignitionUT = plan.StrategicBurnUT - finiteBurnLeadSeconds;
            ut = ignitionUT;
            result.Add(manager.Tick(ut, false, alignmentError, double.NaN));
            if (result.LastDirective != AirlessLandingPhaseDirective.RequireFreshStrategicValidation) return result;
            result.FreshValidationRequired = true;
            result.Add(manager.AcceptStrategicValidation(plan.SnapshotVersion + 1, ut, freshValidationIsValid,
                freshValidationIsValid ? null : "Harness deliberately rejected the fresh ignition snapshot."));
            if (result.LastDirective == AirlessLandingPhaseDirective.Reject) return result;
            result.Add(manager.Tick(ut, false, alignmentError, plan.StrategicDeorbitDeltaVMagnitude));
            if (result.LastDirective != AirlessLandingPhaseDirective.BeginFiniteBurn) return result;
            result.FiniteBurnStarted = true;
            result.DeliveredDeltaV = ExecuteFiniteBurn(manager, result, ref ut, plan.StrategicDeorbitDeltaVMagnitude,
                acceleration, step, alignmentError);
            result.FinalPhase = manager.Phase;
            return result;
        }

        private static bool AdvanceToBurnGate(AirlessLandingPhaseManager manager, AirlessLandingControllerHarnessResult result,
            ref double ut, double alignmentError, double step, bool autoWarp)
        {
            // The flight module gives the phase manager infinity until both the
            // controller and measured thrust vector are inside the one-degree,
            // settled authority gate. The supplied alignmentError models that
            // same contract; the harness cannot authorize an attitude-only warp.
            result.Add(manager.Tick(ut, autoWarp, alignmentError, double.NaN));
            if (result.LastDirective == AirlessLandingPhaseDirective.RequestInitialWarp)
            {
                result.InitialWarpRequested = true;
                ut = result.LastWarpUT;
                result.Add(manager.Tick(ut, autoWarp, alignmentError, double.NaN));
            }
            if (result.LastDirective == AirlessLandingPhaseDirective.RequestAttitude)
            {
                ut += step;
                result.Add(manager.Tick(ut, autoWarp, alignmentError, double.NaN));
            }
            if (result.LastDirective != AirlessLandingPhaseDirective.WarpAuthorized) return false;

            result.Add(manager.Tick(ut, autoWarp, alignmentError, double.NaN));
            if (result.LastDirective == AirlessLandingPhaseDirective.RequestWarp)
            {
                result.FinalWarpRequested = true;
                ut = result.LastWarpUT;
                result.Add(manager.Tick(ut, autoWarp, alignmentError, double.NaN));
            }
            return result.LastDirective == AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude;
        }

        private static double ExecuteFiniteBurn(AirlessLandingPhaseManager manager, AirlessLandingControllerHarnessResult result,
            ref double ut, double plannedDeltaV, double acceleration, double step, double alignmentError)
        {
            var progress = new FiniteBurnProgress(plannedDeltaV);
            while (!progress.IsComplete() && result.WorkUnits < 100000)
            {
                // Mirror CommandFiniteBurnThrottle: MechJeb's 0.5 s
                // deceleration horizon is followed by V2's final engine-limit
                // mapping. A full-throttle harness here used to hide a final
                // one-frame overshoot that the real controller avoids.
                double requestedThrottle = Math.Max(0.01, Math.Min(1.0,
                    progress.RemainingDeltaV / (0.5 * acceleration)));
                AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(progress.RemainingDeltaV,
                    requestedThrottle, 0, acceleration);
                progress.Integrate(step, command.ExpectedAcceleration);
                ut += step;
                result.Add(manager.Tick(ut, false, alignmentError, progress.RemainingDeltaV));
                if (result.LastDirective == AirlessLandingPhaseDirective.FiniteBurnComplete)
                {
                    if (manager.Phase == AirlessLandingPhaseManagerPhase.AwaitStrategicReplan)
                        result.PlaneBurnCompleted = true;
                    else
                        result.FiniteBurnCompleted = true;
                    break;
                }
                if (result.LastDirective != AirlessLandingPhaseDirective.RequestFiniteBurnThrottle) break;
            }
            return progress.DeliveredDeltaV;
        }
    }

    public sealed class AirlessLandingControllerHarnessResult
    {
        public readonly System.Collections.Generic.List<AirlessLandingPhaseDirective> Directives =
            new System.Collections.Generic.List<AirlessLandingPhaseDirective>();
        public bool InitialWarpRequested;
        public bool FinalWarpRequested;
        public bool FreshPlaneValidationRequired;
        public bool FreshValidationRequired;
        public bool PlaneBurnStarted;
        public bool PlaneBurnCompleted;
        public bool FiniteBurnStarted;
        public bool FiniteBurnCompleted;
        public double PlaneDeliveredDeltaV;
        public double DeliveredDeltaV;
        public double LastWarpUT = double.NaN;
        public int WorkUnits;
        public AirlessLandingPhaseManagerPhase FinalPhase;
        public string LastReason;
        public AirlessLandingPhaseDirective LastDirective => Directives.Count == 0
            ? AirlessLandingPhaseDirective.None : Directives[Directives.Count - 1];

        public void Add(AirlessLandingPhaseDecision decision)
        {
            Directives.Add(decision.Directive);
            LastWarpUT = decision.WarpUT;
            LastReason = decision.Reason;
            WorkUnits += decision.WorkUnits;
            FinalPhase = decision.Phase;
        }
    }

    public enum AirlessLandingPhaseManagerPhase
    {
        Idle, InitialWarpToPlaneAlignment, PreparePlaneAlignmentWarp, WarpToPlaneAlignment, AlignPlaneAlignment, PlaneAlignmentBurn, AwaitStrategicReplan,
        InitialWarpToStrategicBurn, PrepareStrategicWarp, WarpToStrategicBurn, AlignStrategicBurn, StrategicBurn, Coast, Rejected
    }

    public enum AirlessLandingPhaseDirective
    {
        None, RequestInitialWarp, WarpAuthorized, RequestWarp, ExitWarpAndRequestAttitude, RequestAttitude, RequireFreshPlaneValidation,
        RequireFreshStrategicValidation, BeginFiniteBurn, RequestFiniteBurnThrottle, FiniteBurnComplete, RequireStrategicReplan, Reject
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
