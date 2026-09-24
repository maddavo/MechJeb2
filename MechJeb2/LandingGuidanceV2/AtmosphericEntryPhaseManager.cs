using System;

namespace MuMech
{
    /// <summary>
    /// Owns authority transitions for V2's finite atmospheric-entry burn. It
    /// deliberately contains no vessel or Unity access, so the live controller
    /// and deterministic harness execute the same warp, attitude, validation,
    /// and finite-burn sequence.
    /// </summary>
    public sealed class AtmosphericEntryPhaseManager
    {
        public const double AttitudeReadyDegrees = AirlessLandingPhaseManager.AttitudeReadyDegrees;
        public const double InitialWarpLeadSeconds = AirlessLandingPhaseManager.InitialWarpLeadSeconds;
        public const double WarpSettleMargin = AirlessLandingPhaseManager.WarpSettleMargin;
        public const double BurnCompleteDeltaV = AirlessLandingPhaseManager.BurnCompleteDeltaV;

        private AtmosphericLandingPlan _plan;
        private readonly double _finiteBurnLeadSeconds;
        private bool _burnGateValidated;

        public AtmosphericEntryPhase Phase { get; private set; } = AtmosphericEntryPhase.Idle;
        public int LastWorkUnits { get; private set; }
        public AtmosphericLandingPlan Plan => _plan;
        public double IgnitionUT => _plan == null ? double.NaN : _plan.StrategicEntryBurnUT - _finiteBurnLeadSeconds;

        public AtmosphericEntryPhaseManager(double finiteBurnLeadSeconds)
        {
            _finiteBurnLeadSeconds = Finite(finiteBurnLeadSeconds) && finiteBurnLeadSeconds >= 0
                ? finiteBurnLeadSeconds : 0;
        }

        public AtmosphericEntryPhaseDecision Start(AtmosphericLandingPlan plan)
        {
            _plan = plan;
            _burnGateValidated = false;
            if (!Executable(plan, out string reason)) return Reject(reason);
            if (plan.StrategicEntryDeltaV.magnitude <= BurnCompleteDeltaV)
            {
                Phase = AtmosphericEntryPhase.Entry;
                return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
            }
            Phase = AtmosphericEntryPhase.InitialWarpToBurn;
            return Decision(AtmosphericEntryDirective.None);
        }

        public AtmosphericEntryPhaseDecision Tick(double ut, bool autoWarp, double attitudeErrorDegrees,
            double remainingBurnDeltaV)
        {
            if (_plan == null) return Reject("V2 atmospheric phase manager has no active entry plan.");
            switch (Phase)
            {
                case AtmosphericEntryPhase.InitialWarpToBurn:
                    if (autoWarp && ut < IgnitionUT - InitialWarpLeadSeconds)
                        return Decision(AtmosphericEntryDirective.RequestInitialWarp, IgnitionUT - InitialWarpLeadSeconds);
                    Phase = AtmosphericEntryPhase.PrepareWarp;
                    return Decision(AtmosphericEntryDirective.RequestAttitude);
                case AtmosphericEntryPhase.PrepareWarp:
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AtmosphericEntryDirective.RequestAttitude);
                    Phase = AtmosphericEntryPhase.WarpToBurn;
                    return Decision(AtmosphericEntryDirective.WarpAuthorized);
                case AtmosphericEntryPhase.WarpToBurn:
                    if (autoWarp && ut < IgnitionUT - WarpSettleMargin)
                        return Decision(AtmosphericEntryDirective.RequestWarp, IgnitionUT - WarpSettleMargin);
                    Phase = AtmosphericEntryPhase.AlignBurn;
                    _burnGateValidated = false;
                    return Decision(AtmosphericEntryDirective.ExitWarpAndRequestAttitude);
                case AtmosphericEntryPhase.AlignBurn:
                    if (ut < IgnitionUT) return Decision(AtmosphericEntryDirective.RequestAttitude);
                    if (!_burnGateValidated) return Decision(AtmosphericEntryDirective.RequireFreshBurnValidation);
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AtmosphericEntryDirective.RequestAttitude);
                    Phase = AtmosphericEntryPhase.Burn;
                    return Decision(AtmosphericEntryDirective.BeginFiniteBurn);
                case AtmosphericEntryPhase.Burn:
                    if (remainingBurnDeltaV > BurnCompleteDeltaV)
                    {
                        if (attitudeErrorDegrees > AttitudeReadyDegrees)
                            return Decision(AtmosphericEntryDirective.RequestAttitude);
                        return Decision(AtmosphericEntryDirective.RequestFiniteBurnThrottle);
                    }
                    Phase = AtmosphericEntryPhase.AwaitPostBurnValidation;
                    return Decision(AtmosphericEntryDirective.FiniteBurnComplete);
                case AtmosphericEntryPhase.AwaitPostBurnValidation:
                    return Decision(AtmosphericEntryDirective.RequirePostBurnValidation);
                case AtmosphericEntryPhase.Entry:
                    return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
                case AtmosphericEntryPhase.Rejected:
                    return Decision(AtmosphericEntryDirective.None);
                default:
                    return Reject("V2 atmospheric phase manager entered an unknown phase.");
            }
        }

        public AtmosphericEntryPhaseDecision AcceptFreshBurnValidation(long snapshotVersion, double snapshotUT,
            bool valid, string rejectionReason = null)
        {
            if (Phase != AtmosphericEntryPhase.AlignBurn)
                return Reject("A V2 atmospheric ignition validation was supplied outside the entry-burn alignment phase.");
            if (snapshotUT < IgnitionUT)
                return Reject("V2 atmospheric ignition validation arrived before the finite-burn ignition gate.");
            if (snapshotUT > _plan.StrategicEntryBurnUT + 0.25)
                return Reject("V2 atmospheric ignition validation missed the planned finite-burn midpoint.");
            if (snapshotVersion <= _plan.SnapshotVersion)
                return Reject("V2 atmospheric ignition validation did not use a fresh post-warp snapshot.");
            if (!valid)
                return Reject(string.IsNullOrEmpty(rejectionReason)
                    ? "V2 atmospheric ignition validation rejected the committed entry burn."
                    : rejectionReason);
            _burnGateValidated = true;
            return Decision(AtmosphericEntryDirective.RequestAttitude);
        }

        public AtmosphericEntryPhaseDecision AcceptFreshBurnValidation(AtmosphericLandingPlan plan, double snapshotUT)
        {
            if (plan == null) return Reject("V2 atmospheric ignition validation did not provide a fresh entry plan.");
            AtmosphericEntryPhaseDecision decision = AcceptFreshBurnValidation(plan.SnapshotVersion, snapshotUT,
                plan.State == AtmosphericLandingPlanState.Candidate, plan.Reason);
            if (decision.Directive == AtmosphericEntryDirective.Reject) return decision;
            _plan = plan;
            // A fresh simulation may show that the actual current orbit has
            // already crossed the atmospheric interface. In that case a
            // second strategic burn is neither required nor safe to force;
            // enter the separately validated atmospheric profile directly.
            if (plan.StrategicEntryDeltaV.magnitude <= BurnCompleteDeltaV)
            {
                Phase = AtmosphericEntryPhase.Entry;
                return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
            }
            return decision;
        }

        public AtmosphericEntryPhaseDecision AcceptPostBurnValidation(AtmosphericLandingPlan plan)
        {
            if (Phase != AtmosphericEntryPhase.AwaitPostBurnValidation)
                return Reject("A V2 atmospheric post-burn plan was supplied outside its validation gate.");
            // The finite entry burn has already been delivered. A later
            // estimator failure must not release the vessel onto an entry
            // trajectory: retain V2 authority and continue the conservative
            // retrograde/parachute or powered-braking profile. Pre-burn
            // validation still rejects before any V2 thrust is commanded.
            if (!Executable(plan, out string reason)) return EnterConservativeEntry(
                "V2 post-burn validation could not certify a replacement entry plan; continuing the conservative entry profile: " + reason);
            if (plan.SnapshotVersion <= _plan.SnapshotVersion)
                return EnterConservativeEntry(
                    "V2 post-burn validation did not provide a fresh snapshot; continuing the conservative entry profile.");
            _plan = plan;
            _burnGateValidated = false;
            if (plan.StrategicEntryDeltaV.magnitude <= BurnCompleteDeltaV)
            {
                Phase = AtmosphericEntryPhase.Entry;
                return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
            }
            Phase = AtmosphericEntryPhase.InitialWarpToBurn;
            return Decision(AtmosphericEntryDirective.None);
        }

        private AtmosphericEntryPhaseDecision EnterConservativeEntry(string reason)
        {
            Phase = AtmosphericEntryPhase.Entry;
            LastWorkUnits = 1;
            return new AtmosphericEntryPhaseDecision(Phase, AtmosphericEntryDirective.EnterAtmosphericEntry,
                double.NaN, reason, LastWorkUnits);
        }

        private static bool Executable(AtmosphericLandingPlan plan, out string reason)
        {
            if (plan == null || plan.State != AtmosphericLandingPlanState.Candidate)
            {
                reason = "V2 requires a candidate atmospheric entry plan.";
                return false;
            }
            if (!Finite(plan.LandingMargin) || plan.LandingMargin < 0)
            {
                reason = "V2 atmospheric entry plan does not preserve its powered terminal reserve.";
                return false;
            }
            if (plan.StrategicEntryDeltaV.magnitude > BurnCompleteDeltaV &&
                (!Finite(plan.StrategicEntryBurnUT) || plan.StrategicEntryBurnUT <= 0))
            {
                reason = "V2 atmospheric entry plan has no finite strategic-burn epoch.";
                return false;
            }
            reason = null;
            return true;
        }

        private AtmosphericEntryPhaseDecision Reject(string reason)
        {
            Phase = AtmosphericEntryPhase.Rejected;
            LastWorkUnits = 1;
            return new AtmosphericEntryPhaseDecision(Phase, AtmosphericEntryDirective.Reject, double.NaN, reason, LastWorkUnits);
        }

        private AtmosphericEntryPhaseDecision Decision(AtmosphericEntryDirective directive, double warpUT = double.NaN)
        {
            LastWorkUnits = 1;
            return new AtmosphericEntryPhaseDecision(Phase, directive, warpUT, null, LastWorkUnits);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    /// <summary>
    /// KSP-free execution of the exact atmospheric burn authority machine.
    /// It is deliberately command-oriented: every warp, attitude gate, fresh
    /// validation and finite throttle command is recorded for regression tests.
    /// </summary>
    public sealed class AtmosphericEntryControllerHarness
    {
        public AtmosphericEntryControllerHarnessResult Execute(AtmosphericLandingPlan plan, double startUT,
            double finiteBurnLeadSeconds, double maximumAcceleration, AtmosphericLandingPlan postBurnPlan,
            double physicsStepSeconds = 0.02, bool freshValidationIsValid = true,
            double attitudeErrorDegrees = 0, bool autoWarp = true)
        {
            var result = new AtmosphericEntryControllerHarnessResult();
            var manager = new AtmosphericEntryPhaseManager(finiteBurnLeadSeconds);
            result.Add(manager.Start(plan));
            if (result.LastDirective == AtmosphericEntryDirective.Reject) return result;
            if (manager.Phase == AtmosphericEntryPhase.Entry)
            {
                result.FinalPhase = manager.Phase;
                return result;
            }

            double ut = startUT;
            double step = Math.Max(0.001, physicsStepSeconds);
            double acceleration = Math.Max(0.001, maximumAcceleration);
            result.Add(manager.Tick(ut, autoWarp, attitudeErrorDegrees, double.NaN));
            if (result.LastDirective == AtmosphericEntryDirective.RequestInitialWarp)
            {
                result.InitialWarpRequested = true;
                ut = result.LastWarpUT;
                result.Add(manager.Tick(ut, autoWarp, attitudeErrorDegrees, double.NaN));
            }
            if (result.LastDirective == AtmosphericEntryDirective.RequestAttitude)
            {
                ut += step;
                result.Add(manager.Tick(ut, autoWarp, attitudeErrorDegrees, double.NaN));
            }
            if (result.LastDirective != AtmosphericEntryDirective.WarpAuthorized) return result;
            result.Add(manager.Tick(ut, autoWarp, attitudeErrorDegrees, double.NaN));
            if (result.LastDirective == AtmosphericEntryDirective.RequestWarp)
            {
                result.FinalWarpRequested = true;
                ut = result.LastWarpUT;
                result.Add(manager.Tick(ut, autoWarp, attitudeErrorDegrees, double.NaN));
            }
            if (result.LastDirective != AtmosphericEntryDirective.ExitWarpAndRequestAttitude) return result;

            ut = manager.IgnitionUT;
            result.Add(manager.Tick(ut, false, attitudeErrorDegrees, double.NaN));
            if (result.LastDirective != AtmosphericEntryDirective.RequireFreshBurnValidation) return result;
            result.FreshBurnValidationRequired = true;
            AtmosphericLandingPlan freshPlan = new AtmosphericLandingPlan(plan.SnapshotVersion + 1,
                freshValidationIsValid ? AtmosphericLandingPlanState.Candidate : AtmosphericLandingPlanState.Rejected,
                plan.PredictedTargetError, plan.EntryCorridorRadius, plan.EndpointUncertainty, plan.TerminalReserve,
                plan.LandingMargin, freshValidationIsValid ? plan.Reason : "Harness deliberately rejected the fresh ignition plan.",
                plan.StrategicEntryDeltaV, plan.StrategicEntryBurnUT, plan.EntryUT, plan.EntryTargetError);
            result.Add(manager.AcceptFreshBurnValidation(freshPlan, ut));
            if (result.LastDirective == AtmosphericEntryDirective.Reject) return result;
            result.Add(manager.Tick(ut, false, attitudeErrorDegrees, plan.StrategicEntryDeltaV.magnitude));
            if (result.LastDirective != AtmosphericEntryDirective.BeginFiniteBurn) return result;
            result.FiniteBurnStarted = true;

            var progress = new FiniteBurnProgress(plan.StrategicEntryDeltaV.magnitude);
            while (!progress.IsComplete() && result.WorkUnits < 100000)
            {
                double requestedThrottle = Math.Max(0.01, Math.Min(1.0,
                    progress.RemainingDeltaV / (0.5 * acceleration)));
                AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(progress.RemainingDeltaV,
                    requestedThrottle, 0, acceleration);
                result.NonzeroThrottleCommanded |= command.RequestedThrottle > 0 && command.ExpectedAcceleration > 0;
                progress.Integrate(step, command.ExpectedAcceleration);
                ut += step;
                result.Add(manager.Tick(ut, false, attitudeErrorDegrees, progress.RemainingDeltaV));
                if (result.LastDirective == AtmosphericEntryDirective.FiniteBurnComplete)
                {
                    result.FiniteBurnCompleted = true;
                    break;
                }
                if (result.LastDirective != AtmosphericEntryDirective.RequestFiniteBurnThrottle) return result;
            }
            result.DeliveredDeltaV = progress.DeliveredDeltaV;
            if (!result.FiniteBurnCompleted) return result;
            result.Add(manager.Tick(ut, false, attitudeErrorDegrees, progress.RemainingDeltaV));
            if (result.LastDirective != AtmosphericEntryDirective.RequirePostBurnValidation) return result;
            result.PostBurnValidationRequired = true;
            result.Add(manager.AcceptPostBurnValidation(postBurnPlan));
            result.FinalPhase = manager.Phase;
            return result;
        }
    }

    public sealed class AtmosphericEntryControllerHarnessResult
    {
        public readonly System.Collections.Generic.List<AtmosphericEntryDirective> Directives =
            new System.Collections.Generic.List<AtmosphericEntryDirective>();
        public bool InitialWarpRequested;
        public bool FinalWarpRequested;
        public bool FreshBurnValidationRequired;
        public bool FiniteBurnStarted;
        public bool NonzeroThrottleCommanded;
        public bool FiniteBurnCompleted;
        public bool PostBurnValidationRequired;
        public double DeliveredDeltaV;
        public double LastWarpUT = double.NaN;
        public int WorkUnits;
        public AtmosphericEntryPhase FinalPhase;
        public string LastReason;
        public AtmosphericEntryDirective LastDirective => Directives.Count == 0
            ? AtmosphericEntryDirective.None : Directives[Directives.Count - 1];

        public void Add(AtmosphericEntryPhaseDecision decision)
        {
            Directives.Add(decision.Directive);
            LastWarpUT = decision.WarpUT;
            LastReason = decision.Reason;
            WorkUnits += decision.WorkUnits;
            FinalPhase = decision.Phase;
        }
    }

    public static class AtmosphericEnergyGate
    {
        public static bool ShouldBeginPoweredBraking(double altitude, double descentSpeed,
            double gravity, double maximumAcceleration)
        {
            if (!Finite(altitude) || !Finite(descentSpeed) || !Finite(gravity) || !Finite(maximumAcceleration)) return false;
            double netAcceleration = maximumAcceleration - gravity;
            if (netAcceleration <= 0 || descentSpeed <= 0) return false;
            // Two times the ideal stopping distance: one stopping-distance
            // margin covers modelled drag variation and finite attitude/engine response.
            return altitude <= descentSpeed * descentSpeed / netAcceleration;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public enum AtmosphericEntryPhase
    {
        Idle, InitialWarpToBurn, PrepareWarp, WarpToBurn, AlignBurn, Burn, AwaitPostBurnValidation, Entry, Rejected
    }

    public enum AtmosphericEntryDirective
    {
        None, RequestInitialWarp, RequestAttitude, WarpAuthorized, RequestWarp, ExitWarpAndRequestAttitude,
        RequireFreshBurnValidation, BeginFiniteBurn, RequestFiniteBurnThrottle, FiniteBurnComplete,
        RequirePostBurnValidation, EnterAtmosphericEntry, Reject
    }

    public sealed class AtmosphericEntryPhaseDecision
    {
        public readonly AtmosphericEntryPhase Phase;
        public readonly AtmosphericEntryDirective Directive;
        public readonly double WarpUT;
        public readonly string Reason;
        public readonly int WorkUnits;

        public AtmosphericEntryPhaseDecision(AtmosphericEntryPhase phase, AtmosphericEntryDirective directive,
            double warpUT, string reason, int workUnits)
        {
            Phase = phase;
            Directive = directive;
            WarpUT = warpUT;
            Reason = reason;
            WorkUnits = workUnits;
        }
    }
}
