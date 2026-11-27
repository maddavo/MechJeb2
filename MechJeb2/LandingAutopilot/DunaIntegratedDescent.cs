using System;
using System.Globalization;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // Explicit experimental controller, never selected by ordinary Land at
    // target. Only a pre-screened, replayed model trajectory may engage.
    // No implicit fallback to legacy braking or unscreened retargeting.
    internal sealed class DunaIntegratedDescent : AutopilotStep
    {
        private readonly DunaIntegratedPreview.Snapshot snapshot;
        private AtmosphericPlanTracker tracker;
        private readonly DunaReplanCoordinator replanner=new DunaReplanCoordinator();
        private readonly DunaFallbackCoordinator fallback=new DunaFallbackCoordinator();
        private D.Environment activeEnvironment;
        private bool emergency,hazardMode,orbitalBurnRequested;
        private string hazardReason;
        internal void CancelReplanning(){replanner.Cancel();fallback.Cancel();}
        private bool warpOwned,warpReturning,warpDisabledForPlan;
        private double warpExit=double.NaN,nextWarpProbe=double.NegativeInfinity;
        private double unpackedUT=double.NaN,warpStartUT=double.NaN;
        private bool warpSawRate;
        internal void CancelWarp()
        {
            if(warpOwned||warpReturning)Core.Warp.StopOwnedLandingWarp();
            warpOwned=false;warpReturning=false;warpExit=double.NaN;unpackedUT=double.NaN;
        }
        private bool OwnsControls()=>Core.Landing.Enabled&&DunaLandingAdmission.OwnsActuators(
            Core.Attitude.Users.Count,Core.Attitude.Users.Contains(Core.Landing),
            Core.Thrust.Users.Count,Core.Thrust.Users.Contains(Core.Thrust),
            Core.Thrust.Users.Contains(Core.Landing),true);
        private bool PhysicsReady()=>DunaLandingAdmission.WarpPhysicsReady(TimeWarp.CurrentRate,Vessel.packed,
            TimeWarp.CurrentRateIndex==0&&Math.Abs(VesselState.Time-Planetarium.GetUniversalTime())<=.05);
        private void ReturnToPhysics()
        {
            Core.Warp.StopOwnedLandingWarp();
            warpReturning=true;warpOwned=false;unpackedUT=double.NaN;
            Status="Returning to 1x for landing trajectory revalidation";
        }
        private AutopilotStep RefreshAfterWarp()
        {
            // Only after actual physics/vessel state has refreshed. The
            // packed orbit is not a live attitude or engine observation.
            string reason=DunaIntegratedPreview.LiveCompatibility(Core,snapshot,true);
            if(reason!=null)return Finish(reason+" after warp");
            var s=VesselState;
            var actual=DunaIntegratedPreview.LiveState(Core,snapshot);
            double cap=Math.Min(snapshot.Cap,Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit));
            var decision=tracker.Track(actual,s.AltitudeBottom,s.MaxThrustAcceleration,cap,
                s.LocalGravity,s.MaxEngineResponseTime);
            // Crossing an intended atmospheric margin is not a missed
            // powered interval. Inspect the actual reference history.
            bool missed=!tracker.ReferenceAt(actual.UT,out _,out var feed)||feed.Throttle>0;
            if(!missed&&D.Finite(warpStartUT))
                foreach(var p in tracker.Plan.Reference)
                {
                    if(p.State.UT>=actual.UT)break;
                    if(p.ValidUntil>warpStartUT&&p.Command.Throttle>0){missed=true;break;}
                }
            UnityEngine.Debug.Log("[DunaIntegratedDescent] warp exit UT="+Num(actual.UT)+
                " planned exit="+Num(warpExit)+" tracking="+decision.Valid+" missed="+missed+
                " positionError="+Num(decision.PositionError)+" velocityError="+Num(decision.VelocityError));
            warpReturning=false;warpOwned=false;warpExit=double.NaN;
            // A changed orbit or missed burn must be re-planned, not
            // silently credited with a burn skipped during warp.
            if((missed||!decision.Valid)&&!orbitalBurnRequested&&
                AtmosphericStrategicPlanner.TryOrbit(activeEnvironment,actual,out var orbit)&&
                orbit.Periapsis>activeEnvironment.Radius+activeEnvironment.AtmosphereTop)
            {
                var planning=new DunaIntegratedPlanning(Core);
                if(!planning.Start(out string why))return Finish("post-warp replan: "+why);
                CancelReplanning();
                return planning;
            }
            if(missed)return Finish("warp overran the unpowered interval; manual control required");
            // The validated trajectory is still current. The regular
            // tracker now uses the measured post-warp state, and the
            // normal approach replan/supervisor remains active.
            warpDisabledForPlan=true;
            nextWarpProbe=actual.UT+1;
            Status=decision.Valid?"Post-warp trajectory revalidated; acquiring burn attitude":
                "Post-warp trajectory requires guidance recovery";
            return this;
        }
        public override AutopilotStep OnFixedUpdate()
        {
            try
            {
                if(!OwnsControls())return Finish("warp actuator ownership changed");
                if(warpOwned||warpReturning)
                {
                    bool context=DunaIntegratedPreview.SameContext(Core,snapshot,true)&&
                        FlightGlobals.ActiveVessel==Vessel&&!Vessel.LandedOrSplashed;
                    if(!context)
                    {CancelWarp();return Finish("landing context changed during warp");}
                    if(warpReturning)
                    {
                        if(!PhysicsReady()){unpackedUT=double.NaN;Core.Warp.StopOwnedLandingWarp();return this;}
                        // Require a subsequent fresh physical tick, not
                        // merely the frame on which KSP unpacked.
                        if(!D.Finite(unpackedUT)){unpackedUT=VesselState.Time;return this;}
                        if(VesselState.Time<=unpackedUT)return this;
                        return RefreshAfterWarp();
                    }
                    double now=Planetarium.GetUniversalTime();
                    // A user returning to 1x manually is an interruption,
                    // not an instruction to immediately accelerate again.
                    if(TimeWarp.CurrentRate>1)warpSawRate=true;
                    if((warpSawRate&&TimeWarp.CurrentRate==1&&!Vessel.packed)||
                        Core.Warp.warpToUT>0)
                    {ReturnToPhysics();return this;}
                    var action=DunaLandingAdmission.AdvanceWarp(now,warpExit,TimeWarp.CurrentRate,Vessel.packed,
                        Core.Node.Autowarp,context,TimeWarp.WarpMode==TimeWarp.Modes.HIGH,Core.Warp.WarpPaused);
                    if(action!=DunaLandingAdmission.WarpAction.Continue)
                    {ReturnToPhysics();return this;}
                    // Regular warp only; never invoke physics warp in air.
                    // Bound the rate and decrease it as the exit approaches.
                    float maxRate=(float)Math.Max(1,Math.Min(100,(warpExit-now)/10));
                    Core.Warp.WarpRegularAtRate(maxRate,true,true);
                    Status="Auto-warp to landing preparation; "+Num(warpExit-now)+" s";
                    return this;
                }
                if(!PhysicsReady())
                {
                    // Manual/unowned warp cannot consume a powered segment.
                    if(TimeWarp.CurrentRate!=1||Vessel.packed)
                    {warpStartUT=lastUT;warpExit=Planetarium.GetUniversalTime();ReturnToPhysics();}
                    return this;
                }
                var s=VesselState;
                if(!Core.Node.Autowarp||Core.Warp.WarpPaused||hazardMode||emergency||terminalExtension||
                    replanner.Busy||fallback.Busy||Core.Warp.warpToUT>0||s.Time<nextWarpProbe)return this;
                // Allow another window after an actual orbital burn. Do
                // not restart the same warp during its preparation period.
                if(warpDisabledForPlan)
                {
                    if(!orbitalBurnRequested||s.Time<snapshot.ApproachStartUT)return this;
                    warpDisabledForPlan=false;
                }
                nextWarpProbe=s.Time+1;
                if(DunaIntegratedPreview.LiveCompatibility(Core,snapshot,false)!=null)return this;
                var actual=DunaIntegratedPreview.LiveState(Core,snapshot);
                double cap=Math.Min(snapshot.Cap,Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit));
                var decision=tracker.Track(actual,s.AltitudeBottom,s.MaxThrustAcceleration,cap,
                    s.LocalGravity,s.MaxEngineResponseTime);
                if(!decision.Valid||decision.Command.Throttle!=0)return this;
                double exit=DunaLandingAdmission.WarpExitUT(activeEnvironment,tracker,actual,decision.Command.Throttle,
                    Vessel.ctrlState.mainThrottle,s.CurrentThrustAcceleration,s.MinThrustAcceleration,true,true,false);
                if(!D.Finite(exit))return this;
                CancelReplanning();
                Core.Thrust.RequestActiveThrottle(0,false,true);
                warpStartUT=s.Time;warpExit=exit;warpOwned=true;warpSawRate=false;
                Core.Warp.WarpRegularAtRate((float)Math.Max(1,Math.Min(100,(exit-s.Time)/10)),true,true);
                Status="Auto-warp to landing preparation; "+Num(exit-s.Time)+" s";
                UnityEngine.Debug.Log("[DunaIntegratedDescent] auto-warp requested UT="+Num(s.Time)+" exit="+Num(exit));
                return this;
            }
            catch(Exception ex){CancelWarp();return Finish("warp controller exception: "+ex.Message);}
        }
        private double nextEngineUT=double.NegativeInfinity,nextLogUT=double.NegativeInfinity,lastUT=double.NegativeInfinity;
        private bool terminalExtension;
        private double nextSurfaceUT=double.NegativeInfinity,deckOffset;
        private string detail="";
        public override string TraceDetails=>detail;
        // GUI must not present the independent legacy predictor as the
        // forecast of this controller. Reproject contact radial at the
        // present body orientation; the plan remains a model forecast.
        internal string ForecastText()
        {
            if(hazardMode)return "Integrated forecast invalidated: "+hazardReason;
            double now=Planetarium.GetUniversalTime();
            if(terminalExtension||now>=tracker.EndUT)return "Integrated contact epoch expired; terminal extension";
            var end=tracker.Plan.Result.End;
            Vector3d world=MainBody.position+DunaIntegratedPreview.WorldAt(MainBody,snapshot.Frame,
                end.R.Unit*MainBody.Radius,end.UT);
            double miss=AtmosphericFeasibleRangePlanner.Miss(snapshot.Environment,snapshot.Initial,end,
                snapshot.Settings.Common.TargetAtInitialUT);
            return "Integrated trajectory (model only)"+(emergency?"; alternative site":"")+
                "\nContact "+MainBody.GetLatitude(world).ToString("F6",CultureInfo.InvariantCulture)+
                "°, "+MainBody.GetLongitude(world).ToString("F6",CultureInfo.InvariantCulture)+"°"+
                "\nOriginal target miss "+Num(miss/1000)+" km"+
                "\n"+tracker.ForecastCountdowns(now,snapshot.Strategic&&!emergency,
                    snapshot.DeorbitStartUT,snapshot.DeorbitEndUT,snapshot.ApproachStartUT)+
                (warpOwned&&now<warpExit?"\nWarp exit in "+AtmosphericPlanTracker.Countdown(warpExit-now):
                    warpReturning?"\nWaiting for 1x physics revalidation":"")+
                "\nModel remaining mass above reserve "+Num(end.Mass-tracker.ReserveMass)+" kg";
        }
        internal DunaIntegratedDescent(MechJebCore core,DunaIntegratedPreview.Snapshot snapshot,
            AtmosphericPlanTracker tracker):base(core)
        {
            this.snapshot=snapshot;this.tracker=tracker;activeEnvironment=snapshot.Environment;
            Status="Integrated descent (experimental)";
        }
        private string Num(double x)=>x.ToString("F3",CultureInfo.InvariantCulture);
        private AutopilotStep Finish(string reason)
        {
            CancelWarp();
            Core.Landing.EndIntegratedDescent(reason);
            return null;
        }

        public override AutopilotStep Drive(FlightCtrlState controls)
        {
            try
            {
                // Drive may not run during packed warp. When it does,
                // it must never write a powered command until revalidated.
                if(warpOwned||warpReturning||!PhysicsReady())return this;
                var s=VesselState;
                if(Vessel.LandedOrSplashed)
                    return Finish("observed contact lat="+Num(MainBody.GetLatitude(s.CoM))+
                        " lon="+Num(MainBody.GetLongitude(s.CoM))+" asl="+Num(s.AltitudeASL)+
                        " vertical="+Num(s.SpeedVertical)+" horizontal="+Num(s.SpeedSurfaceHorizontal)+
                        " massKg="+Num(s.Mass*1000));
                if(!Core.Landing.Enabled||Core.Attitude.Users.Count!=1||
                    !Core.Attitude.Users.Contains(Core.Landing)||Core.Thrust.Users.Count!=2||
                    !Core.Thrust.Users.Contains(Core.Thrust)||!Core.Thrust.Users.Contains(Core.Landing))
                    return Finish("actuator ownership changed; manual control required");
                if(s.Time<lastUT)return Finish("UT regressed");
                lastUT=s.Time;
                bool engines=s.Time>=nextEngineUT;
                if(engines)nextEngineUT=s.Time+1;
                string reason=DunaIntegratedPreview.LiveCompatibility(Core,snapshot,engines);
                if(reason!=null)return Finish(reason+"; manual control required");
                // The live collider-based altitude is not the model's sphere.
                double clearance=s.AltitudeBottom;
                if(clearance<1000&&s.Time>=nextSurfaceUT)
                {
                    nextSurfaceUT=s.Time+.5;
                    double deck=Core.Landing.LandingSurface.Altitude(MainBody,
                        MainBody.GetLatitude(s.CoM),MainBody.GetLongitude(s.CoM),s.SurfaceAltitudeASL);
                    if(Core.Landing.LandingSurface.LastQuerySaturated||!D.Finite(deck))
                        return Finish("unknown local collider surface; manual control required");
                    deckOffset=Math.Max(0,deck-s.SurfaceAltitudeASL);
                }
                clearance-=deckOffset;
                double cap=Math.Min(snapshot.Cap,Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit));
                var actual=DunaIntegratedPreview.LiveState(Core,snapshot);
                var decision=tracker.Track(actual,clearance,s.MaxThrustAcceleration,cap,
                    s.LocalGravity,s.MaxEngineResponseTime);
                bool orbitalPrefix=snapshot.Strategic&&s.Time<snapshot.ApproachStartUT;
                if(!hazardMode&&!emergency&&!orbitalPrefix)
                {
                    // Alternate background work. A ready contingency is not
                    // busy; a pending local terrain screen and nominal solve
                    // do not compete for the same physics-tick budget.
                    var local=decision.Valid?fallback.Tick(Core,snapshot,tracker,actual,decision,clearance,
                        s.MaxThrustAcceleration,cap,s.LocalGravity,s.MaxEngineResponseTime,!replanner.Busy):null;
                    var replacement=replanner.Tick(Core,snapshot,tracker,actual,decision,clearance,
                        s.MaxThrustAcceleration,cap,s.LocalGravity,s.MaxEngineResponseTime,!fallback.Busy);
                    if(!ReferenceEquals(replacement,tracker))
                    {
                        fallback.Cancel();tracker=replacement;terminalExtension=false;
                        decision=tracker.Track(actual,clearance,s.MaxThrustAcceleration,cap,
                            s.LocalGravity,s.MaxEngineResponseTime);
                    }
                    if(!decision.Valid)
                        local=fallback.Tick(Core,snapshot,tracker,actual,decision,clearance,
                            s.MaxThrustAcceleration,cap,s.LocalGravity,s.MaxEngineResponseTime,false);
                    if(local!=null)
                    {
                        replanner.Cancel();tracker=local.Tracker;activeEnvironment=local.Environment;
                        emergency=true;terminalExtension=false;
                        decision=tracker.Track(actual,clearance,s.MaxThrustAcceleration,cap,
                            s.LocalGravity,s.MaxEngineResponseTime);
                    }
                }
                var velocity=activeEnvironment.Surface(actual.R,actual.Velocity);
                // Before an irreversible orbital burn, retain the existing
                // abort policy. In descent, the independent supervisor may
                // invalidate even a nominally valid tracker.
                bool preburn=DunaLandingAdmission.OrbitalPreburn(activeEnvironment,actual,
                    snapshot.Strategic,orbitalBurnRequested);
                double maxFlow=double.NaN;
                if(activeEnvironment.Air(actual.UT,actual.R,out double p,out _))
                {
                    var authority=snapshot.Propulsion.At(p);
                    if(authority.Known)maxFlow=authority.Flow;
                }
                double touch=tracker.Settings.Common.Terminal.TouchdownSpeed;
                var safety=AtmosphericRecoverability.Assess(actual,velocity,clearance,s.MaxThrustAcceleration,
                    cap,s.LocalGravity,snapshot.Limits.TurnRate,s.MaxEngineResponseTime,maxFlow,tracker.ReserveMass,touch);
                D.Command command=default;
                bool extension=!hazardMode&&!decision.Valid&&decision.Reason=="plan epoch expired"&&
                    AtmosphericUncertaintyReplay.TryTerminalExtension(tracker,activeEnvironment,snapshot.Propulsion,
                        actual,velocity,clearance,s.MaxThrustAcceleration,cap,s.LocalGravity,out command);
                if(extension)terminalExtension=true;
                bool imminent=!preburn&&safety.Status==AtmosphericRecoverability.Level.Hazard;
                if(!hazardMode&&(imminent||!decision.Valid&&!extension))
                {
                    if(preburn)return Finish(decision.Reason+"; preburn abort");
                    hazardReason=imminent?safety.Reason:decision.Reason;
                    hazardMode=true;terminalExtension=false;
                    CancelReplanning();
                    UnityEngine.Debug.Log("[DunaIntegratedDescent] independent supervisor invalidated plan: "+hazardReason+
                        "; best-effort hazard control, targeting no longer valid");
                }
                if(hazardMode)
                {
                    if(!AtmosphericRecoverability.TryBestEffort(actual,velocity,clearance,s.MaxThrustAcceleration,
                        cap,s.LocalGravity,s.MaxEngineResponseTime,maxFlow,tracker.ReserveMass,touch,imminent,out command))
                        return Finish("hazard control unavailable: "+hazardReason+"; manual control required");
                }
                else if(decision.Valid)command=decision.Command;
                if(!hazardMode&&!DunaLandingAdmission.OrbitalBurnAligned(tracker,actual,orbitalPrefix,
                    snapshot.DeorbitStartUT,snapshot.DeorbitEndUT))
                    return Finish("orbital burn alignment not demonstrated; manual control required");
                if(!command.Valid||!D.Finite(cap)||command.Throttle>cap+1e-8)
                    return Finish("invalid live command");
                // Exclusive ownership of the existing actuator interfaces.
                // OFF is the thrust controller's direct TargetThrottle path
                // when it has an engaged consumer; no competing speed PID.
                Core.Thrust.Tmode=MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH=false;
                Core.Attitude.attitudeTo(DunaIntegratedPreview.WorldAt(MainBody,snapshot.Frame,command.Direction.Unit,s.Time),
                    AttitudeReference.INERTIAL,Core.Landing);
                Core.Thrust.RequestActiveThrottle((float)command.Throttle,false,true);
                if(orbitalPrefix&&command.Throttle>0)orbitalBurnRequested=true;
                double originalMiss=AtmosphericFeasibleRangePlanner.Miss(snapshot.Environment,snapshot.Initial,
                    tracker.Plan.Result.End,snapshot.Settings.Common.TargetAtInitialUT);
                bool degraded=emergency||originalMiss>snapshot.Settings.Common.TargetTolerance;
                if(hazardMode)originalMiss=double.NaN;
                Status=hazardMode?"Best-effort hazard control; NO VALID LANDING FORECAST\n"+hazardReason:
                    (orbitalPrefix?(s.Time<snapshot.DeorbitStartUT?"Strategic orbital preparation ":s.Time<snapshot.DeorbitEndUT?"Strategic finite deorbit ":"Strategic engine tail "):emergency?"Emergency local-site ":terminalExtension?"Integrated terminal extension ":
                    "Integrated "+(degraded?"degraded ":""))+(command.Throttle>0?"powered":"coast")+
                    "; original target model miss "+(originalMiss/1000).ToString("F2",CultureInfo.InvariantCulture)+" km"+
                    (emergency?"\nFallback site model miss "+(tracker.TargetMiss/1000).ToString("F2",CultureInfo.InvariantCulture)+" km":
                    "\n"+replanner.Status+"; "+fallback.Status)+
                    "\nRadial recoverability: "+safety.Status;
                detail=" planEndUT="+Num(tracker.EndUT)+" clearance="+Num(clearance)+
                    " positionError="+Num(decision.PositionError)+" velocityError="+Num(decision.VelocityError)+
                    " massErrorKg="+Num(decision.MassError)+" throttle="+Num(command.Throttle)+
                    " acceptedReplans="+replanner.Accepted+" rejectedReplans="+replanner.Rejected+
                    " preparedFallbacks="+fallback.Prepared+" emergency="+emergency+" originalMiss="+Num(originalMiss)+
                    " hazardMode="+hazardMode+" recoverability="+safety.Status+
                    " delayedRadialMargin="+Num(safety.Margin)+" radialFuelMarginKg="+Num(safety.FuelMargin);
                if(s.Time>=nextLogUT)
                {
                    nextLogUT=s.Time+1;
                    UnityEngine.Debug.Log("[DunaIntegratedDescent] ut="+Num(s.Time)+detail+
                        " observedAccel="+Num(s.CurrentThrustAcceleration)+" vertical="+Num(s.SpeedVertical)+
                        " horizontal="+Num(s.SpeedSurfaceHorizontal)+" massKg="+Num(actual.Mass)+
                        " reserveKg="+Num(tracker.ReserveMass));
                }
                return this;
            }
            catch(Exception ex){return Finish("controller exception: "+ex.Message+"; manual control required");}
        }
    }
}
