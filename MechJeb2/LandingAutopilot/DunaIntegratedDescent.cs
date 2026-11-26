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
        private bool emergency;
        internal void CancelReplanning(){replanner.Cancel();fallback.Cancel();}
        private double nextEngineUT=double.NegativeInfinity,nextLogUT=double.NegativeInfinity,lastUT=double.NegativeInfinity;
        private bool terminalExtension;
        private double nextSurfaceUT=double.NegativeInfinity,deckOffset;
        private string detail="";
        public override string TraceDetails=>detail;
        internal DunaIntegratedDescent(MechJebCore core,DunaIntegratedPreview.Snapshot snapshot,
            AtmosphericPlanTracker tracker):base(core)
        {
            this.snapshot=snapshot;this.tracker=tracker;activeEnvironment=snapshot.Environment;
            Status="Integrated descent (experimental)";
        }
        private string Num(double x)=>x.ToString("F3",CultureInfo.InvariantCulture);
        private AutopilotStep Finish(string reason)
        {
            Core.Landing.EndIntegratedDescent(reason);
            return null;
        }

        public override AutopilotStep Drive(FlightCtrlState controls)
        {
            try
            {
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
                var actual=DunaIntegratedPreview.LiveState(Core);
                var decision=tracker.Track(actual,clearance,s.MaxThrustAcceleration,cap,
                    s.LocalGravity,s.MaxEngineResponseTime);
                bool orbitalPrefix=snapshot.Strategic&&s.Time<snapshot.ApproachStartUT;
                if(!emergency&&!orbitalPrefix)
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
                D.Command command;
                if(decision.Valid)command=decision.Command;
                else
                {
                    // Small, bounded contact-time tolerance ONLY for an
                    // already slow, near-upright terminal state. No general
                    // emergency controller or legacy handover is implied.
                    if(decision.Reason!="plan epoch expired"||s.Time<tracker.EndUT||
                        s.Time>tracker.EndUT+5||!actual.Valid||actual.Mass<=tracker.ReserveMass||
                        !D.Finite(clearance)||clearance<0||clearance>5||
                        s.SpeedVertical>0||s.SpeedVertical< -2||s.SpeedSurfaceHorizontal>1||
                        D.V.Dot(actual.Forward,actual.R.Unit)<Math.Cos(Math.PI/6)||
                        s.MaxThrustAcceleration*cap<=s.LocalGravity)
                        return Finish(decision.Reason+"; "+(emergency?"local contingency lost":fallback.Status)+"; manual control required");
                    terminalExtension=true;
                    var surface=new D.SurfaceSample(clearance,clearance,clearance,actual.R.Unit,true);
                    var t=AtmosphericFiniteTimePlanner.TerminalCommand(activeEnvironment,snapshot.Propulsion,
                        actual,tracker.Settings,surface);
                    if(!t.Valid||!activeEnvironment.Air(actual.UT,actual.R,out double pressure,out _))
                        return Finish("terminal extension capability unknown");
                    var a=snapshot.Propulsion.At(pressure);
                    if(!a.Known||a.Force<=0)return Finish("terminal extension authority unknown");
                    double throttle=t.Throttle*a.Force/(actual.Mass*s.MaxThrustAcceleration);
                    command=new D.Command(Math.Min(cap,throttle),t.Direction.Unit);
                }
                if(orbitalPrefix&&s.Time>=snapshot.DeorbitStartUT&&s.Time<snapshot.DeorbitEndUT&&
                    (!tracker.ReferenceAt(actual.UT,out _,out var plannedBurn)||
                     D.V.Dot(actual.Forward,plannedBurn.Direction.Unit)<.995))
                    return Finish("orbital burn alignment not demonstrated; manual control required");
                if(!command.Valid||!D.Finite(cap)||command.Throttle>cap+1e-8)
                    return Finish("invalid live command");
                // Exclusive ownership of the existing actuator interfaces.
                // OFF is the thrust controller's direct TargetThrottle path
                // when it has an engaged consumer; no competing speed PID.
                Core.Thrust.Tmode=MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH=false;
                Core.Attitude.attitudeTo(DunaIntegratedPreview.World(command.Direction.Unit),
                    AttitudeReference.INERTIAL,Core.Landing);
                Core.Thrust.RequestActiveThrottle((float)command.Throttle,false,true);
                double originalMiss=AtmosphericFeasibleRangePlanner.Miss(snapshot.Environment,snapshot.Initial,
                    tracker.Plan.Result.End,snapshot.Settings.Common.TargetAtInitialUT);
                bool degraded=emergency||originalMiss>snapshot.Settings.Common.TargetTolerance;
                Status=(orbitalPrefix?(s.Time<snapshot.DeorbitStartUT?"Strategic orbital preparation ":s.Time<snapshot.DeorbitEndUT?"Strategic finite deorbit ":"Strategic engine tail "):emergency?"Emergency local-site ":terminalExtension?"Integrated terminal extension ":
                    "Integrated "+(degraded?"degraded ":""))+(command.Throttle>0?"powered":"coast")+
                    "; original target model miss "+(originalMiss/1000).ToString("F2",CultureInfo.InvariantCulture)+" km"+
                    (emergency?"\nFallback site model miss "+(tracker.TargetMiss/1000).ToString("F2",CultureInfo.InvariantCulture)+" km":
                    "\n"+replanner.Status+"; "+fallback.Status);
                detail=" planEndUT="+Num(tracker.EndUT)+" clearance="+Num(clearance)+
                    " positionError="+Num(decision.PositionError)+" velocityError="+Num(decision.VelocityError)+
                    " massErrorKg="+Num(decision.MassError)+" throttle="+Num(command.Throttle)+
                    " acceptedReplans="+replanner.Accepted+" rejectedReplans="+replanner.Rejected+
                    " preparedFallbacks="+fallback.Prepared+" emergency="+emergency+" originalMiss="+Num(originalMiss);
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
