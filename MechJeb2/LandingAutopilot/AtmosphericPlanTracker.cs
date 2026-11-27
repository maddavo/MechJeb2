using System;
using System.Collections.Generic;
using System.Globalization;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;

namespace MuMech.Landing
{
    // Physics-rate tracking of a single replayed trajectory. No game APIs.
    // Reference tracking is distinct from optimisation and from independent
    // recoverability certification. Loss of the envelope is explicit.
    internal sealed class AtmosphericPlanTracker
    {
        private readonly D.Environment environment;
        private readonly D.Propulsion propulsion;
        private readonly D.Limits limits;
        private readonly F.Settings settings;
        private readonly F.Candidate plan;
        // Build once per accepted trajectory, including replacements.
        // Display updates do not scan the full numerical history.
        private readonly List<(double start,double end)> poweredWindows=new List<(double,double)>();
        internal double StartUT => plan.Reference[0].State.UT;
        internal double EndUT => plan.Result.End.UT;
        internal double TargetMiss => plan.Result.Miss;
        internal double ReserveMass => limits.ReserveMass;
        internal F.Candidate Plan => plan;
        internal F.Settings Settings => settings;

        internal AtmosphericPlanTracker(D.Environment environment,D.Propulsion propulsion,D.Limits limits,
            F.Settings settings,F.Candidate replayedPlan)
        {
            if(environment==null||!environment.Valid||propulsion==null||limits==null||!limits.Valid||
                settings==null||!settings.Valid||replayedPlan==null||!replayedPlan.Result.Feasible||
                replayedPlan.Reference.Count==0||replayedPlan.Reference.Count!=replayedPlan.Commands.Count||
                !replayedPlan.Result.End.Valid||replayedPlan.Result.End.Mass<limits.ReserveMass)
                throw new ArgumentException("invalid trajectory");
            this.environment=environment;this.propulsion=propulsion;this.limits=limits;
            this.settings=settings;plan=replayedPlan;
            double last=plan.Reference[0].State.UT;
            for(int i=0;i<plan.Reference.Count;i++)
            {
                var p=plan.Reference[i];var c=plan.Commands[i];
                if(!p.State.Valid||!p.Command.Valid||p.State.UT!=last||
                    !D.Finite(p.ValidUntil)||p.ValidUntil<=last||p.Command.Throttle>settings.Common.CommandCap||
                    c.StartUT!=last||c.EndUT!=p.ValidUntil||
                    c.Command.Throttle!=p.Command.Throttle||
                    (c.Command.Direction-p.Command.Direction).Length>1e-9)
                    throw new ArgumentException("non-contiguous or invalid trajectory");
                last=p.ValidUntil;
                if(c.Command.Throttle>0&&c.StartUT<EndUT)
                {
                    double end=Math.Min(c.EndUT,EndUT);
                    int n=poweredWindows.Count;
                    if(n>0&&poweredWindows[n-1].end==c.StartUT)
                        poweredWindows[n-1]=(poweredWindows[n-1].start,end);
                    else poweredWindows.Add((c.StartUT,end));
                }
            }
            if(EndUT<=StartUT||EndUT>last)throw new ArgumentException("invalid contact epoch");
        }

        internal readonly struct Decision
        {
            internal readonly bool Valid;
            internal readonly string Reason;
            internal readonly D.Command Command;
            internal readonly double PositionError,VelocityError,MassError,ReferenceClearance;
            internal Decision(string reason,double position=double.NaN,double velocity=double.NaN,
                double mass=double.NaN,double clearance=double.NaN)
            {Valid=false;Reason=reason;Command=default;PositionError=position;VelocityError=velocity;
                MassError=mass;ReferenceClearance=clearance;}
            internal Decision(D.Command command,double position,double velocity,double mass,double clearance)
            {Valid=true;Reason="tracking";Command=command;PositionError=position;VelocityError=velocity;
                MassError=mass;ReferenceClearance=clearance;}
        }

        // Durations are simulated game time, not wall-clock time.
        // Round up so a future event never displays zero prematurely.
        internal static string Countdown(double seconds)
        {
            if(!D.Finite(seconds)||seconds<0||seconds>=TimeSpan.MaxValue.TotalSeconds)return "unknown";
            var span=TimeSpan.FromSeconds(Math.Ceiling(seconds));
            return ((long)span.TotalHours).ToString("00",CultureInfo.InvariantCulture)+":"+
                span.Minutes.ToString("00",CultureInfo.InvariantCulture)+":"+
                span.Seconds.ToString("00",CultureInfo.InvariantCulture);
        }

        internal bool TryPoweredWindow(double now,double notBefore,out double start,out double end)
        {
            start=end=double.NaN;
            if(!D.Finite(now)||!D.Finite(notBefore)||now<StartUT||now>=EndUT)return false;
            double from=Math.Max(now,notBefore);
            int lo=0,hi=poweredWindows.Count;
            while(lo<hi)
            {
                int mid=lo+(hi-lo)/2;
                if(poweredWindows[mid].end<=from)lo=mid+1;else hi=mid;
            }
            if(lo==poweredWindows.Count)return false;
            var window=poweredWindows[lo];
            start=Math.Max(window.start,notBefore);end=window.end;
            return start<end;
        }

        internal string ForecastCountdowns(double now,bool strategic,double deorbitStart,double deorbitEnd,double approachStart)
        {
            if(!D.Finite(now)||now<StartUT||now>=EndUT)return "Manoeuvre countdown unavailable";
            string text="";
            if(strategic&&now<deorbitEnd)
            {
                text=now<deorbitStart?
                    "Deorbit burn in "+Countdown(deorbitStart-now)+"; duration "+Countdown(deorbitEnd-deorbitStart):
                    "Deorbit burn remaining "+Countdown(deorbitEnd-now);
                text+="\n";
            }
            if(TryPoweredWindow(now,strategic?approachStart:StartUT,out double start,out double end))
                text+=now<start?"Next descent burn in "+Countdown(start-now)+"; duration "+Countdown(end-start)+"\n":
                    "Descent burn remaining "+Countdown(end-now)+"\n";
            else text+="No further scheduled burn\n";
            return text+"Touchdown in "+Countdown(EndUT-now)+" (game time)";
        }

        // Search only the remaining validated reference. Do not warp
        // across positive thrust, engine tail or atmosphere entry.
        internal double NextPoweredOrAtmosphericUT(double ut,double boundaryRadius)
        {
            if(!ReferenceAt(ut,out var state,out var command)||command.Throttle!=0||
                state.Throttle>1e-4||state.R.Length<=boundaryRadius)return ut;
            int lo=0,hi=plan.Reference.Count-1;
            while(lo<hi)
            {
                int mid=lo+(hi-lo+1)/2;
                if(plan.Reference[mid].State.UT<=ut)lo=mid;else hi=mid-1;
            }
            for(int i=lo+1;i<plan.Reference.Count;i++)
            {
                var p=plan.Reference[i];
                if(p.Command.Throttle>0||p.State.Throttle>1e-4||p.State.R.Length<=boundaryRadius)
                    return p.State.UT;
            }
            return EndUT;
        }

        internal bool ReferenceAt(double ut,out D.State state,out D.Command command)
        {
            state=default;command=default;
            if(!D.Finite(ut)||ut<StartUT||ut>=EndUT)return false;
            int lo=0,hi=plan.Reference.Count-1;
            while(lo<hi)
            {
                int mid=lo+(hi-lo+1)/2;
                if(plan.Reference[mid].State.UT<=ut)lo=mid;else hi=mid-1;
            }
            var p=plan.Reference[lo];
            D.State a=p.State,b=lo+1<plan.Reference.Count?plan.Reference[lo+1].State:plan.Result.End;
            if(ut>=p.ValidUntil||b.UT<=a.UT)return false;
            double f=(ut-a.UT)/(b.UT-a.UT);
            D.V forward=(a.Forward*(1-f)+b.Forward*f).Unit;
            // An antipodal interpolation has no well-defined direction.
            if(forward.Length<.5)return false;
            state=new D.State(ut,a.R*(1-f)+b.R*f,a.Velocity*(1-f)+b.Velocity*f,
                a.Mass*(1-f)+b.Mass*f,forward,a.Throttle*(1-f)+b.Throttle*f,
                a.PointingRate*(1-f)+b.PointingRate*f);
            command=p.Command;
            return state.Valid;
        }

        // At altitude, differences between the target sphere and local
        // terrain are not necessarily tracking deviations. The actual
        // clearance and stopping bound remain separate admission checks.
        internal static double TerrainTolerance(double clearance)=>Math.Min(10000,Math.Max(2,clearance*.1));

        // Do not use a stale solution or "catch up" by skipping a burn.
        // Interpolated reference is checked at the actual UT; a small
        // command interval must not be treated as a fresh complete plan.
        // For a proportional position term 1/tau², a steady
        // acceleration disturbance produces approximately a*tau²
        // position offset. Choose tau so the modelled 10% drag error
        // uses no more than a quarter of the existing tracking envelope.
        // Keep the same bounded correction acceleration and minimum
        // response time; do not expand positional acceptance.
        internal static double FeedbackTimeConstant(double clearance,double positionLimit,double dragAcceleration)
        {
            double tau=Math.Max(1.5,Math.Min(8,Math.Sqrt(Math.Max(1,clearance))*.35));
            if(D.Finite(dragAcceleration)&&dragAcceleration>0)
                tau=Math.Max(1.5,Math.Min(tau,Math.Sqrt(.25*positionLimit/(.1*dragAcceleration))));
            return tau;
        }

        internal Decision Track(D.State actual,double clearance,double liveMaxAcceleration,double cap,
            double localGravity,double liveResponse)
        {
            if(!actual.Valid||!D.Finite(clearance)||clearance<=0||
                !D.Finite(liveMaxAcceleration)||liveMaxAcceleration<=0||!D.Finite(cap)||cap<=0||cap>1||
                !D.Finite(localGravity)||localGravity<0||!D.Finite(liveResponse)||liveResponse<0)
                return new Decision("invalid live capability or clearance");
            if(actual.Mass<=limits.ReserveMass)return new Decision("propellant reserve reached");
            if(cap+1e-5<settings.Common.CommandCap)return new Decision("throttle authority reduced");
            if(!ReferenceAt(actual.UT,out D.State reference,out D.Command feed))
                return new Decision("plan epoch expired");
            D.V dr=reference.R-actual.R,dv=reference.Velocity-actual.Velocity;
            double pe=dr.Length,ve=dv.Length,me=Math.Abs(reference.Mass-actual.Mass);
            double refClearance=environment.PhysicalClearance(reference);
            Decision Reject(string reason)=>new Decision(reason,pe,ve,me,refClearance);
            if(!D.Finite(refClearance))return Reject("unknown reference clearance");
            // Strict near-contact tolerances; larger but bounded errors
            // at altitude. These are experimental tracking limits.
            double positionLimit=Math.Min(200,Math.Max(3,refClearance*.02));
            double velocityLimit=Math.Min(15,Math.Max(.75,refClearance*.005));
            double initialMass=plan.Reference[0].State.Mass;
            // A 5% lower impulse per kg requires up to 1/.95 times
            // the propellant for the same impulse. Allow the associated
            // cumulative consumption difference in addition to the
            // original measurement allowance, not arbitrary mass loss.
            double consumed=Math.Max(0,initialMass-reference.Mass);
            double massLimit=Math.Max(10,initialMass*.01)+consumed*(1/.95-1);
            // The final segment is a spatial/velocity feedback landing,
            // not tracking a clock. Permit the same bounded local terminal
            // envelope used by the planner once its recorded terminal
            // phase begins. Contact timing variation is not itself unsafe.
            D.V up=actual.R.Unit,w=environment.Surface(actual.R,actual.Velocity);
            double vr=D.V.Dot(w,up),vh=(w-up*vr).Length;
            D.V contact=P.RotateTarget(plan.Result.End.R.Unit,environment.Omega,actual.UT-EndUT);
            double contactDistance=(up-contact).Length*environment.Radius;
            // Timing recovery is vertical only: an arbitrary lateral
            // displacement is NOT corrected by the radial terminal law.
            // Keep the original transverse position and speed envelopes.
            D.V lateralError=dr-up*D.V.Dot(dr,up);
            D.V lateralVelocityError=dv-up*D.V.Dot(dv,up);
            bool terminal=D.Finite(plan.Result.TerminalStartUT)&&actual.UT>=plan.Result.TerminalStartUT&&
                clearance<=Math.Min(50,settings.ApproachHeight*2+5)&&vr<=0&&
                vr>=-settings.TerminalVertical-1&&vh<=Math.Max(1,settings.TerminalHorizontal+1)&&
                D.Finite(contactDistance)&&contactDistance<=Math.Max(5,settings.Common.TargetTolerance)&&
                lateralError.Length<=positionLimit&&lateralVelocityError.Length<=velocityLimit&&
                D.V.Dot(actual.Forward,up)>.75;
            if((!terminal&&(pe>positionLimit||ve>velocityLimit))||me>massLimit)
                return Reject("trajectory deviation");
            double remaining=Math.Max(0,reference.Mass-plan.Result.End.Mass);
            if(actual.Mass-remaining<limits.ReserveMass)
                return Reject("projected propellant reserve threatened");
            // Higher terrain is not itself an unsafe trajectory.
            // The physical stopping-distance screen below uses the
            // actual clearance. Near-ground guidance uses the actual
            // clearance in the terminal feedback law. Reject a trajectory
            // which reaches ground outside its terminal/arrival envelope.
            if(clearance<=3&&!terminal&&refClearance>clearance+3)
                return Reject("unexpected imminent terrain contact");
            double vertical=D.V.Dot(environment.Surface(actual.R,actual.Velocity),actual.R.Unit);
            double down=Math.Max(0,-vertical),available=liveMaxAcceleration*cap-localGravity;
            double touch=settings.Common.Terminal.TouchdownSpeed;
            double stopDistance=available>0?Math.Max(0,down*down-touch*touch)/(2*available):double.PositiveInfinity;
            // Optimistic axial stopping bound, not a certified safety model.
            // Failing even this bound is a reason to reject tracking.
            // A no-drag bound is not an atmospheric reachability proof.
            // Do not discard a fully propagated trajectory far from contact
            // solely for failing this local bound. Near contact it protects
            // against an imminent downward-speed hazard.
            if(down>touch+1&&clearance/Math.Max(.1,down)<=8&&
                (!D.Finite(stopDistance)||stopDistance+down*liveResponse>clearance))
                return Reject("insufficient radial stopping distance");
            if(terminal)
            {
                var surface=new D.SurfaceSample(clearance,clearance,clearance,up,true);
                var terminalCommand=F.TerminalCommand(environment,propulsion,actual,settings,surface);
                if(!terminalCommand.Valid||!environment.Air(actual.UT,actual.R,out double terminalPressure,out _))
                    return Reject("unknown terminal capability");
                var nominal=propulsion.At(terminalPressure);
                if(!nominal.Known||nominal.Force<=0)return Reject("unknown terminal thrust");
                var terminalRequest=new D.Command(Math.Min(cap,terminalCommand.Throttle*nominal.Force/
                    (actual.Mass*liveMaxAcceleration)),terminalCommand.Direction.Unit);
                return terminalRequest.Valid?new Decision(terminalRequest,pe,ve,me,refClearance):Reject("invalid terminal command");
            }
            if(feed.Throttle==0)
                return new Decision(new D.Command(0,feed.Direction.Unit),pe,ve,me,refClearance);
            if(!environment.Air(reference.UT,reference.R,out double pressure,out double density))
                return Reject("unknown reference atmosphere");
            var authority=propulsion.At(pressure);
            if(!authority.Known||authority.Force<=0)return Reject("unknown reference thrust");
            D.V acceleration=feed.Direction.Unit*(feed.Throttle*authority.Force/reference.Mass);
            // Position/velocity correction has limited authority. Preserve
            // scheduled unpowered intervals and use no integral windup.
            double airSpeed=environment.Surface(reference.R,reference.Velocity).Length;
            double dragAcceleration=.5*density*environment.DragArea*airSpeed*airSpeed/reference.Mass;
            double tau=FeedbackTimeConstant(refClearance,positionLimit,dragAcceleration);
            // The reference thrust alone does not reproduce reference
            // acceleration when mass, density or relative air velocity
            // have changed. Compensate the predictable passive-force
            // difference using the SAME nominal model at both states.
            // Uncertainty replay must not reveal its perturbed model here.
            var referenceForces=D.Forces(environment,propulsion,reference);
            var actualForces=D.Forces(environment,propulsion,actual);
            if(!referenceForces.Known||!actualForces.Known)
                return Reject("unknown passive acceleration");
            D.V referencePassive=referenceForces.Gravity+
                (referenceForces.Drag+referenceForces.Lift)*(1/reference.Mass);
            D.V actualPassive=actualForces.Gravity+
                (actualForces.Drag+actualForces.Lift)*(1/actual.Mass);
            D.V correction=dr*(1/(tau*tau))+dv*(2/tau)+referencePassive-actualPassive;
            double correctionCap=.25*liveMaxAcceleration*cap;
            if(correction.Length>correctionCap)correction=correction.Unit*correctionCap;
            acceleration+=correction;
            if(!acceleration.Valid)return Reject("nonfinite acceleration");
            D.V direction=acceleration.Length>1e-8?acceleration.Unit:feed.Direction.Unit;
            double throttle=Math.Min(cap,acceleration.Length/liveMaxAcceleration);
            if(D.V.Dot(actual.Forward,direction)<.75)throttle=0;
            var command=new D.Command(throttle,direction);
            return command.Valid?new Decision(command,pe,ve,me,refClearance):Reject("invalid tracking command");
        }
    }
}
