using System;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;

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
            double massLimit=Math.Max(10,plan.Reference[0].State.Mass*.01);
            if(pe>positionLimit||ve>velocityLimit||me>massLimit)return Reject("trajectory deviation");
            if(clearance<refClearance-TerrainTolerance(refClearance))
                return Reject("terrain clearance below trajectory");
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
            if(feed.Throttle==0)
                return new Decision(new D.Command(0,feed.Direction.Unit),pe,ve,me,refClearance);
            if(!environment.Air(reference.UT,reference.R,out double pressure,out _))
                return Reject("unknown reference atmosphere");
            var authority=propulsion.At(pressure);
            if(!authority.Known||authority.Force<=0)return Reject("unknown reference thrust");
            D.V acceleration=feed.Direction.Unit*(feed.Throttle*authority.Force/reference.Mass);
            // Position/velocity correction has limited authority. Preserve
            // scheduled unpowered intervals and use no integral windup.
            double tau=Math.Max(1.5,Math.Min(8,Math.Sqrt(Math.Max(1,refClearance))*.35));
            D.V correction=dr*(1/(tau*tau))+dv*(2/tau);
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
