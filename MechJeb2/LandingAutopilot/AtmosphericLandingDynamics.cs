using System;
using System.Threading;

namespace MuMech.Landing
{
    // 2.16.0-landing.20261126.duna-live-screen
    // Independent SI, body-centred inertial, non-actuating reference model.
    // Fixed spherical terrain and collinear fixed engines only. No KSP APIs.
    internal static class AtmosphericLandingDynamics
    {
        internal readonly struct V
        {
            internal readonly double X, Y, Z;
            internal V(double x, double y, double z) { X=x; Y=y; Z=z; }
            internal double Length => Math.Sqrt(Dot(this,this));
            internal bool Valid => Finite(X) && Finite(Y) && Finite(Z) && Finite(Length);
            internal V Unit => Length>0 ? this*(1/Length) : default;
            public static V operator +(V a,V b) => new V(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
            public static V operator -(V a,V b) => a+b*(-1);
            public static V operator *(V a,double k) => new V(a.X*k,a.Y*k,a.Z*k);
            internal static double Dot(V a,V b) => a.X*b.X+a.Y*b.Y+a.Z*b.Z;
            internal static V Cross(V a,V b) => new V(a.Y*b.Z-a.Z*b.Y,a.Z*b.X-a.X*b.Z,a.X*b.Y-a.Y*b.X);
        }
        internal static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
        private static double Clamp(double x,double lo,double hi) => Math.Max(lo,Math.Min(hi,x));

        // Explicit interpolation. Tangents optional; linear is synthetic-only
        // unless the underlying curve is verified linear. No extrapolation.
        internal readonly struct Knot
        {
            internal readonly double X,Y,In,Out;
            internal Knot(double x,double y,double incoming=double.NaN,double outgoing=double.NaN)
            { X=x;Y=y;In=incoming;Out=outgoing; }
        }
        internal sealed class Curve
        {
            private readonly Knot[] _keys;
            internal Curve(params Knot[] keys)
            {
                if(keys==null || keys.Length==0) throw new ArgumentException("empty curve");
                _keys=(Knot[])keys.Clone();
                for(int i=0;i<_keys.Length;i++)
                    if(!Finite(_keys[i].X)||!Finite(_keys[i].Y)||_keys[i].Y<0 ||
                        i>0&&_keys[i].X<=_keys[i-1].X) throw new ArgumentException("invalid curve");
            }
            internal double At(double x)
            {
                if(!Finite(x)||x<_keys[0].X||x>_keys[_keys.Length-1].X) return double.NaN;
                for(int i=1;i<_keys.Length;i++)
                {
                    Knot a=_keys[i-1],b=_keys[i];
                    if(x>b.X) continue;
                    double d=b.X-a.X,t=(x-a.X)/d;
                    if(!Finite(a.Out)||!Finite(b.In)) return a.Y+(b.Y-a.Y)*t;
                    double t2=t*t,t3=t2*t;
                    return (2*t3-3*t2+1)*a.Y+(t3-2*t2+t)*d*a.Out+
                        (-2*t3+3*t2)*b.Y+(t3-t2)*d*b.In;
                }
                return _keys[_keys.Length-1].Y;
            }
        }

        internal sealed class Engine
        {
            // MaxFlow kg/s before persistent limit. Pressure curve uses atm.
            // Lever is metres from CoM in a common vessel frame; all engines
            // must be verified collinear with +Y in that frame.
            internal readonly double MaxFlow,Limit,G0;
            internal readonly V Lever;
            internal readonly Curve Isp;
            internal Engine(double flow,double limit,double g0,V lever,Curve isp)
            { MaxFlow=flow;Limit=limit;G0=g0;Lever=lever;Isp=isp; }
        }
        internal readonly struct Authority
        {
            internal readonly bool Known;
            internal readonly double Force,Flow,TorqueResidual;
            internal Authority(bool known,double force,double flow,double residual)
            {Known=known;Force=force;Flow=flow;TorqueResidual=residual;}
        }
        // Exact maximum zero-net-torque allocation for one or two fixed
        // collinear engines. Deliberately no assumption of reaction wheel,
        // gimbal or RCS authority. Unsupported geometry returns unknown;
        // zero force with known geometry is different from unknown capability.
        internal sealed class Propulsion
        {
            private readonly Engine[] _engines;
            private readonly bool _geometryVerified;
            private readonly Curve _allocatedForce;
            private readonly double _allocatedFlow;
            internal Propulsion(bool geometryVerified,params Engine[] engines)
            { _geometryVerified=geometryVerified;_engines=engines==null?Array.Empty<Engine>():(Engine[])engines.Clone(); }
            // Explicit estimated snapshot envelope, not a claim of verified
            // future torque balance. Force N vs pressure Pa, flow kg/s.
            private Propulsion(Curve allocatedForce,double allocatedFlow)
            { _allocatedForce=allocatedForce;_allocatedFlow=allocatedFlow;_engines=Array.Empty<Engine>(); }
            internal static Propulsion FromAllocatedEnvelope(Curve force,double flow) => new Propulsion(force,flow);
            internal Authority At(double pressurePa)
            {
                if(_allocatedForce!=null)
                {
                    double f=_allocatedForce.At(pressurePa);
                    return Finite(f)&&f>=0&&Finite(_allocatedFlow)&&_allocatedFlow>=0
                        ?new Authority(true,f,_allocatedFlow,double.NaN):default;
                }
                if(!_geometryVerified||_engines.Length<1||_engines.Length>2||
                    !Finite(pressurePa)||pressurePa<0) return default;
                var forces=new double[_engines.Length];var flows=new double[_engines.Length];
                var torque=new V[_engines.Length];
                for(int i=0;i<_engines.Length;i++)
                {
                    Engine e=_engines[i];
                    if(e==null||e.Isp==null||!Finite(e.MaxFlow)||e.MaxFlow<0||
                        !Finite(e.Limit)||e.Limit<0||e.Limit>1||!Finite(e.G0)||e.G0<=0||!e.Lever.Valid) return default;
                    double isp=e.Isp.At(pressurePa/101325);
                    if(!Finite(isp)||isp<=0) return default;
                    flows[i]=e.MaxFlow*e.Limit;
                    forces[i]=flows[i]*isp*e.G0;
                    torque[i]=V.Cross(e.Lever,new V(0,1,0))*forces[i];
                    if(!Finite(forces[i])||!torque[i].Valid) return default;
                }
                double x=1,y=_engines.Length==2?1:0;
                const double eps=1e-9;
                double n0=torque[0].Length,n1=_engines.Length==2?torque[1].Length:0;
                if(n0>eps&&n1>eps)
                {
                    if(V.Dot(torque[0],torque[1])>=0 ||
                        V.Cross(torque[0].Unit,torque[1].Unit).Length>1e-10)
                        return default; // unsupported nonopposed torque geometry
                    x=Math.Min(1,n1/n0);y=Math.Min(1,n0/n1);
                }
                else
                {
                    if(n0>eps) x=0;
                    if(n1>eps) y=0;
                }
                V residual=torque[0]*x;
                double f=forces[0]*x,q=flows[0]*x;
                if(_engines.Length==2){residual+=torque[1]*y;f+=forces[1]*y;q+=flows[1]*y;}
                if(!Finite(f)||!Finite(q)||residual.Length>eps+1e-10*(n0+n1))return default;
                return new Authority(true,f,q,residual.Length);
            }
        }

        internal sealed class Environment
        {
            internal readonly double Radius,Mu,AtmosphereTop,TerrainRadius,DragArea,BottomOffset;
            internal readonly V Omega;
            internal readonly Curve Pressure,Density;
            internal readonly Func<V,V,V,double,double,V> PassiveLift;
            // DragArea = Cd*A m², constant synthetic/passive adapter. Lift is
            // NOT assumed zero for a real vessel; unsupported unless supplied
            // via an explicitly supplied deterministic force model (r, air-relative
            // velocity, forward, pressure Pa, density kg/m³ -> force N). Null
            // means explicit zero lift for the model, not actual-vessel evidence.
            internal Environment(double radius,double mu,V omega,double atmosphereTop,
                Curve pressure,Curve density,double dragArea,double terrainRadius,double bottomOffset,
                Func<V,V,V,double,double,V> passiveLift=null)
            {Radius=radius;Mu=mu;Omega=omega;AtmosphereTop=atmosphereTop;Pressure=pressure;Density=density;
                DragArea=dragArea;TerrainRadius=terrainRadius;BottomOffset=bottomOffset;PassiveLift=passiveLift;}
            internal bool Valid => Finite(Radius)&&Radius>0&&Finite(Mu)&&Mu>=0&&Omega.Valid&&
                Finite(AtmosphereTop)&&AtmosphereTop>=0&&Finite(TerrainRadius)&&TerrainRadius>0&&
                Finite(BottomOffset)&&BottomOffset>=0&&Finite(DragArea)&&DragArea>=0&&
                (AtmosphereTop==0||Pressure!=null&&Density!=null);
            internal bool Air(V r,out double p,out double rho)
            {
                double h=r.Length-Radius;
                p=rho=0;
                if(!Finite(h))return false;
                if(AtmosphereTop==0||h>=AtmosphereTop)return true;
                p=Pressure.At(h);rho=Density.At(h);
                return Finite(p)&&p>=0&&Finite(rho)&&rho>=0;
            }
            internal V Surface(V r,V v)=>v-V.Cross(Omega,r);
            internal double Clearance(V r)=>r.Length-TerrainRadius-BottomOffset;
        }
        internal sealed class Limits
        {
            internal readonly double ReserveMass,TurnRate,EngineRise,EngineFall,Step;
            // Turn rate rad/s, engine first-order time constants in s.
            // Zero engine constant means explicitly instantaneous; zero turn
            // rate means no turning authority, not instantaneous pointing.
            internal Limits(double reserveMass,double turnRate,double rise,double fall,double step)
            {ReserveMass=reserveMass;TurnRate=turnRate;EngineRise=rise;EngineFall=fall;Step=step;}
            internal bool Valid => Finite(ReserveMass)&&ReserveMass>0&&Finite(TurnRate)&&TurnRate>=0&&
                Finite(EngineRise)&&EngineRise>=0&&Finite(EngineFall)&&EngineFall>=0&&Finite(Step)&&Step>0&&Step<=1;
        }
        internal readonly struct State
        {
            internal readonly double UT,Mass,Throttle,PointingRate;
            internal readonly V R,Velocity,Forward;
            internal State(double ut,V r,V v,double mass,V forward,double throttle=0,double rate=0)
            {UT=ut;R=r;Velocity=v;Mass=mass;Forward=forward;Throttle=throttle;PointingRate=rate;}
            internal bool Valid => Finite(UT)&&R.Valid&&R.Length>0&&Velocity.Valid&&Finite(Mass)&&Mass>0&&
                Forward.Valid&&Math.Abs(Forward.Length-1)<1e-6&&Finite(Throttle)&&Throttle>=0&&Throttle<=1&&
                Finite(PointingRate)&&PointingRate>=0;
        }
        internal readonly struct Command
        {
            internal readonly double Throttle;
            internal readonly V Direction;
            internal Command(double throttle,V direction){Throttle=throttle;Direction=direction;}
            internal bool Valid=>Finite(Throttle)&&Throttle>=0&&Throttle<=1&&Direction.Valid&&Direction.Length>0;
        }
        internal enum Outcome { Advanced, CaptureEnvelope, ContactWithinLimits, ContactUnsafe,
            ReserveExhausted, UnknownCapability, InvalidInput, Horizon, Cancelled, NoControllableThrust }
        internal readonly struct Result
        {
            internal readonly Outcome Status;
            internal readonly State End;
            internal readonly double MinimumClearance,FirstPoweredUT;
            internal Result(Outcome status,State end,double clearance,double firstPoweredUT=double.NaN)
            {Status=status;End=end;MinimumClearance=clearance;FirstPoweredUT=firstPoweredUT;}
        }
        private static V Turn(V from,V to,double angle)
        {
            to=to.Unit;
            double total=Math.Acos(Clamp(V.Dot(from,to),-1,1));
            if(total<=angle||total<1e-10)return to;
            V tangent=to-from*V.Dot(from,to);
            if(tangent.Length<1e-8)
                tangent=V.Cross(from,Math.Abs(from.X)<.8?new V(1,0,0):new V(0,1,0));
            return (from*Math.Cos(angle)+tangent.Unit*Math.Sin(angle)).Unit;
        }
        private static double Lag(double initial,double command,double tau,double dt)=>
            tau==0?command:command+(initial-command)*Math.Exp(-dt/tau);
        private static bool Force(Environment env,Propulsion propulsion,V r,V v,V f,double throttle,
            double mass,out V acceleration,out double flow)
        {
            acceleration=default;flow=0;
            if(!r.Valid||r.Length<=0||mass<=0||!env.Air(r,out double p,out double density))return false;
            Authority a=propulsion.At(p);
            if(!a.Known)return false;
            V air=env.Surface(r,v);
            V drag=air*(-.5*density*env.DragArea*air.Length);
            V lift=default;
            if(density>0&&env.PassiveLift!=null)
            {
                try { lift=env.PassiveLift(r,air,f,p,density); }
                catch(Exception) { return false; }
                if(!lift.Valid)return false;
            }
            acceleration=r*(-env.Mu/(r.Length*r.Length*r.Length))+(f*(a.Force*throttle)+drag+lift)*(1/mass);
            flow=a.Flow*throttle;
            return acceleration.Valid&&Finite(flow)&&flow>=0;
        }
        // Midpoint integration, time-varying pressure, mass and direction.
        // Explicitly subdivide events at caller; never clamp away spent fuel.
        private static bool RawStep(Environment env,Propulsion propulsion,Limits lim,State s,Command c,double dt,out State next)
        {
            next=s;
            double tau=c.Throttle>=s.Throttle?lim.EngineRise:lim.EngineFall;
            double midThrottle=Lag(s.Throttle,c.Throttle,tau,dt*.5);
            double endThrottle=Lag(s.Throttle,c.Throttle,tau,dt);
            V midForward=Turn(s.Forward,c.Direction,lim.TurnRate*dt*.5);
            V endForward=Turn(s.Forward,c.Direction,lim.TurnRate*dt);
            if(!Force(env,propulsion,s.R,s.Velocity,s.Forward,Lag(s.Throttle,c.Throttle,tau,0),s.Mass,out V a0,out double q0))return false;
            V midR=s.R+s.Velocity*(dt*.5),midV=s.Velocity+a0*(dt*.5);
            double midMass=s.Mass-q0*dt*.5;
            if(!Force(env,propulsion,midR,midV,midForward,midThrottle,midMass,out V am,out double qm))return false;
            double turn=Math.Acos(Clamp(V.Dot(s.Forward,endForward),-1,1));
            next=new State(s.UT+dt,s.R+midV*dt,s.Velocity+am*dt,s.Mass-qm*dt,
                endForward,endThrottle,turn/dt);
            return next.Valid;
        }
        // Spherical contact and reserve crossing are bracketed at substep
        // resolution. Chord check detects interior clearance crossings; no
        // claim of terrain/collider or continuous-time certified clearance.
        internal static Result Advance(Environment env,Propulsion propulsion,Limits lim,State s,Command c,double dt,
            double safeVertical=1,double safeHorizontal=1,double safeAngle=5*Math.PI/180,double safeRate=Math.PI/180)
        {
            if(env==null||propulsion==null||lim==null||!env.Valid||!lim.Valid||!s.Valid||!c.Valid||
                !Finite(dt)||dt<=0||dt>lim.Step||s.Mass<lim.ReserveMass||
                !Finite(safeVertical)||safeVertical<0||!Finite(safeHorizontal)||safeHorizontal<0||
                !Finite(safeAngle)||safeAngle<0||!Finite(safeRate)||safeRate<0)
                return new Result(Outcome.InvalidInput,s,double.NaN);
            double initial=env.Clearance(s.R);
            if(initial<=0)return Contact(env,s,initial,safeVertical,safeHorizontal,safeAngle,safeRate);
            if(!RawStep(env,propulsion,lim,s,c,dt,out State end))return new Result(Outcome.UnknownCapability,s,initial);
            V chord=end.R-s.R;
            double chord2=V.Dot(chord,chord);
            double fraction=chord2>0?Clamp(-V.Dot(s.R,chord)/chord2,0,1):0;
            double minimum=Math.Min(initial,Math.Min(env.Clearance(end.R),env.Clearance(s.R+chord*fraction)));
            bool contact=env.Clearance(end.R)<=0;
            bool fuel=end.Mass<lim.ReserveMass;
            if(!contact&&!fuel&&minimum<=0)return new Result(Outcome.UnknownCapability,s,minimum);
            if(contact||fuel)
            {
                double lo=0,hi=dt;
                State hit=end;
                for(int i=0;i<40;i++)
                {
                    double mid=(lo+hi)*.5;
                    if(!RawStep(env,propulsion,lim,s,c,mid,out State trial))return new Result(Outcome.UnknownCapability,s,minimum);
                    if(env.Clearance(trial.R)<=0||trial.Mass<lim.ReserveMass){hi=mid;hit=trial;}else lo=mid;
                }
                if(hit.Mass<lim.ReserveMass && env.Clearance(hit.R)>0)
                    return new Result(Outcome.ReserveExhausted,hit,Math.Min(initial,env.Clearance(hit.R)));
                if(hit.Mass<lim.ReserveMass-1e-6)return new Result(Outcome.ReserveExhausted,hit,Math.Min(initial,env.Clearance(hit.R)));
                return Contact(env,hit,Math.Min(initial,env.Clearance(hit.R)),safeVertical,safeHorizontal,safeAngle,safeRate);
            }
            return new Result(Outcome.Advanced,end,minimum);
        }
        private static Result Contact(Environment env,State s,double clearance,double maxV,double maxH,double maxAngle,double maxRate)
        {
            V up=s.R.Unit,w=env.Surface(s.R,s.Velocity);
            double radial=V.Dot(w,up),horizontal=(w-up*radial).Length;
            bool valid=radial<=0&&radial>=-maxV&&horizontal<=maxH&&
                V.Dot(s.Forward,up)>=Math.Cos(maxAngle)&&s.PointingRate<=maxRate&&clearance>=-1e-5;
            return new Result(valid?Outcome.ContactWithinLimits:Outcome.ContactUnsafe,s,clearance);
        }

        // Reference coast/finite correction/brake candidate. Preparation is
        // passed separately by the caller; command direction slews in model.
        // Capturing low speed above terrain is NOT a safe touchdown forecast.
        internal static Result Screen(Environment env,Propulsion propulsion,Limits lim,State initial,
            double correctionSeconds,Command correction,double ignitionDelay,double horizon,
            double captureClearance=300,double captureSpeed=15,CancellationToken cancellation=default,double correctionLead=0)
        {
            if(env==null||propulsion==null||lim==null||!env.Valid||!lim.Valid||!initial.Valid||
                initial.Mass<lim.ReserveMass||!correction.Valid||!Finite(correctionSeconds)||correctionSeconds<0||
                !Finite(correctionLead)||correctionLead<0||!Finite(ignitionDelay)||ignitionDelay<correctionLead+correctionSeconds||!Finite(horizon)||horizon<=ignitionDelay||
                horizon>1800||!Finite(captureClearance)||captureClearance<0||!Finite(captureSpeed)||captureSpeed<=0)
                return new Result(Outcome.InvalidInput,initial,double.NaN);
            State state=initial;
            double minimum=env.Clearance(initial.R),first=double.NaN;
            double correctionStart=initial.UT+correctionLead,correctionEnd=correctionStart+correctionSeconds,ignition=initial.UT+ignitionDelay,end=initial.UT+horizon;
            for(int i=0;i<100000&&state.UT<end;i++)
            {
                if(cancellation.IsCancellationRequested)return new Result(Outcome.Cancelled,state,minimum,first);
                V w=env.Surface(state.R,state.Velocity);
                if(state.UT>=ignition&&w.Length<=captureSpeed&&minimum>=captureClearance&&env.Clearance(state.R)>0)
                {
                    if(!env.Air(state.R,out double p,out _))
                        return new Result(Outcome.UnknownCapability,state,minimum,first);
                    Authority authority=propulsion.At(p);
                    if(!authority.Known)return new Result(Outcome.UnknownCapability,state,minimum,first);
                    if(authority.Force<=0||state.Mass<=lim.ReserveMass)
                        return new Result(authority.Force<=0?Outcome.NoControllableThrust:Outcome.ReserveExhausted,state,minimum,first);
                    return new Result(Outcome.CaptureEnvelope,state,minimum,first);
                }
                V retro=w.Length>1e-8?w.Unit*(-1):state.R.Unit;
                bool correcting=state.UT<correctionEnd,braking=state.UT>=ignition;
                // Before ignition, turn toward required braking direction;
                // retain engine tail. Braking throttle uses actual pointing
                // and descent gate. No static hover rejection.
                bool aiming=correcting&&state.UT<correctionStart;
                // Never inject a correction impulse during the slew.
                if(correcting&&!aiming&&V.Dot(state.Forward,correction.Direction.Unit)<Math.Cos(2*Math.PI/180))
                    return new Result(Outcome.UnknownCapability,state,minimum,first);
                Command cmd=correcting?new Command(aiming?0:correction.Throttle,correction.Direction):
                    new Command(braking&&V.Dot(w,state.R)<=0&&V.Dot(state.Forward,retro)>=.75?1:0,retro);
                double dt=Math.Min(lim.Step,end-state.UT);
                if(aiming)dt=Math.Min(dt,correctionStart-state.UT);
                else if(correcting)dt=Math.Min(dt,correctionEnd-state.UT);
                else if(!braking)dt=Math.Min(dt,ignition-state.UT);
                if(dt<=0||state.UT+dt==state.UT)break;
                Result step=Advance(env,propulsion,lim,state,cmd,dt);
                minimum=Math.Min(minimum,step.MinimumClearance);
                if(double.IsNaN(first)&&step.End.UT>state.UT&&(state.Throttle>0||step.End.Throttle>0)&&
                    env.Air(state.R,out double pressure,out _)&&propulsion.At(pressure).Force>0)
                    first=state.UT; // first powered integration interval start, not exact ignition
                state=step.End;
                if(step.Status!=Outcome.Advanced)return new Result(step.Status,state,minimum,first);
            }
            return new Result(Outcome.Horizon,state,minimum,first);
        }

        internal enum Window { CandidateFits, EntryReadinessExpired, TestedBrakingPreparationDoesNotFit, EarlierCandidateOnly,
            NoDemonstratedCandidate, Unresolved }
        // Does not command a burn. Caller supplies an independently calculated
        // absolute entry cutoff and exact tested preparation/correction plan.
        // A pair of failures cannot establish a latest safe ignition.
        internal static Window Classify(double now,double correctionAndTail,double entryCutoff,
            double testedIgnitionUT,double brakingPreparation,Result immediate,Result delayed)
        {
            if(!Finite(now)||!Finite(correctionAndTail)||correctionAndTail<0||!Finite(entryCutoff)||
                !Finite(testedIgnitionUT)||testedIgnitionUT<now||!Finite(brakingPreparation)||brakingPreparation<0)
                return Window.Unresolved;
            if(now+correctionAndTail>=entryCutoff)return Window.EntryReadinessExpired;
            if(now+correctionAndTail+brakingPreparation>testedIgnitionUT)return Window.TestedBrakingPreparationDoesNotFit;
            bool Candidate(Result r)=>r.Status==Outcome.CaptureEnvelope||r.Status==Outcome.ContactWithinLimits;
            if(Candidate(delayed))return Window.CandidateFits;
            bool unresolved=delayed.Status==Outcome.UnknownCapability||delayed.Status==Outcome.InvalidInput||
                delayed.Status==Outcome.Horizon||delayed.Status==Outcome.Cancelled||delayed.Status==Outcome.Advanced;
            if(unresolved)return Window.Unresolved;
            if(Candidate(immediate))return Window.EarlierCandidateOnly;
            if(immediate.Status==Outcome.UnknownCapability||immediate.Status==Outcome.InvalidInput||
                immediate.Status==Outcome.Horizon||immediate.Status==Outcome.Cancelled||immediate.Status==Outcome.Advanced)return Window.Unresolved;
            return Window.NoDemonstratedCandidate;
        }
    }
}
