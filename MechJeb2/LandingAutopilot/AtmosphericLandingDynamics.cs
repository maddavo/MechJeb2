using System;
using System.Threading;

namespace MuMech.Landing
{
    // 2.16.0-landing.20261126.duna-dynamics-m1.3
    // Independent SI, body-centred inertial, non-actuating reference model.
    // Explicit terrain support/corridor adapter, or synthetic sphere. No KSP APIs.
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
            internal Propulsion(bool geometryVerified,params Engine[] engines)
            { _geometryVerified=geometryVerified;_engines=engines==null?Array.Empty<Engine>():(Engine[])engines.Clone(); }
            internal Authority At(double pressurePa)
            {
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

        // Explicit inertial samples. The caller owns the fidelity and domain
        // of a supplied atmosphere/aerodynamics adapter. No silent fallback.
        internal readonly struct AirSample
        {
            internal readonly bool Known;
            internal readonly double Pressure,Density;
            internal AirSample(double pressure,double density)
            {Known=true;Pressure=pressure;Density=density;}
            internal bool Valid=>Known&&Finite(Pressure)&&Pressure>=0&&Finite(Density)&&Density>=0;
        }
        internal readonly struct AeroSample
        {
            internal readonly bool Known;
            internal readonly V Drag,Lift; // forces in N, inertial axes
            internal AeroSample(V drag,V lift){Known=true;Drag=drag;Lift=lift;}
            internal bool Valid=>Known&&Drag.Valid&&Lift.Valid;
        }
        internal readonly struct ForceSample
        {
            internal readonly bool Known;
            internal readonly double Pressure,Density,MassFlow;
            internal readonly V AirVelocity,Gravity,Thrust,Drag,Lift,Acceleration;
            internal ForceSample(double p,double rho,double flow,V air,V gravity,V thrust,V drag,V lift,double mass)
            {
                Pressure=p;Density=rho;MassFlow=flow;AirVelocity=air;Gravity=gravity;
                Thrust=thrust;Drag=drag;Lift=lift;
                Acceleration=gravity+(thrust+drag+lift)*(1/mass);
                Known=Acceleration.Valid&&Finite(flow)&&flow>=0;
            }
        }

        // Surface geometry is not the controller's altitude estimate.
        // Physical clearance includes the supplied attained-attitude support
        // offset. ControllerClearance mirrors the live altitude/deck rule.
        // MinASLTrue is separately required for the live attitude gate.
        internal readonly struct SurfaceSample
        {
            internal readonly bool Known,Landable;
            internal readonly double Clearance,ControllerClearance,MinASLTrue;
            internal readonly V Normal;
            internal SurfaceSample(double clearance,double controller,double minASLTrue,V normal,bool landable)
            {Known=true;Clearance=clearance;ControllerClearance=controller;MinASLTrue=minASLTrue;Normal=normal;Landable=landable;}
            internal bool Valid=>Known&&Finite(Clearance)&&Finite(ControllerClearance)&&
                Finite(MinASLTrue)&&Normal.Valid&&Math.Abs(Normal.Length-1)<1e-6;
        }
        internal sealed class TerrainModel
        {
            internal readonly Func<State,SurfaceSample> Sample;
            // Must supply a conservative lower clearance bound over the ENTIRE
            // numerical interval (not just endpoint samples). Caller must
            // include terrain, changing support geometry and path uncertainty.
            // NaN means not resolved. No automatic sphere fallback.
            internal readonly Func<State,State,Command,double> LowerBound;
            internal TerrainModel(Func<State,SurfaceSample> sample,Func<State,State,Command,double> lowerBound)
            {Sample=sample;LowerBound=lowerBound;}
            internal bool Valid=>Sample!=null&&LowerBound!=null;
        }
        internal sealed class Environment
        {
            internal readonly double Radius,Mu,AtmosphereTop,TerrainRadius,DragArea,BottomOffset;
            internal readonly V Omega;
            internal readonly Curve Pressure,Density;
            internal readonly Func<V,V,V,double,double,V> PassiveLift;
            internal readonly Func<double,V,AirSample> Atmosphere;
            internal readonly Func<double,V,V,V,double,double,AeroSample> Aerodynamics;
            internal readonly TerrainModel Terrain;
            // DragArea = Cd*A m², constant synthetic/passive adapter. Lift is
            // NOT assumed zero for a real vessel; unsupported unless supplied
            // via an explicitly supplied deterministic force model (r, air-relative
            // velocity, forward, pressure Pa, density kg/m³ -> force N). Null
            // means explicit zero lift for the model, not actual-vessel evidence.
            internal Environment(double radius,double mu,V omega,double atmosphereTop,
                Curve pressure,Curve density,double dragArea,double terrainRadius,double bottomOffset,
                Func<V,V,V,double,double,V> passiveLift=null,
                Func<double,V,AirSample> atmosphere=null,
                Func<double,V,V,V,double,double,AeroSample> aerodynamics=null,TerrainModel terrain=null)
            {Radius=radius;Mu=mu;Omega=omega;AtmosphereTop=atmosphereTop;Pressure=pressure;Density=density;
                DragArea=dragArea;TerrainRadius=terrainRadius;BottomOffset=bottomOffset;PassiveLift=passiveLift;
                Atmosphere=atmosphere;Aerodynamics=aerodynamics;Terrain=terrain;}
            internal bool Valid => Finite(Radius)&&Radius>0&&Finite(Mu)&&Mu>=0&&Omega.Valid&&
                Finite(AtmosphereTop)&&AtmosphereTop>=0&&Finite(TerrainRadius)&&TerrainRadius>0&&
                Finite(BottomOffset)&&BottomOffset>=0&&Finite(DragArea)&&DragArea>=0&&
                (AtmosphereTop==0||Atmosphere!=null||Pressure!=null&&Density!=null)&&
                (Aerodynamics==null||PassiveLift==null&&DragArea==0)&&(Terrain==null||Terrain.Valid);
            internal bool Air(double ut,V r,out double p,out double rho)
            {
                p=rho=0;
                if(!Finite(ut)||!r.Valid)return false;
                double h=r.Length-Radius;
                if(!Finite(h))return false;
                if(AtmosphereTop==0||h>=AtmosphereTop)return true;
                if(Atmosphere!=null)
                {
                    AirSample sample;
                    try {sample=Atmosphere(ut,r);}catch(Exception){return false;}
                    if(!sample.Valid)return false;
                    p=sample.Pressure;rho=sample.Density;
                    return true;
                }
                return AirAtAltitude(h,out p,out rho);
            }
            // A verified scalar altitude must not be rebuilt as a vector and
            // rounded below a curve endpoint. Domain remains strict.
            internal bool AirAtAltitude(double h,out double p,out double rho)
            {
                p=rho=0;
                if(!Finite(h))return false;
                if(AtmosphereTop==0||h>=AtmosphereTop)return true;
                if(Atmosphere!=null||Pressure==null||Density==null)return false;
                p=Pressure.At(h);rho=Density.At(h);
                return Finite(p)&&p>=0&&Finite(rho)&&rho>=0;
            }
            internal V Surface(V r,V v)=>v-V.Cross(Omega,r);
            internal double Clearance(V r)=>r.Length-TerrainRadius-BottomOffset;
            internal SurfaceSample SurfaceAt(State state)
            {
                if(!state.Valid)return default;
                if(Terrain!=null)
                {
                    SurfaceSample result;
                    try {result=Terrain.Sample(state);}catch(Exception){return default;}
                    return result.Valid?result:default;
                }
                double asl=state.R.Length-Radius,clearance=Clearance(state.R);
                return new SurfaceSample(clearance,Math.Min(asl,clearance),
                    Math.Min(asl,state.R.Length-TerrainRadius),state.R.Unit,true);
            }
            internal double PhysicalClearance(State state)
            {
                SurfaceSample surface=SurfaceAt(state);
                return surface.Valid?surface.Clearance:double.NaN;
            }
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
            ReserveExhausted, UnknownCapability, InvalidInput, Horizon, Cancelled, NoControllableThrust, UnknownTerrain }
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
        // Shared predicted force decomposition for propagation and independent
        // residual reporting. All vectors are inertial; gravity is m/s²,
        // thrust/drag/lift are N, flow kg/s, pressure Pa, density kg/m³.
        internal static ForceSample Forces(Environment env,Propulsion propulsion,State s)
        {
            if(env==null||propulsion==null||!env.Valid||!s.Valid)return default;
            return Evaluate(env,propulsion,s.UT,s.R,s.Velocity,s.Forward,s.Throttle,s.Mass);
        }
        private static ForceSample Evaluate(Environment env,Propulsion propulsion,double ut,V r,V v,V f,
            double throttle,double mass)
        {
            if(!r.Valid||r.Length<=0||!v.Valid||!f.Valid||!Finite(mass)||mass<=0||
                !Finite(throttle)||throttle<0||throttle>1||!env.Air(ut,r,out double p,out double density))return default;
            Authority a=propulsion.At(p);
            if(!a.Known)return default;
            V air=env.Surface(r,v),drag=default,lift=default;
            if(density>0)
            {
                if(env.Aerodynamics!=null)
                {
                    AeroSample sample;
                    try {sample=env.Aerodynamics(ut,r,air,f,p,density);}catch(Exception){return default;}
                    if(!sample.Valid)return default;
                    drag=sample.Drag;lift=sample.Lift;
                    // Passive drag cannot accelerate along the airflow.
                    if(V.Dot(drag,air)>1e-9*Math.Max(1,drag.Length*air.Length))return default;
                }
                else
                {
                    drag=air*(-.5*density*env.DragArea*air.Length);
                    if(env.PassiveLift!=null)
                    {
                        try {lift=env.PassiveLift(r,air,f,p,density);}catch(Exception){return default;}
                        if(!lift.Valid)return default;
                    }
                }
            }
            V gravity=r*(-env.Mu/(r.Length*r.Length*r.Length));
            return new ForceSample(p,density,a.Flow*throttle,air,gravity,f*(a.Force*throttle),drag,lift,mass);
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
            ForceSample a0=Evaluate(env,propulsion,s.UT,s.R,s.Velocity,s.Forward,Lag(s.Throttle,c.Throttle,tau,0),s.Mass);
            if(!a0.Known)return false;
            V midR=s.R+s.Velocity*(dt*.5),midV=s.Velocity+a0.Acceleration*(dt*.5);
            double midMass=s.Mass-a0.MassFlow*dt*.5;
            ForceSample am=Evaluate(env,propulsion,s.UT+dt*.5,midR,midV,midForward,midThrottle,midMass);
            if(!am.Known)return false;
            double turn=Math.Acos(Clamp(V.Dot(s.Forward,endForward),-1,1));
            next=new State(s.UT+dt,s.R+midV*dt,s.Velocity+am.Acceleration*dt,s.Mass-am.MassFlow*dt,
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
            if(env.Terrain!=null)
            {
                int budget=512;
                return AdvanceTerrain(env,propulsion,lim,s,c,dt,safeVertical,safeHorizontal,safeAngle,safeRate,ref budget);
            }
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
        // Resolve earliest contact/reserve event left-to-right. Unknown
        // corridor or support geometry may never become an accepted landing.
        // The bound is supplied by an immutable terrain/geometry adapter;
        // endpoint samples alone cannot certify a clear swept path.
        private static Result AdvanceTerrain(Environment env,Propulsion propulsion,Limits lim,State s,Command c,double dt,
            double maxV,double maxH,double maxAngle,double maxRate,ref int budget)
        {
            SurfaceSample a=env.SurfaceAt(s);
            if(!a.Valid||budget--<=0)return new Result(Outcome.UnknownTerrain,s,double.NaN);
            if(a.Clearance<=0)return TerrainContact(env,s,a,maxV,maxH,maxAngle,maxRate);
            if(!RawStep(env,propulsion,lim,s,c,dt,out State end))
            {
                // Subdivide instead of extrapolating an atmosphere curve
                // beyond contact or jumping past a propellant boundary.
                if(dt<=1e-7||s.UT+dt*.5==s.UT)return new Result(Outcome.UnknownCapability,s,a.Clearance);
                return SplitTerrain(env,propulsion,lim,s,c,dt,maxV,maxH,maxAngle,maxRate,ref budget);
            }
            SurfaceSample z=env.SurfaceAt(end);
            double lower=double.NaN;
            if(z.Valid)
            {
                try{lower=env.Terrain.LowerBound(s,end,c);}catch(Exception){}
                if(Finite(lower))
                {
                    double endpointMinimum=Math.Min(a.Clearance,z.Clearance);
                    if(lower>endpointMinimum+1e-8)
                        return new Result(Outcome.UnknownTerrain,s,double.NaN);
                    // Even tolerated rounding cannot turn a sampled contact
                    // into positive clearance.
                    lower=Math.Min(lower,endpointMinimum);
                }
            }
            bool bounded=z.Valid&&Finite(lower);
            if(bounded&&lower>0&&end.Mass>=lim.ReserveMass)
                return new Result(Outcome.Advanced,end,lower);
            if(dt<=1e-7||s.UT+dt*.5==s.UT)
            {
                // Earliest event resolved only to this bracket; if ambiguous
                // reserve/contact order, fail closed. An unresolved interior
                // crossing is unknown, not safe endpoint contact.
                if(!bounded)return new Result(Outcome.UnknownTerrain,s,double.NaN);
                if(end.Mass<lim.ReserveMass)
                    return z.Clearance<=0?new Result(Outcome.UnknownTerrain,s,lower):
                        lower>0?new Result(Outcome.ReserveExhausted,end,lower):
                        new Result(Outcome.UnknownTerrain,s,lower);
                if(z.Clearance<=0)return TerrainContact(env,end,z,maxV,maxH,maxAngle,maxRate);
                return new Result(Outcome.UnknownTerrain,s,lower);
            }
            return SplitTerrain(env,propulsion,lim,s,c,dt,maxV,maxH,maxAngle,maxRate,ref budget);
        }
        private static Result SplitTerrain(Environment env,Propulsion propulsion,Limits lim,State s,Command c,double dt,
            double maxV,double maxH,double maxAngle,double maxRate,ref int budget)
        {
            double half=dt*.5;
            Result left=AdvanceTerrain(env,propulsion,lim,s,c,half,maxV,maxH,maxAngle,maxRate,ref budget);
            if(left.Status!=Outcome.Advanced)return left;
            double remaining=(s.UT+dt)-left.End.UT;
            if(remaining<=0)return left;
            Result right=AdvanceTerrain(env,propulsion,lim,left.End,c,remaining,maxV,maxH,maxAngle,maxRate,ref budget);
            return new Result(right.Status,right.End,Math.Min(left.MinimumClearance,right.MinimumClearance));
        }
        private static Result TerrainContact(Environment env,State s,SurfaceSample surface,double maxV,double maxH,double maxAngle,double maxRate)
        {
            V w=env.Surface(s.R,s.Velocity);
            double normal=V.Dot(w,surface.Normal),tangent=(w-surface.Normal*normal).Length;
            bool valid=surface.Landable&&normal<=0&&normal>=-maxV&&tangent<=maxH&&
                V.Dot(s.Forward,surface.Normal)>=Math.Cos(maxAngle)&&s.PointingRate<=maxRate&&surface.Clearance>=-1e-5;
            return new Result(valid?Outcome.ContactWithinLimits:Outcome.ContactUnsafe,s,surface.Clearance);
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
            double captureClearance=300,double captureSpeed=15,CancellationToken cancellation=default)
        {
            if(env==null||propulsion==null||lim==null||!env.Valid||!lim.Valid||!initial.Valid||
                initial.Mass<lim.ReserveMass||!correction.Valid||!Finite(correctionSeconds)||correctionSeconds<0||
                !Finite(ignitionDelay)||ignitionDelay<correctionSeconds||!Finite(horizon)||horizon<=ignitionDelay||
                horizon>1800||!Finite(captureClearance)||captureClearance<0||!Finite(captureSpeed)||captureSpeed<=0)
                return new Result(Outcome.InvalidInput,initial,double.NaN);
            State state=initial;
            double minimum=env.PhysicalClearance(initial),first=double.NaN;
            if(!Finite(minimum))return new Result(Outcome.UnknownTerrain,initial,double.NaN);
            double correctionEnd=initial.UT+correctionSeconds,ignition=initial.UT+ignitionDelay,end=initial.UT+horizon;
            for(int i=0;i<100000&&state.UT<end;i++)
            {
                if(cancellation.IsCancellationRequested)return new Result(Outcome.Cancelled,state,minimum,first);
                V w=env.Surface(state.R,state.Velocity);
                if(state.UT>=ignition&&w.Length<=captureSpeed&&minimum>=captureClearance&&env.PhysicalClearance(state)>0)
                {
                    if(!env.Air(state.UT,state.R,out double p,out _))
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
                Command cmd=correcting?correction:new Command(braking&&V.Dot(w,state.R)<=0&&V.Dot(state.Forward,retro)>=.75?1:0,retro);
                double dt=Math.Min(lim.Step,end-state.UT);
                if(correcting)dt=Math.Min(dt,correctionEnd-state.UT);
                else if(!braking)dt=Math.Min(dt,ignition-state.UT);
                if(dt<=0||state.UT+dt==state.UT)break;
                Result step=Advance(env,propulsion,lim,state,cmd,dt);
                minimum=Math.Min(minimum,step.MinimumClearance);
                if(double.IsNaN(first)&&step.End.UT>state.UT&&(state.Throttle>0||step.End.Throttle>0)&&
                    env.Air(state.UT,state.R,out double pressure,out _)&&propulsion.At(pressure).Force>0)
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
            bool unresolved=delayed.Status==Outcome.UnknownCapability||delayed.Status==Outcome.UnknownTerrain||delayed.Status==Outcome.InvalidInput||
                delayed.Status==Outcome.Horizon||delayed.Status==Outcome.Cancelled||delayed.Status==Outcome.Advanced;
            if(unresolved)return Window.Unresolved;
            if(Candidate(immediate))return Window.EarlierCandidateOnly;
            if(immediate.Status==Outcome.UnknownCapability||immediate.Status==Outcome.UnknownTerrain||immediate.Status==Outcome.InvalidInput||
                immediate.Status==Outcome.Horizon||immediate.Status==Outcome.Cancelled||immediate.Status==Outcome.Advanced)return Window.Unresolved;
            return Window.NoDemonstratedCandidate;
        }
    }
}
