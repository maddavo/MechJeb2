using System;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // 2.16.0-landing.20261126.duna-dynamics-m1.3
    // Offline source-derived KHV/FinalDescent reference controller.
    // Not a new live controller; only the explicitly modelled actuator regime.
    internal static class AtmosphericTerminalReplay
    {
        internal enum Phase { HorizontalKill, FinalDescent }
        internal enum Mode { Off, Direct, KeepSurface, KeepVertical }
        internal readonly struct Memory
        {
            internal readonly Phase Phase;
            internal readonly Mode Mode;
            internal readonly double PreviousError,Integral,PreviousThrottle,LastApplied;
            internal Memory(Phase phase,Mode mode,double error,double integral,double previous,double applied)
            {Phase=phase;Mode=mode;PreviousError=error;Integral=integral;PreviousThrottle=previous;LastApplied=applied;}
            internal bool Valid=>Enum.IsDefined(typeof(Phase),Phase)&&Enum.IsDefined(typeof(Mode),Mode)&&
                D.Finite(PreviousError)&&D.Finite(Integral)&&D.Finite(PreviousThrottle)&&PreviousThrottle>=0&&PreviousThrottle<=1&&
                D.Finite(LastApplied)&&LastApplied>=0&&LastApplied<=1;
        }
        internal readonly struct ControllerAuthority
        {
            internal readonly bool Known,UseTorqueThrottle;
            internal readonly double RawAcceleration,LimitedAcceleration,CommandCap;
            // These are the LIVE controller denominators/limit, not a
            // substitution of sustained modeled thrust for raw authority.
            internal ControllerAuthority(double raw,double limited,double cap,bool torqueThrottle)
            {Known=true;RawAcceleration=raw;LimitedAcceleration=limited;CommandCap=cap;UseTorqueThrottle=torqueThrottle;}
            internal bool Valid=>Known&&D.Finite(RawAcceleration)&&RawAcceleration>0&&
                D.Finite(LimitedAcceleration)&&LimitedAcceleration>=0&&LimitedAcceleration<=RawAcceleration&&
                D.Finite(CommandCap)&&CommandCap>=0&&CommandCap<=1;
        }
        internal sealed class Settings
        {
            internal readonly double Tick,TouchdownSpeed,GeeASL,SmoothingSeconds;
            internal readonly Func<D.State,ControllerAuthority> Authority;
            // Predicted controller attitude error, degrees, from an explicitly
            // supplied attitude model. Not necessarily forward-axis angle.
            internal readonly Func<D.State,D.V,double> AttitudeError;
            // Actual terrain/deck radius for the gravity-turn estimate.
            // Receives UT and estimated landing position in inertial axes.
            internal readonly Func<double,D.V,double> EstimatedTerrainRadius;
            internal Settings(double tick,double touchdown,double gee,double smoothing,
                Func<D.State,ControllerAuthority> authority,Func<D.State,D.V,double> attitudeError,
                Func<double,D.V,double> estimatedTerrain)
            {Tick=tick;TouchdownSpeed=touchdown;GeeASL=gee;SmoothingSeconds=smoothing;
                Authority=authority;AttitudeError=attitudeError;EstimatedTerrainRadius=estimatedTerrain;}
            internal bool Valid=>D.Finite(Tick)&&Tick>0&&Tick<=1&&D.Finite(TouchdownSpeed)&&TouchdownSpeed>0&&
                D.Finite(GeeASL)&&GeeASL>=0&&D.Finite(SmoothingSeconds)&&SmoothingSeconds>=0&&
                Authority!=null&&AttitudeError!=null;
        }
        internal readonly struct Decision
        {
            internal readonly bool Known;
            internal readonly D.Command Command;
            internal readonly Memory Next;
            internal readonly double Setpoint,RequestedThrottle;
            internal Decision(D.Command command,Memory next,double setpoint,double requested)
            {Known=true;Command=command;Next=next;Setpoint=setpoint;RequestedThrottle=requested;}
        }
        private static double Clamp(double x,double lo,double hi)=>Math.Max(lo,Math.Min(hi,x));
        // V1LandingControlPolicy.FinalDescentSpeed, non-airless branch.
        internal static double FinalSpeed(double clearance,double limited,double localGravity,double touchdown)
        {
            double net=Math.Max(0,limited-localGravity);
            double distanceSpeed=.90*Math.Sqrt(2*net*Math.Max(0,clearance));
            double cap=Math.Max(5,Math.Min(25,4*localGravity));
            return Math.Min(-touchdown,-Math.Min(distanceSpeed,cap));
        }
        // GravityTurnDescentSpeedPolicy: reproduce the legacy heuristic
        // only as a controller setpoint. NOT the physical propagation model.
        internal static double GravityTurnSpeed(D.V position,D.V velocity,double terrainRadius,double g,double thrust)
        {
            if(!position.Valid||position.Length<=0||!velocity.Valid||!D.Finite(terrainRadius)||terrainRadius<=0||
                !D.Finite(g)||g<0||!D.Finite(thrust)||thrust<=g)return double.NaN;
            double distance=position.Length-terrainRadius;
            double lo=0,hi=1.1*velocity.Length;
            for(int iteration=0;iteration<128&&hi-lo>.1;iteration++)
            {
                double mid=(lo+hi)*.5;
                D.V x=position,v=velocity.Unit*mid;
                for(int i=0;i<10;i++)
                {
                    double dt=v.Length/thrust/(10-i);
                    D.V next=v+(x.Unit*(-g)+v.Unit*(-thrust))*dt;
                    x+=(v+next)*(dt*.5);v=next;
                }
                double fall=position.Length-x.Length+v.Length*v.Length/(2*(thrust-g));
                if(!D.Finite(fall))return double.NaN;
                if(fall<distance)lo=mid;else hi=mid;
            }
            return .95*(lo+hi)*.5;
        }
        internal static Decision Tick(D.Environment env,Settings settings,D.State state,Memory memory)
        {
            if(env==null||settings==null||!env.Valid||!settings.Valid||!state.Valid||!memory.Valid)return default;
            D.SurfaceSample surface=env.SurfaceAt(state);
            if(!surface.Valid)return default;
            ControllerAuthority authority;
            try{authority=settings.Authority(state);}catch(Exception){return default;}
            if(!authority.Valid)return default;
            D.V up=state.R.Unit,w=env.Surface(state.R,state.Velocity);
            double vertical=D.V.Dot(w,up),speed=w.Length;
            D.V horizontal=w-up*vertical;
            double g=env.Mu/(state.R.Length*state.R.Length);
            Mode mode=memory.Mode;Phase phase=memory.Phase;
            double requested=0,setpoint=0,error=memory.PreviousError,integral=memory.Integral,
                previous=memory.PreviousThrottle;
            bool verticalToDirect=false;
            D.V direction=up;
            if(phase==Phase.HorizontalKill)
            {
                if(horizontal.Length<=1)
                {
                    // KHV.RequestActiveThrottle(0) does not overwrite
                    // _transPrevThrust. Preserve that distinct PID memory.
                    phase=Phase.FinalDescent;
                }
                else
                {
                    direction=(up-horizontal.Unit*.2).Unit;
                    double aligned=D.V.Dot(state.Forward,up)*authority.RawAcceleration;
                    requested=aligned>0?Clamp((g-vertical)/aligned,0,1):0;
                }
            }
            else
            {
                if(authority.LimitedAcceleration<g)
                {mode=Mode.KeepVertical;setpoint=0;}
                else if(surface.ControllerClearance>300)
                {
                    bool ascending=speed>5&&D.V.Dot(w.Unit,up)>Math.Cos(80*Math.PI/180);
                    direction=ascending||speed<1e-9?up:w.Unit*(-1);
                    bool misaligned=speed>5&&D.V.Dot(state.Forward,direction)<Math.Cos(45*Math.PI/180);
                    if(ascending||misaligned){mode=Mode.Direct;setpoint=0;}
                    else
                    {
                        mode=Mode.KeepSurface;
                        if(settings.EstimatedTerrainRadius==null)return default;
                        double radius;
                        try{radius=settings.EstimatedTerrainRadius(state.UT,state.R+w.Unit*
                            (speed*speed/(2*authority.LimitedAcceleration)));}catch(Exception){return default;}
                        setpoint=GravityTurnSpeed(state.R,w,radius,settings.GeeASL*9.81,authority.LimitedAcceleration);
                        if(!D.Finite(setpoint))return default;
                    }
                }
                else
                {mode=Mode.KeepVertical;setpoint=FinalSpeed(surface.ControllerClearance,authority.LimitedAcceleration,g,settings.TouchdownSpeed);}
                if(mode==Mode.KeepVertical)
                {
                    direction=(up*Math.Max(Math.Abs(vertical),20*settings.GeeASL)-horizontal).Unit;
                    if(surface.MinASLTrue>5000&&horizontal.Length>Math.Max(Math.Abs(vertical),100*settings.GeeASL)*2)
                    {mode=Mode.Direct;setpoint=100;direction=horizontal.Unit*(-1);verticalToDirect=true;}
                    if(direction.Length==0)direction=up;
                }
                if(mode!=memory.Mode){error=0;integral=0;}
                // Cumulative PID uses previous requested speed throttle, not
                // the achieved engine throttle. Source: PIDController.Compute.
                double controlled=mode==Mode.KeepSurface?speed:mode==Mode.KeepVertical||verticalToDirect?vertical:0;
                double e=(setpoint-controlled)/authority.RawAcceleration;
                if(mode==Mode.KeepSurface&&D.V.Dot(state.Forward,w)<0)e=-e;
                integral+=e*settings.Tick;
                double action=.05*e+.000001*integral+.05*(e-error)/settings.Tick;
                error=e;
                if(mode==Mode.Direct)requested=Clamp(setpoint/100,0,1);
                else if(mode==Mode.KeepSurface)requested=Clamp(previous+action,0,1);
                else
                {
                    double angle;
                    try{angle=settings.AttitudeError(state,direction);}catch(Exception){return default;}
                    if(!D.Finite(angle)||angle<0||angle>180)return default;
                    bool ready=angle<2||(surface.MinASLTrue<1000&&angle<90);
                    requested=ready?Clamp(previous+action,0,1):authority.UseTorqueThrottle?.1:0;
                }
                previous=requested;
            }
            if(!D.Finite(requested)||!D.Finite(error)||!D.Finite(integral)||!direction.Valid||direction.Length==0)return default;
            double applied=Math.Min(requested,authority.CommandCap);
            if(settings.SmoothingSeconds>0)
            {
                double slew=settings.Tick/settings.SmoothingSeconds;
                applied=Clamp(applied,memory.LastApplied-slew,memory.LastApplied+slew);
            }
            applied=Clamp(applied,0,1);
            var nextMemory=new Memory(phase,mode,error,integral,previous,applied);
            return new Decision(new D.Command(applied,direction),nextMemory,setpoint,requested);
        }
        internal readonly struct Report
        {
            internal readonly D.Outcome Outcome;
            internal readonly D.State End;
            internal readonly Memory Memory;
            internal readonly double MinimumClearance;
            internal readonly int Ticks,IntegrationSteps;
            internal Report(D.Outcome outcome,D.State end,Memory memory,double clearance,int ticks,int steps)
            {Outcome=outcome;End=end;Memory=memory;MinimumClearance=clearance;Ticks=ticks;IntegrationSteps=steps;}
        }
        internal static Report Run(D.Environment env,D.Propulsion propulsion,D.Limits limits,Settings settings,
            D.State initial,Memory memory,int maxTicks,double safeVertical,double safeHorizontal,double safeAngle,double safeRate,
            CancellationToken cancellation=default,int maxIntegrationSteps=1000000,
            Action<D.State,D.Command,double> record=null)
        {
            D.State state=initial;
            double minimum=env==null?double.NaN:env.PhysicalClearance(state);
            int ticks=0,steps=0;
            Report Finish(D.Outcome outcome)=>new Report(outcome,state,memory,minimum,ticks,steps);
            if(env==null||propulsion==null||limits==null||settings==null||!env.Valid||!limits.Valid||!settings.Valid||
                !initial.Valid||!memory.Valid||initial.Mass<limits.ReserveMass||maxTicks<=0||maxTicks>1000000||
                maxIntegrationSteps<=0||maxIntegrationSteps>1000000||
                !D.Finite(safeVertical)||safeVertical<0||!D.Finite(safeHorizontal)||safeHorizontal<0||
                !D.Finite(safeAngle)||safeAngle<0||!D.Finite(safeRate)||safeRate<0)
                return Finish(D.Outcome.InvalidInput);
            if(!D.Finite(minimum))return Finish(D.Outcome.UnknownTerrain);
            // Tick the legacy controller at its own cadence; integration
            // substeps do NOT execute extra PID updates.
            while(ticks<maxTicks)
            {
                if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                Decision decision=Tick(env,settings,state,memory);
                if(!decision.Known)return Finish(D.Outcome.UnknownCapability);
                memory=decision.Next;ticks++;
                double endUT=state.UT+settings.Tick;
                record?.Invoke(state,decision.Command,endUT);
                while(state.UT<endUT)
                {
                    if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                    if(steps>=maxIntegrationSteps)return Finish(D.Outcome.Horizon);
                    double dt=Math.Min(limits.Step,endUT-state.UT);
                    if(dt<=0||state.UT+dt<=state.UT)return Finish(D.Outcome.InvalidInput);
                    D.Result step=D.Advance(env,propulsion,limits,state,decision.Command,dt,safeVertical,safeHorizontal,safeAngle,safeRate);
                    steps++;
                    minimum=Math.Min(minimum,step.MinimumClearance);state=step.End;
                    if(step.Status!=D.Outcome.Advanced)return Finish(step.Status);
                }
            }
            return Finish(D.Outcome.Horizon);
        }
    }
}
