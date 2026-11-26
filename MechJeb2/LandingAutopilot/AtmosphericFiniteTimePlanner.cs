using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;
using R=MuMech.Landing.AtmosphericLandingReplay;

namespace MuMech.Landing
{
    // M3 alternative: bounded nonlinear shooting of finite-time feedback
    // trajectories. Local minimum-energy position/velocity law is a seed,
    // not an SOCP and not proof of global optimality. No KSP calls.
    internal static class AtmosphericFiniteTimePlanner
    {
        internal readonly struct Profile
        {
            internal readonly double Coast,Duration;
            internal Profile(double coast,double duration){Coast=coast;Duration=duration;}
        }
        internal sealed class Settings
        {
            internal readonly P.Settings Common;
            internal readonly double TargetSurfaceRadius,ApproachHeight,ApproachSpeed,
                MinPoweredSeconds,MaxPoweredSeconds,ControlTick,TerminalHorizontal,TerminalVertical;
            internal readonly int CoastIntervals,DurationIntervals,Refinements;
            internal Settings(P.Settings common,double targetRadius,double approachHeight=15,double approachSpeed=2,
                double minPowered=10,double maxPowered=600,double controlTick=.5,int coastIntervals=6,
                int durationIntervals=8,int refinements=3,double terminalHorizontal=1,double terminalVertical=4)
            {
                Common=common;TargetSurfaceRadius=targetRadius;ApproachHeight=approachHeight;ApproachSpeed=approachSpeed;
                MinPoweredSeconds=minPowered;MaxPoweredSeconds=maxPowered;ControlTick=controlTick;
                CoastIntervals=coastIntervals;DurationIntervals=durationIntervals;Refinements=refinements;
                TerminalHorizontal=terminalHorizontal;TerminalVertical=terminalVertical;
            }
            internal bool Valid=>Common!=null&&Common.Valid&&D.Finite(TargetSurfaceRadius)&&TargetSurfaceRadius>0&&
                D.Finite(ApproachHeight)&&ApproachHeight>0&&ApproachHeight<=300&&D.Finite(ApproachSpeed)&&ApproachSpeed>0&&
                D.Finite(MinPoweredSeconds)&&MinPoweredSeconds>0&&D.Finite(MaxPoweredSeconds)&&
                MaxPoweredSeconds>=MinPoweredSeconds&&MaxPoweredSeconds<=Common.Horizon&&
                D.Finite(ControlTick)&&ControlTick>0&&ControlTick<=1&&
                CoastIntervals>=1&&CoastIntervals<=32&&DurationIntervals>=1&&DurationIntervals<=32&&
                Refinements>=0&&Refinements<=8&&D.Finite(TerminalHorizontal)&&TerminalHorizontal>=0&&
                D.Finite(TerminalVertical)&&TerminalVertical>=ApproachSpeed;
        }
        internal readonly struct Point
        {
            internal readonly D.State State;
            internal readonly D.Command Command;
            internal readonly double ValidUntil;
            internal Point(D.State state,D.Command command,double until){State=state;Command=command;ValidUntil=until;}
        }
        internal sealed class Candidate
        {
            internal readonly Profile Profile;
            internal readonly P.Candidate Result;
            internal readonly double RequestedPeakAcceleration,MaxSaturation,TerminalPositionError;
            internal readonly IReadOnlyList<Point> Reference;
            internal readonly IReadOnlyList<R.Segment> Commands;
            internal Candidate(Profile profile,P.Candidate result,double peak,double saturation,double positionError,
                List<Point> reference,List<R.Segment> commands)
            {
                Profile=profile;Result=result;RequestedPeakAcceleration=peak;MaxSaturation=saturation;
                TerminalPositionError=positionError;Reference=Array.AsReadOnly(reference.ToArray());
                Commands=Array.AsReadOnly(commands.ToArray());
            }
        }
        internal sealed class Report
        {
            internal readonly P.SearchStatus Status;
            internal readonly Candidate Best;
            internal readonly int Attempts,Steps,Feasible,Unresolved;
            internal readonly double ElapsedMilliseconds;
            internal Report(P.SearchStatus status,Candidate best,int attempts,int steps,int feasible,int unresolved,double elapsed)
            {Status=status;Best=best;Attempts=attempts;Steps=steps;Feasible=feasible;Unresolved=unresolved;ElapsedMilliseconds=elapsed;}
        }
        // Minimum integral of squared acceleration for double-integrator
        // endpoint position AND velocity, evaluated at start of remaining arc.
        internal static D.V EndpointAcceleration(D.V position,D.V velocity,D.V target,D.V terminalVelocity,double t)
        {
            if(!position.Valid||!velocity.Valid||!target.Valid||!terminalVelocity.Valid||!D.Finite(t)||t<=0)
                return new D.V(double.NaN,0,0);
            return (target-position-velocity*t)*(6/(t*t))-(terminalVelocity-velocity)*(2/t);
        }
        private static D.Command PoweredCommand(D.Environment env,D.Propulsion propulsion,D.State state,
            Settings settings,D.V target,D.V terminalVelocity,double time,out double requested,out double saturation)
        {
            requested=saturation=double.NaN;
            var force=D.Forces(env,propulsion,state);
            if(!force.Known)return default;
            D.Authority authority=propulsion.At(force.Pressure);
            if(!authority.Known||!(authority.Force>0))return default;
            D.V need=EndpointAcceleration(state.R,state.Velocity,target,terminalVelocity,time)-
                force.Gravity-(force.Drag+force.Lift)*(1/state.Mass);
            if(!need.Valid)return default;
            requested=need.Length;
            double demand=requested*state.Mass/authority.Force;
            if(!D.Finite(demand))return default;
            saturation=Math.Max(0,demand-settings.Common.CommandCap);
            D.V direction=requested>1e-9?need.Unit:state.R.Unit;
            // Physical integration uses attained attitude, not requested
            // direction. Do not credit thrust while pointed away.
            double throttle=D.V.Dot(state.Forward,direction)>=.75?
                Math.Min(settings.Common.CommandCap,demand):0;
            return new D.Command(throttle,direction);
        }
        // Terminal velocity feedback with explicit gravity/drag feed-forward.
        // Braking envelope is clearance-dependent all the way to contact,
        // not a fixed 15 m/s hold. This is a candidate control law, whose
        // actual attainable direction/throttle are independently propagated.
        internal static D.Command TerminalCommand(D.Environment env,D.Propulsion propulsion,D.State state,
            Settings settings,D.SurfaceSample surface)
        {
            var force=D.Forces(env,propulsion,state);
            if(!force.Known||!surface.Valid||!settings.Valid)return default;
            D.Authority authority=propulsion.At(force.Pressure);
            if(!authority.Known||!(authority.Force>0))return default;
            D.V up=surface.Normal,w=env.Surface(state.R,state.Velocity);
            double vertical=D.V.Dot(w,up);
            D.V horizontal=w-up*vertical;
            D.V passive=force.Gravity+(force.Drag+force.Lift)*(1/state.Mass)-D.V.Cross(env.Omega,state.Velocity);
            double raw=authority.Force/state.Mass;
            double available=Math.Max(0,raw*settings.Common.CommandCap+D.V.Dot(passive,up));
            double brake=Math.Min(.8,.25*available);
            double touch=settings.Common.Terminal.TouchdownSpeed;
            double envelope=Math.Sqrt(touch*touch+2*brake*Math.Max(0,surface.Clearance));
            double speed=Math.Min(Math.Max(touch,settings.TerminalVertical),envelope);
            double ff=envelope<settings.TerminalVertical?-brake*vertical/Math.Max(touch,speed):0;
            D.V desired=up*(ff+1.5*(-speed-vertical))-horizontal*.8-passive;
            double axial=D.V.Dot(desired,up);
            if(axial<=0)return new D.Command(0,up);
            // Finish upright. Limit lateral demand by available tilt;
            // never remove the vertical braking demand to correct position.
            D.V lateral=desired-up*axial;
            double tilt=Math.Min(Math.PI/6,Math.PI/60+Math.Max(0,surface.Clearance)*Math.PI/600);
            double lateralCap=axial*Math.Tan(tilt);
            if(lateral.Length>lateralCap)lateral=lateral.Unit*lateralCap;
            desired=up*axial+lateral;
            double demand=desired.Length/raw;
            if(!desired.Valid||!D.Finite(demand))return default;
            return new D.Command(D.V.Dot(state.Forward,desired.Unit)>=.75?
                Math.Min(settings.Common.CommandCap,demand):0,desired.Unit);
        }

        internal static Candidate Evaluate(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,Profile profile,int budget,bool record=false,CancellationToken cancellation=default)
        {
            var reference=new List<Point>();var commands=new List<R.Segment>();
            D.State state=initial;
            double min=double.NaN,first=double.NaN,terminal=double.NaN,peak=0,saturation=0,positionError=double.NaN;
            int steps=0;
            Candidate Finish(D.Outcome outcome)
            {
                double miss=outcome==D.Outcome.ContactWithinLimits?P.Miss(env,initial,state,settings.Common.TargetAtInitialUT):double.NaN;
                var p=new P.Profile(profile.Coast,0,new D.Command(0,initial.Forward));
                return new Candidate(profile,new P.Candidate(p,outcome,state,miss,min,first,terminal,steps),
                    peak,saturation,positionError,reference,commands);
            }
            if(env==null||propulsion==null||limits==null||settings==null||!env.Valid||!limits.Valid||
                !settings.Valid||!initial.Valid||initial.Mass<limits.ReserveMass||budget<1||budget>1000000||
                !D.Finite(profile.Coast)||profile.Coast<0||profile.Coast>settings.Common.MaxIgnitionDelay||
                !D.Finite(profile.Duration)||profile.Duration<settings.MinPoweredSeconds||profile.Duration>settings.MaxPoweredSeconds||
                profile.Coast+profile.Duration>settings.Common.Horizon)return Finish(D.Outcome.InvalidInput);
            var common=settings.Common;
            double ignition=initial.UT+profile.Coast, arrival=ignition+profile.Duration, end=initial.UT+common.Horizon;
            if(ignition<initial.UT||arrival<=ignition||end<arrival)return Finish(D.Outcome.InvalidInput);
            D.V upAtArrival=P.RotateTarget(common.TargetAtInitialUT,env.Omega,arrival-initial.UT).Unit;
            D.V target=upAtArrival*(settings.TargetSurfaceRadius+settings.ApproachHeight);
            D.V terminalVelocity=D.V.Cross(env.Omega,target)-upAtArrival*settings.ApproachSpeed;
            min=env.PhysicalClearance(initial);
            if(!D.Finite(min))return Finish(D.Outcome.UnknownTerrain);
            bool terminalMode=false;
            while(state.UT<end)
            {
                if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                if(steps>=budget)return Finish(D.Outcome.Horizon);
                D.SurfaceSample surface=env.SurfaceAt(state);
                if(!surface.Valid)return Finish(D.Outcome.UnknownTerrain);
                D.V w=env.Surface(state.R,state.Velocity);
                double vertical=D.V.Dot(w,surface.Normal);
                double horizontal=(w-surface.Normal*vertical).Length;
                // Terminal transfer only near actual modelled terrain and
                // with independently checked low horizontal/vertical speed.
                if(!terminalMode&&state.UT>=ignition&&surface.Clearance>0&&
                    surface.Clearance<=Math.Min(300,settings.ApproachHeight*2)&&vertical<=0&&
                    vertical>=-settings.TerminalVertical&&horizontal<=settings.TerminalHorizontal&&
                    D.V.Dot(state.Forward,surface.Normal)>=Math.Cos(Math.PI/6))
                {
                    terminalMode=true;terminal=state.UT;
                    positionError=P.Miss(env,initial,state,common.TargetAtInitialUT);
                }
                D.Command command;double interval;
                if(terminalMode)
                {
                    command=TerminalCommand(env,propulsion,state,settings,surface);
                    if(!command.Valid)return Finish(D.Outcome.UnknownCapability);
                    interval=common.Terminal.Tick;
                }
                else
                {
                    double remaining=arrival-state.UT;
                    if(remaining<=0)return Finish(D.Outcome.Horizon);
                    var powered=PoweredCommand(env,propulsion,state,settings,target,terminalVelocity,
                        Math.Max(settings.ControlTick,remaining),out double request,out double sat);
                    if(!powered.Valid)return Finish(D.Outcome.UnknownCapability);
                    peak=Math.Max(peak,request);saturation=Math.Max(saturation,sat);
                    bool coast=state.UT<ignition;
                    command=coast?new D.Command(0,powered.Direction):powered;
                    interval=Math.Min(settings.ControlTick,coast?ignition-state.UT:remaining);
                }
                double boundary=Math.Min(end,state.UT+interval);
                if(boundary<=state.UT)return Finish(D.Outcome.InvalidInput);
                // Record commanded intervals, including the complete last
                // numerical interval; the contact event may stop inside it.
                if(record){reference.Add(new Point(state,command,boundary));commands.Add(new R.Segment(state.UT,boundary,command));}
                while(state.UT<boundary)
                {
                    if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                    if(steps>=budget)return Finish(D.Outcome.Horizon);
                    double dt=Math.Min(limits.Step,boundary-state.UT);
                    if(dt<=0||state.UT+dt<=state.UT)return Finish(D.Outcome.InvalidInput);
                    var step=D.Advance(env,propulsion,limits,state,command,dt,common.SafeVertical,
                        common.SafeHorizontal,common.SafeAngle,common.SafeRate);
                    steps++;
                    if(double.IsNaN(first)&&step.End.UT>state.UT&&(state.Throttle>0||step.End.Throttle>0)&&
                        env.Air(state.UT,state.R,out double pressure,out _)&&propulsion.At(pressure).Force>0)
                        first=state.UT;
                    state=step.End;min=Math.Min(min,step.MinimumClearance);
                    if(step.Status!=D.Outcome.Advanced)return Finish(step.Status);
                }
            }
            return Finish(D.Outcome.Horizon);
        }
        // Evaluate complete coarse lattice, then refine around best feasible
        // seed. No early assumption of monotonicity. Budget counts integrations,
        // including reconstruction and independent validation, not wall time.
        internal static Report Search(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,CancellationToken cancellation=default)
        {
            var clock=Stopwatch.StartNew();Candidate best=null;
            int attempts=0,steps=0,feasible=0,unresolved=0;
            var visited=new HashSet<(double,double)>();
            Report Finish(P.SearchStatus status)=>new Report(status,best,attempts,steps,feasible,unresolved,clock.Elapsed.TotalMilliseconds);
            if(settings==null||!settings.Valid||env==null||!env.Valid||limits==null||!limits.Valid||
                propulsion==null||!initial.Valid||initial.Mass<limits.ReserveMass)return Finish(P.SearchStatus.InvalidInput);
            var common=settings.Common;
            bool exhausted=false;
            void Try(double coast,double duration)
            {
                if(exhausted||cancellation.IsCancellationRequested||coast<0||coast>common.MaxIgnitionDelay||
                    duration<settings.MinPoweredSeconds||duration>settings.MaxPoweredSeconds||
                    coast+duration>common.Horizon||!visited.Add((coast,duration)))return;
                if(attempts>=common.MaxCandidates||steps>=common.MaxSteps){exhausted=true;return;}
                Candidate c=Evaluate(env,propulsion,limits,initial,settings,new Profile(coast,duration),common.MaxSteps-steps,false,cancellation);
                attempts++;steps+=c.Result.Steps;
                if(c.Result.Feasible)
                {
                    feasible++;
                    if(best==null||P.Better(c.Result,best.Result,common.TargetTolerance))best=c;
                }
                else if(c.Result.Outcome==D.Outcome.UnknownCapability||c.Result.Outcome==D.Outcome.UnknownTerrain||
                    c.Result.Outcome==D.Outcome.Horizon||c.Result.Outcome==D.Outcome.Cancelled||
                    c.Result.Outcome==D.Outcome.InvalidInput)unresolved++;
            }
            double dc=common.MaxIgnitionDelay/settings.CoastIntervals,
                dt=(settings.MaxPoweredSeconds-settings.MinPoweredSeconds)/settings.DurationIntervals;
            for(int i=0;i<=settings.CoastIntervals&&!exhausted;i++)
                for(int j=0;j<=settings.DurationIntervals&&!exhausted;j++)
                    Try(i==settings.CoastIntervals?common.MaxIgnitionDelay:i*dc,
                        j==settings.DurationIntervals?settings.MaxPoweredSeconds:settings.MinPoweredSeconds+j*dt);
            for(int n=0;n<settings.Refinements&&!exhausted&&best!=null&&!cancellation.IsCancellationRequested;n++)
            {
                dc*=.5;dt*=.5;Profile center=best.Profile;
                for(int i=-1;i<=1&&!exhausted;i++)
                    for(int j=-1;j<=1&&!exhausted;j++)
                        Try(center.Coast+i*dc,center.Duration+j*dt);
            }
            if(cancellation.IsCancellationRequested)return Finish(P.SearchStatus.Cancelled);
            return Finish(exhausted?P.SearchStatus.BudgetExhausted:best==null?P.SearchStatus.NoDemonstratedLanding:
                best.Result.Miss<=common.TargetTolerance?P.SearchStatus.SampledTargetFeasible:P.SearchStatus.SampledFeasibleMiss);
        }
        // Reconstruct candidate's full control history, then independently
        // propagate exactly those commands with a finer numerical step. A
        // successful coarse candidate is not enough for publication.
        internal static bool Validate(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,Candidate candidate,int maxSteps,double positionTolerance,double massTolerance,
            out Candidate reference,out R.Report replay,CancellationToken cancellation=default)
        {
            reference=null;replay=null;
            if(candidate==null||!candidate.Result.Feasible||limits==null||!limits.Valid||
                !D.Finite(positionTolerance)||positionTolerance<0||!D.Finite(massTolerance)||massTolerance<0||
                maxSteps<2||maxSteps>1000000)return false;
            reference=Evaluate(env,propulsion,limits,initial,settings,candidate.Profile,maxSteps/2,true,cancellation);
            if(!reference.Result.Feasible||reference.Commands.Count==0||reference.Result.Steps>=maxSteps)return false;
            var fine=new D.Limits(limits.ReserveMass,limits.TurnRate,limits.EngineRise,limits.EngineFall,limits.Step*.5);
            var history=new R.Segment[reference.Commands.Count];
            for(int i=0;i<history.Length;i++)history[i]=reference.Commands[i];
            var common=settings.Common;
            replay=R.Run(env,propulsion,fine,initial,history,Array.Empty<R.Observation>(),
                common.SafeVertical,common.SafeHorizontal,common.SafeAngle,common.SafeRate,
                maxSteps-reference.Result.Steps,cancellation);
            return replay.Outcome==D.Outcome.ContactWithinLimits&&
                (reference.Result.Miss>common.TargetTolerance||P.Miss(env,initial,replay.End,common.TargetAtInitialUT)<=common.TargetTolerance)&&
                (reference.Result.End.R-replay.End.R).Length<=positionTolerance&&
                Math.Abs(reference.Result.End.Mass-replay.End.Mass)<=massTolerance&&
                Math.Abs(reference.Result.End.UT-replay.End.UT)<=settings.ControlTick+common.Terminal.Tick;
        }
    }
}
