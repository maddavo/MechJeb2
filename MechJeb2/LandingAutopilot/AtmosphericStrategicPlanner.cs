using System;
using System.Collections.Generic;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using R=MuMech.Landing.AtmosphericLandingReplay;
using T=MuMech.Landing.AtmosphericTerminalReplay;

namespace MuMech.Landing
{
    // Strategic outer search. Impulsive orbital geometry proposes seeds only;
    // all ranked outcomes include finite burns, attitude/engine lag, mass,
    // atmospheric descent and contact. No KSP APIs or global optimum claim.
    internal static class AtmosphericStrategicPlanner
    {
        internal readonly struct Orbit
        {
            internal readonly double SemiMajor,Eccentricity,Periapsis,Apoapsis,Period;
            internal readonly D.V Normal;
            internal Orbit(double a,double e,D.V n,double mu)
            {SemiMajor=a;Eccentricity=e;Normal=n;Periapsis=a*(1-e);Apoapsis=a*(1+e);
                Period=2*Math.PI*Math.Sqrt(a*a*a/mu);}
            internal bool Valid=>D.Finite(Period)&&Period>0&&D.Finite(Periapsis)&&Periapsis>0&&
                D.Finite(Apoapsis)&&Apoapsis>=Periapsis&&Normal.Valid&&Normal.Length>.99;
        }
        internal static bool TryOrbit(D.Environment env,D.State state,out Orbit orbit)
        {
            orbit=default;
            if(env==null||!env.Valid||env.Mu<=0||!state.Valid)return false;
            double r=state.R.Length,v2=D.V.Dot(state.Velocity,state.Velocity);
            D.V h=D.V.Cross(state.R,state.Velocity);
            double energy=v2*.5-env.Mu/r;
            if(!D.Finite(energy)||energy>=0||h.Length<1e-6)return false;
            double a=-env.Mu/(2*energy);
            double e=(D.V.Cross(state.Velocity,h)*(1/env.Mu)-state.R.Unit).Length;
            if(!D.Finite(a)||!D.Finite(e)||a<=0||e>=1)return false;
            orbit=new Orbit(a,e,h.Unit,env.Mu);return orbit.Valid;
        }
        internal sealed class Settings
        {
            internal readonly F.Settings Approach;
            internal readonly double Preparation,EngagementLead,MaxWait,MaxBurn;
            internal readonly int Epochs,MaxPrefixes,MaxSteps,ApproachCandidates;
            internal Settings(F.Settings approach,double preparation=35,double maxWait=3600,double maxBurn=180,
                int epochs=8,int maxPrefixes=48,int maxSteps=800000,int approachCandidates=24,double engagementLead=0)
            {Approach=approach;Preparation=preparation;EngagementLead=engagementLead;MaxWait=maxWait;MaxBurn=maxBurn;Epochs=epochs;
                MaxPrefixes=maxPrefixes;MaxSteps=maxSteps;ApproachCandidates=approachCandidates;}
            internal bool Valid=>Approach!=null&&Approach.Valid&&D.Finite(Preparation)&&Preparation>=0&&
                D.Finite(EngagementLead)&&EngagementLead>=0&&D.Finite(MaxWait)&&MaxWait>=Preparation+EngagementLead&&MaxWait<=10000&&D.Finite(MaxBurn)&&MaxBurn>0&&MaxBurn<=900&&
                Epochs>=1&&Epochs<=32&&MaxPrefixes>=1&&MaxPrefixes<=128&&MaxSteps>=1&&MaxSteps<=1000000&&
                ApproachCandidates>=1&&ApproachCandidates<=64;
        }
        internal readonly struct Burn
        {
            internal readonly double Delay,Duration,NormalFraction,ThrottleFraction;
            internal readonly D.V Direction;
            internal Burn(double delay,double duration,double normal,double throttleFraction=1,D.V direction=default)
            {Delay=delay;Duration=duration;NormalFraction=normal;ThrottleFraction=throttleFraction;Direction=direction;}
        }
        internal sealed class Prefix
        {
            internal readonly Burn Burn;
            internal readonly D.State End;
            internal readonly D.Outcome Outcome;
            internal readonly int Steps;
            internal readonly double MinimumClearance,FirstPoweredUT,BurnEndUT;
            internal readonly List<F.Point> Points;
            internal readonly List<R.Segment> Commands;
            internal Prefix(Burn burn,D.State end,D.Outcome outcome,int steps,double min,double first,double burnEnd,
                List<F.Point> points,List<R.Segment> commands)
            {Burn=burn;End=end;Outcome=outcome;Steps=steps;MinimumClearance=min;FirstPoweredUT=first;
                BurnEndUT=burnEnd;Points=points;Commands=commands;}
        }
        internal sealed class Candidate
        {
            internal readonly Burn Deorbit;
            internal readonly F.Candidate Approach,Combined;
            internal readonly F.Settings ApproachSettings;
            internal readonly double HandoffUT,DeorbitFuel,OriginalMiss;
            internal readonly bool ReplayPassed;
            internal Candidate(Burn burn,F.Candidate approach,F.Candidate combined,F.Settings settings,double handoff,
                double fuel,double miss,bool replay)
            {Deorbit=burn;Approach=approach;Combined=combined;ApproachSettings=settings;HandoffUT=handoff;
                DeorbitFuel=fuel;OriginalMiss=miss;ReplayPassed=replay;}
        }
        internal sealed class Report
        {
            internal readonly P.SearchStatus Status;
            internal readonly Candidate Best;
            internal readonly int Prefixes,Approaches,Steps,Feasible,Unresolved;
            internal readonly AtmosphericUncertaintyReplay.Report Scenarios;
            internal Report(P.SearchStatus status,Candidate best,int prefixes,int approaches,int steps,int feasible,int unresolved,
                AtmosphericUncertaintyReplay.Report scenarios)
            {Status=status;Best=best;Prefixes=prefixes;Approaches=approaches;Steps=steps;Feasible=feasible;Unresolved=unresolved;Scenarios=scenarios;}
        }

        // Tangential retrograde seed for a desired osculating periapsis.
        // No impulsive burn is applied in the simulator.
        internal static double SeedDeltaV(D.Environment env,D.State state,double periapsis)
        {
            double r=state.R.Length;
            if(env==null||env.Mu<=0||!state.Valid||!D.Finite(periapsis)||periapsis<=0||periapsis>=r)return double.NaN;
            D.V up=state.R.Unit;
            double tangent=(state.Velocity-up*D.V.Dot(up,state.Velocity)).Length;
            double desired=Math.Sqrt(env.Mu*(2/r-2/(r+periapsis)));
            return Math.Max(0,tangent-desired);
        }
        // Geometry-only seed for a combined plane change and deorbit.
        // A target outside the orbital plane generally needs BOTH a
        // larger retrograde component and substantial normal impulse.
        // Never use this impulsive estimate as a feasibility certificate.
        internal static bool PlaneChangeSeed(D.V radial,D.V tangent,D.V normal,D.V target,
            double speed,double desiredSpeed,out double deltaV,out double normalFraction)
        {
            deltaV=normalFraction=double.NaN;
            if(!radial.Valid||!tangent.Valid||!normal.Valid||!target.Valid||
                !D.Finite(speed)||speed<=0||!D.Finite(desiredSpeed)||desiredSpeed<=0||desiredSpeed>speed)
                return false;
            D.V projected=target-radial*D.V.Dot(target,radial);
            if(projected.Length<1e-6)return false;
            D.V wanted=projected.Unit;
            double retrograde=speed-desiredSpeed*D.V.Dot(wanted,tangent);
            double cross=desiredSpeed*D.V.Dot(wanted,normal);
            double dv=Math.Sqrt(retrograde*retrograde+cross*cross);
            if(!D.Finite(dv)||dv<=0||retrograde<=0)return false;
            deltaV=dv;normalFraction=cross/dv;
            return D.Finite(normalFraction)&&Math.Abs(normalFraction)<1;
        }

        // Analytic bound-orbit state only for seed generation. The actual
        // finite burn, attitude and subsequent trajectory are propagated.
        internal static bool KeplerState(D.Environment env,D.State initial,Orbit orbit,double elapsed,
            out D.V position,out D.V velocity)
        {
            position=velocity=default;
            if(env==null||!initial.Valid||!orbit.Valid||!D.Finite(elapsed))return false;
            double a=orbit.SemiMajor,e=orbit.Eccentricity,n=2*Math.PI/orbit.Period;
            D.V h=D.V.Cross(initial.R,initial.Velocity);
            D.V eccentricity=D.V.Cross(initial.Velocity,h)*(1/env.Mu)-initial.R.Unit;
            D.V first=e>1e-8?eccentricity.Unit:initial.R.Unit;
            D.V second=D.V.Cross(orbit.Normal,first).Unit;
            double eccentricAnomaly=e>1e-8?
                Math.Atan2(D.V.Dot(initial.R,initial.Velocity)/(e*Math.Sqrt(env.Mu*a)),
                    (1-initial.R.Length/a)/e):0;
            double mean=eccentricAnomaly-e*Math.Sin(eccentricAnomaly)+n*elapsed;
            mean=Math.IEEERemainder(mean,2*Math.PI);
            double anomaly=mean;
            for(int i=0;i<12;i++)anomaly-=(anomaly-e*Math.Sin(anomaly)-mean)/(1-e*Math.Cos(anomaly));
            double c=Math.Cos(anomaly),sn=Math.Sin(anomaly),q=Math.Sqrt(1-e*e);
            position=first*(a*(c-e))+second*(a*q*sn);
            velocity=(first*(-sn)+second*(q*c))*(Math.Sqrt(env.Mu*a)/position.Length);
            return position.Valid&&velocity.Valid&&position.Length>0;
        }

        // Construct an orbit whose periapsis lies on the target radial.
        // Unlike the old tangential seed this determines the required
        // radial, along-track AND cross-track velocity components.
        internal static bool TargetPeriapsisVelocity(double mu,D.V position,D.V target,double periapsis,
            out D.V velocity,out double flightTime)
        {
            velocity=default;flightTime=double.NaN;
            double r=position.Length;
            if(!D.Finite(mu)||mu<=0||!position.Valid||r<=0||!target.Valid||target.Length<=0||
                !D.Finite(periapsis)||periapsis<=0||periapsis>=r)return false;
            D.V up=position.Unit,goal=target.Unit;
            double dot=Math.Max(-1,Math.Min(1,D.V.Dot(up,goal)));
            D.V tangent=goal-up*dot;
            if(tangent.Length<1e-6)return false;
            double e=(r-periapsis)/(periapsis-r*dot);
            if(!D.Finite(e)||e<=0||e>=.95)return false;
            double a=periapsis/(1-e),h=Math.Sqrt(mu*periapsis*(1+e));
            double sin=Math.Sqrt(Math.Max(0,1-dot*dot));
            velocity=tangent.Unit*(h/r)-up*(mu/h*e*sin);
            double E=Math.Atan2(-Math.Sqrt(1-e*e)*sin,e+dot);
            flightTime=-(E-e*Math.Sin(E))*Math.Sqrt(a*a*a/mu);
            return velocity.Valid&&D.Finite(flightTime)&&flightTime>0;
        }

        internal static bool TargetTransferSeed(D.Environment env,D.State initial,Orbit orbit,D.Authority authority,
            double delay,double periapsis,double throttle,D.V target,double reserve,double maxBurn,
            out Burn burn,double commandFraction)
        {
            burn=default;
            double duration=0,flight=orbit.Period*.25;D.V impulse=default;
            // Centre the finite burn on the impulsive estimate and iterate
            // the moving target epoch and variable-mass burn duration.
            for(int i=0;i<8;i++)
            {
                double centre=delay+duration*.5;
                if(!KeplerState(env,initial,orbit,centre,out var position,out var velocity))return false;
                var goal=P.RotateTarget(target,env.Omega,centre+flight);
                if(!TargetPeriapsisVelocity(env.Mu,position,goal,periapsis,out var wanted,out double nextFlight))
                    return false;
                impulse=wanted-velocity;
                double nextDuration=SeedBurnTime(impulse.Length,initial.Mass,authority.Force,authority.Flow,throttle);
                if(!D.Finite(nextDuration)||nextDuration<.5||nextDuration>maxBurn||
                    nextDuration*authority.Flow*throttle>=initial.Mass-reserve)return false;
                duration=.5*(duration+nextDuration);flight=.5*(flight+nextFlight);
            }
            if(!impulse.Valid||impulse.Length<.1||duration<.5)return false;
            burn=new Burn(delay,duration,D.V.Dot(impulse.Unit,orbit.Normal),commandFraction,impulse.Unit);
            return true;
        }

        // Variable mass burn time, rather than constant initial acceleration.
        // This proposes a seed only; full dynamics and reserve checks decide.
        internal static double SeedBurnTime(double deltaV,double mass,double force,double flow,double throttle)
        {
            if(!D.Finite(deltaV)||deltaV<=0||!D.Finite(mass)||mass<=0||
                !D.Finite(force)||force<=0||!D.Finite(flow)||flow<=0||
                !D.Finite(throttle)||throttle<=0||throttle>1)return double.NaN;
            double exhaust=force/flow;
            return mass*(1-Math.Exp(-deltaV/exhaust))/(flow*throttle);
        }

        // Propagate a scheduled coast, preparation, finite vector burn and
        // engine tail. State/command history is continuous through handoff.
        internal static Prefix EvaluatePrefix(D.Environment env,D.Propulsion propulsion,D.Limits limits,
            D.State initial,Settings settings,Burn burn,int budget,bool record=false,CancellationToken token=default)
        {
            var points=new List<F.Point>();var commands=new List<R.Segment>();
            D.State s=initial;int steps=0;double min=double.NaN,first=double.NaN,burnEnd=double.NaN;
            Prefix Finish(D.Outcome status)=>new Prefix(burn,s,status,steps,min,first,burnEnd,points,commands);
            if(env==null||propulsion==null||limits==null||settings==null||!env.Valid||!limits.Valid||!settings.Valid||
                !initial.Valid||initial.Mass<=limits.ReserveMass||initial.Throttle>.01||budget<1||budget>1000000||
                !D.Finite(burn.Delay)||burn.Delay<settings.Preparation+settings.EngagementLead||burn.Delay>settings.MaxWait||
                !D.Finite(burn.Duration)||burn.Duration<=0||burn.Duration>settings.MaxBurn||
                !D.Finite(burn.NormalFraction)||Math.Abs(burn.NormalFraction)>1||
                !burn.Direction.Valid||burn.Direction.Length>0&&Math.Abs(burn.Direction.Length-1)>1e-6||
                !D.Finite(burn.ThrottleFraction)||burn.ThrottleFraction<=0||burn.ThrottleFraction>1)return Finish(D.Outcome.InvalidInput);
            var common=settings.Approach.Common;
            double begin=initial.UT+burn.Delay,prep=begin-settings.Preparation;
            burnEnd=begin+burn.Duration;
            double end=burnEnd+Math.Max(.5,5*limits.EngineFall);
            min=env.PhysicalClearance(s);
            if(!D.Finite(min)||min<=0)return Finish(D.Outcome.UnknownTerrain);
            D.V pointing=initial.Forward;
            // One fixed inertial vector is determined at preparation start.
            // Along-track and normal steering occur in the SAME finite burn.
            bool aimed=false;
            while(s.UT<end)
            {
                if(token.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                if(steps>=budget)return Finish(D.Outcome.Horizon);
                if(!aimed&&s.UT>=prep)
                {
                    D.V up=s.R.Unit,normal=D.V.Cross(s.R,s.Velocity).Unit;
                    D.V tangent=(s.Velocity-up*D.V.Dot(up,s.Velocity)).Unit;
                    if(normal.Length<.5||tangent.Length<.5)return Finish(D.Outcome.InvalidInput);
                    pointing=burn.Direction.Length>.5?burn.Direction:
                        (tangent*(-Math.Sqrt(Math.Max(0,1-burn.NormalFraction*burn.NormalFraction)))+
                        normal*burn.NormalFraction).Unit;aimed=true;
                }
                bool powered=s.UT>=begin&&s.UT<burnEnd;
                if(powered&&D.V.Dot(s.Forward,pointing)<.995)
                    return Finish(D.Outcome.UnknownCapability);
                D.Command command=new D.Command(powered?common.CommandCap*burn.ThrottleFraction:0,pointing);
                double boundary=Math.Min(end,s.UT+Math.Min(1,limits.Step));
                if(s.UT<prep)boundary=Math.Min(boundary,prep);
                else if(s.UT<begin)boundary=Math.Min(boundary,begin);
                else if(s.UT<burnEnd)boundary=Math.Min(boundary,burnEnd);
                double dt=boundary-s.UT;
                if(dt<=0||s.UT+dt<=s.UT)return Finish(D.Outcome.InvalidInput);
                if(record){points.Add(new F.Point(s,command,boundary));commands.Add(new R.Segment(s.UT,boundary,command));}
                var result=D.Advance(env,propulsion,limits,s,command,dt,common.SafeVertical,
                    common.SafeHorizontal,common.SafeAngle,common.SafeRate);
                steps++;min=Math.Min(min,result.MinimumClearance);
                if(powered&&double.IsNaN(first)&&result.End.UT>s.UT)first=s.UT;
                s=result.End;
                if(result.Status!=D.Outcome.Advanced)return Finish(result.Status);
            }
            // Require actual descending approach and osculating atmospheric
            // intersection (or already in atmosphere). No missed deorbit
            // or upward orbital arc is sent to the descent optimiser.
            if(D.V.Dot(s.R,s.Velocity)>=0)return Finish(D.Outcome.NoControllableThrust);
            if(s.R.Length>env.Radius+env.AtmosphereTop&&
                (!TryOrbit(env,s,out var orbit)||orbit.Periapsis>env.Radius+env.AtmosphereTop))
                return Finish(D.Outcome.NoControllableThrust);
            return Finish(D.Outcome.Advanced);
        }

        internal static F.Settings ApproachAt(D.Environment env,D.Propulsion propulsion,D.State initial,
            D.State handoff,Settings settings)
        {
            var old=settings.Approach;var common=old.Common;
            if(!env.Air(handoff.UT,handoff.R,out double pressure,out _))return null;
            var a=propulsion.At(pressure);double raw=a.Force/handoff.Mass*common.CommandCap;
            if(!a.Known||!D.Finite(raw)||raw<=0)return null;
            double height=env.PhysicalClearance(handoff),g=env.Mu/(handoff.R.Length*handoff.R.Length);
            if(!D.Finite(height)||height<=0)return null;
            double speed=env.Surface(handoff.R,handoff.Velocity).Length;
            double scale=Math.Max(8,Math.Max(Math.Sqrt(2*height/Math.Max(.1,g)),speed/Math.Max(.1,raw)));
            double maxCoast=Math.Min(600,scale*2),min=Math.Max(3,Math.Min(600,scale*.5)),
                max=Math.Min(900,Math.Max(min,scale*3));
            var target=P.RotateTarget(common.TargetAtInitialUT,env.Omega,handoff.UT-initial.UT).Unit;
            var rebased=new P.Settings(target,Math.Min(1800,maxCoast+max+120),maxCoast,common.TargetTolerance,
                common.Terminal,new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,handoff.Throttle,handoff.Throttle),
                intervals:2,refine:0,maxCandidates:settings.ApproachCandidates,maxSteps:Math.Min(60000,common.MaxSteps),
                captureClearance:common.CaptureClearance,captureSpeed:common.CaptureSpeed,
                safeVertical:common.SafeVertical,safeHorizontal:common.SafeHorizontal,safeAngle:common.SafeAngle,
                safeRate:common.SafeRate,commandCap:common.CommandCap);
            return new F.Settings(rebased,old.TargetSurfaceRadius,old.ApproachHeight,old.ApproachSpeed,
                min,max,old.ControlTick,coastIntervals:2,durationIntervals:3,refinements:2,
                terminalHorizontal:old.TerminalHorizontal,terminalVertical:old.TerminalVertical);
        }

        // Join without resetting mass, position, velocity, throttle or epoch.
        // Whole-history replay starts from the ORIGINAL orbital snapshot.
        internal static Candidate Combine(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,Prefix prefix,F.Settings approachSettings,F.Candidate approach,int budget,
            out int replaySteps,CancellationToken token=default,Action<string> diagnostic=null)
        {
            replaySteps=0;
            if(prefix==null||prefix.Outcome!=D.Outcome.Advanced||prefix.Points.Count==0||approach==null||
                !approach.Result.Feasible||approach.Reference.Count==0||budget<1)return null;
            D.State join=approach.Reference[0].State;
            if(join.UT!=prefix.End.UT||(join.R-prefix.End.R).Length>1e-8||
                (join.Velocity-prefix.End.Velocity).Length>1e-8||join.Mass!=prefix.End.Mass||
                join.Throttle!=prefix.End.Throttle||(join.Forward-prefix.End.Forward).Length>1e-8||
                join.PointingRate!=prefix.End.PointingRate)return null;
            var points=new List<F.Point>(prefix.Points);points.AddRange(approach.Reference);
            var commands=new List<R.Segment>(prefix.Commands);commands.AddRange(approach.Commands);
            var common=settings.Approach.Common;
            double miss=P.Miss(env,initial,approach.Result.End,common.TargetAtInitialUT);
            var fine=new D.Limits(limits.ReserveMass,limits.TurnRate,limits.EngineRise,limits.EngineFall,limits.Step*.5);
            var replay=R.Run(env,propulsion,fine,initial,commands.ToArray(),Array.Empty<R.Observation>(),
                common.SafeVertical,common.SafeHorizontal,common.SafeAngle,common.SafeRate,budget,token,
                D.Finite(approach.Result.TerminalStartUT)&&commands[commands.Count-1].StartUT>=approach.Result.TerminalStartUT ? .5 : 0);
            double historyMass=replay.End.Mass;
            bool continued=false;
            if(replay.Outcome==D.Outcome.Advanced&&replay.HistoryCompleted&&
                D.Finite(approach.Result.TerminalStartUT)&&commands[commands.Count-1].StartUT>=approach.Result.TerminalStartUT&&
                replay.Steps<budget)
            {
                // The actual controller already has a bounded terminal
                // extension. Prove this SAME feedback policy reaches
                // contact instead of demanding sub-metre open-loop
                // coincidence after a long orbital history.
                var terminalTracker=new AtmosphericPlanTracker(env,propulsion,limits,approachSettings,approach);
                var extended=AtmosphericUncertaintyReplay.CompleteTerminalReplay(env,propulsion,fine,
                    terminalTracker,replay,budget,token);
                continued=extended.Steps>replay.Steps;
                replay=extended;
            }
            replaySteps=replay.Steps;
            bool passed=replay.Outcome==D.Outcome.ContactWithinLimits&&
                (miss>common.TargetTolerance||P.Miss(env,initial,replay.End,common.TargetAtInitialUT)<=common.TargetTolerance)&&
                P.ContactPositionDifference(env,approach.Result.End,replay.End)<=20&&
                // Numerical mass consistency is measured at the end of
                // the recorded history; additional extension fuel is real
                // consumption, not integration error. Final reserve remains
                // mandatory and the extension itself is fully propagated.
                Math.Abs((continued?historyMass:replay.End.Mass)-approach.Result.End.Mass)<=Math.Max(.5,initial.Mass*.0005)&&
                replay.End.Mass>=limits.ReserveMass&&
                Math.Abs(replay.End.UT-approach.Result.End.UT)<=(continued?5:1);
            if(!passed)
            {
                diagnostic?.Invoke("join replay="+replay.Outcome+" miss="+miss+
                    " fineMiss="+P.Miss(env,initial,replay.End,common.TargetAtInitialUT)+
                    " positionError="+P.ContactPositionDifference(env,approach.Result.End,replay.End)+
                    " massError="+Math.Abs(replay.End.Mass-approach.Result.End.Mass)+
                    " historyMassError="+Math.Abs(historyMass-approach.Result.End.Mass)+
                    " timeError="+Math.Abs(replay.End.UT-approach.Result.End.UT)+
                    " endClearance="+env.PhysicalClearance(replay.End)+
                    " endSpeed="+env.Surface(replay.End.R,replay.End.Velocity).Length);
                return null;
            }
            var profile=new F.Profile(prefix.End.UT-initial.UT+approach.Profile.Coast,approach.Profile.Duration,approach.Profile.Curved);
            var p=new P.Profile(profile.Coast,0,new D.Command(0,initial.Forward));
            var result=new P.Candidate(p,approach.Result.Outcome,approach.Result.End,miss,
                Math.Min(prefix.MinimumClearance,approach.Result.MinimumClearance),prefix.FirstPoweredUT,
                approach.Result.TerminalStartUT,prefix.Steps+approach.Result.Steps+replay.Steps);
            var combined=new F.Candidate(profile,result,approach.RequestedPeakAcceleration,approach.MaxSaturation,
                approach.TerminalPositionError,points,commands);
            return new Candidate(prefix.Burn,approach,combined,approachSettings,prefix.End.UT,
                initial.Mass-prefix.End.Mass,miss,true);
        }

        internal static Report Search(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,CancellationToken token=default,Action<string> diagnostic=null)
        {
            Candidate best=null;int prefixes=0,approaches=0,steps=0,feasible=0,unresolved=0;
            bool exhausted=false;
            AtmosphericUncertaintyReplay.Report bestScenarios=null;
            Report Finish(P.SearchStatus status)=>new Report(status,best,prefixes,approaches,steps,feasible,unresolved,bestScenarios);
            if(settings==null||!settings.Valid||propulsion==null||limits==null||!limits.Valid||
                !TryOrbit(env,initial,out var orbit)||initial.Mass<=limits.ReserveMass||initial.Throttle>.01||
                orbit.Periapsis<=env.Radius+env.AtmosphereTop||settings.Approach.Common.CommandCap<=0)
                return Finish(P.SearchStatus.InvalidInput);
            double earliest=settings.Preparation+settings.EngagementLead;
            double wait=Math.Min(settings.MaxWait,orbit.Period+earliest);
            // Order epoch seeds by approximate prograde downrange geometry
            // at a fraction of the orbit; this only orders bounded search,
            // never discards a candidate as infeasible based on geometry.
            var epochs=new List<(double delay,double score)>();
            D.V up=initial.R.Unit,along=D.V.Cross(orbit.Normal,up).Unit;
            for(int i=0;i<settings.Epochs;i++)
            {
                double delay=earliest+(wait-earliest)*i/settings.Epochs;
                double angle=2*Math.PI*delay/orbit.Period+Math.PI*.65;
                D.V ahead=up*Math.Cos(angle)+along*Math.Sin(angle);
                D.V target=P.RotateTarget(settings.Approach.Common.TargetAtInitialUT,env.Omega,delay+orbit.Period*.325);
                epochs.Add((delay,D.V.Dot(ahead,target)));
            }
            // Add a high-resolution cheap geometry seed and neighbouring
            // epochs. Avoid quantising orbital targeting to hundreds of
            // seconds while retaining the original coarse coverage.
            double geometryDelay=earliest,geometryScore=double.NegativeInfinity;
            for(int i=0;i<=64;i++)
            {
                double delay=earliest+(wait-earliest)*i/64;
                double angle=2*Math.PI*delay/orbit.Period+Math.PI*.65;
                D.V ahead=up*Math.Cos(angle)+along*Math.Sin(angle);
                D.V target=P.RotateTarget(settings.Approach.Common.TargetAtInitialUT,env.Omega,delay+orbit.Period*.325);
                double score=D.V.Dot(ahead,target);
                if(score>geometryScore){geometryScore=score;geometryDelay=delay;}
            }
            double spacing=(wait-earliest)/64;
            foreach(double delay in new[]{geometryDelay,geometryDelay-spacing,geometryDelay+spacing})
            {
                if(delay<earliest||delay>wait||epochs.Exists(e=>Math.Abs(e.delay-delay)<.001))continue;
                double angle=2*Math.PI*delay/orbit.Period+Math.PI*.65;
                D.V target=P.RotateTarget(settings.Approach.Common.TargetAtInitialUT,env.Omega,delay+orbit.Period*.325);
                epochs.Add((delay,D.V.Dot(up*Math.Cos(angle)+along*Math.Sin(angle),target)));
            }
            epochs.Sort((a,b)=>b.score.CompareTo(a.score));
            foreach(var epoch in epochs)
            {
                if(token.IsCancellationRequested)return Finish(P.SearchStatus.Cancelled);
                // Circular/elliptic local seed at original state. Actual
                // finite propagation, not this approximation, is ranked.
                double r=initial.R.Length;
                double normal=D.V.Dot(orbit.Normal,P.RotateTarget(settings.Approach.Common.TargetAtInitialUT,
                    env.Omega,epoch.delay+orbit.Period*.325));
                double tilt=Math.Max(-.65,Math.Min(.65,normal*2));
                // A coplanar target needs no plane-change seed.
                // Do not spend most of the shared replay/scenario budget
                // on mirrored, physically equivalent out-of-plane burns.
                var tilts=Math.Abs(tilt)<.002?new[]{0.0}:
                    Math.Abs(tilt)<.02?new[]{0.0,tilt}:new[]{tilt,0.0,tilt*.5};
                foreach(double depth in new[]{env.Radius+env.AtmosphereTop*.35,env.Radius*.97})
                {
                    double dv=SeedDeltaV(env,initial,Math.Min(r-1,depth));
                    if(!D.Finite(dv)||dv<=0)continue;
                    if(!env.Air(initial.UT,initial.R,out double pressure,out _))continue;
                    var authority=propulsion.At(pressure);
                    double accel=authority.Force/initial.Mass*settings.Approach.Common.CommandCap;
                    if(!authority.Known||!D.Finite(accel)||accel<=0)continue;
                    // Preserve the existing small-crossrange search, but
                    // do not constrain a large plane change to a small
                    // correction on top of a coplanar deorbit impulse.
                    const double burnFraction=.85;
                    var seeds=new List<Burn>();
                    if(Math.Abs(normal)>.1)
                    {
                        if(TargetTransferSeed(env,initial,orbit,authority,epoch.delay,Math.Min(r-1,depth),
                            settings.Approach.Common.CommandCap*burnFraction,
                            settings.Approach.Common.TargetAtInitialUT,limits.ReserveMass,settings.MaxBurn,
                            out var transfer,burnFraction))
                            seeds.Add(transfer);
                        double angle=2*Math.PI*(epoch.delay-settings.Preparation)/orbit.Period;
                        D.V radial=up*Math.Cos(angle)+along*Math.Sin(angle);
                        D.V tangent=D.V.Cross(orbit.Normal,radial).Unit;
                        D.V future=P.RotateTarget(settings.Approach.Common.TargetAtInitialUT,
                            env.Omega,epoch.delay+orbit.Period*.325);
                        double speed=(initial.Velocity-initial.R.Unit*D.V.Dot(initial.R.Unit,initial.Velocity)).Length;
                        if(PlaneChangeSeed(radial,tangent,orbit.Normal,future,speed,speed-dv,
                            out double planeDv,out double planeFraction))
                        {
                            double time=SeedBurnTime(planeDv,initial.Mass,authority.Force,authority.Flow,
                                settings.Approach.Common.CommandCap*burnFraction);
                            if(D.Finite(time)&&time>=.5&&time<=settings.MaxBurn&&
                                time*authority.Flow*settings.Approach.Common.CommandCap*burnFraction<
                                initial.Mass-limits.ReserveMass)
                                seeds.Add(new Burn(epoch.delay,time,planeFraction,burnFraction));
                        }
                    }
                    foreach(double fraction in tilts)
                        seeds.Add(new Burn(epoch.delay,Math.Min(settings.MaxBurn,Math.Max(.5,
                            dv/(accel*burnFraction)/Math.Sqrt(1-fraction*fraction))),fraction,burnFraction));
                    foreach(var seed in seeds)
                    {
                        if(token.IsCancellationRequested)return Finish(P.SearchStatus.Cancelled);
                        if(prefixes>=settings.MaxPrefixes||steps>=settings.MaxSteps){exhausted=true;break;}
                        var burn=seed;
                        double fraction=burn.NormalFraction,duration=burn.Duration;
                        var prefix=EvaluatePrefix(env,propulsion,limits,initial,settings,burn,
                            settings.MaxSteps-steps,false,token);
                        prefixes++;steps+=prefix.Steps;
                        diagnostic?.Invoke("prefix "+prefixes+" delay="+epoch.delay+" duration="+duration+
                            " normal="+fraction+" transfer="+(burn.Direction.Length>.5)+" status="+prefix.Outcome+" height="+env.PhysicalClearance(prefix.End)+
                            " vRadial="+D.V.Dot(prefix.End.Velocity,prefix.End.R.Unit)+
                            " mass="+prefix.End.Mass+" steps="+steps);
                        if(prefix.Outcome!=D.Outcome.Advanced){unresolved++;continue;}
                        var approachSettings=ApproachAt(env,propulsion,initial,prefix.End,settings);
                        if(approachSettings==null){unresolved++;continue;}
                        // Enforce one shared numerical-work ceiling.
                        if(settings.MaxSteps-steps<approachSettings.Common.MaxSteps){exhausted=true;break;}
                        // Bounded diagnostic sample: detailed failures for the
                        // first two seed epochs, aggregate later.
                        Action<F.Candidate> describe=null;
                        if(diagnostic!=null&&(prefixes<=12||prefixes%6==4))
                            describe=c=>diagnostic("  candidate coast="+c.Profile.Coast+" duration="+c.Profile.Duration+" curved="+c.Profile.Curved+
                                " outcome="+c.Result.Outcome+" endH="+env.PhysicalClearance(c.Result.End)+
                                " endV="+env.Surface(c.Result.End.R,c.Result.End.Velocity).Length+
                                " endMass="+c.Result.End.Mass+" elapsed="+(c.Result.End.UT-prefix.End.UT)+
                                " endMiss="+P.Miss(env,prefix.End,c.Result.End,approachSettings.Common.TargetAtInitialUT)+
                                " terminal="+c.Result.TerminalStartUT+" saturation="+c.MaxSaturation);
                        var report=F.Search(env,propulsion,limits,prefix.End,approachSettings,token,diagnostic:describe,refineNearContact:true,curvedCandidates:Math.Abs(normal)>.1);
                        approaches+=report.Attempts;steps+=report.Steps;
                        diagnostic?.Invoke("approach "+report.Status+" attempts="+report.Attempts+
                            " nominal="+report.Feasible+" unresolved="+report.Unresolved+
                            " bestMiss="+report.Best?.Result.Miss+" steps="+steps);
                        if(report.Best==null){unresolved++;continue;}
                        if(best!=null&&!P.Better(report.Best.Result,best.Combined.Result,
                            settings.Approach.Common.TargetTolerance))continue;
                        if(settings.MaxSteps-steps<1000){exhausted=true;break;}
                        // Reconstruct the orbital prefix once; try ranked
                        // approach alternatives through whole-history replay
                        // and closed-loop scenarios before ranking a join.
                        var recorded=EvaluatePrefix(env,propulsion,limits,initial,settings,burn,
                            Math.Min(20000,settings.MaxSteps-steps),true,token);
                        steps+=recorded.Steps;
                        if(recorded.Outcome!=D.Outcome.Advanced){unresolved++;continue;}
                        int tried=0;
                        foreach(var proposal in report.Ranked)
                        {
                            if(token.IsCancellationRequested)return Finish(P.SearchStatus.Cancelled);
                            if(tried>=12)break;
                            if(best!=null&&!P.Better(proposal.Result,best.Combined.Result,
                                settings.Approach.Common.TargetTolerance))break;
                            if(steps>=settings.MaxSteps){exhausted=true;break;}
                            tried++;
                            var descent=F.Evaluate(env,propulsion,limits,recorded.End,approachSettings,proposal.Profile,
                                Math.Min(20000,settings.MaxSteps-steps),true,token);
                            steps+=descent.Result.Steps;
                            if(steps>=settings.MaxSteps){exhausted=true;break;}
                            var joined=Combine(env,propulsion,limits,initial,settings,recorded,approachSettings,descent,
                                Math.Min(60000,settings.MaxSteps-steps),out int replaySteps,token,diagnostic);
                            steps+=replaySteps;
                            if(joined==null){unresolved++;continue;}
                            if(steps>=settings.MaxSteps){exhausted=true;break;}
                            var tracker=new AtmosphericPlanTracker(env,propulsion,limits,settings.Approach,joined.Combined);
                            var screen=AtmosphericUncertaintyReplay.Validate(env,propulsion,limits,tracker,
                                Math.Min(240000,settings.MaxSteps-steps),token);
                            steps+=screen.Steps;
                            if(!screen.Passed){diagnostic?.Invoke("scenario rejected: "+screen.Reason);unresolved++;continue;}
                            if(token.IsCancellationRequested)return Finish(P.SearchStatus.Cancelled);
                            feasible++;
                            if(best==null||P.Better(joined.Combined.Result,best.Combined.Result,
                                settings.Approach.Common.TargetTolerance))
                            {best=joined;bestScenarios=screen;}
                            break;
                        }
                        if(exhausted)break;
                    }
                    if(exhausted)break;
                }
                if(exhausted)break;
            }
            if(token.IsCancellationRequested)return Finish(P.SearchStatus.Cancelled);
            return Finish(exhausted?P.SearchStatus.BudgetExhausted:best==null?P.SearchStatus.NoDemonstratedLanding:
                best.OriginalMiss<=settings.Approach.Common.TargetTolerance?P.SearchStatus.SampledTargetFeasible:P.SearchStatus.SampledFeasibleMiss);
        }
    }
}
