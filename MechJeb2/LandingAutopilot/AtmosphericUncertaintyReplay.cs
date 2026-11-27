using System;
using System.Collections.Generic;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using R=MuMech.Landing.AtmosphericLandingReplay;

namespace MuMech.Landing
{
    // Explicit scenario screen of the CLOSED LOOP tracker, not just another
    // nominal open-loop history replay. This is finite model stress testing;
    // the perturbations are provisional, not measured uncertainty bounds.
    internal static class AtmosphericUncertaintyReplay
    {
        // The real strategic execution boundaries, not inferred from the
        // first nonzero throttle in an independently perturbed history.
        internal readonly struct OrbitalExecution
        {
            internal readonly double BurnStart,BurnEnd,Handoff;
            internal OrbitalExecution(double start,double end,double handoff)
            {BurnStart=start;BurnEnd=end;Handoff=handoff;}
            internal bool Valid=>D.Finite(BurnStart)&&D.Finite(BurnEnd)&&D.Finite(Handoff)&&
                BurnEnd>BurnStart&&Handoff>=BurnEnd;
        }
        internal readonly struct Scenario
        {
            internal readonly string Name;
            internal readonly double ImpulseScale,DragScale,ResponseScale,TurnScale;
            internal Scenario(string name,double impulse,double drag,double response,double turn)
            {Name=name;ImpulseScale=impulse;DragScale=drag;ResponseScale=response;TurnScale=turn;}
        }
        internal readonly struct Trial
        {
            internal readonly string Name,Reason;
            internal readonly D.Outcome Outcome;
            internal readonly double Miss,Mass,EndUT;
            internal readonly int Steps;
            internal Trial(string name,string reason,D.Outcome outcome,double miss,double mass,double ut,int steps)
            {Name=name;Reason=reason;Outcome=outcome;Miss=miss;Mass=mass;EndUT=ut;Steps=steps;}
            internal bool Passed=>Outcome==D.Outcome.ContactWithinLimits&&D.Finite(Miss);
        }
        internal sealed class Report
        {
            internal readonly IReadOnlyList<Trial> Trials;
            internal readonly bool Passed;
            internal readonly string Reason;
            internal readonly int Steps;
            internal readonly double WorstMiss,MinimumEndMass;
            internal Report(List<Trial> trials,bool passed,string reason,int steps)
            {
                Trials=Array.AsReadOnly(trials.ToArray());Passed=passed;Reason=reason;Steps=steps;
                double miss=0,mass=double.PositiveInfinity;int count=0;
                foreach(var t in trials)if(t.Passed){count++;miss=Math.Max(miss,t.Miss);mass=Math.Min(mass,t.Mass);}
                WorstMiss=count>0?miss:double.NaN;MinimumEndMass=count>0?mass:double.NaN;
            }
        }
        internal static Scenario[] Standard()=>new[]{
            new Scenario("nominal closed loop",1,1,1,1),
            new Scenario("5% less impulse",.95,1,1,1),
            new Scenario("10% less drag",1,.9,1,1),
            new Scenario("combined adverse model",.95,1.1,1.2,.9)
        };

        // The near-contact timing extension is shared by numerical testing
        // and live control. It is explicitly bounded and cannot replace a
        // lost high-altitude trajectory.
        internal static bool TryTerminalExtension(AtmosphericPlanTracker tracker,D.Environment model,
            D.Propulsion propulsion,D.State actual,D.V surfaceVelocity,double clearance,
            double raw,double cap,double gravity,out D.Command command)
        {
            command=default;
            if(tracker==null||model==null||propulsion==null||!actual.Valid||!surfaceVelocity.Valid||
                !D.Finite(clearance)||clearance<=0||clearance>5||!D.Finite(raw)||raw<=0||
                !D.Finite(cap)||cap<=0||cap>1||!D.Finite(gravity)||gravity<0||
                actual.UT<tracker.EndUT||actual.UT>tracker.EndUT+5||actual.Mass<=tracker.ReserveMass||
                raw*cap<=gravity||D.V.Dot(actual.Forward,actual.R.Unit)<Math.Cos(Math.PI/6))return false;
            double vertical=D.V.Dot(surfaceVelocity,actual.R.Unit);
            if(vertical>0||vertical< -2||(surfaceVelocity-actual.R.Unit*vertical).Length>1)return false;
            var surface=new D.SurfaceSample(clearance,clearance,clearance,actual.R.Unit,true);
            var t=AtmosphericFiniteTimePlanner.TerminalCommand(model,propulsion,actual,tracker.Settings,surface);
            if(!t.Valid||!model.Air(actual.UT,actual.R,out double p,out _))return false;
            var a=propulsion.At(p);
            if(!a.Known||a.Force<=0)return false;
            command=new D.Command(Math.Min(cap,t.Throttle*a.Force/(actual.Mass*raw)),t.Direction.Unit);
            return command.Valid;
        }

        // Complete only a fully replayed terminal command history using
        // the same bounded feedback extension used by live guidance.
        // Never alter the generic exact-history replay contract.
        internal static R.Report CompleteTerminalReplay(D.Environment env,D.Propulsion propulsion,D.Limits limits,
            AtmosphericPlanTracker tracker,R.Report replay,int budget,CancellationToken token=default)
        {
            if(replay==null||replay.Outcome!=D.Outcome.Advanced||!replay.HistoryCompleted||
                tracker==null||replay.Steps>=budget)return replay;
            var state=replay.End;
            int steps=replay.Steps;
            double minimum=replay.MinimumClearance;
            var common=tracker.Settings.Common;
            R.Report Finish(D.Outcome status)=>new R.Report(status,state,minimum,steps,replay.CompletedSegments,
                replay.HistoryCompleted,new List<R.Comparison>(replay.Comparisons));
            double end=tracker.EndUT+5;
            while(state.UT<end)
            {
                if(token.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                if(steps>=budget)return Finish(D.Outcome.Horizon);
                if(!env.Air(state.UT,state.R,out double pressure,out _))return Finish(D.Outcome.UnknownCapability);
                var authority=propulsion.At(pressure);
                if(!authority.Known||authority.Force<=0)return Finish(D.Outcome.UnknownCapability);
                double raw=authority.Force/state.Mass,g=env.Mu/(state.R.Length*state.R.Length);
                if(!TryTerminalExtension(tracker,env,propulsion,state,env.Surface(state.R,state.Velocity),
                    env.PhysicalClearance(state),raw,common.CommandCap,g,out var command))return Finish(D.Outcome.Advanced);
                double dt=Math.Min(Math.Min(.2,limits.Step),end-state.UT);
                if(dt<=0||state.UT+dt<=state.UT)return Finish(D.Outcome.InvalidInput);
                var result=D.Advance(env,propulsion,limits,state,command,dt,common.SafeVertical,
                    common.SafeHorizontal,common.SafeAngle,common.SafeRate);
                steps++;state=result.End;minimum=Math.Min(minimum,result.MinimumClearance);
                if(result.Status!=D.Outcome.Advanced)return Finish(result.Status);
            }
            return Finish(D.Outcome.Advanced);
        }

        internal static Trial Run(D.Environment model,D.Propulsion propulsion,D.Limits limits,
            AtmosphericPlanTracker tracker,Scenario scenario,int budget,CancellationToken token=default,
            OrbitalExecution? execution=null)
        {
            D.State state=tracker==null?default:tracker.Plan.Reference[0].State;
            D.State initial=state;int steps=0;bool burnRequested=false;
            Trial Finish(D.Outcome status,string reason)=>new Trial(scenario.Name,reason,status,
                status==D.Outcome.ContactWithinLimits?P.Miss(model,initial,state,tracker.Settings.Common.TargetAtInitialUT):double.NaN,
                state.Mass,state.UT,steps);
            if(model==null||!model.Valid||propulsion==null||limits==null||!limits.Valid||tracker==null||
                model.PassiveLift!=null||model.Aerodynamics!=null||
                execution.HasValue&&!execution.Value.Valid||
                !D.Finite(scenario.ImpulseScale)||scenario.ImpulseScale<=0||
                !D.Finite(scenario.DragScale)||scenario.DragScale<0||
                !D.Finite(scenario.ResponseScale)||scenario.ResponseScale<=0||
                !D.Finite(scenario.TurnScale)||scenario.TurnScale<=0||budget<1||budget>1000000)
                return Finish(D.Outcome.InvalidInput,"unsupported scenario model");
            var env=model.WithDrag(model.DragArea*scenario.DragScale);
            var engine=propulsion.WithImpulseScale(scenario.ImpulseScale);
            var lim=new D.Limits(limits.ReserveMass,limits.TurnRate*scenario.TurnScale,
                limits.EngineRise*scenario.ResponseScale,limits.EngineFall*scenario.ResponseScale,limits.Step*.5);
            var common=tracker.Settings.Common;double cap=common.CommandCap;
            double end=tracker.EndUT+5;
            while(state.UT<end)
            {
                if(token.IsCancellationRequested)return Finish(D.Outcome.Cancelled,"cancelled");
                if(steps>=budget)return Finish(D.Outcome.Horizon,"scenario step budget");
                if(!env.Air(state.UT,state.R,out double pressure,out _))return Finish(D.Outcome.UnknownCapability,"atmosphere");
                var a=engine.At(pressure);
                if(!a.Known||a.Force<=0)return Finish(D.Outcome.UnknownCapability,"engine");
                double raw=a.Force/state.Mass,g=env.Mu/(state.R.Length*state.R.Length);
                double clearance=env.PhysicalClearance(state);
                var d=tracker.Track(state,clearance,raw,cap,g,lim.EngineRise);
                D.Command command;
                if(d.Valid)command=d.Command;
                else if(d.Reason=="plan epoch expired"&&TryTerminalExtension(tracker,model,propulsion,state,
                    env.Surface(state.R,state.Velocity),clearance,raw,cap,g,out var t))command=t;
                else return Finish(D.Outcome.UnknownCapability,"tracking: "+d.Reason+" at UT "+state.UT+
                    " clearance="+clearance+" positionError="+d.PositionError+" velocityError="+d.VelocityError+" massError="+d.MassError);
                if(execution.HasValue)
                {
                    var timing=execution.Value;
                    bool prefix=state.UT<timing.Handoff;
                    bool preburn=DunaLandingAdmission.OrbitalPreburn(model,state,true,burnRequested);
                    if(!preburn)
                    {
                        var safety=AtmosphericRecoverability.Assess(state,env.Surface(state.R,state.Velocity),
                            clearance,raw,cap,g,limits.TurnRate,lim.EngineRise,a.Flow,
                            tracker.ReserveMass,common.Terminal.TouchdownSpeed);
                        if(safety.Status==AtmosphericRecoverability.Level.Hazard)
                            return Finish(D.Outcome.UnknownCapability,"live supervisor: "+safety.Reason+" at UT "+state.UT+
                                " clearance="+clearance+" down="+safety.DownSpeed);
                    }
                    if(prefix&&state.UT>=timing.BurnStart&&state.UT<timing.BurnEnd&&
                        !DunaLandingAdmission.OrbitalCommandAligned(state,command))
                        command=new D.Command(0,command.Direction);
                    if(prefix&&command.Throttle>0)burnRequested=true;
                }
                // Re-evaluate feedback at <=0.2s intervals, split at
                // every command boundary and integrate with <=half step.
                double boundary=Math.Min(end,state.UT+.2);
                if(state.UT<tracker.EndUT)
                {
                    // Reference boundaries are absolute; don't accidentally
                    // extend a powered command past its scheduled interval.
                    var points=tracker.Plan.Reference;int lo=0,hi=points.Count-1;
                    while(lo<hi){int mid=lo+(hi-lo+1)/2;if(points[mid].State.UT<=state.UT)lo=mid;else hi=mid-1;}
                    boundary=Math.Min(boundary,Math.Min(tracker.EndUT,points[lo].ValidUntil));
                }
                while(state.UT<boundary)
                {
                    if(token.IsCancellationRequested)return Finish(D.Outcome.Cancelled,"cancelled");
                    if(steps>=budget)return Finish(D.Outcome.Horizon,"scenario step budget");
                    double dt=Math.Min(lim.Step,boundary-state.UT);
                    if(dt<=0||state.UT+dt<=state.UT)return Finish(D.Outcome.InvalidInput,"epoch resolution");
                    var step=D.Advance(env,engine,lim,state,command,dt,common.SafeVertical,common.SafeHorizontal,
                        common.SafeAngle,common.SafeRate);
                    steps++;state=step.End;
                    if(step.Status!=D.Outcome.Advanced)
                    {
                        D.V w=env.Surface(state.R,state.Velocity),up=state.R.Unit;
                        double vertical=D.V.Dot(w,up),horizontal=(w-up*vertical).Length;
                        double angle=Math.Acos(Math.Max(-1,Math.Min(1,D.V.Dot(state.Forward,up))))*180/Math.PI;
                        return Finish(step.Status,step.Status+" vertical="+vertical+" horizontal="+horizontal+
                            " angleDeg="+angle+" rateDegS="+state.PointingRate*180/Math.PI+" mass="+state.Mass);
                    }
                }
            }
            return Finish(D.Outcome.Horizon,"contact not demonstrated");
        }

        internal static Report Validate(D.Environment env,D.Propulsion propulsion,D.Limits limits,
            AtmosphericPlanTracker tracker,int budget,CancellationToken token=default,
            OrbitalExecution? execution=null)
        {
            var trials=new List<Trial>();int steps=0;
            if(budget<1||budget>1000000)return new Report(trials,false,"invalid scenario budget",steps);
            foreach(var scenario in Standard())
            {
                if(token.IsCancellationRequested)return new Report(trials,false,"cancelled",steps);
                if(steps>=budget)return new Report(trials,false,"scenario budget exhausted",steps);
                Trial t=Run(env,propulsion,limits,tracker,scenario,budget-steps,token,execution);
                trials.Add(t);steps+=t.Steps;
                if(!t.Passed)return new Report(trials,false,t.Name+": "+t.Reason,steps);
                if((trials.Count==1||execution.HasValue)&&tracker.TargetMiss<=tracker.Settings.Common.TargetTolerance&&
                    t.Miss>tracker.Settings.Common.TargetTolerance)
                    return new Report(trials,false,t.Name+": closed-loop target miss exceeds tolerance",steps);
            }
            return new Report(trials,true,"all sampled scenarios reached model contact within limits",steps);
        }
    }
}
