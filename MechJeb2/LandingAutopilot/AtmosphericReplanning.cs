using System;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;

namespace MuMech.Landing
{
    // Bounded, model-only receding horizon. The currently admitted tracker
    // remains responsible during search. Replacement has a future absolute
    // activation epoch and cannot skip the candidate's first powered step.
    internal static class AtmosphericReplanning
    {
        internal sealed class Request
        {
            internal readonly int Generation;
            internal readonly D.State Observed;
            internal readonly double SwitchUT,Cap;
            internal readonly D.Environment Environment;
            internal readonly D.Propulsion Propulsion;
            internal readonly D.Limits Limits;
            internal readonly F.Settings Settings;
            internal readonly AtmosphericPlanTracker Previous;
            internal Request(int generation,D.State observed,double switchUT,double cap,
                D.Environment env,D.Propulsion propulsion,D.Limits limits,F.Settings settings,AtmosphericPlanTracker previous)
            {Generation=generation;Observed=observed;SwitchUT=switchUT;Cap=cap;Environment=env;
                Propulsion=propulsion;Limits=limits;Settings=settings;Previous=previous;}
        }
        internal sealed class Result
        {
            internal readonly int Generation;
            internal readonly double SwitchUT;
            internal readonly D.State Anchor;
            internal readonly AtmosphericPlanTracker Tracker;
            internal readonly string Reason;
            internal readonly int Attempts,Steps;
            internal readonly double SolveMs;
            internal Result(int generation,double ut,D.State anchor,AtmosphericPlanTracker tracker,string reason,
                int attempts,int steps,double ms)
            {Generation=generation;SwitchUT=ut;Anchor=anchor;Tracker=tracker;Reason=reason;
                Attempts=attempts;Steps=steps;SolveMs=ms;}
        }

        // Predict from the actually observed state, not a copied future
        // nominal state. Simulate the OLD tracker until the scheduled handoff.
        // Same model as the planner, not proof of actual force parity.
        internal static bool Project(Request request,out D.State state,out string reason,CancellationToken token=default)
        {
            state=request==null?default:request.Observed;
            reason="invalid request";
            if(request==null||request.Environment==null||!request.Environment.Valid||request.Propulsion==null||
                request.Limits==null||!request.Limits.Valid||request.Settings==null||!request.Settings.Valid||
                request.Previous==null||!ReferenceEquals(request.Settings,request.Previous.Settings)||!state.Valid||state.Mass<=request.Limits.ReserveMass||
                request.Limits.ReserveMass<request.Previous.ReserveMass||
                !D.Finite(request.Cap)||request.Cap<=0||request.Cap>1||
                !D.Finite(request.SwitchUT)||request.SwitchUT<=state.UT||
                request.SwitchUT-state.UT>10||request.SwitchUT>=request.Previous.EndUT)return false;
            var env=request.Environment;var settings=request.Settings.Common;
            int count=0;
            while(state.UT<request.SwitchUT)
            {
                if(token.IsCancellationRequested){reason="cancelled";return false;}
                if(++count>512){reason="prefix budget";return false;}
                if(!env.Air(state.UT,state.R,out double pressure,out _))
                {reason="prefix atmosphere unknown";return false;}
                var authority=request.Propulsion.At(pressure);
                if(!authority.Known||authority.Force<=0){reason="prefix thrust unknown";return false;}
                double gravity=env.Mu/(state.R.Length*state.R.Length);
                var d=request.Previous.Track(state,env.PhysicalClearance(state),authority.Force/state.Mass,
                    request.Cap,gravity,request.Limits.EngineRise);
                if(!d.Valid){reason="prefix: "+d.Reason;return false;}
                double dt=Math.Min(Math.Min(.1,request.Limits.Step),request.SwitchUT-state.UT);
                if(dt<=0||state.UT+dt<=state.UT){reason="prefix epoch resolution";return false;}
                var step=D.Advance(env,request.Propulsion,request.Limits,state,d.Command,dt,
                    settings.SafeVertical,settings.SafeHorizontal,settings.SafeAngle,settings.SafeRate);
                state=step.End;
                if(step.Status!=D.Outcome.Advanced){reason="prefix: "+step.Status;return false;}
            }
            reason=null;return state.UT==request.SwitchUT;
        }

        internal static F.Settings Rebase(Request r,D.State anchor)
        {
            var original=r.Settings;var common=original.Common;var previous=r.Previous;
            // Preserve original reserve and requested landing location in
            // body-fixed coordinates. Rebase its inertial radial at anchor UT.
            var target=P.RotateTarget(common.TargetAtInitialUT,r.Environment.Omega,anchor.UT-previous.StartUT).Unit;
            double arrival=previous.StartUT+previous.Plan.Profile.Coast+previous.Plan.Profile.Duration;
            double time=Math.Max(4,arrival-anchor.UT);
            double maxCoast=Math.Min(60,Math.Max(0,previous.StartUT+previous.Plan.Profile.Coast-anchor.UT+5));
            double minDuration=Math.Max(1,time*.55),maxDuration=Math.Min(900,Math.Max(minDuration,time*1.6));
            double horizon=Math.Min(1800,maxCoast+maxDuration+120);
            // No artificial 15-second coast is imposed on an active burn.
            var settings=new P.Settings(target,horizon,maxCoast,common.TargetTolerance,
                common.Terminal,new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,anchor.Throttle,anchor.Throttle),
                intervals:2,refine:0,maxCandidates:24,maxSteps:45000,captureClearance:common.CaptureClearance,
                captureSpeed:common.CaptureSpeed,safeVertical:common.SafeVertical,safeHorizontal:common.SafeHorizontal,
                safeAngle:common.SafeAngle,safeRate:common.SafeRate,entryCutoffDelay:0,commandCap:r.Cap);
            return new F.Settings(settings,original.TargetSurfaceRadius,original.ApproachHeight,original.ApproachSpeed,
                minDuration,maxDuration,original.ControlTick,coastIntervals:2,durationIntervals:4,refinements:1,
                terminalHorizontal:original.TerminalHorizontal,terminalVertical:original.TerminalVertical);
        }
        internal static Result Solve(Request r,CancellationToken token=default)
        {
            var clock=System.Diagnostics.Stopwatch.StartNew();
            D.State anchor=default;int attempts=0,steps=0;
            Result Finish(AtmosphericPlanTracker plan,string reason)=>new Result(r?.Generation??-1,
                r?.SwitchUT??double.NaN,anchor,plan,reason,attempts,steps,clock.Elapsed.TotalMilliseconds);
            try
            {
                if(!Project(r,out anchor,out string reason,token))return Finish(null,reason);
                var settings=Rebase(r,anchor);
                if(!settings.Valid)return Finish(null,"invalid remaining horizon");
                double oldIgnition=r.Previous.StartUT+r.Previous.Plan.Profile.Coast;
                double oldArrival=oldIgnition+r.Previous.Plan.Profile.Duration;
                double coast=Math.Max(0,oldIgnition-anchor.UT);
                coast=Math.Min(settings.Common.MaxIgnitionDelay,coast);
                double duration=Math.Max(settings.MinPoweredSeconds,Math.Min(settings.MaxPoweredSeconds,
                    oldArrival-anchor.UT-coast));
                var report=F.Search(r.Environment,r.Propulsion,r.Limits,anchor,settings,token,new F.Profile(coast,duration));
                attempts=report.Attempts;steps=report.Steps;
                if(token.IsCancellationRequested)return Finish(null,"cancelled");
                if(report.Best==null)return Finish(null,"search "+report.Status);
                var selection=F.SelectValidated(r.Environment,r.Propulsion,r.Limits,anchor,settings,report,
                    20000,60000,10,Math.Max(.5,anchor.Mass*.00025),token);
                steps+=selection.ReplaySteps+selection.ScenarioSteps;
                if(selection.Candidate==null)return Finish(null,selection.Reason);
                var tracker=new AtmosphericPlanTracker(r.Environment,r.Propulsion,r.Limits,settings,selection.Candidate);
                return Finish(tracker,tracker.TargetMiss<=settings.Common.TargetTolerance?"target candidate":"degraded candidate");
            }
            catch(Exception ex){return Finish(null,ex.GetType().Name+": "+ex.Message);}
        }

        // No epoch skipping. Tiny positive time tolerance accounts for the
        // discrete physics callback crossing a scheduled absolute UT.
        // Require close agreement with the candidate's own attained state,
        // not only the much wider tracking envelope.
        internal static string Admission(Result result,int expectedGeneration,D.State actual,
            double clearance,double liveAccel,double cap,double gravity,double response,
            bool oldValid,double targetTolerance,double reserve)
        {
            if(result==null||result.Generation!=expectedGeneration||result.Tracker==null)return "stale or absent candidate";
            if(!actual.Valid||!D.Finite(result.SwitchUT)||actual.UT<result.SwitchUT||
                actual.UT-result.SwitchUT>.08)return "handoff epoch missed";
            var t=result.Tracker;
            if(t.StartUT!=result.SwitchUT||t.ReserveMass<reserve||actual.Mass<=reserve)
                return "candidate reserve or epoch mismatch";
            if(!t.ReferenceAt(actual.UT,out var reference,out _))return "candidate not current";
            double pe=(actual.R-reference.R).Length,ve=(actual.Velocity-reference.Velocity).Length;
            if(pe>Math.Min(10,Math.Max(1,clearance*.002))||ve>1.0||
                Math.Abs(actual.Mass-reference.Mass)>Math.Max(2,actual.Mass*.001)||
                Math.Abs(actual.Throttle-reference.Throttle)>.15||
                D.V.Dot(actual.Forward,reference.Forward)<Math.Cos(8*Math.PI/180))
                return "handoff state differs from projected anchor";
            if(oldValid&&t.TargetMiss>targetTolerance)return "retain current plan over degraded candidate";
            var d=t.Track(actual,clearance,liveAccel,cap,gravity,response);
            return d.Valid?null:d.Reason;
        }
    }
}
