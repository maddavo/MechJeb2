using System;
using System.Collections.Generic;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;

namespace MuMech.Landing
{
    // Independent local-site search. Does not optimise miss to the original
    // target, does not consume its protected mass reserve, and never calls
    // KSP terrain APIs from a worker. Sites are sampled inputs, not certified
    // obstacle-free touchdown areas.
    internal static class AtmosphericFallbackPlanner
    {
        internal readonly struct Site
        {
            internal readonly D.V Radial;
            internal readonly double SurfaceRadius;
            internal Site(D.V radial,double radius){Radial=radial;SurfaceRadius=radius;}
            internal bool Valid=>Radial.Valid&&Math.Abs(Radial.Length-1)<1e-6&&
                D.Finite(SurfaceRadius)&&SurfaceRadius>0;
        }
        internal sealed class Result
        {
            internal readonly int Generation,Attempts,Steps;
            internal readonly double SwitchUT,OriginalMiss;
            internal readonly AtmosphericPlanTracker Tracker;
            internal readonly D.Environment Environment;
            internal readonly string Reason;
            internal Result(int generation,double ut,AtmosphericPlanTracker tracker,D.Environment environment,
                double miss,int attempts,int steps,string reason)
            {Generation=generation;SwitchUT=ut;Tracker=tracker;Environment=environment;OriginalMiss=miss;
                Attempts=attempts;Steps=steps;Reason=reason;}
        }

        // Candidate radials at measured epoch. A range seed comes from the
        // current tangent speed and estimated braking time; all directions
        // are merely proposals for numerical propagation and terrain checks.
        internal static D.V[] SiteRadials(D.Environment env,D.State state,double accel,double lead)
        {
            if(env==null||!env.Valid||!state.Valid||!D.Finite(accel)||accel<=0||
                !D.Finite(lead)||lead<0||lead>10)return Array.Empty<D.V>();
            D.V up=state.R.Unit,w=env.Surface(state.R,state.Velocity);
            D.V horizontal=w-up*D.V.Dot(w,up);
            if(horizontal.Length<.1)return new[]{up};
            D.V along=horizontal.Unit;
            double speed=horizontal.Length;
            double distance=Math.Min(env.Radius*.7,speed*lead+speed*speed/(2*Math.Max(.2,accel*.6)));
            D.V Offset(double d)=> (up*Math.Cos(d/env.Radius)+along*Math.Sin(d/env.Radius)).Unit;
            return new[]{Offset(distance*.65),Offset(distance),Offset(distance*1.35),Offset(speed*lead),up};
        }

        internal static D.Environment SiteEnvironment(D.Environment env,Site site)
        {
            // A fallback changes the destination, not the planet.
            if(env==null||!env.Valid||!site.Valid||env.Aerodynamics!=null||env.PassiveLift!=null)
                return null;
            return env.WithSiteRadius(site.SurfaceRadius);
        }

        internal static Result Solve(AtmosphericReplanning.Request request,Site[] sites,CancellationToken token=default)
        {
            int attempts=0,steps=0;
            Result Finish(AtmosphericPlanTracker tracker,D.Environment env,double miss,string reason)=>
                new Result(request?.Generation??-1,request?.SwitchUT??double.NaN,tracker,env,miss,attempts,steps,reason);
            if(sites==null||sites.Length==0||sites.Length>5)return Finish(null,null,double.NaN,"no sampled sites");
            sites=(Site[])sites.Clone();
            try
            {
                if(!AtmosphericReplanning.Project(request,out var anchor,out string reason,token))
                    return Finish(null,null,double.NaN,reason);
                var old=request.Settings;var common=old.Common;
                D.V original=P.RotateTarget(common.TargetAtInitialUT,request.Environment.Omega,
                    anchor.UT-request.Previous.StartUT).Unit;
                F.Candidate best=null;D.Environment bestEnv=null;F.Settings bestSettings=null;
                double bestMiss=double.NaN;
                foreach(var site in sites)
                {
                    if(token.IsCancellationRequested)return Finish(null,null,double.NaN,"cancelled");
                    var env=SiteEnvironment(request.Environment,site);
                    if(env==null||env.PhysicalClearance(anchor)<=0||
                        !env.Air(anchor.UT,anchor.R,out double pressure,out _))continue;
                    var authority=request.Propulsion.At(pressure);
                    if(!authority.Known||authority.Force<=0)continue;
                    double raw=authority.Force/anchor.Mass*request.Cap;
                    double g=env.Mu/(anchor.R.Length*anchor.R.Length);
                    if(raw<=g)continue;
                    double speed=env.Surface(anchor.R,anchor.Velocity).Length;
                    double height=env.PhysicalClearance(anchor);
                    double scale=Math.Max(5,Math.Max(speed/Math.Max(.2,raw-g),
                        Math.Sqrt(2*height/Math.Max(.2,g))));
                    double min=Math.Max(2,Math.Min(500,scale*.5)),max=Math.Min(900,Math.Max(min,scale*2.5));
                    var radial=P.RotateTarget(site.Radial,env.Omega,anchor.UT-request.Observed.UT).Unit;
                    // Independent safety objective: land at the sampled site
                    // with maximum remaining mass, not nearest to the nominal
                    // target. Site tolerance caps modelled contact displacement.
                    var settings=new P.Settings(radial,Math.Min(1800,max+120),0,100,
                        common.Terminal,new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,anchor.Throttle,anchor.Throttle),
                        intervals:1,refine:0,maxCandidates:8,maxSteps:12000,captureClearance:common.CaptureClearance,
                        captureSpeed:common.CaptureSpeed,safeVertical:common.SafeVertical,safeHorizontal:common.SafeHorizontal,
                        safeAngle:common.SafeAngle,safeRate:common.SafeRate,commandCap:request.Cap);
                    var f=new F.Settings(settings,site.SurfaceRadius+env.BottomOffset,old.ApproachHeight,old.ApproachSpeed,
                        min,max,old.ControlTick,coastIntervals:1,durationIntervals:5,refinements:1,
                        terminalHorizontal:old.TerminalHorizontal,terminalVertical:old.TerminalVertical);
                    var report=F.Search(env,request.Propulsion,request.Limits,anchor,f,token);
                    attempts+=report.Attempts;steps+=report.Steps;
                    if(report.Best==null||report.Best.Result.Miss>settings.TargetTolerance)continue;
                    if(token.IsCancellationRequested)return Finish(null,null,double.NaN,"cancelled");
                    // Test alternatives within each site before comparing
                    // sites. A nominal optimum is not a validated fallback.
                    var selection=F.SelectValidated(env,request.Propulsion,request.Limits,anchor,f,report,
                        16000,40000,10,Math.Max(.5,anchor.Mass*.00025),token);
                    steps+=selection.ReplaySteps+selection.ScenarioSteps;
                    var replayed=selection.Candidate;
                    if(replayed==null||replayed.Result.Miss>settings.TargetTolerance)continue;
                    double miss=P.Miss(env,anchor,replayed.Result.End,original);
                    if(!D.Finite(miss))continue;
                    if(best==null||replayed.Result.End.Mass>best.Result.End.Mass||
                        replayed.Result.End.Mass==best.Result.End.Mass&&miss<bestMiss)
                    {best=replayed;bestEnv=env;bestSettings=f;bestMiss=miss;}
                }
                if(token.IsCancellationRequested)return Finish(null,null,double.NaN,"cancelled");
                if(best==null)return Finish(null,null,double.NaN,"no replayed local-site contact");
                var tracker=new AtmosphericPlanTracker(bestEnv,request.Propulsion,request.Limits,bestSettings,best);
                return Finish(tracker,bestEnv,bestMiss,"local-site candidate");
            }
            catch(Exception ex){return Finish(null,null,double.NaN,ex.GetType().Name+": "+ex.Message);}
        }

        // A completed local candidate can be taken only when the actual
        // state agrees with its reference NOW. No blind elapsed-burn credit.
        // Short age window allows a rolling contingency to be available
        // across several physics callbacks, not just one exact tick.
        internal static string Admission(Result result,int generation,D.State actual,double clearance,
            double liveAcceleration,double cap,double gravity,double response,double reserve)
        {
            if(result==null||result.Tracker==null||result.Generation!=generation)return "stale fallback";
            if(!actual.Valid||actual.UT<result.SwitchUT||actual.UT>result.SwitchUT+2)
                return "fallback activation window missed";
            var t=result.Tracker;
            if(t.StartUT!=result.SwitchUT||t.ReserveMass<reserve||actual.Mass<=reserve)
                return "fallback reserve or epoch mismatch";
            if(!t.ReferenceAt(actual.UT,out var expected,out _))return "fallback expired";
            if((actual.R-expected.R).Length>Math.Min(5,Math.Max(1,clearance*.001))||
                (actual.Velocity-expected.Velocity).Length>1||
                Math.Abs(actual.Mass-expected.Mass)>Math.Max(2,actual.Mass*.001)||
                Math.Abs(actual.Throttle-expected.Throttle)>.1||
                D.V.Dot(actual.Forward,expected.Forward)<Math.Cos(5*Math.PI/180))
                return "fallback state mismatch";
            var d=t.Track(actual,clearance,liveAcceleration,cap,gravity,response);
            return d.Valid?null:d.Reason;
        }
    }
}
