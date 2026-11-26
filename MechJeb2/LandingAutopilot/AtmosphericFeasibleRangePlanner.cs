using System;
using System.Collections.Generic;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using T=MuMech.Landing.AtmosphericTerminalReplay;

namespace MuMech.Landing
{
    // M2 reference planner: fixed approach, finite correction and coast/brake
    // candidates, actual terminal replay. Independent SI model; no KSP calls.
    // Sampled feasibility is not a proof of reachability or infeasibility.
    internal static class AtmosphericFeasibleRangePlanner
    {
        internal readonly struct Profile
        {
            internal readonly double CorrectionSeconds, IgnitionDelay;
            internal readonly D.Command Correction;
            internal Profile(double ignition, double correctionSeconds, D.Command correction)
            {IgnitionDelay=ignition;CorrectionSeconds=correctionSeconds;Correction=correction;}
        }
        internal sealed class Settings
        {
            internal readonly double Horizon, CaptureClearance, CaptureSpeed, TargetTolerance, MaxIgnitionDelay,EntryCutoffDelay,CommandCap;
            internal readonly double SafeVertical,SafeHorizontal,SafeAngle,SafeRate;
            internal readonly int CoarseIntervals,RefineLevels,MaxCandidates,MaxSteps;
            internal readonly T.Settings Terminal;
            internal readonly T.Memory TerminalMemory;
            // Target is unit inertial radial vector at Initial.UT. Rotation
            // uses the model's signed inertial omega, not a longitude sign guess.
            internal readonly D.V TargetAtInitialUT;
            internal Settings(D.V target,double horizon,double maxIgnition,double tolerance,
                T.Settings terminal,T.Memory memory,int intervals=12,int refine=2,int maxCandidates=128,
                int maxSteps=200000,double captureClearance=300,double captureSpeed=15,
                double safeVertical=1,double safeHorizontal=1,double safeAngle=5*Math.PI/180,double safeRate=Math.PI/180,
                double entryCutoffDelay=0,double commandCap=1)
            {
                TargetAtInitialUT=target;Horizon=horizon;MaxIgnitionDelay=maxIgnition;TargetTolerance=tolerance;
                Terminal=terminal;TerminalMemory=memory;CoarseIntervals=intervals;RefineLevels=refine;
                MaxCandidates=maxCandidates;MaxSteps=maxSteps;CaptureClearance=captureClearance;CaptureSpeed=captureSpeed;
                SafeVertical=safeVertical;SafeHorizontal=safeHorizontal;SafeAngle=safeAngle;SafeRate=safeRate;
                EntryCutoffDelay=entryCutoffDelay;CommandCap=commandCap;
            }
            internal bool Valid=>TargetAtInitialUT.Valid&&Math.Abs(TargetAtInitialUT.Length-1)<1e-6&&
                D.Finite(Horizon)&&Horizon>0&&Horizon<=1800&&D.Finite(MaxIgnitionDelay)&&MaxIgnitionDelay>=0&&
                MaxIgnitionDelay<Horizon&&D.Finite(EntryCutoffDelay)&&EntryCutoffDelay>=0&&EntryCutoffDelay<=Horizon&&D.Finite(CommandCap)&&CommandCap>=0&&CommandCap<=1&&D.Finite(TargetTolerance)&&TargetTolerance>=0&&Terminal!=null&&Terminal.Valid&&
                TerminalMemory.Valid&&CoarseIntervals>=1&&CoarseIntervals<=64&&RefineLevels>=0&&RefineLevels<=6&&
                MaxCandidates>=1&&MaxCandidates<=512&&MaxSteps>=1&&MaxSteps<=1000000&&
                D.Finite(CaptureClearance)&&CaptureClearance>=0&&D.Finite(CaptureSpeed)&&CaptureSpeed>0&&
                D.Finite(SafeVertical)&&SafeVertical>=0&&D.Finite(SafeHorizontal)&&SafeHorizontal>=0&&
                D.Finite(SafeAngle)&&SafeAngle>=0&&SafeAngle<=Math.PI&&D.Finite(SafeRate)&&SafeRate>=0;
        }
        internal readonly struct Candidate
        {
            internal readonly Profile Profile;
            internal readonly D.Outcome Outcome;
            internal readonly D.State End;
            internal readonly double Miss,MinimumClearance,FirstPoweredIntervalUT,TerminalStartUT;
            internal readonly int Steps;
            internal bool Feasible=>Outcome==D.Outcome.ContactWithinLimits&&D.Finite(Miss);
            internal Candidate(Profile p,D.Outcome outcome,D.State end,double miss,double clearance,
                double first,double terminal,int steps)
            {Profile=p;Outcome=outcome;End=end;Miss=miss;MinimumClearance=clearance;
                FirstPoweredIntervalUT=first;TerminalStartUT=terminal;Steps=steps;}
        }
        internal enum SearchStatus { SampledTargetFeasible, SampledFeasibleMiss, NoDemonstratedLanding,
            BudgetExhausted, Cancelled, InvalidInput }
        internal sealed class Report
        {
            internal readonly SearchStatus Status;
            internal readonly IReadOnlyList<Candidate> Candidates;
            internal readonly Candidate? Best;
            internal readonly int Steps;
            internal readonly bool RequestedGridCompleted;
            internal readonly int FeasibleCount,UnresolvedCount;
            internal readonly double MinimumSampledMiss,MaximumSampledMiss;
            internal Report(SearchStatus status,List<Candidate> candidates,Candidate? best,int steps,bool complete)
            {
                Status=status;Candidates=Array.AsReadOnly(candidates.ToArray());Best=best;Steps=steps;RequestedGridCompleted=complete;
                double lo=double.PositiveInfinity,hi=double.NegativeInfinity;
                int feasible=0,unresolved=0;
                foreach(var a in candidates)
                {
                    if(a.Feasible){feasible++;lo=Math.Min(lo,a.Miss);hi=Math.Max(hi,a.Miss);}
                    else if(a.Outcome==D.Outcome.UnknownCapability||a.Outcome==D.Outcome.UnknownTerrain||
                        a.Outcome==D.Outcome.InvalidInput||a.Outcome==D.Outcome.Horizon||a.Outcome==D.Outcome.Cancelled||
                        a.Outcome==D.Outcome.CaptureEnvelope||a.Outcome==D.Outcome.Advanced)unresolved++;
                }
                FeasibleCount=feasible;UnresolvedCount=unresolved;
                MinimumSampledMiss=feasible>0?lo:double.NaN;MaximumSampledMiss=feasible>0?hi:double.NaN;
            }
        }
        internal static D.V RotateTarget(D.V target,D.V omega,double elapsed)
        {
            double angle=omega.Length*elapsed;
            if(omega.Length==0)return target;
            D.V axis=omega.Unit;
            return target*Math.Cos(angle)+D.V.Cross(axis,target)*Math.Sin(angle)+
                axis*(D.V.Dot(axis,target)*(1-Math.Cos(angle)));
        }
        internal static double Miss(D.Environment env,D.State initial,D.State contact,D.V target)
        {
            D.V a=RotateTarget(target,env.Omega,contact.UT-initial.UT).Unit,b=contact.R.Unit;
            return env.Radius*Math.Atan2(D.V.Cross(a,b).Length,D.V.Dot(a,b));
        }
        // Contact states can be at different epochs. Compare their
        // body-fixed positions at a common epoch, not the motion of
        // the surface through inertial space between those epochs.
        internal static double ContactPositionDifference(D.Environment env,D.State a,D.State b)
        {
            if(env==null||!a.Valid||!b.Valid)return double.NaN;
            return (a.R-RotateTarget(b.R,env.Omega,a.UT-b.UT)).Length;
        }
        // Lexicographic: within tolerance prefer fuel, otherwise reduce miss.
        // Only actual model contact within limits may enter the feasible set.
        internal static bool Better(Candidate a,Candidate b,double tolerance)
        {
            if(!a.Feasible)return false;
            if(!b.Feasible)return true;
            bool ta=a.Miss<=tolerance,tb=b.Miss<=tolerance;
            if(ta!=tb)return ta;
            if(!ta&&a.Miss!=b.Miss)return a.Miss<b.Miss;
            if(a.End.Mass!=b.End.Mass)return a.End.Mass>b.End.Mass;
            if(a.Miss!=b.Miss)return a.Miss<b.Miss;
            return a.End.UT<b.End.UT;
        }
        internal static Candidate Evaluate(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,Profile p,int budget,CancellationToken cancellation=default,
            Action<D.State,D.Command,double> record=null)
        {
            D.State state=initial;
            double minimum=double.NaN,first=double.NaN,terminal=double.NaN;
            int steps=0;
            Candidate Finish(D.Outcome result)=>new Candidate(p,result,state,
                result==D.Outcome.ContactWithinLimits?Miss(env,initial,state,settings.TargetAtInitialUT):double.NaN,
                minimum,first,terminal,steps);
            if(env==null||propulsion==null||limits==null||settings==null||!env.Valid||!limits.Valid||
                !settings.Valid||!initial.Valid||initial.Mass<limits.ReserveMass||budget<1||budget>1000000||
                !p.Correction.Valid||p.Correction.Throttle>settings.CommandCap||!D.Finite(p.CorrectionSeconds)||p.CorrectionSeconds<0||
                !D.Finite(p.IgnitionDelay)||p.IgnitionDelay<p.CorrectionSeconds||
                p.IgnitionDelay>settings.MaxIgnitionDelay||p.CorrectionSeconds>settings.EntryCutoffDelay)return Finish(D.Outcome.InvalidInput);
            minimum=env.PhysicalClearance(initial);
            if(!D.Finite(minimum))return Finish(D.Outcome.UnknownTerrain);
            double correctionEnd=initial.UT+p.CorrectionSeconds, ignition=initial.UT+p.IgnitionDelay,
                end=initial.UT+settings.Horizon;
            while(state.UT<end)
            {
                if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                if(steps>=budget)return Finish(D.Outcome.Horizon);
                D.V w=env.Surface(state.R,state.Velocity);
                double clearance=env.PhysicalClearance(state);
                if(!D.Finite(clearance))return Finish(D.Outcome.UnknownTerrain);
                // A capture envelope is an intermediate state only.
                // Continue with source-derived KHV/final controller.
                if(state.UT>=ignition&&w.Length<=settings.CaptureSpeed&&clearance>0&&
                    clearance>=settings.CaptureClearance)
                {
                    terminal=state.UT;
                    var mem=settings.TerminalMemory;
                    // Seed applied-throttle history from attained state; PID
                    // memory is explicitly supplied for terminal entry.
                    mem=new T.Memory(mem.Phase,mem.Mode,mem.PreviousError,mem.Integral,
                        mem.PreviousThrottle,state.Throttle);
                    int ticks=(int)Math.Min(1000000,Math.Floor((end-state.UT)/settings.Terminal.Tick));
                    if(ticks<1)return Finish(D.Outcome.Horizon);
                    // Modelled terminal authority must respect the candidate-wide
                    // command cap too. An inconsistent adapter is unknown.
                    var terminalSettings=new T.Settings(settings.Terminal.Tick,settings.Terminal.TouchdownSpeed,
                        settings.Terminal.GeeASL,settings.Terminal.SmoothingSeconds,
                        st=>{
                            var a=settings.Terminal.Authority(st);
                            return a.Valid&&a.CommandCap<=settings.CommandCap?a:default;
                        },settings.Terminal.AttitudeError,settings.Terminal.EstimatedTerrainRadius);
                    var t=T.Run(env,propulsion,limits,terminalSettings,state,mem,ticks,
                        settings.SafeVertical,settings.SafeHorizontal,settings.SafeAngle,settings.SafeRate,
                        cancellation,budget-steps,record);
                    state=t.End;steps+=t.IntegrationSteps;minimum=Math.Min(minimum,t.MinimumClearance);
                    return Finish(t.Outcome);
                }
                bool correcting=state.UT<correctionEnd,braking=state.UT>=ignition;
                D.V retro=w.Length>1e-8?w.Unit*(-1):state.R.Unit;
                var command=correcting?p.Correction:new D.Command(
                    braking&&D.V.Dot(w,state.R)<=0&&D.V.Dot(state.Forward,retro)>=.75?settings.CommandCap:0,retro);
                double dt=Math.Min(limits.Step,end-state.UT);
                if(correcting)dt=Math.Min(dt,correctionEnd-state.UT);
                else if(!braking)dt=Math.Min(dt,ignition-state.UT);
                if(dt<=0||state.UT+dt<=state.UT)return Finish(D.Outcome.InvalidInput);
                record?.Invoke(state,command,state.UT+dt);
                var r=D.Advance(env,propulsion,limits,state,command,dt,settings.SafeVertical,
                    settings.SafeHorizontal,settings.SafeAngle,settings.SafeRate);
                steps++;
                if(double.IsNaN(first)&&r.End.UT>state.UT&&(state.Throttle>0||r.End.Throttle>0)&&
                    env.Air(state.UT,state.R,out double pressure,out _)&&propulsion.At(pressure).Force>0)
                    first=state.UT;
                state=r.End;minimum=Math.Min(minimum,r.MinimumClearance);
                if(r.Status!=D.Outcome.Advanced)return Finish(r.Status);
            }
            return Finish(D.Outcome.Horizon);
        }
        // Automatically generate sizeable finite correction candidates, rather
        // than assuming a sequence of small, impulsive local corrections.
        // Durations and throttle are supplied by the capability/deadline
        // adapter; the basis spans downrange, crossrange and radial directions.
        // These are search seeds, not assumed beneficial or executable burns.
        internal static CorrectionOption[] GenerateCorrections(D.Environment env,D.State initial,
            Settings settings,double[] durations,double throttle)
        {
            if(env==null||!env.Valid||!initial.Valid||settings==null||!settings.Valid||
                durations==null||durations.Length>3||!D.Finite(throttle)||throttle<=0||throttle>settings.CommandCap)
                return Array.Empty<CorrectionOption>();
            D.V up=initial.R.Unit,w=env.Surface(initial.R,initial.Velocity);
            D.V along=(w-up*D.V.Dot(w,up)).Unit;
            if(along.Length<.5)
            {
                D.V target=settings.TargetAtInitialUT;
                along=(target-up*D.V.Dot(target,up)).Unit;
            }
            if(along.Length<.5)
                along=D.V.Cross(up,Math.Abs(up.X)<.8?new D.V(1,0,0):new D.V(0,1,0)).Unit;
            D.V cross=D.V.Cross(up,along).Unit;
            var directions=new[]{along,along*(-1),cross,cross*(-1),up,up*(-1),
                (along+cross).Unit,(along-cross).Unit,(cross-along).Unit,(along+cross).Unit*(-1)};
            var options=new List<CorrectionOption>{new CorrectionOption(0,new D.Command(0,initial.Forward))};
            var seen=new HashSet<double>();
            foreach(double duration in durations)
            {
                if(!D.Finite(duration)||duration<=0||duration>settings.MaxIgnitionDelay||
                    duration>settings.EntryCutoffDelay||!seen.Add(duration))continue;
                foreach(var direction in directions)
                    options.Add(new CorrectionOption(duration,new D.Command(throttle,direction)));
            }
            return options.ToArray();
        }

        // Joint reference search. Evaluate the same timing grid for each
        // caller-supplied finite correction (including zero correction).
        // Distribute work round-robin, so an early candidate cannot consume
        // every timing evaluation merely because it is listed first.
        internal readonly struct CorrectionOption
        {
            internal readonly double Seconds;
            internal readonly D.Command Command;
            internal CorrectionOption(double seconds,D.Command command){Seconds=seconds;Command=command;}
        }
        internal static Report SearchJoint(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,CorrectionOption[] corrections,CancellationToken cancellation=default)
        {
            var candidates=new List<Candidate>();Candidate? best=null;
            int steps=0;bool complete=false;
            Report Finish(SearchStatus status)=>new Report(status,candidates,best,steps,complete);
            if(env==null||propulsion==null||limits==null||settings==null||!env.Valid||!limits.Valid||
                !settings.Valid||!initial.Valid||initial.Mass<limits.ReserveMass||corrections==null||
                corrections.Length<1||corrections.Length>64)return Finish(SearchStatus.InvalidInput);
            corrections=(CorrectionOption[])corrections.Clone();
            foreach(var c in corrections)
                if(!c.Command.Valid||c.Command.Throttle>settings.CommandCap||!D.Finite(c.Seconds)||c.Seconds<0||c.Seconds>settings.MaxIgnitionDelay||c.Seconds>settings.EntryCutoffDelay)
                    return Finish(SearchStatus.InvalidInput);
            var seen=new HashSet<double>[corrections.Length];
            for(int k=0;k<seen.Length;k++)seen[k]=new HashSet<double>();
            for(int level=0;level<=settings.RefineLevels;level++)
            {
                int intervals=settings.CoarseIntervals*(1<<level);
                for(int i=0;i<=intervals;i++)
                {
                    if(level>0&&(i&1)==0)continue;
                    for(int k=0;k<corrections.Length;k++)
                    {
                        if(cancellation.IsCancellationRequested)return Finish(SearchStatus.Cancelled);
                        var c=corrections[k];
                        double ignition=i==intervals?settings.MaxIgnitionDelay:
                            c.Seconds+(settings.MaxIgnitionDelay-c.Seconds)*i/intervals;
                        if(!seen[k].Add(ignition))continue;
                        if(candidates.Count>=settings.MaxCandidates||steps>=settings.MaxSteps)
                            return Finish(SearchStatus.BudgetExhausted);
                        var a=Evaluate(env,propulsion,limits,initial,settings,
                            new Profile(ignition,c.Seconds,c.Command),settings.MaxSteps-steps,cancellation);
                        candidates.Add(a);steps+=a.Steps;
                        if(a.Feasible&&(!best.HasValue||Better(a,best.Value,settings.TargetTolerance)))best=a;
                        if(a.Outcome==D.Outcome.Cancelled)return Finish(SearchStatus.Cancelled);
                    }
                }
            }
            complete=true;
            return Finish(best.HasValue?best.Value.Miss<=settings.TargetTolerance?
                SearchStatus.SampledTargetFeasible:SearchStatus.SampledFeasibleMiss:SearchStatus.NoDemonstratedLanding);
        }

        // Stage A: sample ALL coarse ignition intervals, including endpoints.
        // Then refine each interval on a bounded uniform grid. No monotonic
        // assumption or interpolation of missing/unsafe branches. The caller
        // may use alternate finite corrections with the same evaluator.
        internal static Report Search(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            Settings settings,double correctionSeconds,D.Command correction,CancellationToken cancellation=default)
        {
            return SearchJoint(env,propulsion,limits,initial,settings,
                new[]{new CorrectionOption(correctionSeconds,correction)},cancellation);
        }
    }
}
