using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using R=MuMech.Landing.AtmosphericLandingReplay;

namespace MuMech.Landing
{
    // Same snapshot, reserve, target, integration/validation budgets.
    // A uses its existing timing-only reference policy, B the existing
    // nonlinear feedback shooting, C convex local acceleration shooting.
    // No production solver selection or actuator calls.
    internal static class AtmosphericSolverBenchmark
    {
        internal enum Method { TimingReference, FiniteTimeShooting, ConvexShooting }
        internal sealed class Trial
        {
            internal readonly Method Solver;
            internal readonly P.SearchStatus Status;
            internal readonly P.Candidate? Nominal;
            internal readonly bool FineReplayPassed,ScenarioPassed;
            internal readonly string Reason;
            internal readonly int Attempts,SearchSteps,ValidationSteps;
            internal readonly double SearchMs,TotalMs,SampledWorstMiss,SampledMinimumMass;
            internal bool Validated=>FineReplayPassed&&ScenarioPassed&&Nominal.HasValue&&Nominal.Value.Feasible;
            internal Trial(Method solver,P.SearchStatus status,P.Candidate? nominal,bool fine,bool scenarios,string reason,
                int attempts,int searchSteps,int validationSteps,double searchMs,double totalMs,
                double worstMiss,double minimumMass)
            {Solver=solver;Status=status;Nominal=nominal;FineReplayPassed=fine;ScenarioPassed=scenarios;Reason=reason;
                Attempts=attempts;SearchSteps=searchSteps;ValidationSteps=validationSteps;SearchMs=searchMs;
                TotalMs=totalMs;SampledWorstMiss=worstMiss;SampledMinimumMass=minimumMass;}
        }

        internal static Trial Run(Method solver,D.Environment env,D.Propulsion propulsion,D.Limits limits,
            D.State initial,F.Settings settings,int validationBudget=120000,int wallMilliseconds=10000,
            CancellationToken cancellation=default)
        {
            var clock=Stopwatch.StartNew();double searchMs=0,worstMiss=double.NaN,minimumMass=double.NaN;
            P.SearchStatus status=P.SearchStatus.InvalidInput;P.Candidate? nominal=null;
            bool fine=false,scenarios=false;int attempts=0,searchSteps=0,validationSteps=0;
            Trial Finish(string reason)=>new Trial(solver,status,nominal,fine,scenarios,reason,attempts,searchSteps,
                validationSteps,searchMs,clock.Elapsed.TotalMilliseconds,worstMiss,minimumMass);
            if(settings==null||!settings.Valid||settings.MinCoast!=0||env==null||!env.Valid||
                propulsion==null||limits==null||!limits.Valid||!initial.Valid||
                validationBudget<2||validationBudget>1000000||wallMilliseconds<0||wallMilliseconds>120000||
                !Enum.IsDefined(typeof(Method),solver))return Finish("invalid common fixture (minimum coast must be zero)");
            using(var timeout=CancellationTokenSource.CreateLinkedTokenSource(cancellation))
            {
                if(wallMilliseconds>0)timeout.CancelAfter(wallMilliseconds);
                var token=timeout.Token;
                try
                {
                    var common=settings.Common;
                    if(solver!=Method.TimingReference)
                    {
                        bool convex=solver==Method.ConvexShooting;
                        var report=F.Search(env,propulsion,limits,initial,settings,token,convex:convex);
                        status=report.Status;nominal=report.Best?.Result;
                        attempts=report.Attempts;searchSteps=report.Steps;searchMs=clock.Elapsed.TotalMilliseconds;
                        if(token.IsCancellationRequested){status=P.SearchStatus.Cancelled;return Finish("cancelled or timeout during search");}
                        if(!nominal.HasValue)return Finish("no modelled contact candidate");
                        var selection=F.SelectValidated(env,propulsion,limits,initial,settings,report,
                            Math.Max(2,validationBudget/3),validationBudget-validationBudget/3,
                            20,Math.Max(.5,initial.Mass*.0005),token,convex);
                        validationSteps=selection.ReplaySteps+selection.ScenarioSteps;
                        fine=selection.FineReplayPassed;scenarios=selection.Candidate!=null;
                        if(selection.Scenarios!=null)
                        {
                            worstMiss=selection.Scenarios.WorstMiss;minimumMass=selection.Scenarios.MinimumEndMass;
                        }
                        if(scenarios)nominal=selection.Candidate.Result;
                        return Finish(selection.Reason+"; validation attempts="+selection.Attempts);
                    }
                    // Keep A as the timing-only reference, but validate
                    // its ranked alternatives with the same total budget.
                    var timing=P.Search(env,propulsion,limits,initial,common,0,new D.Command(0,initial.Forward),token);
                    status=timing.Status;nominal=timing.Best;attempts=timing.Candidates.Count;searchSteps=timing.Steps;
                    searchMs=clock.Elapsed.TotalMilliseconds;
                    if(token.IsCancellationRequested){status=P.SearchStatus.Cancelled;return Finish("cancelled or timeout during search");}
                    if(!nominal.HasValue)return Finish("no modelled contact candidate");
                    var ranked=new List<P.Candidate>();
                    foreach(var candidate in timing.Candidates)if(candidate.Feasible)ranked.Add(candidate);
                    ranked.Sort((a,b)=>P.Better(a,b,common.TargetTolerance)?-1:P.Better(b,a,common.TargetTolerance)?1:0);
                    int tried=0,replayUsed=0,scenarioUsed=0;
                    int replayBudget=Math.Max(2,validationBudget/3),scenarioBudget=validationBudget-replayBudget;
                    string reason="no modelled contact candidate";
                    foreach(var candidate in ranked)
                    {
                        if(token.IsCancellationRequested)return Finish("cancelled");
                        if(tried>=12||replayBudget-replayUsed<2||scenarioBudget-scenarioUsed<1)
                            return Finish("validation budget exhausted: "+reason);
                        tried++;
                        var points=new List<F.Point>();var history=new List<R.Segment>();
                        var a=P.Evaluate(env,propulsion,limits,initial,common,candidate.Profile,
                            (replayBudget-replayUsed)/2,token,(state,command,end)=>{
                                points.Add(new F.Point(state,command,end));
                                history.Add(new R.Segment(state.UT,end,command));
                            });
                        replayUsed+=a.Steps;validationSteps=replayUsed+scenarioUsed;
                        var recorded=new F.Candidate(new F.Profile(a.Profile.IgnitionDelay,settings.MaxPoweredSeconds),
                            a,0,0,double.NaN,points,history);
                        bool pass=F.ValidateRecorded(env,propulsion,limits,initial,settings,recorded,replayBudget-replayUsed,
                            20,Math.Max(.5,initial.Mass*.0005),out var replay,out reason,token);
                        replayUsed+=replay?.Steps??0;validationSteps=replayUsed+scenarioUsed;
                        if(!pass)continue;
                        fine=true;
                        var tracker=new AtmosphericPlanTracker(env,propulsion,limits,settings,recorded);
                        var uncertainty=AtmosphericUncertaintyReplay.Validate(env,propulsion,limits,tracker,scenarioBudget-scenarioUsed,token);
                        scenarioUsed+=uncertainty.Steps;validationSteps=replayUsed+scenarioUsed;
                        worstMiss=uncertainty.WorstMiss;minimumMass=uncertainty.MinimumEndMass;
                        reason=uncertainty.Reason;
                        if(uncertainty.Passed&&!token.IsCancellationRequested)
                        {
                            scenarios=true;nominal=a;
                            return Finish("all sampled model contacts validated; validation attempts="+tried);
                        }
                    }
                    return Finish(reason+"; validation attempts="+tried);
                }
                catch(Exception ex){return Finish(ex.GetType().Name+": "+ex.Message);}
            }
        }

        internal static Trial[] Compare(D.Environment env,D.Propulsion propulsion,D.Limits limits,D.State initial,
            F.Settings settings,int validationBudget=120000,int wallMilliseconds=10000,CancellationToken token=default)
        {
            return new[]{
                Run(Method.TimingReference,env,propulsion,limits,initial,settings,validationBudget,wallMilliseconds,token),
                Run(Method.FiniteTimeShooting,env,propulsion,limits,initial,settings,validationBudget,wallMilliseconds,token),
                Run(Method.ConvexShooting,env,propulsion,limits,initial,settings,validationBudget,wallMilliseconds,token)
            };
        }
    }
}
