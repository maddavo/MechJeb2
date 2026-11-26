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
                    F.Candidate selected=null;
                    if(solver==Method.TimingReference)
                    {
                        var report=P.Search(env,propulsion,limits,initial,settings.Common,0,
                            new D.Command(0,initial.Forward),token);
                        status=report.Status;nominal=report.Best;attempts=report.Candidates.Count;searchSteps=report.Steps;
                    }
                    else
                    {
                        var report=F.Search(env,propulsion,limits,initial,settings,token,convex:solver==Method.ConvexShooting);
                        status=report.Status;selected=report.Best;nominal=selected?.Result;
                        attempts=report.Attempts;searchSteps=report.Steps;
                    }
                    searchMs=clock.Elapsed.TotalMilliseconds;
                    if(token.IsCancellationRequested){status=P.SearchStatus.Cancelled;return Finish("cancelled or timeout during search");}
                    if(!nominal.HasValue)return Finish("no modelled contact candidate");
                    // Reconstruct the selected policy at the common model
                    // step. Capture A's ACTUAL terminal PID commands, not
                    // the direct planner's terminal commands.
                    F.Candidate recorded;
                    if(solver==Method.TimingReference)
                    {
                        var points=new List<F.Point>();var history=new List<R.Segment>();
                        var a=P.Evaluate(env,propulsion,limits,initial,settings.Common,nominal.Value.Profile,
                            validationBudget/2,token,(state,command,end)=>{
                                points.Add(new F.Point(state,command,end));
                                history.Add(new R.Segment(state.UT,end,command));
                            });
                        recorded=new F.Candidate(new F.Profile(a.Profile.IgnitionDelay,settings.MaxPoweredSeconds),
                            a,0,0,double.NaN,points,history);
                    }
                    else recorded=F.Evaluate(env,propulsion,limits,initial,settings,selected.Profile,
                        validationBudget/2,true,token,solver==Method.ConvexShooting);
                    validationSteps+=recorded.Result.Steps;
                    if(!recorded.Result.Feasible||recorded.Commands.Count==0||validationSteps>=validationBudget)
                        return Finish("selected trajectory reconstruction failed: "+recorded.Result.Outcome);
                    var commands=new R.Segment[recorded.Commands.Count];
                    for(int i=0;i<commands.Length;i++)commands[i]=recorded.Commands[i];
                    var common=settings.Common;
                    var finer=new D.Limits(limits.ReserveMass,limits.TurnRate,limits.EngineRise,limits.EngineFall,limits.Step*.5);
                    var replay=R.Run(env,propulsion,finer,initial,commands,Array.Empty<R.Observation>(),
                        common.SafeVertical,common.SafeHorizontal,common.SafeAngle,common.SafeRate,
                        validationBudget-validationSteps,token);
                    validationSteps+=replay.Steps;
                    fine=replay.Outcome==D.Outcome.ContactWithinLimits&&
                        (recorded.Result.End.R-replay.End.R).Length<=20&&
                        Math.Abs(recorded.Result.End.Mass-replay.End.Mass)<=Math.Max(.5,initial.Mass*.0005)&&
                        Math.Abs(recorded.Result.End.UT-replay.End.UT)<=settings.ControlTick+common.Terminal.Tick&&
                        (recorded.Result.Miss>common.TargetTolerance||
                            P.Miss(env,initial,replay.End,common.TargetAtInitialUT)<=common.TargetTolerance);
                    if(!fine)return Finish("finer replay rejected: "+replay.Outcome);
                    if(validationSteps>=validationBudget)return Finish("validation budget exhausted");
                    var tracker=new AtmosphericPlanTracker(env,propulsion,limits,settings,recorded);
                    var uncertainty=AtmosphericUncertaintyReplay.Validate(env,propulsion,limits,tracker,
                        validationBudget-validationSteps,token);
                    validationSteps+=uncertainty.Steps;scenarios=uncertainty.Passed;
                    worstMiss=uncertainty.WorstMiss;minimumMass=uncertainty.MinimumEndMass;
                    return Finish(scenarios?"all sampled model contacts validated":uncertainty.Reason);
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
