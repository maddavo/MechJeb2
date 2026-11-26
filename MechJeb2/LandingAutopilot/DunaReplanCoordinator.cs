using System;
using System.Threading;
using System.Threading.Tasks;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using Replanning=MuMech.Landing.AtmosphericReplanning;

namespace MuMech.Landing
{
    // One outstanding solve per active landing. Main-thread lifecycle and
    // corridor queries; worker receives only numerical immutable inputs.
    internal sealed class DunaReplanCoordinator
    {
        private Task<Replanning.Result> job;
        private CancellationTokenSource cancel;
        private Replanning.Result candidate;
        private DunaPlanCorridor corridor;
        private int generation;
        private double switchUT,nextRequestUT;
        private string status="initial plan";
        internal string Status=>status;
        internal bool Busy=>job!=null||candidate!=null;
        internal int Accepted {get;private set;}
        internal int Rejected {get;private set;}
        private void Clear()
        {
            if(cancel!=null){cancel.Cancel();cancel.Dispose();cancel=null;}
            job=null;candidate=null;corridor=null;
        }
        internal void Cancel()
        {
            generation++;
            Clear();
        }
        private void Reject(string reason,double ut)
        {
            Rejected++;
            status="retained current plan: "+reason;
            UnityEngine.Debug.Log("[DunaReplan] rejected generation="+generation+" UT="+ut+" "+reason);
            Cancel();
            nextRequestUT=ut+5;
        }
        internal AtmosphericPlanTracker Tick(MechJebCore core,DunaIntegratedPreview.Snapshot snapshot,
            AtmosphericPlanTracker current,D.State actual,AtmosphericPlanTracker.Decision oldDecision,
            double clearance,double rawAccel,double cap,double gravity,double response,bool allowNew=true)
        {
            double now=actual.UT;
            // Never delay a loss-of-control decision waiting for a worker.
            // No speculative new burn is commanded during search.
            if(job!=null&&job.IsCompleted)
            {
                try{candidate=job.GetAwaiter().GetResult();}
                catch(Exception ex){Reject("worker exception: "+ex.Message,now);return current;}
                finally{job=null;cancel?.Dispose();cancel=null;}
                if(candidate==null||candidate.Generation!=generation||candidate.Tracker==null)
                {Reject(candidate?.Reason??"no candidate",now);return current;}
                corridor=new DunaPlanCorridor(snapshot.Environment,candidate.Tracker,snapshot.Frame);
                status="screening replacement";
            }
            if(candidate!=null)
            {
                if(now>switchUT+.08)
                {Reject("handoff deadline missed",now);return current;}
                if(!corridor.Finished&&now<switchUT)corridor.Advance(core);
                if(corridor.Finished&&!corridor.Passed)
                {Reject(corridor.Status,now);return current;}
                if(now>=switchUT)
                {
                    string reason=!corridor.Passed?"terrain screen incomplete":
                        Replanning.Admission(candidate,generation,actual,clearance,rawAccel,cap,gravity,response,
                            oldDecision.Valid,current.Settings.Common.TargetTolerance,current.ReserveMass);
                    if(reason==null&&oldDecision.Valid&&
                        candidate.Tracker.Plan.Result.End.Mass<current.Plan.Result.End.Mass-Math.Max(10,actual.Mass*.002)&&
                        oldDecision.PositionError<5&&oldDecision.VelocityError<.5)
                        reason="replacement spends additional fuel without significant deviation";
                    if(reason!=null){Reject(reason,now);return current;}
                    AtmosphericPlanTracker next=candidate.Tracker;
                    bool degraded=next.TargetMiss>current.Settings.Common.TargetTolerance;
                    status=degraded?"degraded replacement: modelled off-target contact":"replacement accepted";
                    Accepted++;
                    UnityEngine.Debug.Log("[DunaReplan] accepted generation="+generation+" UT="+now+
                        " degraded="+degraded+" predictedMiss="+next.TargetMiss+" endMassKg="+next.Plan.Result.End.Mass+
                        " attempts="+candidate.Attempts+" steps="+candidate.Steps+" solveMs="+candidate.SolveMs);
                    Cancel();nextRequestUT=now+10;
                    return next;
                }
            }
            else if(job!=null&&now>=switchUT)
            {Reject("worker exceeded handoff deadline",now);return current;}

            if(allowNew&&job==null&&candidate==null&&oldDecision.Valid&&now>=nextRequestUT&&
                clearance>500&&current.EndUT-now>25&&
                (!D.Finite(current.Plan.Result.TerminalStartUT)||current.Plan.Result.TerminalStartUT-now>15))
            {
                // Periodic refresh, faster when approaching the envelope.
                // Use the original absolute mass floor; never reset reserve
                // to a percentage of the fuel remaining at each replan.
                nextRequestUT=now+10;
                switchUT=now+5;
                var request=new Replanning.Request(++generation,actual,switchUT,cap,snapshot.Environment,
                    snapshot.Propulsion,snapshot.Limits,current.Settings,current);
                cancel=new CancellationTokenSource();cancel.CancelAfter(750);
                CancellationToken token=cancel.Token;
                job=Task.Run(()=>Replanning.Solve(request,token));
                status="replanning to UT "+switchUT.ToString("F2");
                UnityEngine.Debug.Log("[DunaReplan] requested generation="+generation+" UT="+now+
                    " switchUT="+switchUT+" positionError="+oldDecision.PositionError+
                    " velocityError="+oldDecision.VelocityError+" reserveKg="+current.ReserveMass);
            }
            return current;
        }
    }
}
