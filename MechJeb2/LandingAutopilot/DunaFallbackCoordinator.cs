using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using Fallback=MuMech.Landing.AtmosphericFallbackPlanner;
using Replanning=MuMech.Landing.AtmosphericReplanning;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;

namespace MuMech.Landing
{
    // Rolling, precomputed local-site contingency. No solve or terrain
    // search may delay a live fault decision. KSP APIs only on main thread.
    internal sealed class DunaFallbackCoordinator
    {
        private Task<Fallback.Result> job;
        private CancellationTokenSource cancel;
        private Replanning.Request request;
        private D.V[] radials;
        private readonly List<Fallback.Site> sites=new List<Fallback.Site>();
        private int index,generation;
        private double nextSampleUT,nextRequestUT,deadline;
        private Fallback.Result pending,ready;
        private DunaPlanCorridor corridor;
        private AtmosphericPlanTracker owner;
        internal bool Busy=>request!=null||job!=null||pending!=null;
        internal string Status {get;private set;}="no local-site contingency";
        internal int Prepared {get;private set;}
        private void ClearPending()
        {
            if(cancel!=null){cancel.Cancel();cancel.Dispose();cancel=null;}
            job=null;request=null;radials=null;sites.Clear();pending=null;corridor=null;
        }
        internal void Cancel()
        {
            generation++;ClearPending();ready=null;owner=null;
            Status="no local-site contingency";
        }
        private void Reject(string reason,double ut)
        {
            Status="fallback unavailable: "+reason;
            UnityEngine.Debug.Log("[DunaFallback] rejected generation="+generation+" UT="+ut+" "+reason);
            ClearPending();nextRequestUT=ut+2;
        }

        internal Fallback.Result Tick(MechJebCore core,DunaIntegratedPreview.Snapshot snapshot,
            AtmosphericPlanTracker current,D.State actual,AtmosphericPlanTracker.Decision decision,
            double clearance,double rawAccel,double cap,double gravity,double response,bool allowNew)
        {
            try
            {
                return TickCore(core,snapshot,current,actual,decision,clearance,rawAccel,cap,gravity,response,allowNew);
            }
            catch(Exception ex)
            {
                Reject("contingency preparation: "+ex.Message,actual.UT);
                return null;
            }
        }
        private Fallback.Result TickCore(MechJebCore core,DunaIntegratedPreview.Snapshot snapshot,
            AtmosphericPlanTracker current,D.State actual,AtmosphericPlanTracker.Decision decision,
            double clearance,double rawAccel,double cap,double gravity,double response,bool allowNew)
        {
            double now=actual.UT;
            if(!ReferenceEquals(current,owner))
            {
                Cancel();owner=current;nextRequestUT=now;
            }
            if(ready!=null&&(now>ready.SwitchUT+2||now>=ready.Tracker.EndUT))ready=null;
            // Only a previously replayed and screened candidate can be
            // used on fault. No waiting for a new worker or PQS query.
            if(!decision.Valid)
            {
                if(ready!=null)
                {
                    string reason=Fallback.Admission(ready,ready.Generation,actual,clearance,
                        rawAccel,cap,gravity,response,current.ReserveMass);
                    if(reason==null)
                    {
                        var result=ready;
                        UnityEngine.Debug.Log("[DunaFallback] engaged local-site contingency; original target miss="+
                            result.OriginalMiss+" site miss="+result.Tracker.TargetMiss+" reserveKg="+result.Tracker.ReserveMass);
                        Cancel();return result;
                    }
                    Status="fallback refused: "+reason;
                }
                return null;
            }
            if(Busy&&now>=deadline)
            {Reject("local-site preparation exceeded handoff deadline",now);return null;}
            if(request!=null&&now>=nextSampleUT)
            {
                nextSampleUT=now+.02;
                var body=core.vessel.mainBody;
                D.V radial=radials[index++];
                // These input radials refer to observed UT; sample their
                // body-fixed locations at the present body orientation.
                Vector3d world=body.position+DunaIntegratedPreview.WorldAt(body,snapshot.Frame,
                    radial*body.Radius,request.Observed.UT);
                double lat=body.GetLatitude(world),lon=body.GetLongitude(world);
                double ground=body.TerrainAltitude(lat,lon);
                if(D.Finite(ground)&&ground>=0)sites.Add(new Fallback.Site(radial,body.Radius+ground));
                if(index==radials.Length)
                {
                    var work=request;var copy=sites.ToArray();
                    request=null;radials=null;sites.Clear();
                    cancel=new CancellationTokenSource();cancel.CancelAfter(750);
                    CancellationToken token=cancel.Token;
                    job=Task.Run(()=>Fallback.Solve(work,copy,token));
                    Status="searching local-site trajectories";
                }
            }
            if(job!=null&&job.IsCompleted)
            {
                try{pending=job.GetAwaiter().GetResult();}
                catch(Exception ex){Reject(ex.Message,now);return null;}
                finally{job=null;cancel?.Dispose();cancel=null;}
                if(pending==null||pending.Generation!=generation||pending.Tracker==null)
                {Reject(pending?.Reason??"no candidate",now);return null;}
                corridor=new DunaPlanCorridor(pending.Environment,pending.Tracker,snapshot.Frame,true);
                Status="screening local-site trajectory";
            }
            if(pending!=null)
            {
                if(now>=pending.SwitchUT)
                {Reject("fallback screening deadline missed",now);return null;}
                corridor.Advance(core);
                if(corridor.Finished)
                {
                    if(!corridor.Passed){Reject(corridor.Status,now);return null;}
                    ready=pending;pending=null;corridor=null;Prepared++;
                    // Do not replace a still-admissible ready contingency
                    // with a future candidate before its window ends.
                    nextRequestUT=ready.SwitchUT+2;
                    Status="local-site candidate screened; activation UT "+ready.SwitchUT.ToString("F2");
                    UnityEngine.Debug.Log("[DunaFallback] ready generation="+ready.Generation+" switchUT="+ready.SwitchUT+
                        " originalMiss="+ready.OriginalMiss+" siteMiss="+ready.Tracker.TargetMiss+
                        " endMassKg="+ready.Tracker.Plan.Result.End.Mass+" attempts="+ready.Attempts+" steps="+ready.Steps);
                }
            }
            if(allowNew&&!Busy&&now>=nextRequestUT&&clearance>500&&current.EndUT-now>25&&
                (!D.Finite(current.Plan.Result.TerminalStartUT)||current.Plan.Result.TerminalStartUT-now>15))
            {
                const double lead=5;
                radials=Fallback.SiteRadials(snapshot.Environment,actual,rawAccel*cap,lead);
                if(radials.Length==0){nextRequestUT=now+2;return null;}
                deadline=now+lead;
                request=new Replanning.Request(++generation,actual,deadline,cap,snapshot.Environment,
                    snapshot.Propulsion,snapshot.Limits,current.Settings,current);
                index=0;sites.Clear();nextSampleUT=now;nextRequestUT=now+lead;
                Status="sampling alternative sites";
            }
            return null;
        }
    }
}
