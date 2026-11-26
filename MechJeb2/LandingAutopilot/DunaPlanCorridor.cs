using System;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;

namespace MuMech.Landing
{
    // Reused for initial and replacement plans. Sampled terrain only;
    // no claim of a certified continuous corridor. Main-thread only.
    internal sealed class DunaPlanCorridor
    {
        private readonly D.Environment environment;
        private readonly AtmosphericPlanTracker tracker;
        private int index,queries;
        private double nextUT=double.NegativeInfinity;
        internal bool Passed {get;private set;}
        internal bool Finished {get;private set;}
        internal string Status {get;private set;}="Checking sampled terrain";
        internal DunaPlanCorridor(D.Environment env,AtmosphericPlanTracker tracker)
        {environment=env;this.tracker=tracker;}
        private void Reject(string reason){Status="Rejected: "+reason;Finished=true;}
        internal void Advance(MechJebCore core)
        {
            if(Finished)return;
            try
            {
                double now=core.VesselState.Time;
                if(now>=tracker.EndUT){Reject("trajectory expired");return;}
                if(now<nextUT)return;
                nextUT=now+.02;
                var points=tracker.Plan.Reference;
                int stride=Math.Max(1,(points.Count+127)/128);
                var body=core.vessel.mainBody;
                int work=0,scanned=0;
                var clock=System.Diagnostics.Stopwatch.StartNew();
                while(index<=points.Count&&work<8&&scanned++<128&&
                    (work==0||clock.Elapsed.TotalMilliseconds<1))
                {
                    int i=index++;
                    bool contact=i==points.Count;
                    D.State pt=contact?tracker.Plan.Result.End:points[i].State;
                    double modelClearance=environment.PhysicalClearance(pt);
                    if(!D.Finite(modelClearance)){Reject("unknown reference clearance");return;}
                    if(!contact&&i%stride!=0&&modelClearance>300)continue;
                    if(++queries>512){Reject("corridor sample budget");return;}
                    work++;
                    D.V radial=P.RotateTarget(pt.R.Unit,environment.Omega,now-pt.UT);
                    Vector3d world=body.position+DunaIntegratedPreview.World(radial*body.Radius);
                    double lat=body.GetLatitude(world),lon=body.GetLongitude(world);
                    double ground=body.TerrainAltitude(lat,lon);
                    if(contact||modelClearance<=300)
                    {
                        ground=core.Landing.LandingSurface.Altitude(body,lat,lon,ground);
                        if(core.Landing.LandingSurface.LastQuerySaturated)
                        {Reject("local collider surface query saturated");return;}
                    }
                    if(!D.Finite(ground)){Reject("unknown terrain");return;}
                    double sampledClearance=pt.R.Length-body.Radius-ground-environment.BottomOffset;
                    if(contact||modelClearance<=300)
                    {
                        if(Math.Abs(sampledClearance-modelClearance)>2)
                        {Reject("near-ground terrain differs from model");return;}
                    }
                    else if(sampledClearance<modelClearance-Math.Min(100,Math.Max(2,modelClearance*.1)))
                    {Reject("terrain rises into model corridor");return;}
                }
                Passed=index>points.Count;Finished=Passed;
                Status=Passed?"Sampled corridor passed (not swept validation)":
                    "Checking sampled terrain ("+queries+")";
            }
            catch(Exception ex){Reject(ex.Message);}
        }
    }
}
