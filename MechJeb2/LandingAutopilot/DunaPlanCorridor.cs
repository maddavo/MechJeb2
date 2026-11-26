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
        private readonly DunaLandingAdmission.SolverFrame frame;
        private int index,queries,footprintIndex;
        private readonly bool requireFootprint;
        private double contactGround;
        private double nextUT=double.NegativeInfinity;
        internal bool Passed {get;private set;}
        internal bool NeedsTerrainRefinement {get;private set;}
        internal double ConflictLatitude {get;private set;}
        internal double ConflictLongitude {get;private set;}
        internal bool Finished {get;private set;}
        internal string Status {get;private set;}="Checking sampled terrain";
        internal DunaPlanCorridor(D.Environment env,AtmosphericPlanTracker tracker,DunaLandingAdmission.SolverFrame frame,bool requireFootprint=false)
        {environment=env;this.tracker=tracker;this.frame=frame;this.requireFootprint=requireFootprint;}
        private void Reject(string reason){Status="Rejected: "+reason;Finished=true;}
        // Compare actual available clearance with the physical trajectory,
        // not with the arbitrary height of a target-elevation sphere.
        internal static bool TerrainConflict(double clearance,double modelClearance,bool contact)
            =>contact?Math.Abs(clearance)>2:clearance<=0||
                modelClearance<=30&&Math.Abs(clearance-modelClearance)>2;
        private void Refine(double lat,double lon,double clearance,double modelClearance)
        {
            NeedsTerrainRefinement=true;ConflictLatitude=lat;ConflictLongitude=lon;
            Reject("terrain requires trajectory refinement at "+lat.ToString("F5")+","+lon.ToString("F5")+
                " clearance="+clearance.ToString("F1")+" model="+modelClearance.ToString("F1"));
        }
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
                int stride=Math.Max(1,(points.Count+255)/256);
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
                    // Fine samples where actual terrain could approach the
                    // path. Coarser sampling in high-altitude flight.
                    double datum=pt.R.Length-environment.Radius-environment.BottomOffset;
                    double top=environment.Grid==null?environment.TerrainRadius-environment.Radius:environment.Grid.Max;
                    if(!contact&&i%stride!=0&&datum>top+300&&modelClearance>300)continue;
                    if(++queries>4096){Reject("corridor sample budget");return;}
                    work++;
                    Vector3d world=body.position+DunaIntegratedPreview.WorldAt(body,frame,pt.R.Unit*body.Radius,pt.UT);
                    double lat=body.GetLatitude(world),lon=body.GetLongitude(world);
                    double ground=body.TerrainAltitude(lat,lon);
                    if(contact||modelClearance<=300)
                    {
                        ground=core.Landing.LandingSurface.Altitude(body,lat,lon,ground);
                        if(core.Landing.LandingSurface.LastQuerySaturated)
                        {Reject("local collider surface query saturated");return;}
                    }
                    if(!D.Finite(ground)){Reject("unknown terrain");return;}
                    if(contact)contactGround=ground;
                    double sampledClearance=pt.R.Length-body.Radius-ground-environment.BottomOffset;
                    if(TerrainConflict(sampledClearance,modelClearance,contact))
                    {
                        Refine(lat,lon,sampledClearance,modelClearance);return;
                    }
                }
                // Additional local footprint samples for a fallback site.
                // This 25m cross is a screening heuristic, not a swept
                // collider or slope certification for the vessel footprint.
                if(index>points.Count&&requireFootprint&&footprintIndex<4&&work<8&&
                    (work==0||clock.Elapsed.TotalMilliseconds<1))
                {
                    D.State end=tracker.Plan.Result.End;
                    D.V up=end.R.Unit;
                    D.V tangent=D.V.Cross(up,Math.Abs(up.X)<.8?new D.V(1,0,0):new D.V(0,1,0)).Unit;
                    D.V across=D.V.Cross(up,tangent).Unit;
                    D.V axis=footprintIndex<2?tangent:across;
                    double angle=(footprintIndex%2==0?1:-1)*25/environment.Radius;
                    D.V radial=(up*Math.Cos(angle)+axis*Math.Sin(angle)).Unit;
                    Vector3d world=body.position+DunaIntegratedPreview.WorldAt(body,frame,radial*body.Radius,end.UT);
                    double lat=body.GetLatitude(world),lon=body.GetLongitude(world);
                    double ground=body.TerrainAltitude(lat,lon);
                    if(++queries>4096){Reject("footprint sample budget");return;}
                    ground=core.Landing.LandingSurface.Altitude(body,lat,lon,ground);
                    if(core.Landing.LandingSurface.LastQuerySaturated||!D.Finite(ground)||
                        Math.Abs(ground-contactGround)>2)
                    {Reject("fallback footprint height variation or unknown collider");return;}
                    footprintIndex++;
                }
                Passed=index>points.Count&&(!requireFootprint||footprintIndex==4);Finished=Passed;
                Status=Passed?"Sampled corridor passed (not swept validation)":
                    "Checking sampled terrain ("+queries+")";
            }
            catch(Exception ex){Reject(ex.Message);}
        }
    }
}
