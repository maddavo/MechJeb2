using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;
using MechJebLibBindings;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;
using Debug=UnityEngine.Debug;

namespace MuMech.Landing
{
    // Experimental model preview only. All Unity/KSP access is on the
    // caller's main thread. The worker receives numerical copies, never a
    // Vessel, CelestialBody, Part, Unity curve, or mutable game reference.
    internal sealed class DunaIntegratedPreview
    {
        private sealed class EngineCopy
        {
            internal double Flow,G;
            internal double[] Isp;
        }
        internal sealed class Snapshot
        {
            internal Guid Vessel;
            internal uint[] Parts;
            internal uint ReferencePart;
            internal string EngineConfiguration;
            internal double Latitude,Longitude,UT,Mass,Cap,Warp,DragArea,UsableFuel;
            internal D.State Initial;
            internal DunaLandingAdmission.SolverFrame Frame;
            internal D.Environment Environment;
            internal D.Propulsion Propulsion;
            internal D.Limits Limits;
            internal F.Settings Settings;
            internal bool Strategic;
            internal double ApproachStartUT,DeorbitStartUT,DeorbitEndUT;
            internal double PreferredBurnUT=double.NaN;
            internal F.Profile? PreferredApproach;
        }
        private sealed class Answer
        {
            internal P.Report Reference;
            internal F.Report Direct;
            internal AtmosphericStrategicPlanner.Report Strategic;
            internal F.Candidate Validated;
            internal bool ReplayPassed;
            internal AtmosphericUncertaintyReplay.Report Scenarios;
            internal double ReferenceMs,ReplayMs;
            internal string Error;
        }
        private Task<Answer> job;
        private CancellationTokenSource cancel;
        private Snapshot snapshot;
        private Answer answer;
        private string status="Not evaluated";
        private bool logged;
        private AtmosphericPlanTracker tracker;
        private string corridorStatus;
        private bool corridorPassed;
        private string executionStatus;
        private TerrainAcquisition terrainCapture;
        private D.TerrainGrid terrainGrid;
        private int terrainRefinements;
        private double protectedReserve;
        private double preferredBurnUT=double.NaN;
        private F.Profile? preferredApproach;
        // Incremental main-thread PQS sampling. Use body-fixed coordinates
        // so a fresh solver epoch does not invalidate acquired elevations.
        private sealed class TerrainAcquisition
        {
            private const int N=12,PatchN=10;
            private readonly double[] globe;
            private readonly List<(D.V centre,double half)> requests=new List<(D.V,double)>();
            private double[] patch;
            private int index,patchIndex;
            internal D.TerrainGrid Grid {get;private set;}
            internal bool Complete=>Grid!=null&&patchIndex>=requests.Count;
            internal int Samples {get;private set;}
            internal TerrainAcquisition(D.TerrainGrid previous,double latitude,double longitude,double radius,
                IEnumerable<D.V> approachRadials=null)
            {
                Grid=previous;globe=previous==null?new double[6*(N+1)*(N+1)]:null;
                double lat=latitude*Math.PI/180,lon=longitude*Math.PI/180;
                D.V q=new D.V(Math.Cos(lat)*Math.Cos(lon),Math.Cos(lat)*Math.Sin(lon),Math.Sin(lat));
                // Regional path patches first, fine destination last.
                if(approachRadials!=null)
                    foreach(var radial in approachRadials)
                        requests.Add((radial.Unit,Math.Min(.25,6000/radius)));
                requests.Add((q,Math.Min(.25,4000/radius)));
                requests.Add((q,Math.Min(.01,250/radius)));
                patch=new double[(PatchN+1)*(PatchN+1)];
            }
            private static double Sample(MechJebCore core,D.V q,bool collider)
            {
                double lat=Math.Asin(Math.Max(-1,Math.Min(1,q.Z)))*180/Math.PI;
                double lon=Math.Atan2(q.Y,q.X)*180/Math.PI;
                var body=core.vessel.mainBody;
                double h=body.TerrainAltitude(lat,lon);
                if(collider)
                {
                    h=core.Landing.LandingSurface.Altitude(body,lat,lon,h);
                    Require(!core.Landing.LandingSurface.LastQuerySaturated,"terrain collider query saturated");
                }
                Require(D.Finite(h),"terrain sample unavailable");return h;
            }
            internal void Advance(MechJebCore core)
            {
                var clock=Stopwatch.StartNew();
                for(int work=0;work<24&&!Complete&&(work==0||clock.Elapsed.TotalMilliseconds<2);work++)
                {
                    if(Grid==null)
                    {
                        int side=N+1,face=index/(side*side),k=index%(side*side);
                        globe[index++]=Sample(core,D.TerrainGrid.Direction(face,N,k%side,k/side),false);Samples++;
                        if(index==globe.Length){Grid=new D.TerrainGrid(N,globe);index=0;}
                        continue;
                    }
                    var request=requests[patchIndex];
                    // Same frame as immutable Patch.Direction; no
                    // allocation of a temporary patch for each sample.
                    D.V centre=request.centre;
                    D.V a=D.V.Cross(centre,Math.Abs(centre.X)<.8?new D.V(1,0,0):new D.V(0,1,0)).Unit;
                    D.V b=D.V.Cross(centre,a).Unit;
                    int n=PatchN+1,i=index%n,j=index/n;
                    D.V q=(centre+a*(request.half*(2.0*i/PatchN-1))+
                        b*(request.half*(2.0*j/PatchN-1))).Unit;
                    patch[index++]=Sample(core,q,false);Samples++;
                    if(index==patch.Length)
                    {
                        Grid=Grid.Refine(new D.TerrainGrid.Patch(centre,request.half,PatchN,patch));
                        index=0;patchIndex++;patch=new double[patch.Length];
                    }
                }
            }
        }
        internal static D.V V(Vector3d x)=>new D.V(x.x,x.y,x.z);
        internal static Vector3d World(D.V x)=>new Vector3d(x.X,x.Y,x.Z);
        private static string Num(double x,string format="F1")=>D.Finite(x)?x.ToString(format,CultureInfo.InvariantCulture):"unknown";
        private static void Require(bool ok,string reason){if(!ok)throw new InvalidOperationException(reason);}

        // Configuration identity excludes attained throttle, pressure and
        // current thrust (those vary during the validated trajectory).
        internal static string EngineSignature(Vessel v)
        {
            var text=new System.Text.StringBuilder();
            int modules=0;
            foreach(Part part in v.parts)
                for(int i=0;i<part.Modules.Count;i++)
                {
                    if(++modules>2048)throw new InvalidOperationException("module budget");
                    if(!(part.Modules[i] is ModuleEngines e))continue;
                    text.Append(part.flightID).Append(':').Append(i).Append(':')
                        .Append(e.EngineIgnited).Append(':').Append(e.isEnabled).Append(':')
                        .Append(e.isOperational).Append(':').Append(e.throttleLocked).Append(':')
                        .Append(Num(e.maxFuelFlow,"R")).Append(':').Append(Num(e.minFuelFlow,"R")).Append(':')
                        .Append(Num(e.thrustPercentage,"R")).Append(';');
                }
            return text.ToString();
        }
        internal static DunaLandingAdmission.BodyBasis BodyBasis(CelestialBody body)
            =>new DunaLandingAdmission.BodyBasis(V(body.GetSurfaceNVector(0,0)),
                V(body.GetSurfaceNVector(0,90)),V(body.GetSurfaceNVector(90,0)));
        internal static Vector3d WorldAt(CelestialBody body,DunaLandingAdmission.SolverFrame frame,D.V vector,double ut)
            =>World(frame.ToWorld(BodyBasis(body),vector,ut));
        internal static D.State LiveState(MechJebCore core,Snapshot snap)
        {
            var s=core.VesselState;
            double attained=s.MaxThrustAcceleration>0?
                Math.Max(0,Math.Min(1,s.CurrentThrustAcceleration/s.MaxThrustAcceleration)):double.NaN;
            var basis=BodyBasis(core.vessel.mainBody);
            return new D.State(s.Time,snap.Frame.ToSolver(basis,V(s.OrbitalPosition),s.Time),
                snap.Frame.ToSolver(basis,V(s.OrbitalVelocity),s.Time),s.Mass*1000,
                snap.Frame.ToSolver(basis,V(s.Forward),s.Time).Unit,attained,core.vessel.angularVelocity.magnitude);
        }

        // Snapshot uses a reduced-order equivalent axial engine, frozen
        // measured/estimated drag area, sampled atmosphere at target, and
        // spherical terrain at target ASL. These are explicitly NOT verified
        // real-engine, lift, crossfeed, attitude or swept terrain models.
        // Results are for comparing algorithms, never authorization to burn.
        private static Snapshot Capture(MechJebCore core,double reserveFloor=0)
        {
            Vessel v=core.vessel;
            VesselState s=core.VesselState;
            CelestialBody body=v.mainBody;
            Require(body!=null&&body.bodyName=="Duna"&&body.atmosphere,"Duna atmosphere required");
            Debug.Log("[DunaIntegratedPreview] capture request UT="+Num(s.Time,"R")+" vessel="+v.id+
                " body="+body.bodyName+" ASL="+Num(s.AltitudeASL,"R")+" clearance="+Num(s.AltitudeBottom,"R")+
                " vertical="+Num(s.SpeedVertical,"R")+" horizontal="+Num(s.SpeedSurfaceHorizontal,"R")+
                " massKg="+Num(s.Mass*1000,"R")+" currentThrottle="+Num(v.ctrlState.mainThrottle,"R")+
                " maxAcceleration="+Num(s.MaxThrustAcceleration,"R")+" minAcceleration="+Num(s.MinThrustAcceleration,"R")+
                " liveAreaDrag="+Num(s.AreaDrag,"R")+" chuteOption="+core.Landing.DeployChutes+
                " chuteCount="+s.Parachutes.Count+" RCS="+v.ActionGroups[KSPActionGroup.RCS]+
                " warp="+Num(TimeWarp.CurrentRate,"R"));
            string context=DunaLandingAdmission.CheckContext(FlightGlobals.ActiveVessel==v,v.packed,
                v.LandedOrSplashed,TimeWarp.CurrentRate,core.Target.PositionTargetExists&&core.Target.targetBody==body,
                core.Landing.DeployChutes,s.Parachutes.Count,s.ParachuteDeployed,v.ActionGroups[KSPActionGroup.RCS],
                core.Thrust.LimiterMinThrottle,core.Thrust.DifferentialThrottle,core.Thrust.SmoothThrottle,
                ReflectionUtils.IsLoadedFAR,v.parts==null?0:v.parts.Count);
            Require(context==null,context);
            Require(D.Finite(s.Mass)&&s.Mass>0&&D.Finite(s.Time),"invalid vessel mass or epoch");
            Require(v.GetReferenceTransformPart()!=null,"vessel reference part unavailable");
            double lat=core.Target.targetLatitude,lon=core.Target.targetLongitude;
            double asl=body.TerrainAltitude(lat,lon);
            double radius=body.Radius,mu=body.gravParameter,top=body.RealMaxAtmosphereAltitude();
            double offset=s.TerminalBottomOffset();
            Require(D.Finite(asl)&&D.Finite(radius)&&radius>0&&D.Finite(mu)&&mu>0&&
                D.Finite(top)&&top>0&&asl>=0&&D.Finite(offset)&&offset>=0,"invalid body or target terrain");
            D.V pos=V(s.OrbitalPosition),vel=V(s.OrbitalVelocity),omega=V(body.angularVelocity),forward=V(s.Forward).Unit;
            // Frame sign is checked against the very same sampled inertial
            // and surface velocities; do not silently flip rotation.
            Require((vel-V(s.SurfaceVelocity)-D.V.Cross(omega,pos)).Length<.5,"surface/inertial rotation mismatch");
            var frame=new DunaLandingAdmission.SolverFrame(BodyBasis(body),omega,s.Time);
            D.V target=V(body.GetSurfaceNVector(lat,lon)).Unit;
            Require(target.Valid&&Math.Abs(target.Length-1)<1e-6,"invalid target radial");
            double mass=s.Mass*1000;
            double cap=Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit);
            Require(D.Finite(cap)&&cap>0&&cap<=1&&s.MaxThrustAcceleration>0&&s.MinThrustAcceleration<1e-6,
                "no known throttleable axial thrust");

            // Fixed sample count. KSP evaluates the atmospheric functions
            // only here on the main thread. Use Pa and kg/m³ thereafter.
            const int N=64;
            var pressure=new D.Knot[N+1];var density=new D.Knot[N+1];
            var pressures=new double[N+1];double maxP=0;
            // Copy effective KSP altitude and solar/latitude curves. Use
            // GetTemperature(altitude), not a world-position temperature
            // overload whose captured M5.6 output was near space temperature.
            D.Curve Sample(double lo,double hi,int count,Func<double,double> evaluate,bool signed=false)
            {
                var keys=new D.Knot[count+1];
                for(int i=0;i<=count;i++)
                {
                    double x=lo+(hi-lo)*i/count;
                    keys[i]=new D.Knot(x,evaluate(x));
                }
                return new D.Curve(signed,keys);
            }
            const int airIntervals=256;
            var pCurve=Sample(0,top,airIntervals,h=>body.GetPressure(h)*1000);
            var tCurve=Sample(0,top,airIntervals,h=>body.GetTemperature(h));
            var aSun=Sample(0,top,airIntervals,h=>body.atmosphereTemperatureSunMultCurve.Evaluate((float)h),true);
            var lBias=Sample(0,90,90,latDeg=>body.latitudeTemperatureBiasCurve.Evaluate((float)latDeg),true);
            var lSun=Sample(0,90,90,latDeg=>body.latitudeTemperatureSunMultCurve.Evaluate((float)latDeg),true);
            var xSun=Sample(-1,1,128,dot=>body.axialTemperatureSunMultCurve.Evaluate((float)dot),true);
            // Derive and cross-check the gas coefficient from the actual
            // stock density API (kPa, kelvin), not an assumed gas composition.
            double gas=FlightGlobals.getAtmDensity(1,200,body)*200/1000;
            double gasCheck=FlightGlobals.getAtmDensity(5,300,body)*300/5000;
            Require(D.Finite(gas)&&gas>0&&D.Finite(gasCheck)&&Math.Abs(gas-gasCheck)<=gas*1e-5,
                "stock density evaluation inconsistent");
            var atmosphere=new D.StockAtmosphere(radius,top,gas,V(body.bodyTransform.up),
                V(FlightGlobals.Bodies[0].position-body.position),Math.Sign(body.rotationPeriod),
                pCurve,tCurve,aSun,lBias,lSun,xSun);
            var temperatures=new System.Text.StringBuilder();
            for(int i=0;i<=N;i++)
            {
                double h=top*i/N;
                double kpa=body.GetPressure(h),temp=atmosphere.Temperature(target*(radius+h));
                double rho=FlightGlobals.getAtmDensity(kpa,temp,body);
                Require(D.Finite(kpa)&&kpa>=0&&D.Finite(temp)&&temp>0&&D.Finite(rho)&&rho>=0,
                    "invalid stock atmosphere sample");
                Require(Math.Abs(rho-gas*kpa*1000/temp)<=Math.Max(1e-10,rho*1e-5),
                    "stock density coefficient mismatch");
                temperatures.Append(Num(h,"R")).Append(':').Append(Num(temp,"R")).Append(';');
                pressures[i]=kpa*1000;maxP=Math.Max(maxP,pressures[i]);
                pressure[i]=new D.Knot(h,pressures[i]);density[i]=new D.Knot(h,rho);
            }
            Require(maxP>0,"atmosphere pressure unavailable");
            Debug.Log("[DunaIntegratedPreview] stock atmosphere gasKgKPerPaM3="+Num(gas,"R")+
                " samples(ASL:K)="+temperatures);

            // Common propellant ratio and tank inventory. This is a
            // vessel-wide inventory estimate, not a proof of crossfeed.
            // Reject mixed fuels, locked or non-axial active engines.
            var engines=new List<EngineCopy>();
            var inventory=new Dictionary<int,double>();Dictionary<int,double> mixture=null;
            int modules=0,resources=0;
            foreach(Part part in v.parts)
            {
                foreach(PartResource r in part.Resources)
                {
                    Require(++resources<=4096,"resource limit");
                    if(!r.flowState||r.info==null)continue;
                    double amount=r.amount;
                    Require(D.Finite(amount)&&amount>=0,"invalid resource amount");
                    int id=r.info.id;inventory.TryGetValue(id,out double previous);
                    inventory[id]=previous+amount;
                }
                foreach(PartModule module in part.Modules)
                {
                    Require(++modules<=2048,"module limit");
                    if(!(module is ModuleEngines e)||!e.EngineIgnited||!e.isEnabled||!e.isOperational)continue;
                    Require(engines.Count<16&&!e.throttleLocked&&e.minFuelFlow<1e-8,"unsupported active engine");
                    Require(e.thrustTransforms!=null&&e.thrustTransformMultipliers!=null&&e.thrustTransforms.Count>0&&
                        e.thrustTransforms.Count==e.thrustTransformMultipliers.Count&&e.thrustTransforms.Count<=32,
                        "unknown thrust transforms");
                    double multipliers=0;
                    for(int i=0;i<e.thrustTransforms.Count;i++)
                    {
                        var tf=e.thrustTransforms[i];double mult=e.thrustTransformMultipliers[i];
                        Require(tf!=null&&D.Finite(mult)&&mult>=0,"invalid nozzle");
                        if(mult>0)Require(D.V.Dot(V(-(Vector3d)tf.forward).Unit,forward)>.99,"nonaxial thrust unsupported");
                        multipliers+=mult;
                    }
                    Require(Math.Abs(multipliers-1)<.01,"unsupported nozzle weighting");
                    var mix=new Dictionary<int,double>();double sum=0;
                    foreach(Propellant prop in e.propellants)
                    {
                        var def=PartResourceLibrary.Instance.GetDefinition(prop.id);
                        Require(def!=null&&def.density>0&&D.Finite(prop.ratio)&&prop.ratio>0,"unsupported propellant");
                        Require(!mix.ContainsKey(prop.id),"duplicate propellant");
                        mix[prop.id]=prop.ratio;sum+=prop.ratio;
                    }
                    Require(sum>0&&mix.Count<=8,"unknown propellant mixture");
                    var ids=new List<int>(mix.Keys);
                    foreach(int id in ids)mix[id]/=sum;
                    if(mixture==null)mixture=mix;
                    else
                    {
                        Require(mixture.Count==mix.Count,"mixed engine propellants");
                        foreach(var kv in mixture)
                            Require(mix.TryGetValue(kv.Key,out double ratio)&&Math.Abs(ratio-kv.Value)<1e-6,"mixed engine ratios");
                    }
                    double flow=e.maxFuelFlow*1000*e.thrustPercentage/100*e.flowMultiplier;
                    Require(D.Finite(flow)&&flow>0&&D.Finite(e.g)&&e.g>0&&e.atmosphereCurve!=null,"unknown engine flow/Isp");
                    var copy=new EngineCopy{Flow=flow,G=e.g,Isp=new double[N+1]};
                    for(int i=0;i<=N;i++)
                    {
                        copy.Isp[i]=e.atmosphereCurve.Evaluate((float)(maxP*i/N/101325));
                        Require(D.Finite(copy.Isp[i])&&copy.Isp[i]>0,"invalid engine pressure curve");
                    }
                    engines.Add(copy);
                }
            }
            Require(engines.Count>0&&mixture!=null,"no active engine");
            double flowTotal=0;foreach(var e in engines)flowTotal+=e.Flow;
            double usable=DunaLandingAdmission.UsableFuel(inventory,mixture,
                id=>PartResourceLibrary.Instance.GetDefinition(id).density*1000,mass);
            Require(D.Finite(usable)&&usable>0&&usable<mass,"unknown usable propellant inventory");
            // Retain 5% of estimated mixture; crossfeed/staging is not
            // modelled. Do not infer a guaranteed reserve from total tanks.
            double reserve=DunaLandingAdmission.RecoveryReserve(reserveFloor,mass-.95*usable);
            Require(reserve<mass,"protected propellant reserve reached");
            var ispKeys=new D.Knot[N+1];
            double nominalCurrent=0;
            double currentPa=body.GetPressure(s.AltitudeASL)*1000;
            Require(D.Finite(currentPa)&&currentPa>=0&&currentPa<=maxP+1e-6,"current pressure outside sampled domain");
            for(int i=0;i<=N;i++)
            {
                double force=0;
                foreach(var e in engines)force+=e.Flow*e.G*e.Isp[i];
                ispKeys[i]=new D.Knot(maxP*i/N/101325,force/(flowTotal*9.80665));
            }
            var curve=new D.Curve(ispKeys);
            nominalCurrent=flowTotal*9.80665*curve.At(Math.Min(maxP,currentPa)/101325);
            Require(nominalCurrent>0&&D.Finite(nominalCurrent),"unknown nominal current thrust");
            // Reduce impulse per kg rather than reduce fuel rate together
            // with thrust. Frozen flow multipliers remain an approximation.
            double derate=DunaLandingAdmission.ImpulseDerate(s.MaxThrustAcceleration,mass,nominalCurrent);
            Require(D.Finite(derate)&&derate>0,"unknown aggregate authority");
            for(int i=0;i<ispKeys.Length;i++)ispKeys[i]=new D.Knot(ispKeys[i].X,ispKeys[i].Y*derate);
            var propulsion=new D.Propulsion(true,new D.Engine(flowTotal,1,9.80665,default,new D.Curve(ispKeys)));
            // AreaDrag is a current-flight sample and can be zero in
            // vacuum. Copy the direction-independent drag-cube faces used
            // by the stock MechJeb landing estimate, not current aero force.
            var drag=new DunaLandingAdmission.DragAccumulator();
            int aeroParts=0;
            foreach(Part part in v.parts)
            {
                if(part.ShieldedFromAirstream||part.DragCubes.None)continue;
                for(int face=0;face<6;face++)
                    drag.AddFace(part.DragCubes.WeightedDrag[face],part.DragCubes.AreaOccluded[face]);
                aeroParts++;
            }
            double area=drag.Finish(PhysicsGlobals.DragCubeMultiplier,PhysicsGlobals.DragMultiplier);
            Debug.Log("[DunaIntegratedPreview] aero liveCdA="+Num(s.AreaDrag,"R")+
                " prospectiveCdA="+Num(area,"R")+" aeroParts="+aeroParts+
                " pressurePa="+Num(currentPa,"R")+" ASL="+Num(s.AltitudeASL,"R"));
            Require(D.Finite(area)&&area>0,"prospective drag area unavailable");
            double terrainRadius=radius+asl;
            var environment=new D.Environment(radius,mu,omega,top,new D.Curve(pressure),new D.Curve(density),
                area,terrainRadius,offset,atmosphere:atmosphere.At);
            double response=Math.Max(.1,s.MaxEngineResponseTime);
            Require(D.Finite(response)&&response<=30,"unknown actuator response");
            // 6 deg/s deliberately explicit, not an observed worst-case
            // turn guarantee. The selected model is not executable.
            var limits=new D.Limits(reserve,Math.PI/30,response,response,.5);
            double attained=Math.Max(0,Math.Min(1,s.CurrentThrustAcceleration/s.MaxThrustAcceleration));
            var initial=new D.State(s.Time,pos,vel,mass,forward,attained,v.angularVelocity.magnitude);
            Require(initial.Valid&&environment.Valid&&limits.Valid,"invalid numerical snapshot");
            var mode=DunaLandingAdmission.Choose(environment,initial,v.ctrlState.mainThrottle,s.SpeedVertical,out string reason);
            Require(mode!=DunaLandingAdmission.Mode.Refused,reason);
            bool strategic=mode==DunaLandingAdmission.Mode.Orbit;
            var settings=DunaLandingAdmission.Settings(environment,propulsion,initial,target,cap,
                s.MaxThrustAcceleration,derate,(double)core.Landing.TouchdownSpeed);
            Require(settings!=null,"invalid solver settings or horizon");
            // One bounded model record per admitted snapshot, sufficient
            // to reconstruct the numerical input without a second flight.
            var samples=new System.Text.StringBuilder();
            for(int i=0;i<=N;i++)
                samples.Append(Num(pressure[i].X,"R")).Append(':').Append(Num(pressure[i].Y,"R")).Append(':')
                    .Append(Num(density[i].Y,"R")).Append(':').Append(Num(ispKeys[i].X,"R")).Append(':')
                    .Append(Num(ispKeys[i].Y,"R")).Append(';');
            Debug.Log("[DunaIntegratedPreview] model radius="+Num(radius,"R")+" mu="+Num(mu,"R")+
                " top="+Num(top,"R")+" terrain="+Num(terrainRadius,"R")+" bottom="+Num(offset,"R")+
                " omega="+Num(omega.X,"R")+","+Num(omega.Y,"R")+","+Num(omega.Z,"R")+
                " target="+Num(target.X,"R")+","+Num(target.Y,"R")+","+Num(target.Z,"R")+
                " forward="+Num(forward.X,"R")+","+Num(forward.Y,"R")+","+Num(forward.Z,"R")+
                " pointingRate="+Num(initial.PointingRate,"R")+" attained="+Num(attained,"R")+
                " flowKgS="+Num(flowTotal,"R")+" derate="+Num(derate,"R")+" fuelKg="+Num(usable,"R")+
                " reserveKg="+Num(reserve,"R")+" touchdown="+Num(settings.Common.Terminal.TouchdownSpeed,"R")+
                " samples(ASL:Pa:rho:atm:Isp)="+samples);
            var partIds=new uint[v.parts.Count];for(int i=0;i<partIds.Length;i++)partIds[i]=v.parts[i].flightID;
            return new Snapshot{Vessel=v.id,Parts=partIds,Latitude=lat,Longitude=lon,UT=s.Time,Mass=mass,
                Cap=cap,Warp=TimeWarp.CurrentRate,DragArea=area,UsableFuel=usable,Initial=initial,
                Environment=environment,Propulsion=propulsion,Limits=limits,Settings=settings,Frame=frame,
                Strategic=strategic,ApproachStartUT=initial.UT,
                ReferencePart=v.GetReferenceTransformPart().flightID,EngineConfiguration=EngineSignature(v)};
        }
        private static Answer Solve(Snapshot s,CancellationToken token)
        {
            var a=new Answer();
            try
            {
                var clock=Stopwatch.StartNew();
                using var budget=new CancellationTokenSource();
                budget.CancelAfter(s.Strategic?20000:8000);
                using var linked=CancellationTokenSource.CreateLinkedTokenSource(token,budget.Token);
                var workToken=linked.Token;
                if(s.Strategic)
                {
                    var settings=new AtmosphericStrategicPlanner.Settings(s.Settings,preparation:45,engagementLead:45,maxWait:6000,maxBurn:600);
                    a.Strategic=AtmosphericStrategicPlanner.Search(s.Environment,s.Propulsion,s.Limits,s.Initial,settings,token,
                        preferredDelay:s.PreferredBurnUT-s.UT,preferredApproach:s.PreferredApproach,
                        optimizationBudget:budget.Token,stopAtFirstTarget:true);
                    a.ReplayPassed=a.Strategic.Best?.ReplayPassed==true;
                    a.Scenarios=a.Strategic.Scenarios;
                    if(!token.IsCancellationRequested)
                        a.Validated=a.Strategic.ValidatedTarget(s.Settings.Common.TargetTolerance)?.Combined;
                    if(a.Validated==null)
                        a.Error=a.Strategic.Status==P.SearchStatus.BudgetExhausted?
                            "Optimisation budget exhausted without a validated targeted landing":
                            a.Strategic.Status==P.SearchStatus.Cancelled?"Planning cancelled":"No validated targeted landing found";
                    return a;
                }
                a.Reference=P.Search(s.Environment,s.Propulsion,s.Limits,s.Initial,s.Settings.Common,
                    0,new D.Command(0,s.Initial.Forward),workToken);
                a.ReferenceMs=clock.Elapsed.TotalMilliseconds;
                if(workToken.IsCancellationRequested){a.Error=token.IsCancellationRequested?"Planning cancelled":"Approach planning budget exhausted";return a;}
                a.Direct=F.Search(s.Environment,s.Propulsion,s.Limits,s.Initial,s.Settings,workToken,curvedCandidates:true);
                if(workToken.IsCancellationRequested||a.Direct.Best==null)
                {a.Error=token.IsCancellationRequested?"Planning cancelled":budget.IsCancellationRequested?
                    "Approach planning budget exhausted without a validated landing":"No approach candidate";return a;}
                clock.Restart();
                var selection=F.SelectValidated(s.Environment,s.Propulsion,s.Limits,s.Initial,s.Settings,a.Direct,
                    30000,120000,20,Math.Max(.5,s.Mass*.0005),workToken);
                a.ReplayMs=clock.Elapsed.TotalMilliseconds;
                a.ReplayPassed=selection.FineReplayPassed;
                a.Scenarios=selection.Scenarios;
                a.Validated=token.IsCancellationRequested?null:selection.Candidate;
                if(a.Validated==null)a.Error=token.IsCancellationRequested?"Planning cancelled":
                    budget.IsCancellationRequested?"Approach validation budget exhausted":selection.Reason;
            }
            catch(Exception ex){a.Error=ex.GetType().Name+": "+ex.Message;}
            return a;
        }
        internal bool Start(MechJebCore core,out string reason,double reserveFloor=0,D.TerrainGrid previousTerrain=null)
        {
            Cancel();
            try
            {
                protectedReserve=reserveFloor;
                snapshot=Capture(core,protectedReserve);answer=null;logged=false;
                var copy=snapshot;
                if(previousTerrain!=null)
                {
                    // Body-fixed samples remain valid after a changed orbit.
                    terrainGrid=previousTerrain;
                    StartTerrainSolve(core);
                }
                else if(copy.Strategic)
                {
                    // Initial target-sphere solution is a routing hint,
                    // never executable landing authority.
                    cancel=new CancellationTokenSource();
                    CancellationToken token=cancel.Token;
                    job=Task.Run(()=>Solve(copy,token));
                    status="Computing coarse orbital approach";
                }
                else
                {
                    terrainCapture=new TerrainAcquisition(null,copy.Latitude,copy.Longitude,copy.Environment.Radius);
                    status="Acquiring approach terrain";
                }
                Debug.Log("[DunaIntegratedPreview] snapshot UT="+Num(copy.UT,"R")+" vessel="+copy.Vessel+
                    " target="+Num(copy.Latitude,"R")+","+Num(copy.Longitude,"R")+" massKg="+Num(copy.Mass)+
                    " mixtureKg="+Num(copy.UsableFuel)+" cap="+Num(copy.Cap,"F3")+
                    " dragArea="+Num(copy.DragArea)+" model=approximate_no_actuation"+
                    " mode="+(copy.Strategic?"orbit":"descent")+" position="+copy.Initial.R.X.ToString("R",CultureInfo.InvariantCulture)+","+
                    copy.Initial.R.Y.ToString("R",CultureInfo.InvariantCulture)+","+copy.Initial.R.Z.ToString("R",CultureInfo.InvariantCulture)+
                    " velocity="+Num(copy.Initial.Velocity.X,"R")+","+Num(copy.Initial.Velocity.Y,"R")+","+Num(copy.Initial.Velocity.Z,"R")+
                    " reserveKg="+Num(copy.Limits.ReserveMass,"R")+" thrustAccel="+Num(core.VesselState.MaxThrustAcceleration,"R")+
                    " offset="+Num(copy.Environment.BottomOffset,"R")+" response="+Num(copy.Limits.EngineRise,"R"));
                reason=null;return true;
            }
            catch(Exception ex)
            {
                reason=ex.Message;Cancel();status="Unavailable: "+reason;
                Debug.Log("[DunaIntegratedPreview] "+status);return false;
            }
        }
        private void StartTerrainSolve(MechJebCore core)
        {
            // The unpowered vessel has moved during PQS sampling. Capture
            // its current epoch/pose rather than skipping a planned burn.
            snapshot=Capture(core,protectedReserve);
            var basis=snapshot.Frame.AtCapture;
            snapshot.Environment=snapshot.Environment.WithGrid(terrainGrid,snapshot.UT,basis.A,basis.B,basis.C);
            snapshot.PreferredBurnUT=preferredBurnUT;snapshot.PreferredApproach=preferredApproach;
            var copy=snapshot;
            cancel=new CancellationTokenSource();
            CancellationToken token=cancel.Token;
            job=Task.Run(()=>Solve(copy,token));
            answer=null;tracker=null;corridor=null;corridorPassed=false;corridorStatus=null;logged=false;
            status=copy.Strategic?"Computing terrain-aware orbital landing":"Computing terrain-aware descent";
            Debug.Log("[DunaIntegratedPreview] terrain-aware solve UT="+Num(copy.UT,"R")+
                " terrain min="+Num(terrainGrid.Min)+" max="+Num(terrainGrid.Max)+" refinements="+terrainRefinements);
            Debug.Log("[DunaIntegratedPreview] terrain samples metres="+terrainGrid.Evidence());
        }
        internal bool Running=>terrainCapture!=null||job!=null&&!job.IsCompleted;
        internal void Cancel()
        {
            if(cancel!=null){cancel.Cancel();cancel.Dispose();cancel=null;}
            job=null;snapshot=null;answer=null;logged=false;status="Not evaluated";
            terrainCapture=null;terrainGrid=null;terrainRefinements=0;protectedReserve=0;preferredBurnUT=double.NaN;preferredApproach=null;
            tracker=null;corridor=null;
            corridorStatus=null;corridorPassed=false;executionStatus=null;
        }
        private static string CandidateText(P.Candidate? c,double initialMass)
        {
            if(!c.HasValue)return "no demonstrated landing";
            var a=c.Value;
            return "miss "+Num(a.Miss/1000,"F2")+" km; fuel "+Num(initialMass-a.End.Mass, "F0")+" kg; coast "+
                Num(a.Profile.IgnitionDelay)+" s";
        }
        internal static bool SameContext(MechJebCore core,Snapshot snapshot,bool ownedWarp=false)
        {
            if(snapshot==null||core.vessel==null||core.vessel.id!=snapshot.Vessel||
                core.vessel.mainBody==null||core.vessel.mainBody.bodyName!="Duna"||
                !core.Target.PositionTargetExists||core.Target.targetBody!=core.vessel.mainBody||
                (double)core.Target.targetLatitude!=snapshot.Latitude||(double)core.Target.targetLongitude!=snapshot.Longitude||
                (!ownedWarp&&(TimeWarp.CurrentRate!=snapshot.Warp||core.vessel.packed))||
                core.vessel.parts.Count!=snapshot.Parts.Length||
                core.vessel.GetReferenceTransformPart()==null||
                core.vessel.GetReferenceTransformPart().flightID!=snapshot.ReferencePart)return false;
            if((ownedWarp?Planetarium.GetUniversalTime():core.VesselState.Time)<snapshot.UT)return false;
            for(int i=0;i<snapshot.Parts.Length;i++)if(core.vessel.parts[i].flightID!=snapshot.Parts[i])return false;
            return true;
        }
        internal static string LiveCompatibility(MechJebCore core,Snapshot snap,bool checkEngines)
        {
            if(!SameContext(core,snap)||FlightGlobals.ActiveVessel!=core.vessel)return "vessel, target, reference part, topology, warp or epoch changed";
            var v=core.vessel;var s=core.VesselState;var body=v.mainBody;
            if((core.Landing.DeployChutes&&s.Parachutes.Count>0)||s.ParachuteDeployed||core.Thrust.LimiterMinThrottle||
                core.Thrust.DifferentialThrottle||core.Thrust.SmoothThrottle||v.ActionGroups[KSPActionGroup.RCS])
                return "unsupported active actuator or throttle setting";
            if(ReflectionUtils.IsLoadedFAR)return "unsupported aero model";
            double cap=Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit);
            if(!D.Finite(cap)||cap+1e-5<snap.Cap||s.MinThrustAcceleration>1e-6)
                return "throttle authority changed";
            if(Math.Abs((double)core.Landing.TouchdownSpeed-snap.Settings.Common.Terminal.TouchdownSpeed)>1e-6)
                return "touchdown setting changed";
            var basis=BodyBasis(body);
            D.V pos=V(s.OrbitalPosition),vel=V(s.OrbitalVelocity);
            D.V omega=V(body.angularVelocity);
            if((vel-V(s.SurfaceVelocity)-D.V.Cross(omega,pos)).Length>.5)
                return "inertial velocity mismatch";
            if((snap.Frame.ToSolver(basis,omega,s.Time)-snap.Environment.Omega).Length>1e-7)
                return "body angular velocity changed";
            D.V target=V(body.GetSurfaceNVector(snap.Latitude,snap.Longitude)).Unit;
            if(!snap.Frame.TargetMatches(basis,target,snap.Settings.Common.TargetAtInitialUT,s.Time))
                return "target frame mismatch";
            if(checkEngines&&EngineSignature(v)!=snap.EngineConfiguration)return "engine configuration changed";
            return null;
        }

        private DunaPlanCorridor corridor;
        private void AdvanceCorridor(MechJebCore core)
        {
            if(answer?.Validated==null||corridorPassed||corridorStatus!=null&&corridorStatus.StartsWith("Rejected:"))return;
            try
            {
                if(tracker==null)
                {
                    if(snapshot.Strategic)
                    {
                        var best=answer.Strategic?.Best;
                        if(best==null){corridorStatus="Rejected: no replayed orbital trajectory";return;}
                        snapshot.ApproachStartUT=best.HandoffUT;
                        snapshot.DeorbitStartUT=snapshot.UT+best.Deorbit.Delay;
                        snapshot.DeorbitEndUT=snapshot.DeorbitStartUT+best.Deorbit.Duration;
                    }
                    tracker=new AtmosphericPlanTracker(snapshot.Environment,snapshot.Propulsion,snapshot.Limits,
                        snapshot.Settings,answer.Validated);
                    if(tracker.TargetMiss>snapshot.Settings.Common.TargetTolerance)
                    {corridorStatus="Rejected: modelled target miss exceeds tolerance";return;}
                    corridor=new DunaPlanCorridor(snapshot.Environment,tracker,snapshot.Frame);
                }
                corridor.Advance(core);
                corridorPassed=corridor.Passed;
                corridorStatus=corridor.Status;
                if(corridor.NeedsTerrainRefinement&&snapshot.Strategic&&terrainRefinements<4)
                {
                    Debug.Log("[DunaIntegratedPreview] "+corridorStatus+"; acquiring additional terrain and replanning");
                    terrainRefinements++;
                    if(answer.Strategic?.Best!=null)
                    {
                        preferredBurnUT=snapshot.UT+answer.Strategic.Best.Deorbit.Delay;
                        preferredApproach=answer.Strategic.Best.Approach.Profile;
                    }
                    terrainCapture=new TerrainAcquisition(terrainGrid,corridor.ConflictLatitude,
                        corridor.ConflictLongitude,snapshot.Environment.Radius);
                    answer=null;tracker=null;corridor=null;corridorStatus=null;corridorPassed=false;
                    status="Refining terrain and recalculating landing";
                }
            }
            catch(Exception ex){corridorStatus="Rejected: "+ex.Message;}
        }

        internal bool ExecutionAvailable=>tracker!=null&&corridorPassed;
        internal bool Finished=>!Running&&(snapshot==null||answer==null||answer.Validated==null||corridorPassed||
            corridorStatus!=null&&corridorStatus.StartsWith("Rejected:"));
        internal Snapshot Input=>snapshot;
        internal bool TryTakePlan(MechJebCore core,out Snapshot input,out AtmosphericPlanTracker plan,out string reason,bool pending=false)
        {
            input=null;plan=null;
            if(!ExecutionAvailable){reason="no replayed and corridor-screened model trajectory";return false;}
            try
            {
                reason=LiveCompatibility(core,snapshot,true);
                if(reason!=null)return false;
                if(core.Landing.Enabled!=pending||
                    !DunaLandingAdmission.OwnsActuators(core.Attitude.Users.Count,core.Attitude.Users.Contains(core.Landing),
                        core.Thrust.Users.Count,core.Thrust.Users.Contains(core.Thrust),core.Thrust.Users.Contains(core.Landing),pending))
                {reason="another autopilot owns attitude or throttle";return false;}
                var s=core.VesselState;
                if(core.vessel.ctrlState.mainThrottle>1e-4||s.CurrentThrustAcceleration>.05)
                {reason="engagement requires unpowered coast";return false;}
                var decision=tracker.Track(LiveState(core,snapshot),s.AltitudeBottom,s.MaxThrustAcceleration,
                    Math.Min(snapshot.Cap,Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit)),s.LocalGravity,s.MaxEngineResponseTime);
                if(!decision.Valid){reason=decision.Reason;return false;}
                if(snapshot.Strategic)
                {
                    var actual=LiveState(core,snapshot);
                    if(!tracker.ReferenceAt(actual.UT,out var expected,out _)||
                        D.V.Dot(actual.Forward,expected.Forward)<Math.Cos(8*Math.PI/180)||
                        actual.UT>=snapshot.DeorbitStartUT)
                    {reason="orbital preparation state or burn deadline missed; evaluate again";return false;}
                }
                // Refuse engagement after skipping a planned positive-thrust
                // interval, even if the position tolerance is still met.
                foreach(var p in tracker.Plan.Reference)
                {
                    if(p.State.UT>=s.Time)break;
                    if(p.Command.Throttle>1e-4&&s.Time-p.State.UT>.1)
                    {reason="required powered interval already elapsed; evaluate again";return false;}
                }
                input=snapshot;plan=tracker;reason=null;return true;
            }
            catch(Exception ex){reason=ex.Message;return false;}
        }
        internal void SetExecutionStatus(string message)=>executionStatus=message;
        internal string Poll(MechJebCore core)
        {
            if(snapshot!=null&&!SameContext(core,snapshot)){Cancel();status="Cancelled: vessel, target, warp or topology changed";
                Debug.Log("[DunaIntegratedPreview] "+status);}
            if(terrainCapture!=null&&snapshot!=null)
            {
                try
                {
                    terrainCapture.Advance(core);
                    if(!terrainCapture.Complete)return status+" ("+terrainCapture.Samples+" samples)";
                    terrainGrid=terrainCapture.Grid;terrainCapture=null;
                    StartTerrainSolve(core);
                }
                catch(Exception ex)
                {
                    terrainCapture=null;status="Terrain acquisition failed: "+ex.Message;
                    answer=new Answer{Error=status};Debug.Log("[DunaIntegratedPreview] "+status);
                }
            }
            if(job!=null&&job.IsCompleted&&answer==null)
            {
                try{answer=job.GetAwaiter().GetResult();status=answer.Error??"Snapshot complete";}
                catch(Exception ex){status="Preview failed: "+ex.Message;}
                finally{job=null;cancel?.Dispose();cancel=null;}
            }
            if(answer==null||snapshot==null)return status;
            if(snapshot.Strategic&&terrainGrid==null&&terrainCapture==null)
            {
                // Use the coarse predicted approach to demand-load actual
                // terrain in body-fixed coordinates. Limit to eight
                // regional samples; destination patches remain fine.
                var best=answer.Strategic?.Best;
                var radials=new List<D.V>();
                if(best!=null)
                {
                    preferredBurnUT=snapshot.UT+best.Deorbit.Delay;
                    preferredApproach=best.Approach.Profile;
                    var basis=snapshot.Frame.AtCapture;
                    var target=snapshot.Settings.Common.TargetAtInitialUT;
                    D.V fixedTarget=new D.V(D.V.Dot(target,basis.A),D.V.Dot(target,basis.B),D.V.Dot(target,basis.C));
                    var route=new List<D.V>();
                    foreach(var point in best.Approach.Reference)
                    {
                        var state=point.State;
                        if(state.R.Length-snapshot.Environment.Radius>20000)continue;
                        var radial=P.RotateTarget(state.R.Unit,snapshot.Environment.Omega,snapshot.UT-state.UT);
                        var q=new D.V(D.V.Dot(radial,basis.A),D.V.Dot(radial,basis.B),D.V.Dot(radial,basis.C)).Unit;
                        if((q-fixedTarget).Length*snapshot.Environment.Radius<4000)continue;
                        if(route.Count==0||(q-route[route.Count-1]).Length*snapshot.Environment.Radius>=6000)
                            route.Add(q);
                    }
                    int count=Math.Min(8,route.Count);
                    for(int i=0;i<count;i++)radials.Add(route[count==1?0:i*(route.Count-1)/(count-1)]);
                }
                terrainCapture=new TerrainAcquisition(null,snapshot.Latitude,snapshot.Longitude,
                    snapshot.Environment.Radius,radials);
                answer=null;status="Acquiring terrain along predicted approach";
                return status;
            }
            AdvanceCorridor(core);
            if(answer==null)return status;
            double age=core.VesselState.Time-snapshot.UT;
            string lines=status+" (age "+Num(age)+" s; experimental model)\n";
            if(answer.Strategic!=null)
            {
                var report=answer.Strategic;var best=report.Best;
                string searchStatus=report.Status==P.SearchStatus.BudgetExhausted&&answer.Validated!=null?
                    "Budget reached; validated trajectory retained":report.Status.ToString();
                lines+="Orbital search: "+searchStatus+"; "+report.Prefixes+" prefixes, "+report.Approaches+" descent candidates\n";
                if(best!=null)
                    lines+="Deorbit in "+Num(best.Deorbit.Delay-age)+" s; burn "+Num(best.Deorbit.Duration)+
                        " s; normal component "+Num(best.Deorbit.NormalFraction*100)+"%; deorbit fuel "+
                        Num(best.DeorbitFuel,"F0")+" kg\nWhole landing: "+CandidateText(best.Combined.Result,snapshot.Mass)+"\n";
            }
            if(answer.Reference!=null)
                lines+="A: "+CandidateText(answer.Reference.Best,snapshot.Mass)+"; "+answer.Reference.Status+
                    " ("+answer.Reference.Candidates.Count+" tests, "+Num(answer.ReferenceMs,"F0")+" ms)\n";
            if(answer.Direct!=null)
                lines+="B: "+CandidateText((answer.Validated??answer.Direct.Best)?.Result,snapshot.Mass)+"; "+answer.Direct.Status+
                    (answer.Direct.Best==null?"":"; powered "+Num((answer.Validated??answer.Direct.Best).Profile.Duration)+" s")+
                    " ("+answer.Direct.Attempts+" tests, "+Num(answer.Direct.ElapsedMilliseconds,"F0")+" ms; min prep "+Num(snapshot.Settings.MinCoast)+" s)\n";
            lines+=(snapshot.Strategic?"Whole-history finer-step replay: ":"B finer-step replay: ")+(answer.ReplayPassed?"PASS (model only)":"not demonstrated")+
                "; approximate terrain/aero/actuators.";
            if(answer.Scenarios!=null)
            {
                var scenarios=answer.Scenarios;
                lines+="\nClosed-loop scenarios: "+(scenarios.Passed?"PASS (model only)":"REJECTED: "+scenarios.Reason)+
                    "; "+scenarios.Trials.Count+" trials; max sampled miss "+Num(scenarios.WorstMiss/1000,"F2")+
                    " km; min contact mass "+Num(scenarios.MinimumEndMass,"F0")+" kg";
            }
            else lines+="\nClosed-loop scenario screen not demonstrated.";
            if(corridorStatus!=null)lines+="\n"+corridorStatus;
            if(executionStatus!=null)lines+="\n"+executionStatus;
            if(!logged){logged=true;Debug.Log("[DunaIntegratedPreview] "+lines.Replace('\n',' '));}
            return lines;
        }
    }
}
