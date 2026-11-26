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
            internal D.Environment Environment;
            internal D.Propulsion Propulsion;
            internal D.Limits Limits;
            internal F.Settings Settings;
            internal bool Strategic;
            internal double ApproachStartUT,DeorbitStartUT,DeorbitEndUT;
        }
        private sealed class Answer
        {
            internal P.Report Reference;
            internal F.Report Direct;
            internal AtmosphericStrategicPlanner.Report Strategic;
            internal F.Candidate Validated;
            internal bool ReplayPassed;
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
        internal static D.State LiveState(MechJebCore core)
        {
            var s=core.VesselState;
            double attained=s.MaxThrustAcceleration>0?
                Math.Max(0,Math.Min(1,s.CurrentThrustAcceleration/s.MaxThrustAcceleration)):double.NaN;
            return new D.State(s.Time,V(s.OrbitalPosition),V(s.OrbitalVelocity),s.Mass*1000,
                V(s.Forward).Unit,attained,core.vessel.angularVelocity.magnitude);
        }

        // Snapshot uses a reduced-order equivalent axial engine, frozen
        // measured/estimated drag area, sampled atmosphere at target, and
        // spherical terrain at target ASL. These are explicitly NOT verified
        // real-engine, lift, crossfeed, attitude or swept terrain models.
        // Results are for comparing algorithms, never authorization to burn.
        private static Snapshot Capture(MechJebCore core,bool strategic)
        {
            Vessel v=core.vessel;
            VesselState s=core.VesselState;
            CelestialBody body=v.mainBody;
            Require(body!=null&&body.bodyName=="Duna"&&body.atmosphere,"Duna atmosphere required");
            Require(!v.packed&&!v.LandedOrSplashed&&TimeWarp.CurrentRate==1,"unpacked flight at 1x required");
            Require(core.Target.PositionTargetExists&&core.Target.targetBody==body,"select a Duna landing target");
            Require(!core.Landing.DeployChutes&&!s.ParachuteDeployed,"parachute dynamics unsupported");
            Require(!core.Thrust.LimiterMinThrottle,"minimum throttle floor unsupported");
            Require(!core.Thrust.DifferentialThrottle,"differential throttle model not yet supported");
            Require(!core.Thrust.SmoothThrottle,"throttle smoothing model not yet supported for preview");
            Require(!ReflectionUtils.IsLoadedFAR,"FAR aerodynamics unsupported by preview");
            Require(v.parts!=null&&v.parts.Count>0&&v.parts.Count<=512,"unsupported part count");
            Require(D.Finite(s.Mass)&&s.Mass>0&&D.Finite(s.Time)&&(strategic||s.SpeedVertical<0),"descending state required for descent-only preview");
            double lat=core.Target.targetLatitude,lon=core.Target.targetLongitude;
            double asl=body.TerrainAltitude(lat,lon);
            double radius=body.Radius,mu=body.gravParameter,top=body.RealMaxAtmosphereAltitude();
            double offset=s.TerminalBottomOffset();
            Require(D.Finite(asl)&&asl>=0&&D.Finite(radius)&&radius>0&&D.Finite(mu)&&mu>0&&
                D.Finite(top)&&top>0&&D.Finite(offset)&&offset>=0,"invalid body or target terrain");
            D.V pos=V(s.OrbitalPosition),vel=V(s.OrbitalVelocity),omega=V(body.angularVelocity),forward=V(s.Forward).Unit;
            // Frame sign is checked against the very same sampled inertial
            // and surface velocities; do not silently flip rotation.
            Require((vel-V(s.SurfaceVelocity)-D.V.Cross(omega,pos)).Length<.5,"surface/inertial rotation mismatch");
            D.V target=V(body.GetWorldSurfacePosition(lat,lon,0)-body.position).Unit;
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
            for(int i=0;i<=N;i++)
            {
                double h=top*i/N;
                double kpa=body.GetPressure(h);
                double temp=body.GetFullTemperature(body.GetWorldSurfacePosition(lat,lon,h));
                double rho=FlightGlobals.getAtmDensity(kpa,temp,body);
                Require(D.Finite(kpa)&&kpa>=0&&D.Finite(temp)&&temp>0&&D.Finite(rho)&&rho>=0,"invalid atmosphere sample");
                pressures[i]=kpa*1000;maxP=Math.Max(maxP,pressures[i]);
                pressure[i]=new D.Knot(h,pressures[i]);density[i]=new D.Knot(h,rho);
            }
            Require(maxP>0,"atmosphere pressure unavailable");

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
            double ratioUnits=double.PositiveInfinity,densityMix=0;
            foreach(var kv in mixture)
            {
                inventory.TryGetValue(kv.Key,out double amount);
                ratioUnits=Math.Min(ratioUnits,amount/kv.Value);
                densityMix+=kv.Value*PartResourceLibrary.Instance.GetDefinition(kv.Key).density*1000;
            }
            double usable=ratioUnits*densityMix;
            Require(D.Finite(usable)&&usable>0&&usable<mass,"unknown usable propellant inventory");
            // Retain 5% of estimated mixture; crossfeed/staging is not
            // modelled. Do not infer a guaranteed reserve from total tanks.
            double reserve=mass-.95*usable;
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
            double derate=.90*Math.Min(1,s.MaxThrustAcceleration*mass/nominalCurrent);
            Require(D.Finite(derate)&&derate>0,"unknown aggregate authority");
            for(int i=0;i<ispKeys.Length;i++)ispKeys[i]=new D.Knot(ispKeys[i].X,ispKeys[i].Y*derate);
            var propulsion=new D.Propulsion(true,new D.Engine(flowTotal,1,9.80665,default,new D.Curve(ispKeys)));
            double area=s.AreaDrag;
            Require(D.Finite(area)&&area>=0,"unknown drag area");
            double terrainRadius=radius+asl;
            var environment=new D.Environment(radius,mu,omega,top,new D.Curve(pressure),new D.Curve(density),
                area,terrainRadius,offset);
            double response=Math.Max(.1,s.MaxEngineResponseTime);
            Require(D.Finite(response)&&response<=30,"unknown actuator response");
            // 6 deg/s deliberately explicit, not an observed worst-case
            // turn guarantee. The selected model is not executable.
            var limits=new D.Limits(reserve,Math.PI/30,response,response,.5);
            double attained=Math.Max(0,Math.Min(1,s.CurrentThrustAcceleration/s.MaxThrustAcceleration));
            var initial=new D.State(s.Time,pos,vel,mass,forward,attained,v.angularVelocity.magnitude);
            Require(initial.Valid&&environment.Valid&&limits.Valid,"invalid numerical snapshot");
            double clearance=environment.PhysicalClearance(initial);
            Require(clearance>0&&clearance<=200000,"outside preview altitude domain");
            if(strategic)
            {
                Require(initial.Throttle<=.01&&v.ctrlState.mainThrottle<1e-4,"orbital preview requires unpowered coast");
                Require(AtmosphericStrategicPlanner.TryOrbit(environment,initial,out var orbit)&&
                    orbit.Periapsis>radius+top&&orbit.Eccentricity<=.2&&orbit.Apoapsis-radius<=200000,
                    "strategic preview requires low bound orbit above atmosphere (e <= 0.2)");
                Require(area>0,"orbital aero area unavailable");
            }
            double gravity=mu/(radius*radius);
            double acceleration=s.MaxThrustAcceleration*cap*derate;
            double scale=Math.Max(12,Math.Max(Math.Sqrt(2*clearance/gravity),s.SpeedSurface/Math.Max(.1,acceleration)));
            double coast=Math.Min(300,Math.Max(0,scale*.75));
            double minDuration=Math.Max(8,scale*.65),maxDuration=Math.Min(900,Math.Max(minDuration,scale*3));
            Require(minDuration<=maxDuration,"descent horizon unsupported");
            double touchdown=(double)core.Landing.TouchdownSpeed;
            // Terminal replay authority is computed only from numerical
            // samples; no live callback is retained. Its 0.2 s controller
            // cadence is a documented screening approximation.
            var terminal=new T.Settings(.2,touchdown,gravity/9.81,0,
                st=>{
                    if(!environment.Air(st.UT,st.R,out double pressurePa,out _))return default;
                    var a=propulsion.At(pressurePa);
                    double raw=a.Force/st.Mass;
                    return a.Known&&raw>0?new T.ControllerAuthority(raw,raw*cap,cap,false):default;
                },
                (st,dir)=>Math.Acos(Math.Max(-1,Math.Min(1,D.V.Dot(st.Forward,dir.Unit))))*180/Math.PI,
                (ut,p)=>terrainRadius);
            var memory=new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,attained,attained);
            var common=new P.Settings(target,1800,coast,100,terminal,memory,intervals:8,refine:1,
                maxCandidates:64,maxSteps:150000,captureClearance:50,captureSpeed:15,
                safeVertical:Math.Max(1,touchdown),safeHorizontal:1,safeAngle:5*Math.PI/180,
                safeRate:Math.PI/180,entryCutoffDelay:0,commandCap:cap);
            var settings=new F.Settings(common,terrainRadius+offset,minPowered:minDuration,maxPowered:maxDuration,
                controlTick:.5,coastIntervals:4,durationIntervals:6,refinements:2,
                minCoast:Math.Min(15,coast));
            Require(settings.Valid,"invalid solver settings");
            var partIds=new uint[v.parts.Count];for(int i=0;i<partIds.Length;i++)partIds[i]=v.parts[i].flightID;
            return new Snapshot{Vessel=v.id,Parts=partIds,Latitude=lat,Longitude=lon,UT=s.Time,Mass=mass,
                Cap=cap,Warp=TimeWarp.CurrentRate,DragArea=area,UsableFuel=usable,Initial=initial,
                Environment=environment,Propulsion=propulsion,Limits=limits,Settings=settings,
                Strategic=strategic,ApproachStartUT=initial.UT,
                ReferencePart=v.GetReferenceTransformPart().flightID,EngineConfiguration=EngineSignature(v)};
        }
        private static Answer Solve(Snapshot s,CancellationToken token)
        {
            var a=new Answer();
            try
            {
                var clock=Stopwatch.StartNew();
                if(s.Strategic)
                {
                    var settings=new AtmosphericStrategicPlanner.Settings(s.Settings,preparation:45);
                    a.Strategic=AtmosphericStrategicPlanner.Search(s.Environment,s.Propulsion,s.Limits,s.Initial,settings,token);
                    a.ReplayPassed=a.Strategic.Best?.ReplayPassed==true;
                    if(a.ReplayPassed&&!token.IsCancellationRequested)a.Validated=a.Strategic.Best.Combined;
                    return a;
                }
                a.Reference=P.Search(s.Environment,s.Propulsion,s.Limits,s.Initial,s.Settings.Common,
                    0,new D.Command(0,s.Initial.Forward),token);
                a.ReferenceMs=clock.Elapsed.TotalMilliseconds;
                if(token.IsCancellationRequested)return a;
                a.Direct=F.Search(s.Environment,s.Propulsion,s.Limits,s.Initial,s.Settings,token);
                if(token.IsCancellationRequested||a.Direct.Best==null)return a;
                clock.Restart();
                a.ReplayPassed=F.Validate(s.Environment,s.Propulsion,s.Limits,s.Initial,s.Settings,a.Direct.Best,
                    30000,20,Math.Max(.5,s.Mass*.0005),out var reconstructed,out _,token);
                a.ReplayMs=clock.Elapsed.TotalMilliseconds;
                if(a.ReplayPassed)a.Validated=reconstructed;
            }
            catch(Exception ex){a.Error=ex.GetType().Name+": "+ex.Message;}
            return a;
        }
        internal void Start(MechJebCore core,bool strategic=false)
        {
            Cancel();
            try
            {
                snapshot=Capture(core,strategic);answer=null;logged=false;
                var copy=snapshot;
                cancel=new CancellationTokenSource();cancel.CancelAfter(strategic?20000:8000);
                CancellationToken token=cancel.Token;
                job=Task.Run(()=>Solve(copy,token));
                status=strategic?"Computing orbital landing preview":"Computing snapshot preview";
                Debug.Log("[DunaIntegratedPreview] snapshot UT="+Num(copy.UT,"R")+" vessel="+copy.Vessel+
                    " target="+Num(copy.Latitude,"R")+","+Num(copy.Longitude,"R")+" massKg="+Num(copy.Mass)+
                    " mixtureKg="+Num(copy.UsableFuel)+" cap="+Num(copy.Cap,"F3")+
                    " dragArea="+Num(copy.DragArea)+" model=approximate_no_actuation");
            }
            catch(Exception ex){status="Unavailable: "+ex.Message;snapshot=null;}
        }
        internal bool Running=>job!=null&&!job.IsCompleted;
        internal void Cancel()
        {
            if(cancel!=null){cancel.Cancel();cancel.Dispose();cancel=null;}
            job=null;snapshot=null;answer=null;logged=false;status="Not evaluated";
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
        internal static bool SameContext(MechJebCore core,Snapshot snapshot)
        {
            if(snapshot==null||core.vessel==null||core.vessel.id!=snapshot.Vessel||
                core.vessel.mainBody==null||core.vessel.mainBody.bodyName!="Duna"||
                !core.Target.PositionTargetExists||core.Target.targetBody!=core.vessel.mainBody||
                (double)core.Target.targetLatitude!=snapshot.Latitude||(double)core.Target.targetLongitude!=snapshot.Longitude||
                TimeWarp.CurrentRate!=snapshot.Warp||core.vessel.packed||core.vessel.parts.Count!=snapshot.Parts.Length||
                core.vessel.GetReferenceTransformPart()==null||
                core.vessel.GetReferenceTransformPart().flightID!=snapshot.ReferencePart)return false;
            if(core.VesselState.Time<snapshot.UT)return false;
            for(int i=0;i<snapshot.Parts.Length;i++)if(core.vessel.parts[i].flightID!=snapshot.Parts[i])return false;
            return true;
        }
        internal static string LiveCompatibility(MechJebCore core,Snapshot snap,bool checkEngines)
        {
            if(!SameContext(core,snap)||FlightGlobals.ActiveVessel!=core.vessel)return "vessel, target, reference part, topology, warp or epoch changed";
            var v=core.vessel;var s=core.VesselState;var body=v.mainBody;
            if(core.Landing.DeployChutes||s.ParachuteDeployed||core.Thrust.LimiterMinThrottle||
                core.Thrust.DifferentialThrottle||core.Thrust.SmoothThrottle||v.ActionGroups[KSPActionGroup.RCS])
                return "unsupported active actuator or throttle setting";
            if(ReflectionUtils.IsLoadedFAR)return "unsupported aero model";
            double cap=Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit);
            if(!D.Finite(cap)||cap+1e-5<snap.Cap||s.MinThrustAcceleration>1e-6)
                return "throttle authority changed";
            if(Math.Abs((double)core.Landing.TouchdownSpeed-snap.Settings.Common.Terminal.TouchdownSpeed)>1e-6)
                return "touchdown setting changed";
            D.V pos=V(s.OrbitalPosition),vel=V(s.OrbitalVelocity);
            if((vel-V(s.SurfaceVelocity)-D.V.Cross(snap.Environment.Omega,pos)).Length>.5)
                return "inertial frame changed";
            D.V target=V(body.GetWorldSurfacePosition(snap.Latitude,snap.Longitude,0)-body.position).Unit;
            D.V expected=P.RotateTarget(snap.Settings.Common.TargetAtInitialUT,snap.Environment.Omega,s.Time-snap.UT);
            if(!target.Valid||(target-expected).Length>1e-5)return "target frame mismatch";
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
                    corridor=new DunaPlanCorridor(snapshot.Environment,tracker);
                }
                corridor.Advance(core);
                corridorPassed=corridor.Passed;
                corridorStatus=corridor.Status;
            }
            catch(Exception ex){corridorStatus="Rejected: "+ex.Message;}
        }

        internal bool ExecutionAvailable=>tracker!=null&&corridorPassed;
        internal bool TryTakePlan(MechJebCore core,out Snapshot input,out AtmosphericPlanTracker plan,out string reason)
        {
            input=null;plan=null;
            if(!ExecutionAvailable){reason="no replayed and corridor-screened model trajectory";return false;}
            try
            {
                reason=LiveCompatibility(core,snapshot,true);
                if(reason!=null)return false;
                if(core.Landing.Enabled||core.Attitude.Users.Count!=0||core.Thrust.Users.Count!=1||
                    !core.Thrust.Users.Contains(core.Thrust))
                {reason="another autopilot owns attitude or throttle";return false;}
                var s=core.VesselState;
                if(core.vessel.ctrlState.mainThrottle>1e-4||s.CurrentThrustAcceleration>.05)
                {reason="engagement requires unpowered coast";return false;}
                var decision=tracker.Track(LiveState(core),s.AltitudeBottom,s.MaxThrustAcceleration,
                    Math.Min(snapshot.Cap,Math.Min(s.ThrottleLimit,s.ThrottleFixedLimit)),s.LocalGravity,s.MaxEngineResponseTime);
                if(!decision.Valid){reason=decision.Reason;return false;}
                if(snapshot.Strategic)
                {
                    var actual=LiveState(core);
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
            if(snapshot!=null&&!SameContext(core,snapshot)){Cancel();status="Cancelled: vessel, target, warp or topology changed";}
            if(job!=null&&job.IsCompleted&&answer==null)
            {
                try{answer=job.GetAwaiter().GetResult();status=answer.Error??"Snapshot complete";}
                catch(Exception ex){status="Preview failed: "+ex.Message;}
                finally{job=null;cancel?.Dispose();cancel=null;}
            }
            if(answer==null||snapshot==null)return status;
            AdvanceCorridor(core);
            double age=core.VesselState.Time-snapshot.UT;
            string lines=status+" (age "+Num(age)+" s; experimental model)\n";
            if(answer.Strategic!=null)
            {
                var report=answer.Strategic;var best=report.Best;
                lines+="Orbital search: "+report.Status+"; "+report.Prefixes+" prefixes, "+report.Approaches+" descent candidates\n";
                if(best!=null)
                    lines+="Deorbit in "+Num(best.Deorbit.Delay-age)+" s; burn "+Num(best.Deorbit.Duration)+
                        " s; normal component "+Num(best.Deorbit.NormalFraction*100)+"%; deorbit fuel "+
                        Num(best.DeorbitFuel,"F0")+" kg\nWhole landing: "+CandidateText(best.Combined.Result,snapshot.Mass)+"\n";
            }
            if(answer.Reference!=null)
                lines+="A: "+CandidateText(answer.Reference.Best,snapshot.Mass)+"; "+answer.Reference.Status+
                    " ("+answer.Reference.Candidates.Count+" tests, "+Num(answer.ReferenceMs,"F0")+" ms)\n";
            if(answer.Direct!=null)
                lines+="B: "+CandidateText(answer.Direct.Best?.Result,snapshot.Mass)+"; "+answer.Direct.Status+
                    (answer.Direct.Best==null?"":"; powered "+Num(answer.Direct.Best.Profile.Duration)+" s")+
                    " ("+answer.Direct.Attempts+" tests, "+Num(answer.Direct.ElapsedMilliseconds,"F0")+" ms; min prep "+Num(snapshot.Settings.MinCoast)+" s)\n";
            lines+=(snapshot.Strategic?"Whole-history finer-step replay: ":"B finer-step replay: ")+(answer.ReplayPassed?"PASS (model only)":"not demonstrated")+
                "; approximate terrain/aero/actuators.";
            if(corridorStatus!=null)lines+="\n"+corridorStatus;
            if(executionStatus!=null)lines+="\n"+executionStatus;
            if(!logged){logged=true;Debug.Log("[DunaIntegratedPreview] "+lines.Replace('\n',' '));}
            return lines;
        }
    }
}
