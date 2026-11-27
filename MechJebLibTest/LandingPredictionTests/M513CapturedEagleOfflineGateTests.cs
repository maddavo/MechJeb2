using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using MuMech.Landing;
using Xunit;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using S=MuMech.Landing.AtmosphericStrategicPlanner;

namespace MechJebLibTest.LandingPredictionTests
{
    public sealed class M513CapturedEagleOfflineGateTests
    {
        private sealed class Capture
        {
            internal readonly Dictionary<string,string> Meta=new Dictionary<string,string>();
            internal readonly List<double[]> Air=new List<double[]>();
            internal readonly List<double[]> Temperature=new List<double[]>();
            internal int TerrainN;
            internal double[] Terrain;
            internal readonly List<D.TerrainGrid.Patch> Patches=new List<D.TerrainGrid.Patch>();
        }
        private static readonly CultureInfo Invariant=CultureInfo.InvariantCulture;
        private static double Number(Capture c,string key)=>double.Parse(c.Meta[key],Invariant);
        private static D.V Vector(Capture c,string key)
        {
            var a=c.Meta[key].Split(',').Select(x=>double.Parse(x,Invariant)).ToArray();
            if(a.Length!=3)throw new InvalidDataException("Expected 3-vector: "+key);
            return new D.V(a[0],a[1],a[2]);
        }
        private static Capture ReadCapture()
        {
            string nested=Path.Combine(AppContext.BaseDirectory,"LandingPredictionTests","Fixtures","M5.12-Eagle-orbit-capture.csv");
            string flat=Path.Combine(AppContext.BaseDirectory,"Fixtures","M5.12-Eagle-orbit-capture.csv");
            string path=File.Exists(nested)?nested:flat;
            var capture=new Capture();
            foreach(string raw in File.ReadLines(path))
            {
                if(String.IsNullOrWhiteSpace(raw)||raw[0]=='#')continue;
                string[] f=raw.Split(',');
                switch(f[0])
                {
                    case "M":
                        if(f.Length<3||capture.Meta.ContainsKey(f[1]))
                            throw new InvalidDataException("Duplicate or malformed metadata: "+raw);
                        capture.Meta.Add(f[1],String.Join(",",f.Skip(2)));
                        break;
                    case "A":
                        if(f.Length!=6)throw new InvalidDataException("Malformed atmosphere row: "+raw);
                        capture.Air.Add(f.Skip(1).Select(x=>double.Parse(x,Invariant)).ToArray());
                        break;
                    case "T":
                        if(f.Length!=3)throw new InvalidDataException("Malformed temperature row: "+raw);
                        capture.Temperature.Add(f.Skip(1).Select(x=>double.Parse(x,Invariant)).ToArray());
                        break;
                    case "G":
                        if(f.Length<3||capture.Terrain!=null)throw new InvalidDataException("Malformed terrain grid");
                        capture.TerrainN=int.Parse(f[1],Invariant);
                        capture.Terrain=f.Skip(2).Select(x=>double.Parse(x,Invariant)).ToArray();
                        break;
                    case "P":
                        if(f.Length<127)throw new InvalidDataException("Malformed terrain patch");
                        double[] values=f.Skip(1).Select(x=>double.Parse(x,Invariant)).ToArray();
                        int n=(int)values[4];
                        if(values.Length!=5+(n+1)*(n+1))throw new InvalidDataException("Terrain patch sample count mismatch");
                        capture.Patches.Add(new D.TerrainGrid.Patch(new D.V(values[0],values[1],values[2]),
                            values[3],n,values.Skip(5).ToArray()));
                        break;
                    default:throw new InvalidDataException("Unknown captured record: "+f[0]);
                }
            }
            foreach(string required in new[]{"schema","sourceSha256","loadedVersion","vesselGuid","body",
                "captureUT","terrainCaptureUT","targetLatitude","targetLongitude","position","orbitalVelocity",
                "surfaceVelocityDerivedFromLoggedOrbitAndOmega","omega","forward","targetRadial","frameA","frameB","frameC",
                "radius","mu","atmosphereTop","TerrainAsl","bottomOffset","massKg","usableFuelKg","reserveKg",
                "flowKgPerSecond","impulseDerate","dragArea","maxAcceleration","turnRate","engineResponse",
                "touchdownSpeed","gasFactor"})
                if(!capture.Meta.ContainsKey(required))throw new InvalidDataException("Missing required captured field: "+required);
            if(capture.Meta["schema"]!="M5.12-Eagle-captured-runtime-v1"||
                capture.Meta["loadedVersion"]!="2.16.0-landing.20261127.duna-integrated-m5.12"||
                capture.Meta["vesselGuid"]!="5ec86e8a-29c5-4d6e-a975-0d2cf487c436")
                throw new InvalidDataException("Captured runtime identity mismatch");
            if(capture.Air.Count!=65||capture.Temperature.Count!=65||capture.Terrain==null||
                capture.Terrain.Length!=6*(capture.TerrainN+1)*(capture.TerrainN+1)||capture.Patches.Count!=10)
                throw new InvalidDataException("Captured atmosphere/terrain dataset incomplete");
            return capture;
        }
        private static D.Curve Curve(IEnumerable<double[]> rows,int x,int y)
            =>new D.Curve(rows.Select(r=>new D.Knot(r[x],r[y])).ToArray());
        private static D.Propulsion Propulsion(Capture c)
        {
            var knots=c.Air.Select(r=>new D.Knot(r[3],r[4])).ToArray();
            return new D.Propulsion(true,new D.Engine(Number(c,"flowKgPerSecond"),1,9.80665,default,new D.Curve(knots)));
        }
        private static DunaIntegratedSnapshotBuilder.Input Input(Capture c)
        {
            double mass=Number(c,"massKg"),flow=Number(c,"flowKgPerSecond"),derate=Number(c,"impulseDerate"),maxAccel=Number(c,"maxAcceleration");
            var grid=new D.TerrainGrid(c.TerrainN,c.Terrain,c.Patches.ToArray());
            return new DunaIntegratedSnapshotBuilder.Input
            {
                Body=c.Meta["body"],HasAtmosphere=true,Active=true,Targeted=true,Packed=false,Landed=false,
                ChuteOption=false,ChuteCount=0,ChuteDeployed=false,Rcs=false,MinimumThrottle=false,
                DifferentialThrottle=false,Smoothing=false,Far=false,PartCount=55,Warp=1,
                UT=Number(c,"captureUT"),MassKg=mass,AltitudeAsl=99079.449365078763,
                VerticalSpeed=-.22674064559748786,CommandedThrottle=0,MaxAcceleration=maxAccel,MinAcceleration=0,
                FixedThrottleLimit=1,ThrottleLimit=1,TargetLatitude=Number(c,"targetLatitude"),
                TargetLongitude=Number(c,"targetLongitude"),TerrainAsl=Number(c,"TerrainAsl"),
                Radius=Number(c,"radius"),Mu=Number(c,"mu"),AtmosphereTop=Number(c,"atmosphereTop"),
                BottomOffset=Number(c,"bottomOffset"),TouchdownSpeed=Number(c,"touchdownSpeed"),
                EngineResponse=Number(c,"engineResponse"),TurnRate=Number(c,"turnRate"),
                FlowKgPerSecond=flow,UsableFuelKg=Number(c,"usableFuelKg"),ReserveFloorKg=Number(c,"reserveKg"),
                NominalThrustNewtons=mass*maxAccel/derate,ImpulseDerate=derate,DragArea=Number(c,"dragArea"),
                AttainedThrottle=0,PointingRate=Number(c,"pointingRate"),
                Position=Vector(c,"position"),OrbitalVelocity=Vector(c,"orbitalVelocity"),
                SurfaceVelocity=Vector(c,"surfaceVelocityDerivedFromLoggedOrbitAndOmega"),
                Omega=Vector(c,"omega"),Forward=Vector(c,"forward"),TargetRadial=Vector(c,"targetRadial"),
                PressureByAltitude=Curve(c.Air,0,1),DensityByAltitude=Curve(c.Air,0,2),
                EngineIspByPressure=Curve(c.Air,3,4),Atmosphere=(ut,pos)=>
                {
                    double height=pos.Length-Number(c,"radius");
                    if(height<0||height>=Number(c,"atmosphereTop"))return new D.AirSample(0,0);
                    var row=c.Air.OrderBy(item=>Math.Abs(item[0]-height)).First();
                    if(Math.Abs(row[0]-height)>781.2501)return default;
                    return new D.AirSample(row[1],row[2]);
                },Propulsion=Propulsion(c),Terrain=grid,
                FrameA=Vector(c,"frameA"),FrameB=Vector(c,"frameB"),FrameC=Vector(c,"frameC"),
                Parts=Enumerable.Range(0,55).Select(i=>(uint)(i+1)).ToArray(),ReferencePart=1,
                EngineConfiguration="captured aggregate; per-engine topology unavailable"
            };
        }

        [Fact]
        public void CapturedM512RuntimeInputsPassSharedProductionAdmission()
        {
            var c=ReadCapture();var result=DunaIntegratedSnapshotBuilder.Build(Input(c));
            Assert.True(result.Accepted,result.Reason);
            Assert.Equal(DunaLandingAdmission.Mode.Orbit,result.Mode);
            Assert.Equal(Number(c,"captureUT"),result.Initial.UT,6);
            Assert.Equal(Number(c,"reserveKg"),result.Limits.ReserveMass,3);
            Assert.Equal(65,c.Air.Count);Assert.Equal(65,c.Temperature.Count);
            Assert.Equal(10,c.Patches.Count);Assert.Equal(322626.89560281805,result.Environment.TerrainRadius,6);
            Assert.True(result.Environment.Grid!=null);
            double gas=Number(c,"gasFactor");
            for(int i=0;i<c.Air.Count;i++)
            {
                Assert.Equal(c.Air[i][0],c.Temperature[i][0],8);
                double rhoFromCapturedTemperature=gas*c.Air[i][1]/c.Temperature[i][1];
                Assert.InRange(Math.Abs(rhoFromCapturedTemperature-c.Air[i][2]),0,Math.Max(1e-8,c.Air[i][2]*1e-5));
            }
            Assert.InRange(Math.Abs(D.V.Dot(result.Frame.AtCapture.A,result.Frame.AtCapture.B)),0,1e-6);
        }

        [Theory]
        [InlineData("Rcs",true,"active RCS unsupported")]
        [InlineData("ChuteDeployed",true,"parachute dynamics unsupported")]
        [InlineData("DifferentialThrottle",true,"differential throttle model unsupported")]
        [InlineData("Smoothing",true,"throttle smoothing model unsupported")]
        [InlineData("Far",true,"FAR aerodynamics unsupported")]
        public void CapturedInputMutationsRefuseUnsupportedProductionConfigurations(string field,bool value,string reason)
        {
            var input=Input(ReadCapture());
            typeof(DunaIntegratedSnapshotBuilder.Input).GetField(field,
                System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Public)
                .SetValue(input,value);
            var result=DunaIntegratedSnapshotBuilder.Build(input);
            Assert.False(result.Accepted);
            Assert.Equal(reason,result.Reason);
        }

        [Fact]
        public void CapturedM512StockColumnAndTerrainCanDriveActualStrategicReplay()
        {
            var c=ReadCapture();var built=DunaIntegratedSnapshotBuilder.Build(Input(c));
            Assert.True(built.Accepted,built.Reason);
            var common=new S.Settings(built.Settings,preparation:45,engagementLead:45,maxWait:6000,maxBurn:600);
            var report=S.Search(built.Environment,built.Propulsion,built.Limits,built.Initial,common,
                stopAtFirstTarget:true);
            var candidate=report.ValidatedTarget(built.Settings.Common.TargetTolerance);
            Assert.True(candidate!=null,"Captured M5.12 input did not yield validated current-model contact: "+report.Status+
                " prefixes="+report.Prefixes+" approaches="+report.Approaches+" steps="+report.Steps);
            Assert.True(report.Scenarios.Passed,report.Scenarios?.Reason);
            Assert.Equal(4,report.Scenarios.Trials.Count);
            Assert.True(candidate.Combined.Result.End.Mass>=built.Limits.ReserveMass);
            Assert.Equal(built.Initial.UT,candidate.Combined.Reference[0].State.UT);
            Assert.Equal(built.Initial.Mass,candidate.Combined.Reference[0].State.Mass);
            Assert.InRange(candidate.OriginalMiss,0,built.Settings.Common.TargetTolerance);
        }
    }
}
