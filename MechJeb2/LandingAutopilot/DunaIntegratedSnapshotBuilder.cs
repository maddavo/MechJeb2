using System;
using System.Collections.Generic;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // Pure conversion/admission seam shared by production KSP capture and
    // offline records made from an existing, source-identified KSP log.
    // Acquisition of Unity/KSP objects remains on the main thread.
    internal static class DunaIntegratedSnapshotBuilder
    {
        internal sealed class Input
        {
            internal string Body;
            internal bool HasAtmosphere,Active,Targeted,Packed,Landed,ChuteOption,
                ChuteDeployed,Rcs,MinimumThrottle,DifferentialThrottle,Smoothing,Far;
            internal int ChuteCount,PartCount;
            internal double Warp,UT,MassKg,AltitudeAsl,VerticalSpeed,CommandedThrottle,
                MaxAcceleration,MinAcceleration,FixedThrottleLimit,ThrottleLimit,TargetLatitude,TargetLongitude,
                TerrainAsl,Radius,Mu,AtmosphereTop,BottomOffset,TouchdownSpeed,EngineResponse,TurnRate,
                FlowKgPerSecond,UsableFuelKg,ReserveFloorKg,NominalThrustNewtons,ImpulseDerate,DragArea,
                AttainedThrottle,PointingRate;
            internal D.V Position,OrbitalVelocity,SurfaceVelocity,Omega,Forward,TargetRadial;
            internal D.Curve PressureByAltitude,DensityByAltitude,EngineIspByPressure;
            internal Func<double,D.V,D.AirSample> Atmosphere;
            internal D.Propulsion Propulsion;
            internal D.TerrainGrid Terrain;
            internal D.V FrameA,FrameB,FrameC;
            internal uint[] Parts;
            internal uint ReferencePart;
            internal string EngineConfiguration;
        }

        internal sealed class Result
        {
            internal readonly DunaLandingAdmission.Mode Mode;
            internal readonly DunaLandingAdmission.SolverFrame Frame;
            internal readonly D.Environment Environment;
            internal readonly D.State Initial;
            internal readonly D.Limits Limits;
            internal readonly D.Propulsion Propulsion;
            internal readonly AtmosphericFiniteTimePlanner.Settings Settings;
            internal readonly double Cap,UsableFuel,Reserve;
            internal readonly string Reason;
            internal bool Accepted=>Reason==null;
            internal Result(DunaLandingAdmission.Mode mode,DunaLandingAdmission.SolverFrame frame,
                D.Environment environment,D.State initial,D.Limits limits,D.Propulsion propulsion,
                AtmosphericFiniteTimePlanner.Settings settings,double cap,double usable,double reserve,string reason)
            {Mode=mode;Frame=frame;Environment=environment;Initial=initial;Limits=limits;Propulsion=propulsion;
                Settings=settings;Cap=cap;UsableFuel=usable;Reserve=reserve;Reason=reason;}
        }

        private static Result Refuse(string reason)=>new Result(DunaLandingAdmission.Mode.Refused,
            null,null,default,null,null,null,double.NaN,double.NaN,double.NaN,
            String.IsNullOrWhiteSpace(reason)?"invalid numerical snapshot":reason);

        internal static Result Build(Input x)
        {
            if(x==null)return Refuse("snapshot unavailable");
            string context=DunaLandingAdmission.CheckContext(x.Active,x.Packed,x.Landed,x.Warp,
                x.Targeted,x.ChuteOption,x.ChuteCount,x.ChuteDeployed,x.Rcs,x.MinimumThrottle,
                x.DifferentialThrottle,x.Smoothing,x.Far,x.PartCount);
            if(context!=null)return Refuse(context);
            if(x.Body!="Duna"||!x.HasAtmosphere)return Refuse("Duna atmosphere required");
            if(!D.Finite(x.UT)||!D.Finite(x.MassKg)||x.MassKg<=0||!D.Finite(x.AltitudeAsl)||
                !D.Finite(x.VerticalSpeed)||!D.Finite(x.CommandedThrottle)||
                !D.Finite(x.MaxAcceleration)||x.MaxAcceleration<=0||!D.Finite(x.MinAcceleration)||x.MinAcceleration>=1e-6||
                !D.Finite(x.FixedThrottleLimit)||!D.Finite(x.ThrottleLimit))return Refuse("invalid vessel mass, epoch or thrust limits");
            if(!D.Finite(x.Radius)||x.Radius<=0||!D.Finite(x.Mu)||x.Mu<=0||
                !D.Finite(x.AtmosphereTop)||x.AtmosphereTop<=0||!D.Finite(x.TerrainAsl)||x.TerrainAsl<0||
                !D.Finite(x.BottomOffset)||x.BottomOffset<0)return Refuse("invalid body or target terrain");
            if(!x.Position.Valid||!x.OrbitalVelocity.Valid||!x.SurfaceVelocity.Valid||!x.Omega.Valid||
                !x.Forward.Valid||!x.TargetRadial.Valid||Math.Abs(x.TargetRadial.Length-1)>1e-6||
                !D.Finite(x.PointingRate)||x.PointingRate<0||!D.Finite(x.AttainedThrottle)||
                x.AttainedThrottle<0||x.AttainedThrottle>1)
                return Refuse("invalid frame or target vector");
            if((x.OrbitalVelocity-x.SurfaceVelocity-D.V.Cross(x.Omega,x.Position)).Length>=.5)
                return Refuse("surface/inertial rotation mismatch");
            if(x.PressureByAltitude==null&&x.Atmosphere==null||
                x.DensityByAltitude==null&&x.Atmosphere==null||x.EngineIspByPressure==null||
                x.Propulsion==null)return Refuse("snapshot inputs unavailable");
            if(!D.Finite(x.DragArea)||x.DragArea<=0)return Refuse("prospective drag area unavailable");
            if(!D.Finite(x.FlowKgPerSecond)||x.FlowKgPerSecond<=0||
                !D.Finite(x.NominalThrustNewtons)||x.NominalThrustNewtons<=0||
                !D.Finite(x.ImpulseDerate)||x.ImpulseDerate<=0||
                !D.Finite(x.TouchdownSpeed)||x.TouchdownSpeed<=0||
                !D.Finite(x.EngineResponse)||x.EngineResponse<0||
                !D.Finite(x.TurnRate)||x.TurnRate<0)return Refuse("unknown propulsion or actuator capability");
            double cap=Math.Min(x.FixedThrottleLimit,x.ThrottleLimit);
            if(!D.Finite(cap)||cap<=0||cap>1)return Refuse("no known throttleable axial thrust");
            double reserve=DunaLandingAdmission.RecoveryReserve(x.ReserveFloorKg,
                x.MassKg-.95*x.UsableFuelKg);
            if(!D.Finite(x.UsableFuelKg)||x.UsableFuelKg<=0||x.UsableFuelKg>=x.MassKg||
                !D.Finite(reserve)||reserve<=0||reserve>=x.MassKg)return Refuse("unknown usable propellant inventory or reserve");
            try
            {
                var frame=new DunaLandingAdmission.SolverFrame(
                    new DunaLandingAdmission.BodyBasis(x.FrameA,x.FrameB,x.FrameC),x.Omega,x.UT);
                var initial=new D.State(x.UT,x.Position,x.OrbitalVelocity,x.MassKg,x.Forward.Unit,
                    x.AttainedThrottle,x.PointingRate);
                var environment=new D.Environment(x.Radius,x.Mu,x.Omega,x.AtmosphereTop,
                    x.PressureByAltitude,x.DensityByAltitude,x.DragArea,x.Radius+x.TerrainAsl,
                    x.BottomOffset,atmosphere:x.Atmosphere,grid:x.Terrain,
                    terrainEpoch:x.UT,terrainA:x.FrameA,terrainB:x.FrameB,terrainC:x.FrameC);
                if(!initial.Valid||!environment.Valid)return Refuse("invalid numerical snapshot");
                var mode=DunaLandingAdmission.Choose(environment,initial,x.CommandedThrottle,
                    x.VerticalSpeed,out string admissionReason);
                if(mode==DunaLandingAdmission.Mode.Refused)return Refuse(admissionReason);
                var limits=new D.Limits(reserve,x.TurnRate,x.EngineResponse,x.EngineResponse,.5);
                if(!limits.Valid)return Refuse("unknown actuator response");
                var settings=DunaLandingAdmission.Settings(environment,x.Propulsion,initial,
                    x.TargetRadial,cap,x.MaxAcceleration,x.ImpulseDerate,x.TouchdownSpeed);
                if(settings==null)return Refuse("invalid solver settings or horizon");
                return new Result(mode,frame,environment,initial,limits,x.Propulsion,settings,
                    cap,x.UsableFuelKg,reserve,null);
            }
            catch(Exception ex){return Refuse(ex.Message);}
        }
    }
}
