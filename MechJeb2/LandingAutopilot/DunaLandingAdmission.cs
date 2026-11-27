using System;
using System.Collections.Generic;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;

namespace MuMech.Landing
{
    // Shared production admission decisions, independent of Unity. A
    // caller supplies measurements from KSP; tests supply equivalent
    // measured or representative inputs. No test-only alternate policy.
    internal static class DunaLandingAdmission
    {
        internal enum Mode { Legacy, Orbit, Descent, Refused }

        // Fixed inertial solver axes at capture. KSP world axes can rotate
        // with the body. Three body-fixed axes identify current orientation.
        internal readonly struct BodyBasis
        {
            internal readonly D.V A,B,C;
            internal BodyBasis(D.V a,D.V b,D.V c){A=a.Unit;B=b.Unit;C=c.Unit;}
            internal double Handedness=>D.V.Dot(D.V.Cross(A,B),C);
            internal bool Valid=>A.Valid&&B.Valid&&C.Valid&&
                Math.Abs(Math.Abs(Handedness)-1)<1e-5&&
                Math.Abs(D.V.Dot(A,B))<1e-5&&Math.Abs(D.V.Dot(A,C))<1e-5&&Math.Abs(D.V.Dot(B,C))<1e-5;
            internal D.V MapTo(BodyBasis other,D.V v)=>
                other.A*D.V.Dot(v,A)+other.B*D.V.Dot(v,B)+other.C*D.V.Dot(v,C);
        }

        internal sealed class SolverFrame
        {
            internal readonly BodyBasis AtCapture;
            internal readonly D.V Omega;
            internal readonly double Epoch;
            internal SolverFrame(BodyBasis basis,D.V omega,double epoch)
            {
                if(!basis.Valid||!omega.Valid||!D.Finite(epoch))throw new ArgumentException("invalid solver frame");
                AtCapture=basis;Omega=omega;Epoch=epoch;
            }
            private void Check(BodyBasis current,double ut)
            {
                if(!current.Valid||current.Handedness*AtCapture.Handedness<0||!D.Finite(ut))
                    throw new ArgumentException("invalid live body basis or epoch");
            }
            // Orbital velocity already includes physical body rotation;
            // rotate components only, never add omega cross r again.
            internal D.V ToSolver(BodyBasis current,D.V world,double ut)
            {
                Check(current,ut);
                return P.RotateTarget(current.MapTo(AtCapture,world),Omega,ut-Epoch);
            }
            // Current UT for commands; predicted UT for a future ground
            // position queried against the CURRENT body axes.
            internal D.V ToWorld(BodyBasis current,D.V solver,double vectorUT)
            {
                Check(current,vectorUT);
                return AtCapture.MapTo(current,P.RotateTarget(solver,Omega,Epoch-vectorUT));
            }
            internal bool TargetMatches(BodyBasis current,D.V world,D.V initial,double ut)
                =>world.Valid&&initial.Valid&&
                    (ToSolver(current,world,ut)-P.RotateTarget(initial,Omega,ut-Epoch)).Length<=1e-5;
        }

        internal static bool UsesIntegratedTargeted(string body,bool atmosphere,bool targeted)
            =>body=="Duna"&&atmosphere&&targeted;

        internal static string CheckContext(bool active,bool packed,bool landed,double warp,bool target,
            bool deployChutes,int parachuteCount,bool parachuteDeployed,bool rcsActive,bool minThrottle,
            bool differential,bool smoothing,bool far,int parts)
        {
            if(!active||packed||landed||warp!=1)return "active unpacked flight at 1x required";
            if(!target)return "select a Duna landing target";
            if(parachuteDeployed||(deployChutes&&parachuteCount>0))return "parachute dynamics unsupported";
            if(rcsActive)return "active RCS unsupported";
            if(minThrottle)return "minimum throttle floor unsupported";
            if(differential)return "differential throttle model unsupported";
            if(smoothing)return "throttle smoothing model unsupported";
            if(far)return "FAR aerodynamics unsupported";
            if(parts<1||parts>512)return "unsupported part count";
            return null;
        }

        // Stock KSP drag cubes provide prospective area/coefficient even
        // without dynamic pressure. Use the same six-face average as the
        // existing landing autopilot, multiplied by both stock factors.
        // Invalid samples are unknown, not zero; intentionally shielded
        // or cube-less parts are skipped by the acquisition adapter.
        internal sealed class DragAccumulator
        {
            private double sum;
            private int count;
            private bool valid=true;
            internal void AddFace(double coefficient,double area)
            {
                if(!D.Finite(coefficient)||coefficient<0||!D.Finite(area)||area<0)
                    {valid=false;return;}
                sum+=coefficient*area;count++;
            }
            internal double Finish(double cubeMultiplier,double dragMultiplier)
            {
                if(!valid||count<6||count%6!=0||!D.Finite(cubeMultiplier)||cubeMultiplier<=0||
                    !D.Finite(dragMultiplier)||dragMultiplier<=0)return double.NaN;
                double result=sum/6*cubeMultiplier*dragMultiplier;
                return D.Finite(result)&&result>0?result:double.NaN;
            }
        }

        internal static Mode Choose(D.Environment env,D.State state,double commandedThrottle,
            double verticalSpeed,out string reason)
        {
            reason=null;
            if(env==null||!env.Valid||!state.Valid){reason="invalid numerical snapshot";return Mode.Refused;}
            if(!(env.DragArea>0)){reason="prospective drag area unavailable";return Mode.Refused;}
            double h=env.PhysicalClearance(state);
            if(!D.Finite(h)||h<=0||h>200000){reason="outside landing altitude domain";return Mode.Refused;}
            if(!D.Finite(commandedThrottle)||commandedThrottle>=1e-4||commandedThrottle<0||state.Throttle>.01)
            {reason="engagement requires unpowered coast";return Mode.Refused;}
            if(AtmosphericStrategicPlanner.TryOrbit(env,state,out var orbit)&&orbit.Periapsis>env.Radius+env.AtmosphereTop)
            {
                if(orbit.Eccentricity>.2||orbit.Apoapsis-env.Radius>200000)
                {reason="strategic landing requires low bound orbit above atmosphere (e <= 0.2)";return Mode.Refused;}
                return Mode.Orbit;
            }
            if(!D.Finite(verticalSpeed)||verticalSpeed>=0)
            {reason="descending state required for entry trajectory";return Mode.Refused;}
            return Mode.Descent;
        }

        // Same resource-mixture arithmetic and checks for acquired KSP
        // resource samples and offline fixtures. Density is kg per unit.
        internal static double UsableFuel(IDictionary<int,double> inventory,IDictionary<int,double> mixture,
            Func<int,double> density,double mass)
        {
            if(inventory==null||mixture==null||mixture.Count==0||density==null||!D.Finite(mass)||mass<=0)
                return double.NaN;
            double units=double.PositiveInfinity,mixMass=0;
            foreach(var kv in mixture)
            {
                if(!inventory.TryGetValue(kv.Key,out double amount))amount=0;
                double d=density(kv.Key);
                if(!D.Finite(kv.Value)||kv.Value<=0||!D.Finite(amount)||amount<0||!D.Finite(d)||d<=0)
                    return double.NaN;
                units=Math.Min(units,amount/kv.Value);
                mixMass+=kv.Value*d;
            }
            double fuel=units*mixMass;
            return D.Finite(fuel)&&fuel>0&&fuel<mass?fuel:double.NaN;
        }

        internal static double ImpulseDerate(double maxAcceleration,double mass,double nominalThrust)
        {
            if(!D.Finite(maxAcceleration)||maxAcceleration<=0||!D.Finite(mass)||mass<=0||
                !D.Finite(nominalThrust)||nominalThrust<=0)return double.NaN;
            return .90*Math.Min(1,maxAcceleration*mass/nominalThrust);
        }

        // The real snapshot adapter and offline cases construct exactly
        // the same solver settings. No bespoke easier test settings.
        internal static F.Settings Settings(D.Environment environment,D.Propulsion propulsion,D.State initial,
            D.V target,double cap,double maxAcceleration,double derate,double touchdown)
        {
            if(environment==null||!environment.Valid||propulsion==null||!initial.Valid||
                !target.Valid||Math.Abs(target.Length-1)>1e-6||
                !D.Finite(cap)||cap<=0||cap>1||!D.Finite(maxAcceleration)||maxAcceleration<=0||
                !D.Finite(derate)||derate<=0||!D.Finite(touchdown)||touchdown<=0)return null;
            double clearance=environment.PhysicalClearance(initial);
            double gravity=environment.Mu/(environment.Radius*environment.Radius);
            if(!D.Finite(clearance)||clearance<=0||gravity<=0)return null;
            double speed=environment.Surface(initial.R,initial.Velocity).Length;
            double scale=Math.Max(12,Math.Max(Math.Sqrt(2*clearance/gravity),speed/Math.Max(.1,maxAcceleration*cap*derate)));
            double coast=Math.Min(300,Math.Max(0,scale*.75));
            double minDuration=Math.Max(8,scale*.65),maxDuration=Math.Min(900,Math.Max(minDuration,scale*3));
            if(minDuration>maxDuration)return null;
            var terminal=new T.Settings(.2,touchdown,gravity/9.81,0,
                st=>{
                    if(!environment.Air(st.UT,st.R,out double pressurePa,out _))return default;
                    var a=propulsion.At(pressurePa);
                    double raw=a.Force/st.Mass;
                    return a.Known&&raw>0?new T.ControllerAuthority(raw,raw*cap,cap,false):default;
                },
                (st,dir)=>Math.Acos(Math.Max(-1,Math.Min(1,D.V.Dot(st.Forward,dir.Unit))))*180/Math.PI,
                (ut,p)=>environment.TerrainRadius);
            var memory=new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,initial.Throttle,initial.Throttle);
            var common=new P.Settings(target,1800,coast,100,terminal,memory,intervals:8,refine:1,
                maxCandidates:64,maxSteps:150000,captureClearance:50,captureSpeed:15,
                safeVertical:Math.Max(1,touchdown),safeHorizontal:1,safeAngle:5*Math.PI/180,
                safeRate:Math.PI/180,entryCutoffDelay:0,commandCap:cap);
            var settings=new F.Settings(common,environment.TerrainRadius+environment.BottomOffset,
                minPowered:minDuration,maxPowered:maxDuration,controlTick:.5,coastIntervals:4,
                durationIntervals:6,refinements:2,minCoast:Math.Min(15,coast));
            return settings.Valid?settings:null;
        }

        // Shared live/orbital acceptance decisions. A safe unpowered
        // bound orbit may abort; once committed, the independent hazard
        // supervisor can take over. No separate test-only policy.
        internal static bool OrbitalPreburn(D.Environment env,D.State actual,bool strategic,bool burnRequested)
            =>strategic&&(!burnRequested||
                AtmosphericStrategicPlanner.TryOrbit(env,actual,out var orbit)&&
                orbit.Periapsis>env.Radius+env.AtmosphereTop);

        internal static bool OrbitalBurnAligned(AtmosphericPlanTracker tracker,D.State actual,
            bool orbitalPrefix,double start,double end)
            =>!orbitalPrefix||actual.UT<start||actual.UT>=end||
                tracker.ReferenceAt(actual.UT,out _,out var planned)&&
                D.V.Dot(actual.Forward,planned.Direction.Unit)>=.995;

        internal static bool TerrainConflict(double clearance,double modelClearance,bool contact)
            =>!D.Finite(clearance)||!D.Finite(modelClearance)||(contact?Math.Abs(clearance)>2:
                clearance<=0||modelClearance<=30&&Math.Abs(clearance-modelClearance)>2);

        // A regular-warp window must be both a validated zero-thrust
        // reference interval and vacuum on the current osculating orbit.
        // Similar to the airless preparation policy, return to physics
        // early enough for a complete attitude slew and settling.
        internal const double WarpPreparationSeconds=60;
        internal const double WarpAtmosphereMargin=5000;
        internal static double VacuumBoundaryUT(D.Environment env,D.State actual,double boundaryRadius)
        {
            if(env==null||!actual.Valid||!D.Finite(boundaryRadius)||boundaryRadius<=0||
                actual.R.Length<=boundaryRadius||
                !AtmosphericStrategicPlanner.TryOrbit(env,actual,out var orbit))return actual.UT;
            if(orbit.Periapsis>boundaryRadius)return double.PositiveInfinity;
            double a=orbit.SemiMajor,e=orbit.Eccentricity;
            if(e<1e-8)return actual.UT;
            double cos=Math.Max(-1,Math.Min(1,(1-actual.R.Length/a)/e));
            double sin=D.V.Dot(actual.R,actual.Velocity)/(e*Math.Sqrt(env.Mu*a));
            double eccentric=Math.Atan2(sin,cos);
            double mean=eccentric-e*Math.Sin(eccentric);
            double entry=-Math.Acos(Math.Max(-1,Math.Min(1,(1-boundaryRadius/a)/e)));
            double target=entry-e*Math.Sin(entry);
            double delta=(target-mean)%(2*Math.PI);
            if(delta<0)delta+=2*Math.PI;
            return actual.UT+delta*orbit.Period/(2*Math.PI);
        }
        internal static double WarpExitUT(D.Environment env,AtmosphericPlanTracker tracker,D.State actual,
            double commandedThrottle,double appliedThrottle,double acceleration,double minimumAcceleration,
            bool autoWarp,bool physicsReady,bool recoveryActive)
        {
            if(!autoWarp||!physicsReady||recoveryActive||env==null||tracker==null||!actual.Valid||
                !D.Finite(commandedThrottle)||Math.Abs(commandedThrottle)>1e-4||
                !D.Finite(appliedThrottle)||Math.Abs(appliedThrottle)>1e-4||actual.Throttle>1e-4||
                !D.Finite(acceleration)||Math.Abs(acceleration)>.001||
                !D.Finite(minimumAcceleration)||minimumAcceleration>1e-6)return double.NaN;
            double boundary=env.Radius+env.AtmosphereTop+WarpAtmosphereMargin;
            if(actual.R.Length<=boundary||!tracker.ReferenceAt(actual.UT,out _,out var command)||command.Throttle!=0)
                return double.NaN;
            double next=tracker.NextPoweredOrAtmosphericUT(actual.UT,boundary);
            double entry=VacuumBoundaryUT(env,actual,boundary);
            double exit=Math.Min(next,entry)-WarpPreparationSeconds;
            return D.Finite(exit)&&exit-actual.UT>=30?exit:double.NaN;
        }
        internal enum WarpAction { Continue, Stop, WaitForPhysics, Revalidate }
        internal static WarpAction AdvanceWarp(double now,double exit,double rate,bool packed,
            bool autoWarp,bool contextValid,bool regular,bool paused)
        {
            if(!contextValid||!autoWarp||paused||!regular||!D.Finite(now)||!D.Finite(exit)||now>=exit-10)
                return packed||rate!=1?WarpAction.Stop:WarpAction.Revalidate;
            if(rate==1&&!packed&&exit-now<30)return WarpAction.Revalidate;
            return WarpAction.Continue;
        }
        // Alarm braking can precede the popup. Preserve the original
        // preparation deadline, but do not confuse this with manual stop.
        internal enum AlarmWarpAction { Normal, Suspend, WaitForPhysics, Resume, EndIntent }
        internal static bool PendingWarpAlarm(bool triggered,bool actioned,bool haltWarp,bool pauseGame)
            =>triggered&&!actioned&&(haltWarp||pauseGame);
        internal static AlarmWarpAction AlarmWarpTransition(double now,double exit,bool suspended,
            bool alarmAffectingWarp,bool pendingAlarm,bool manualCancel,bool autoWarp,bool paused,
            bool contextValid,bool competingWarp,bool physicsReady)
        {
            if(manualCancel||!autoWarp||paused||!contextValid||competingWarp||
                !D.Finite(now)||!D.Finite(exit)||now>=exit-10)
                return AlarmWarpAction.EndIntent;
            if(alarmAffectingWarp||pendingAlarm)return AlarmWarpAction.Suspend;
            if(!suspended)return AlarmWarpAction.Normal;
            // Same minimum useful preparation-safe window as initial warp.
            if(exit-now<30)return AlarmWarpAction.EndIntent;
            return physicsReady?AlarmWarpAction.Resume:AlarmWarpAction.WaitForPhysics;
        }
        internal static bool WarpPhysicsReady(double rate,bool packed,bool fresh)
            =>rate==1&&!packed&&fresh;

        internal static bool OwnsActuators(int attitudeUsers,bool landingAttitude,int thrustUsers,
            bool thrustSelf,bool landingThrust,bool pending)
            =>pending?attitudeUsers==1&&landingAttitude&&thrustUsers==2&&thrustSelf&&landingThrust:
                attitudeUsers==0&&thrustUsers==1&&thrustSelf&&!landingThrust;

        // Explicit one-click lifecycle. An unsupported or stale plan
        // cannot fall through into the legacy targeted controller.
        internal enum Progress { Waiting, Engage, Refuse }
        internal static Progress Next(bool contextValid,bool ownersValid,bool running,
            bool corridorReady,bool finished,bool admissionPassed)
        {
            if(!contextValid||!ownersValid)return Progress.Refuse;
            if(corridorReady)return admissionPassed?Progress.Engage:Progress.Refuse;
            return running||!finished?Progress.Waiting:Progress.Refuse;
        }
    }
}
