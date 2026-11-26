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
