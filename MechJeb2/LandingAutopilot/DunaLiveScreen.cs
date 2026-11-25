using System;
using System.Collections.Generic;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // Main-thread snapshot: current allocated forward authority, actual
    // connected homogeneous LF/Ox mixture, pressure/Isp. No aerodynamic
    // credit. Future trim/CoM/flow changes remain an explicit uncertainty.
    internal sealed class DunaLiveScreen
    {
        internal D.Environment Environment;
        internal D.Propulsion Propulsion;
        internal D.Limits Limits;
        internal D.State State;
        internal double UsableMass,AllocatedForce,AllocatedFlow,TerminalRadius,TorqueBalancedForce=double.NaN;
        internal string EngineDetails="none";
        internal string AuthoritySource="allocated_margin";
        internal static D.V Vector(Vector3d v)=>new D.V(v.x,v.y,v.z);
        internal static bool TryBuild(Vessel vessel,VesselState vs,double available,double terrain,double turnRate,
            double response,out DunaLiveScreen snapshot,out string reason)
        {
            snapshot=null;reason="invalid_state";
            try
            {
                if(vessel==null||vs==null||vessel.mainBody==null||!D.Finite(available)||available<=0||
                    !D.Finite(terrain)||terrain<=0||!D.Finite(vs.Mass)||vs.Mass<=0||
                    !D.Finite(turnRate)||turnRate<=0||!D.Finite(response)||response<0)return false;
                CelestialBody body=vessel.mainBody;
                var wrappers=vs.EngineWrappers;
                reason="engine_count";
                if(wrappers.Count==0||wrappers.Count>8)return false;
                double cap=Math.Min(1,Math.Min(vs.ThrottleFixedLimit,vs.ThrottleLimit));
                reason="throttle_or_flow";
                if(!D.Finite(cap)||cap<=0||vs.MinimumEngineMassFlow>1e-8)return false;
                double mass=vs.Mass*1000,allocated=available*mass,flow=vs.MaximumEngineMassFlow*1000*cap;
                if(!D.Finite(flow)||flow<=0)return false;
                double maxMix=double.PositiveInfinity,mixDensity=0;
                var ratios=new Dictionary<int,double>();
                var engines=new List<ModuleEngines>();
                foreach(var wrapper in wrappers)
                {
                    ModuleEngines e=wrapper.Engine;
                    reason="unsupported_engine";
                    if(e==null||!e.EngineIgnited||!e.isEnabled||!e.isOperational||e.throttleLocked||
                        e.minFuelFlow>1e-8||e.atmChangeFlow||e.useAtmCurve||e.useAtmCurveIsp||e.useThrottleIspCurve||
                        e.useVelCurve||e.useVelCurveIsp||e.useThrustCurve)return false;
                    reason="unsupported_mixture";
                    if(e.propellants.Count!=2)return false;
                    double density=0;
                    var current=new Dictionary<int,double>();
                    foreach(Propellant p in e.propellants)
                    {
                        var def=PartResourceLibrary.Instance.GetDefinition(p.id);
                        if(def==null||(def.name!="LiquidFuel"&&def.name!="Oxidizer")||p.ignoreForIsp||
                            !D.Finite(p.ratio)||p.ratio<=0||def.density<=0||current.ContainsKey(p.id))return false;
                        current.Add(p.id,p.ratio);
                        density+=p.ratio*def.density*1000;
                        // Query actual resource flow mode. Minimum across
                        // engines avoids double-counting shared pools.
                        reason="connected_fuel";
                        e.part.GetConnectedResourceTotals(p.id,p.GetFlowMode(),out double amount,out double capacity,true);
                        if(!D.Finite(amount)||amount<0)return false;
                        maxMix=Math.Min(maxMix,amount/p.ratio);
                    }
                    reason="mixed_engine_ratios";
                    if(engines.Count==0){ratios=current;mixDensity=density;}
                    else foreach(var kv in ratios)
                        if(!current.TryGetValue(kv.Key,out double ratio)||Math.Abs(ratio-kv.Value)>1e-6)return false;
                    engines.Add(e);
                }
                double usable=maxMix*mixDensity;
                reason="invalid_connected_fuel";
                if(!D.Finite(usable)||usable<=0||usable>=mass)return false;
                double top=body.RealMaxAtmosphereAltitude();
                reason="body_pressure";
                if(!D.Finite(top)||top<=0)return false;
                double pressure=vessel.staticPressurekPa*1000;
                double maxP=0;
                var pressureKeys=new D.Knot[65];var densityKeys=new D.Knot[65];
                // Pressure is sampled on the flight thread, no drag credit.
                double terminalRadius=Math.Min(body.Radius,body.pqsController.radiusMin);
                if(!D.Finite(terminalRadius)||terminalRadius<=0)return false;
                double bottom=terminalRadius-body.Radius;
                for(int i=0;i<pressureKeys.Length;i++)
                {
                    double h=bottom+(top-bottom)*i/(pressureKeys.Length-1);
                    double p=i==pressureKeys.Length-1?0:body.GetPressure(h)*1000;
                    if(!D.Finite(p)||p<0)return false;
                    pressureKeys[i]=new D.Knot(h,p);densityKeys[i]=new D.Knot(h,0);maxP=Math.Max(maxP,p);
                }
                maxP=Math.Max(maxP,pressure);
                if(!D.Finite(maxP)||maxP<=0)return false;
                // Restricted neutral torque model using current allocated
                // upper bounds. No increase above measured allocation.
                var balanceEngines=new List<D.Engine>();
                bool balanceSupported=engines.Count<=2;
                foreach(var wrapper in wrappers)
                {
                    var e=wrapper.Engine;
                    Vector3d f=wrapper.MaxVariableForce,t=wrapper.MaxVariableTorque;
                    if(f.y<=0||Math.Abs(f.x)+Math.Abs(f.z)>1e-6*f.y||Math.Abs(t.y)>1e-6*Math.Max(1,t.magnitude))
                    {balanceSupported=false;break;}
                    var keys=new D.Knot[33];
                    for(int i=0;i<keys.Length;i++)
                    {
                        double atm=maxP*i/32/101325;
                        keys[i]=new D.Knot(atm,e.atmosphereCurve.Evaluate((float)atm));
                    }
                    balanceEngines.Add(new D.Engine(e.maxFuelFlow*e.flowMultiplier*1000,
                        e.thrustPercentage/100.0*cap,e.g,new D.V(t.z/f.y,0,-t.x/f.y),new D.Curve(keys)));
                }
                D.Propulsion balance=balanceSupported?new D.Propulsion(true,balanceEngines.ToArray()):null;
                D.Authority balanced=balance==null?default:balance.At(pressure);
                // A zero neutral allocation does not imply impossible
                // steering with other actuators. Use observed allocation
                // explicitly as an experimental, uncertain force envelope.
                double forceCap=balanced.Known&&balanced.Force>0?Math.Min(allocated,balanced.Force):allocated;
                double scale=forceCap/allocated;
                var forceKeys=new D.Knot[33];
                reason="engine_isp";
                for(int i=0;i<forceKeys.Length;i++)
                {
                    double p=maxP*i/(forceKeys.Length-1),ratio=1;
                    foreach(ModuleEngines e in engines)
                    {
                        double isp=e.atmosphereCurve.Evaluate((float)(pressure/101325)),
                            future=e.atmosphereCurve.Evaluate((float)(p/101325));
                        if(!D.Finite(isp)||isp<=0||!D.Finite(future)||future<=0)return false;
                        ratio=Math.Min(ratio,future/isp);
                    }
                    double f=forceCap*ratio;
                    if(balance!=null)
                    {
                        D.Authority q=balance.At(p);
                        if(q.Known&&q.Force>0)f=Math.Min(f,q.Force);
                    }
                    forceKeys[i]=new D.Knot(p,DunaDynamicScreen.Margin*f);
                }
                snapshot=new DunaLiveScreen
                {
                    Environment=new D.Environment(body.Radius,body.gravParameter,Vector(body.angularVelocity),top,
                        new D.Curve(pressureKeys),new D.Curve(densityKeys),0,terrain,0),
                    Propulsion=D.Propulsion.FromAllocatedEnvelope(new D.Curve(forceKeys),flow*scale*DunaDynamicScreen.Margin),
                    Limits=new D.Limits(mass-usable*(1-DunaDynamicScreen.FuelReserveFraction),turnRate,response,response,1),
                    State=new D.State(vs.Time,Vector(vs.CoM-body.position),Vector(vs.OrbitalVelocity),mass,Vector(vs.Forward).Unit,
                        Math.Max(0,Math.Min(1,vessel.ctrlState.mainThrottle))),
                    UsableMass=usable,AllocatedForce=allocated,AllocatedFlow=flow,TerminalRadius=terminalRadius,
                    TorqueBalancedForce=balanced.Known?balanced.Force:double.NaN,
                    AuthoritySource=balanced.Known&&balanced.Force>0?"bounded_neutral_balance":"allocated_margin"
                };
                var details=new System.Text.StringBuilder();
                for(int i=0;i<engines.Count;i++)
                {
                    ModuleEngines e=engines[i];
                    if(i>0)details.Append(';');
                    details.AppendFormat(System.Globalization.CultureInfo.InvariantCulture,
                        "{0}:{1:R},{2:R},{3:R},{4:R},{5:R}",i,e.thrustPercentage,e.maxFuelFlow,e.flowMultiplier,
                        e.atmosphereCurve.Evaluate((float)(pressure/101325)),e.finalThrust);
                }
                snapshot.EngineDetails=details.ToString();
                reason="ok";
                if(snapshot.State.Valid&&snapshot.Limits.Valid&&snapshot.Environment.Valid)return true;
                snapshot=null;reason="invalid_model";return false;
            }
            catch(Exception e){reason="snapshot_"+e.GetType().Name;snapshot=null;return false;}
        }
    }
}
