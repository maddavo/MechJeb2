using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading;
using UnityEngine;

namespace MuMech.Landing
{
    // Optional, short calibration burst. No hard dependency on Harmony or MFI,
    // no ownership of their override slots, no change to force lists or controls.
    // Neither a prefix nor a postfix certifies a completed Unity physics step.
    internal sealed class DunaM1ForceProbe
    {
        internal const double WindowSeconds = 0.5;
        private const int MaxEvents = 40000, MaxBufferedEvents = 4096,
            MaxBufferedChars = 2 * 1024 * 1024, MaxHolders = 32;
        private const string Owner = "maddavo.mechjeb.duna.m1.forceprobe";
        private static volatile DunaM1ForceProbe active;
        private readonly Guid vessel;
        private readonly int thread;
        private readonly List<MethodBase> patched = new List<MethodBase>();
        private readonly List<string> events = new List<string>();
        private readonly List<string> routeInfo = new List<string>();
        private object harmony;
        private MethodInfo unpatch;
        private MethodInfo stock;
        private long seq;
        private int bufferedChars, accepted, dropped, errors;
        private bool enabled;
        private double armReadUT;
        internal string WindowStopReason { get; private set; }
        private static readonly FieldInfo PartForce = typeof(Part).GetField("force", BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic);
        private static readonly FieldInfo PartTorque = typeof(Part).GetField("torque", BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic);
        private static readonly MethodInfo PrefixMethod = typeof(DunaM1ForceProbe).GetMethod(nameof(Prefix),
            BindingFlags.Static | BindingFlags.NonPublic);
        private static readonly MethodInfo PostfixMethod = typeof(DunaM1ForceProbe).GetMethod(nameof(Postfix),
            BindingFlags.Static | BindingFlags.NonPublic);

        private static string N(double x) => DunaM1Capture.N(x);
        private static string Q(string s) => DunaM1Capture.Q(s);
        private static string B(bool b) => b ? "true" : "false";
        private static string I(long n) => n.ToString(CultureInfo.InvariantCulture);
        private static string V(Vector3d v) => "["+N(v.x)+","+N(v.y)+","+N(v.z)+"]";
        private static string ArrayJson(IEnumerable<string> values) => "["+string.Join(",",values)+"]";
        private static void Field(StringBuilder b,string key,string json) =>
            b.Append(',').Append(Q(key)).Append(':').Append(json);

        private DunaM1ForceProbe(Guid id)
        {
            vessel=id; thread=Thread.CurrentThread.ManagedThreadId;
        }

        // Only an already loaded, exactly identified compatible assembly is used.
        private static Assembly Loaded(string name) =>
            AppDomain.CurrentDomain.GetAssemblies().SingleOrDefault(a=>a.GetName().Name==name);

        private static MethodInfo Integrate(Type t)
        {
            MethodInfo m=t?.GetMethod("Integrate",BindingFlags.Public|BindingFlags.NonPublic|
                BindingFlags.Instance|BindingFlags.DeclaredOnly,null,new[]{typeof(Part)},null);
            if(m==null || m.ReturnType!=typeof(void)) throw new MissingMethodException("Integrate(Part)");
            return m;
        }

        private static string Owners(MethodInfo getInfo, MethodBase original)
        {
            object info=getInfo.Invoke(null,new object[]{original});
            if(info==null)return "[]";
            var owners=info.GetType().GetProperty("Owners")?.GetValue(info,null) as IEnumerable;
            if(owners==null)throw new InvalidOperationException("Harmony patch owners unavailable");
            var names=new List<string>();
            foreach(object o in owners)names.Add(Q(o as string));
            return ArrayJson(names);
        }

        internal static DunaM1ForceProbe Start(Guid vessel,out string status)
        {
            var p=new DunaM1ForceProbe(vessel);
            try
            {
                if(active!=null)throw new InvalidOperationException("Probe already active");
                Assembly h=Loaded("0Harmony");
                if(h==null || h.GetName().Version!=new Version(2,2,1,0))
                    throw new InvalidOperationException("Loaded Harmony 2.2.1.0 required");
                Type ht=h.GetType("HarmonyLib.Harmony",true), hm=h.GetType("HarmonyLib.HarmonyMethod",true);
                p.harmony=Activator.CreateInstance(ht,new object[]{Owner});
                MethodInfo patch=ht.GetMethod("Patch",new[]{typeof(MethodBase),hm,hm,hm,hm});
                p.unpatch=ht.GetMethod("Unpatch",new[]{typeof(MethodBase),typeof(MethodInfo)});
                MethodInfo getInfo=ht.GetMethod("GetPatchInfo",new[]{typeof(MethodBase)});
                FieldInfo priority=hm.GetField("priority");
                if(patch==null || p.unpatch==null || getInfo==null || priority==null)
                    throw new MissingMethodException("Harmony observer API");
                object pre=Activator.CreateInstance(hm,new object[]{PrefixMethod});
                object post=Activator.CreateInstance(hm,new object[]{PostfixMethod});
                // Best effort only: other patches can share priorities or mutate/skip original.
                priority.SetValue(pre,0); priority.SetValue(post,800);
                p.stock=Integrate(typeof(Part).Assembly.GetType("FlightIntegrator"));
                var targets=new List<MethodInfo>{p.stock};
                Assembly mfi=Loaded("ModularFlightIntegrator");
                if(mfi!=null)
                {
                    if(mfi.GetName().Version!=new Version(1,0,0,0))
                        throw new InvalidOperationException("Unexpected MFI assembly version");
                    targets.Add(Integrate(mfi.GetType("ModularFI.ModularFlightIntegrator")));
                }
                foreach(MethodInfo target in targets)
                {
                    string before=Owners(getInfo,target);
                    if(before.Contains(Q(Owner)))throw new InvalidOperationException("Probe owner already present on target");
                    // Track the target even if Patch partially succeeds then throws.
                    p.patched.Add(target);
                    patch.Invoke(p.harmony,new object[]{target,pre,post,null,null});
                    string after=Owners(getInfo,target);
                    if(!after.Contains(Q(Owner)))throw new InvalidOperationException("Patch owner not reported");
                    p.routeInfo.Add("{\"method\":"+Q(target.DeclaringType.FullName+".Integrate")+
                        ",\"ownersBefore\":"+before+",\"ownersAfter\":"+after+"}");
                }
                p.armReadUT=Planetarium.GetUniversalTime();
                if(double.IsNaN(p.armReadUT)||double.IsInfinity(p.armReadUT))
                    throw new InvalidOperationException("Invalid initial observation UT");
                p.enabled=true; active=p;
                status="{\"status\":\"armed_unvalidated\",\"windowSeconds\":"+N(WindowSeconds)+
                    ",\"armReadUT\":"+N(p.armReadUT)+",\"maxEvents\":"+I(MaxEvents)+",\"maxHoldersPerPartEvent\":"+I(MaxHolders)+
                    ",\"mfiAssemblyLoaded\":"+B(mfi!=null)+",\"routes\":"+ArrayJson(p.routeInfo)+
                    ",\"prefixPriority\":0,\"postfixPriority\":800,\"physicsEpochVerified\":false}";
                return p;
            }
            catch(Exception ex)
            {
                bool cleaned=p.Stop();
                status="{\"status\":\"unavailable\",\"errorType\":"+Q(ex.GetType().Name)+
                    ",\"error\":"+Q(ex.InnerException?.Message??ex.Message)+",\"cleanupOk\":"+B(cleaned)+"}";
                return null;
            }
        }

        internal bool LimitReached => accepted>=MaxEvents;
        private bool CanObserve(Part p) =>
            enabled && Thread.CurrentThread.ManagedThreadId==thread && p!=null &&
            p.vessel!=null && p.vessel.id==vessel && p.vessel.mainBody!=null &&
            p.vessel.mainBody.bodyName=="Duna";

        // The hook stops collecting even if the Drive callback stalls or
        // goes away; the next normal lifecycle callback removes the patches.
        private bool InsideWindow()
        {
            if(WindowStopReason!=null)return false;
            double ut=Planetarium.GetUniversalTime();
            if(double.IsNaN(ut)||double.IsInfinity(ut)||ut<armReadUT)WindowStopReason="read_ut_invalid_or_regressed";
            else if(TimeWarp.CurrentRate!=1)WindowStopReason="warp_changed";
            else if(ut-armReadUT>WindowSeconds)WindowStopReason="window_complete";
            return WindowStopReason==null;
        }

        private bool Add(string json)
        {
            if(accepted>=MaxEvents || events.Count>=MaxBufferedEvents ||
                json.Length>MaxBufferedChars-bufferedChars)
            {
                if(dropped<int.MaxValue)dropped++;
                return false;
            }
            events.Add(json); bufferedChars+=json.Length; accepted++; return true;
        }

        private StringBuilder Begin(string kind,long sequence)
        {
            var b=new StringBuilder("{\"kind\":").Append(Q(kind));
            Field(b,"sequence",I(sequence));
            Field(b,"readUT",N(Planetarium.GetUniversalTime()));
            Field(b,"unityFixedTimeRaw",N(Time.fixedTime));
            Field(b,"unityFrameCount",I(Time.frameCount));
            Field(b,"fixedDeltaTime",N(TimeWarp.fixedDeltaTime));
            Field(b,"warpRate",N(TimeWarp.CurrentRate));
            return b;
        }

        private static string RawAccumulator(FieldInfo field,Part part)
        {
            object value=field?.GetValue(part);
            if(value is Vector3d d)return V(d);
            if(value is Vector3 f)return V(f);
            return "null";
        }

        private static string Holders(Part part,out int count)
        {
            count=-1;
            if(part.forces==null)return "null";
            count=part.forces.Count;
            var b=new StringBuilder("[");
            for(int i=0;i<Math.Min(count,MaxHolders);i++)
            {
                if(i!=0)b.Append(',');
                var h=part.forces[i];
                b.Append("{\"forceRawKspKn\":").Append(V(h.force))
                    .Append(",\"positionWorld\":").Append(V(h.pos)).Append('}');
            }
            return b.Append(']').ToString();
        }

        private static void Prefix(MethodBase __originalMethod, Part __0, out long __state)
        {
            __state=0;
            DunaM1ForceProbe p=active;
            if(p==null)return;
            try
            {
                if(!p.CanObserve(__0)||!p.InsideWindow())return;
                if(p.LimitReached){if(p.dropped<int.MaxValue)p.dropped++;return;}
                long seq=++p.seq;
                bool stock=__originalMethod.Equals(p.stock);
                var b=p.Begin(stock?"stock_integrate_prefix":"mfi_integrate_prefix",seq);
                Field(b,"partFlightID",I(__0.flightID));
                Field(b,"rigidbodyMassKspTonnesRaw",__0.rb==null?"null":N(__0.rb.mass));
                // MFI wrapper is route evidence only. Never double-count its list
                // when it dispatches to stock Integrate.
                if(stock)
                {
                    string holders=Holders(__0,out int count);
                    Field(b,"partForceAccumulatorRaw",RawAccumulator(PartForce,__0));
                    Field(b,"partTorqueAccumulatorRaw",RawAccumulator(PartTorque,__0));
                    Field(b,"pendingHolderCount",count<0?"null":I(count));
                    Field(b,"pendingHolders",holders);
                    Field(b,"holdersTruncated",count<0?"null":B(count>MaxHolders));
                }
                if(p.Add(b.Append('}').ToString()))__state=seq;
            }
            catch(Exception){if(p.errors<int.MaxValue)p.errors++;}
        }

        private static void Postfix(MethodBase __originalMethod, Part __0, long __state, bool __runOriginal)
        {
            DunaM1ForceProbe p=active;
            if(p==null || __state==0)return;
            try
            {
                if(!p.CanObserve(__0))return;
                bool stock=__originalMethod.Equals(p.stock);
                var b=p.Begin(stock?"stock_integrate_postfix":"mfi_integrate_postfix",++p.seq);
                Field(b,"prefixSequence",I(__state));
                Field(b,"partFlightID",I(__0.flightID));
                Field(b,"harmonyRunOriginalFlag",B(__runOriginal));
                if(stock)Field(b,"holdersAfter",__0.forces==null?"null":I(__0.forces.Count));
                // A return flag does not prove each holder was applied, or that
                // PhysX has solved the step. Other patches may alter the list.
                p.Add(b.Append('}').ToString());
            }
            catch(Exception){if(p.errors<int.MaxValue)p.errors++;}
        }

        internal void ObserveState(Vessel v,VesselState s,string phase)
        {
            try
            {
                if(!enabled || v.id!=vessel || Thread.CurrentThread.ManagedThreadId!=thread)return;
                var b=Begin("drive_state",++seq);
                Field(b,"phase",Q(phase)); Field(b,"stateUT",N(s.Time));
                Field(b,"coMWorld",V(s.CoM)); Field(b,"positionBCIWorldAxes",V(s.OrbitalPosition));
                Field(b,"orbitalVelocityWorld",V(s.OrbitalVelocity));
                Field(b,"surfaceVelocityWorld",V(s.SurfaceVelocity));
                Field(b,"gravityWorld",V(s.GravityForce));
                Field(b,"massKspTonnesRaw",N(s.Mass));
                Field(b,"mainThrottleRaw",N(v.ctrlState.mainThrottle));
                Add(b.Append('}').ToString());
            }
            catch(Exception){if(errors<int.MaxValue)errors++;}
        }

        internal string Drain()
        {
            string json="{\"events\":"+ArrayJson(events)+",\"acceptedTotal\":"+I(accepted)+
                ",\"droppedTotal\":"+I(dropped)+",\"errorsTotal\":"+I(errors)+
                ",\"lastSequence\":"+I(seq)+",\"windowStopReason\":"+Q(WindowStopReason)+"}";
            events.Clear();bufferedChars=0;return json;
        }

        internal bool Stop()
        {
            enabled=false;
            if(ReferenceEquals(active,this))active=null;
            bool ok=true;
            foreach(MethodBase m in patched)
            {
                // Only remove our own method patches, never an entire target
                // or another owner's patches. Inert if cleanup cannot complete.
                try{unpatch.Invoke(harmony,new object[]{m,PrefixMethod});}catch(Exception){ok=false;}
                try{unpatch.Invoke(harmony,new object[]{m,PostfixMethod});}catch(Exception){ok=false;}
            }
            patched.Clear();
            return ok;
        }
    }
}
