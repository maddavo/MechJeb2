#!/usr/bin/env python3
"""Offline integrated-landing M0 evidence harness; no game or save mutations.

Legacy trace has no vessel/session ID. Explicit source-line range is required
for analysis. M1 GUID capture has identity and vector forces. Neither
predictor LANDED nor termination of a trace establishes actual touchdown.
"""
import argparse
import collections
import hashlib
import json
import math
import pathlib
import tempfile
import unittest

STATE = ("guidance_state", "guidance_transition")
START_PHASES = ("PlaneChange", "DeorbitBurn", "LowDeorbitBurn")
EPS = 1e-7

def finite(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)

def vec(x):
    return isinstance(x, (list, tuple)) and len(x) == 3 and all(finite(a) for a in x)

def add(a,b): return tuple(x+y for x,y in zip(a,b))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def mul(a,k): return tuple(x*k for x in a)
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(a): return math.sqrt(dot(a,a))
def unit(a): return mul(a,1/norm(a)) if norm(a)>EPS else None

def records(path, first=1, last=None):
    """Read in physical file order; a malformed selected row is an error."""
    with pathlib.Path(path).open(encoding="utf-8-sig") as f:
        for line, raw in enumerate(f,1):
            if line < first: continue
            if last is not None and line > last: break
            try:
                data=json.loads(raw)
            except ValueError as e:
                raise ValueError(f"Malformed JSON at source line {line}") from e
            if not isinstance(data,dict):
                raise ValueError(f"Invalid record at source line {line}")
            yield line,data

def fingerprint(path):
    s=pathlib.Path(path).stat()
    return (s.st_size,s.st_mtime_ns,s.st_ino)

def unchanged(path,before):
    if fingerprint(path)!=before:
        raise ValueError("Source changed during analysis; use a stable copy")

def file_digest(path):
    h=hashlib.sha256()
    with pathlib.Path(path).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""): h.update(b)
    return h.hexdigest()

def target(r):
    if isinstance(r.get("body"),str) and finite(r.get("targetLat")) and finite(r.get("targetLon")):
        return (r["body"],float(r["targetLat"]),float(r["targetLon"]))
    return None

def epoch(r):
    # Do not use prediction input/end UT as a live clock.
    return r.get("processUT") if r.get("recordType")=="predictor_result" else r.get("ut")

def catalogue(path):
    """Candidate legacy windows, NOT authenticated sessions. Use file order.

    State UT regression/new approach after terminal flight and target changes
    are evidence of boundaries. Old/new runs with identical target and UT can
    still be indistinguishable; operator must confirm log and save lineage.
    """
    before=fingerprint(path)
    windows=[]
    window=None
    last_state_ut=None
    phase=None
    def finish():
        if window is not None:
            windows.append(window)
    for line,r in records(path):
        kind=r.get("recordType")
        if kind not in STATE and kind!="predictor_result": continue
        t=epoch(r)
        if not finite(t): continue
        q=target(r)
        reason=None
        if window is None: reason="file_start"
        elif kind in STATE and last_state_ut is not None and t<last_state_ut-EPS:
            reason="state_ut_regression"
        elif (kind=="guidance_transition" and r.get("phase") in START_PHASES
              and phase not in (None,*START_PHASES)):
            reason="new_approach_transition"
        elif q and window["target"] and q!=tuple(window["target"]):
            reason="predictor_target_change"
        if reason:
            finish()
            window={"first":line,"last":line,"boundary":reason,"firstUT":t,
                    "lastUT":t,"target":None,"states":0,"predictions":0,
                    "phases":[],"firstStateUT":None,"lastStateUT":None}
            last_state_ut=None
            phase=None
        window["last"]=line
        window["lastUT"]=t
        if q: window["target"]=q
        if kind in STATE:
            window["states"]+=1
            if window["firstStateUT"] is None: window["firstStateUT"]=t
            window["lastStateUT"]=t
            last_state_ut=t
            phase=r.get("phase")
            if not window["phases"] or phase!=window["phases"][-1]:
                window["phases"].append(phase)
        else: window["predictions"]+=1
    finish()
    unchanged(path,before)
    return {"source":str(path),"identity":"candidate_windows_not_sessions","windows":windows}

def interval_ok(a,b,max_gap):
    dt=b["ut"]-a["ut"]
    return (0<dt<=max_gap and a.get("warpRate")==1 and b.get("warpRate")==1
            and a.get("phase")==b.get("phase"))

def actual_impulse(samples,max_gap,mode):
    """Integrate sampled delivered acceleration, NOT indicated remaining dV.
    Legacy scalar is forward projection only. M1 full thrust vector kN/t.
    Do not integrate across gaps, warp, duplicate epochs or phase changes.
    Report valid time coverage so partial sums cannot masquerade as totals.
    """
    out=collections.defaultdict(lambda:{"secondsIntegrated":0.0,"secondsPositive":0.0,
        "scalarDv_m_s":0.0,"alongSurfaceDv_m_s":0.0,"validIntervals":0,
        "skippedIntervals":0,"secondsSkipped":0.0})
    def acc(r):
        if mode=="m1":
            if not (vec(r.get("thrustVectorNEU")) and finite(r.get("mass")) and r["mass"]>0
                    and vec(r.get("surfaceVelocityNEU"))): return None
            a=mul(r["thrustVectorNEU"],1/r["mass"])
            u=unit(r["surfaceVelocityNEU"])
            return norm(a),dot(a,u) if u is not None else 0.0
        if not (finite(r.get("thrustAcceleration")) and vec(r.get("forward"))
                and vec(r.get("surfaceVelocity"))): return None
        u=unit(r["surfaceVelocity"]); f=unit(r["forward"])
        if f is None: return None
        x=max(0.0,r["thrustAcceleration"])
        return x,x*dot(f,u) if u is not None else 0.0
    for a,b in zip(samples,samples[1:]):
        d=out[str(a.get("phase"))]
        dt=b["ut"]-a["ut"]
        if dt<=0: continue
        aa,bb=acc(a),acc(b)
        if not interval_ok(a,b,max_gap) or aa is None or bb is None:
            d["skippedIntervals"]+=1; d["secondsSkipped"]+=dt; continue
        d["validIntervals"]+=1
        d["secondsIntegrated"]+=dt
        if aa[0]>EPS or bb[0]>EPS: d["secondsPositive"]+=dt
        d["scalarDv_m_s"]+=(aa[0]+bb[0])*dt/2
        d["alongSurfaceDv_m_s"]+=(aa[1]+bb[1])*dt/2
    return dict(out)

def select_legacy(path,first,last,expected,max_gap):
    if first<1 or last<first: raise ValueError("Invalid source line range")
    before=fingerprint(path)
    selected=list(records(path,first,last))
    if len(selected)!=last-first+1: raise ValueError("Range extends past EOF")
    states=[]; predictions=[]; transitions=[]
    last_ut=None
    previous_phase=None
    for l,r in selected:
        k=r.get("recordType")
        if k in STATE:
            t=r.get("ut")
            if not finite(t): raise ValueError(f"Missing state UT at line {l}")
            if last_ut is not None and t<last_ut-EPS: raise ValueError("Mixed run: state UT regressed")
            last_ut=t
            if k=="guidance_transition":
                if r.get("phase") in START_PHASES and previous_phase not in (None,*START_PHASES):
                    raise ValueError("Selected range contains a new approach transition")
                transitions.append({"sourceLine":l,"record":r})
            previous_phase=r.get("phase")
            # Prefer actual guidance_state over a same-epoch transition.
            if states and t==states[-1]["ut"]: states[-1]=r
            else: states.append(r)
        elif k=="predictor_result":
            q=target(r)
            if q!=expected:
                raise ValueError(f"Wrong or missing predictor target at line {l}: {q!r}, expected {expected!r}")
            predictions.append((l,r))
    if not states or not predictions: raise ValueError("Require guidance AND target-matched predictor evidence")
    last_prediction=predictions[-1]
    report={"identity":"operator_selected_legacy_range_not_authenticated",
        "first":first,"last":last,"target":expected,"records":len(selected),
        "guidanceSamples":len(states),"predictorRows":len(predictions),
        "firstState":states[0],"lastState":states[-1],"transitions":transitions,
        "lastPrediction":{"sourceLine":last_prediction[0],"record":last_prediction[1]},
        "impulseMethod":"forward acceleration projection proxy; not actual thrust vector",
        "phaseImpulse":actual_impulse(states,max_gap,"legacy"),
        "actualTouchdown":"unobserved_in_legacy_schema",
        "sourceSHA256":file_digest(path)}
    unchanged(path,before)
    return report,selected

def load_m1(path):
    rows=list(records(path))
    if not rows: raise ValueError("Empty capture")
    header=rows[0][1]
    if header.get("recordType")!="session_start" or header.get("schemaVersion")!=1:
        raise ValueError("Expected M1 schema 1 session_start")
    identity=tuple(header.get(k) for k in ("session","vessel","body","targetLat","targetLon"))
    if not all(isinstance(s,str) and s for s in identity[:3]) or target(header) is None:
        raise ValueError("Missing M1 identity")
    samples=[]; terminal=None; ended=False
    for l,r in rows:
        if r.get("schemaVersion")!=1 or tuple(r.get(k) for k in ("session","vessel","body","targetLat","targetLon"))!=identity:
            raise ValueError(f"Mixed or unsupported session at {l}")
        if ended: raise ValueError("Rows after session_end")
        if l!=rows[0][0] and r.get("recordType")=="session_start":
            raise ValueError("Repeated session_start")
        if r.get("recordType")=="session_end":
            terminal=r.get("reason"); ended=True
        if r.get("recordType")=="live":
            t=r.get("ut")
            if not finite(t) or (samples and t<samples[-1]["ut"]-EPS): raise ValueError("Invalid live UT lineage")
            if samples and t==samples[-1]["ut"]: samples[-1]=r
            else: samples.append(r)
    if not samples: raise ValueError("No M1 live samples")
    return header,samples,terminal

def inertial(s,t0,radius,omega,mu):
    """Abstract RH axes: equator lon0 X, east Y, north Z.
    Signed omega about +Z must be independently verified for this convention.
    Uses vessel latitude/longitude as CoM proxy, not exact CoM vector.
    Measured thrust kN/t, drag/lift m/s²; actual force closure, not prediction.
    """
    if not all(finite(s.get(k)) for k in ("ut","latitude","longitude","altitudeASL","mass")):
        return None
    if s["mass"]<=0 or radius+s["altitudeASL"]<=0 or abs(s["latitude"])>90: return None
    if not all(vec(s.get(k)) for k in ("surfaceVelocityNEU","thrustVectorNEU","dragAccelNEU","liftAccelNEU")):
        return None
    p=math.radians(s["latitude"])
    l=math.radians(s["longitude"])+omega*(s["ut"]-t0)
    up=(math.cos(p)*math.cos(l),math.cos(p)*math.sin(l),math.sin(p))
    north=(-math.sin(p)*math.cos(l),-math.sin(p)*math.sin(l),math.cos(p))
    east=(-math.sin(l),math.cos(l),0)
    def convert(v): return add(add(mul(north,v[0]),mul(east,v[1])),mul(up,v[2]))
    r=mul(up,radius+s["altitudeASL"])
    w=convert(s["surfaceVelocityNEU"])
    v=add(w,(-omega*r[1],omega*r[0],0))
    a=add(add(mul(up,-mu/norm(r)**2),convert(mul(s["thrustVectorNEU"],1/s["mass"]))),
          convert(add(s["dragAccelNEU"],s["liftAccelNEU"])))
    return r,v,a

def force_closure(samples,radius,omega,mu,max_gap):
    """Trapezoidal integration over *observed* force histories. Per-step
    residuals; do not reset missing data to zero or bridge unsupported gaps.
    No inferred controller parity, terrain/contact or authority validation.
    """
    d=collections.defaultdict(lambda:{"intervals":0,"skipped":0,"seconds":0.0,
        "velocityResidualSquared":0.0,"positionResidualSquared":0.0,
        "maxVelocityResidual_m_s":0.0,"maxPositionResidual_m":0.0})
    if not all(finite(x) for x in (radius,omega,mu,max_gap)) or radius<=0 or mu<0 or max_gap<=0:
        raise ValueError("Invalid independently supplied body or interval parameters")
    t0=samples[0]["ut"]
    for a,b in zip(samples,samples[1:]):
        p=d[str(a.get("phase"))]
        x,y=inertial(a,t0,radius,omega,mu),inertial(b,t0,radius,omega,mu)
        if not interval_ok(a,b,max_gap) or x is None or y is None:
            p["skipped"]+=1; continue
        dt=b["ut"]-a["ut"]
        r,v,f=x; r1,v1,f1=y
        ev=norm(sub(v1,add(v,mul(add(f,f1),dt/2))))
        er=norm(sub(r1,add(r,mul(add(v,v1),dt/2))))
        p["intervals"]+=1;p["seconds"]+=dt
        p["velocityResidualSquared"]+=ev*ev;p["positionResidualSquared"]+=er*er
        p["maxVelocityResidual_m_s"]=max(p["maxVelocityResidual_m_s"],ev)
        p["maxPositionResidual_m"]=max(p["maxPositionResidual_m"],er)
    for p in d.values():
        n=p["intervals"]
        v2=p.pop("velocityResidualSquared");r2=p.pop("positionResidualSquared")
        p["rmsVelocityResidual_m_s"]=math.sqrt(v2/n) if n else None
        p["rmsPositionResidual_m"]=math.sqrt(r2/n) if n else None
        if not n: p["maxVelocityResidual_m_s"]=p["maxPositionResidual_m"]=None
    return dict(d)

def measurement_inventory(samples,max_gap):
    """Describe actual M1 fields and derived observations; not a fitted model.

    NEU forces in kN (thrust) and m/s² (drag/lift), mass tonnes.
    Dynamic pressure recomputed from observed density and surface speed.
    Effective area uses the observed force projection, not assumed Cd.
    Mass differences include all mass changes, not just engine flow.
    """
    scalar=("latitude","longitude","altitudeASL","altitudeBottom","mass",
        "density","pressureKPa","mach","dynamicPressurePa","commandedThrottle",
        "appliedThrottle","thrustAcceleration","limitedMaxThrustAcceleration",
        "engineResponseSeconds","maximumMassFlow","attitudeErrorDegrees","physicsDt")
    vectors=("surfaceVelocityNEU","forwardNEU","thrustVectorNEU","dragAccelNEU","liftAccelNEU")
    by_phase=collections.defaultdict(list)
    for s in samples:by_phase[str(s.get("phase"))].append(s)
    out={}
    def stats(values):
        v=[x for x in values if finite(x)]
        return {"count":len(v),"min":min(v) if v else None,"max":max(v) if v else None,
            "mean":sum(v)/len(v) if v else None}
    for phase,rows in by_phase.items():
        coverage={k:sum(finite(s.get(k)) for s in rows) for k in scalar}
        coverage.update({k:sum(vec(s.get(k)) for s in rows) for k in vectors})
        area=[];drag_along=[];lift_along=[];thrust_error=[];q_error=[];powered=0
        for s in rows:
            m=s.get("mass");w=s.get("surfaceVelocityNEU");rho=s.get("density")
            f=s.get("forwardNEU");thrust=s.get("thrustVectorNEU")
            d=s.get("dragAccelNEU");lift=s.get("liftAccelNEU")
            if finite(m) and m>0 and vec(thrust) and vec(f) and unit(f) is not None:
                if norm(thrust)>EPS:powered+=1
                if finite(s.get("thrustAcceleration")):
                    thrust_error.append(dot(thrust,unit(f))/m-s["thrustAcceleration"])
            if not vec(w) or unit(w) is None:continue
            u=unit(w)
            if vec(d):drag_along.append(dot(d,u))
            if vec(lift):lift_along.append(dot(lift,u))
            if finite(rho) and rho>0:
                q=.5*rho*dot(w,w)
                if finite(s.get("dynamicPressurePa")):q_error.append(q-s["dynamicPressurePa"])
                if q>1 and finite(m) and m>0 and vec(d):
                    area.append(-dot(d,u)*m*1000/q)
        rates=[]
        for a,b in zip(rows,rows[1:]):
            if interval_ok(a,b,max_gap) and finite(a.get("mass")) and finite(b.get("mass")):
                rates.append((a["mass"]-b["mass"])*1000/(b["ut"]-a["ut"]))
        out[phase]={"rows":len(rows),"coverage":coverage,"positiveMeasuredThrustRows":powered,
            "effectiveDragArea_m2":stats(area),"dragAlongAir_m_s2":stats(drag_along),
            "liftAlongAir_m_s2":stats(lift_along),"thrustProjectionResidual_m_s2":stats(thrust_error),
            "dynamicPressureResidual_Pa":stats(q_error),"observedTotalMassLossRate_kg_s":stats(rates)}
    return {"byPhase":out,"notRecordedByM1Schema":[
        "full vessel attitude including roll and angular-rate vector",
        "desired inertial thrust direction at each controller tick",
        "per-engine attained throttle/force/flow and allocation at each epoch",
        "actual engine-only mass flow and resource/crossfeed state",
        "terrain support normal, swept corridor and attained-orientation collider offset",
        "terminal PID previous error, integral, previous requested speed throttle and applied limiter state"],
        "limitations":["Effective drag area is a measured projection at sampled conditions, not a validated aerodynamic model.",
            "Total mass-loss rate is not engine mass flow; staging/RCS/other resource changes can contribute.",
            "Do not assume fields absent from the schema are zero. Other matching local evidence may supply them."]}

def m1_report(path,radius,omega,mu,max_gap):
    before=fingerprint(path)
    header,samples,terminal=load_m1(path)
    report={"identity":"guid_identified_capture", "session":header,
        "sourceSHA256":file_digest(path),"samples":len(samples),
        "firstLive":samples[0],"lastLive":samples[-1],"sessionEndReason":terminal,
        "actualTouchdown":"observed_landed_or_splashed_state" if any(s.get("situation") in ("LANDED","SPLASHED") for s in samples) else "not_observed",
        "phaseImpulse":actual_impulse(samples,max_gap,"m1"),
        "bodyParameters":{"radius_m":radius,"signedOmega_rad_s":omega,"mu_m3_s2":mu,"origin":"caller_supplied"},
        "forceClosure":force_closure(samples,radius,omega,mu,max_gap),
        "measurementInventory":measurement_inventory(samples,max_gap),
        "limitations":["Measured force closure only; NOT independent actuator/aerodynamic prediction parity.",
            "Verify signed rotation convention and CoM/NEU basis against matching runtime state.",
            "Per-step residuals at recorded cadence; no interpolation across warp/gaps.",
            "Actual landed state distinct from predictor outcome; no touchdown-speed certification."]}
    unchanged(path,before)
    return report

class Tests(unittest.TestCase):
    def write(self,rows):
        f=tempfile.NamedTemporaryFile(mode="w",encoding="utf-8",delete=False)
        p=pathlib.Path(f.name)
        self.addCleanup(p.unlink)
        with f:
            for r in rows: f.write(json.dumps(r)+"\n")
        return p
    def legacy_state(self,t,phase="CourseCorrection"):
        return {"recordType":"guidance_state","ut":t,"phase":phase,"warpRate":1,
                "thrustAcceleration":2,"forward":[1,0,0],"surfaceVelocity":[1,0,0]}
    def test_reloaded_epoch_is_distinct_window(self):
        p=self.write([self.legacy_state(10),self.legacy_state(11),self.legacy_state(10)])
        a=catalogue(p)["windows"]
        self.assertEqual(len(a),2);self.assertEqual(a[1]["boundary"],"state_ut_regression")
    def test_target_mismatch_rejected(self):
        p=self.write([self.legacy_state(10),{"recordType":"predictor_result","processUT":10,
            "body":"Duna","targetLat":1,"targetLon":2}])
        with self.assertRaises(ValueError): select_legacy(p,1,2,("Duna",1,3),1.25)
    def test_partial_impulse_does_not_bridge_gap(self):
        rows=[self.legacy_state(t) for t in (0,1,4,5)]
        rows[-1]["forward"]=[-1,0,0]
        a=actual_impulse(rows,1.25,"legacy")["CourseCorrection"]
        self.assertEqual(a["scalarDv_m_s"],4);self.assertEqual(a["alongSurfaceDv_m_s"],2)
        self.assertEqual(a["secondsSkipped"],3)
    def test_actual_landed_not_predictor_landed(self):
        h={"recordType":"session_start","schemaVersion":1,"session":"s","vessel":"v","body":"Duna","targetLat":1,"targetLon":2}
        a=dict(h,recordType="live",ut=1,situation="FLYING",phase="FinalDescent")
        p=self.write([h,a,dict(h,recordType="prediction_result",outcome="LANDED")])
        self.assertEqual(m1_report(p,320000,.0001,3e11,1.25)["actualTouchdown"],"not_observed")
    def test_mixed_m1_identity_rejected(self):
        h={"recordType":"session_start","schemaVersion":1,"session":"s","vessel":"v","body":"Duna","targetLat":1,"targetLon":2}
        p=self.write([h,dict(h,recordType="live",ut=1,session="other")])
        with self.assertRaises(ValueError): load_m1(p)
    def test_force_closure_constant_radial_acceleration(self):
        # Synthetic nonrotating zero gravity, 2 kN / 1 tonne = 2 m/s².
        def row(t):
            return {"ut":t,"phase":"DecelerationBurn","warpRate":1,"latitude":0,"longitude":0,
                "altitudeASL":1000+t*t,"mass":1,"surfaceVelocityNEU":[0,0,2*t],
                "thrustVectorNEU":[0,0,2],"dragAccelNEU":[0,0,0],"liftAccelNEU":[0,0,0]}
        a=force_closure([row(0),row(1),row(2)],320000,0,0,1.25)["DecelerationBurn"]
        self.assertEqual(a["intervals"],2);self.assertLess(a["maxVelocityResidual_m_s"],1e-10)
        self.assertLess(a["maxPositionResidual_m"],1e-10)
    def test_rotation_and_neu_convention(self):
        s={"ut":0,"latitude":0,"longitude":0,"altitudeASL":1000,"mass":2,
           "surfaceVelocityNEU":[1,2,3],"thrustVectorNEU":[2,4,6],
           "dragAccelNEU":[0,-1,0],"liftAccelNEU":[1,0,0]}
        r,v,a=inertial(s,0,320000,.0001,0)
        self.assertEqual(r,(321000,0,0))
        self.assertAlmostEqual(v[1],34.1);self.assertEqual((v[0],v[2]),(3,1))
        self.assertEqual(a,(3,1,2))
        s["dragAccelNEU"]=None
        self.assertIsNone(inertial(s,0,320000,.0001,0))
    def test_same_target_new_approach_rejected(self):
        a=self.legacy_state(1,"FinalDescent")
        b=dict(self.legacy_state(2,"DeorbitBurn"),recordType="guidance_transition")
        p=self.write([a,b,{"recordType":"predictor_result","processUT":2,
            "body":"Duna","targetLat":1,"targetLon":2}])
        self.assertEqual(catalogue(p)["windows"][1]["boundary"],"new_approach_transition")
        with self.assertRaises(ValueError): select_legacy(p,1,3,("Duna",1,2),1.25)
    def test_m1_rows_after_end_rejected(self):
        h={"recordType":"session_start","schemaVersion":1,"session":"s","vessel":"v","body":"Duna","targetLat":1,"targetLon":2}
        p=self.write([h,dict(h,recordType="session_end",reason=None),dict(h,recordType="live",ut=1)])
        with self.assertRaises(ValueError): load_m1(p)
    def test_measurement_inventory_units_and_missing_fields(self):
        a={"ut":1,"phase":"DecelerationBurn","warpRate":1,"mass":2,
           "surfaceVelocityNEU":[0,10,0],"forwardNEU":[0,-1,0],"density":1,
           "thrustVectorNEU":[0,-8,0],"thrustAcceleration":4,"dragAccelNEU":[0,-.5,0],
           "liftAccelNEU":[1,0,0],"dynamicPressurePa":50}
        b=dict(a,ut=2,mass=1.999)
        report=measurement_inventory([a,b],1.25)
        p=report["byPhase"]["DecelerationBurn"]
        self.assertEqual(p["coverage"]["pressureKPa"],0)
        self.assertEqual(p["positiveMeasuredThrustRows"],2)
        self.assertAlmostEqual(p["effectiveDragArea_m2"]["max"],20)
        self.assertEqual(p["liftAlongAir_m_s2"]["max"],0)
        self.assertAlmostEqual(p["thrustProjectionResidual_m_s2"]["min"],0)
        self.assertAlmostEqual(p["observedTotalMassLossRate_kg_s"]["mean"],1)
        self.assertTrue(report["notRecordedByM1Schema"])
    def test_measurement_inventory_does_not_invent_air_or_bridge_gap(self):
        a={"ut":1,"phase":"CourseCorrection","warpRate":1,"mass":2}
        b=dict(a,ut=100,mass=1)
        p=measurement_inventory([a,b],1.25)["byPhase"]["CourseCorrection"]
        self.assertEqual(p["effectiveDragArea_m2"]["count"],0)
        self.assertIsNone(p["observedTotalMassLossRate_kg_s"]["mean"])
        self.assertEqual(p["coverage"]["thrustVectorNEU"],0)
    def test_legacy_no_contact_claim(self):
        p=self.write([self.legacy_state(1),{"recordType":"predictor_result","processUT":1,
            "body":"Duna","targetLat":1,"targetLon":2,"outcome":"LANDED"}])
        a,_=select_legacy(p,1,2,("Duna",1,2),1.25)
        self.assertEqual(a["actualTouchdown"],"unobserved_in_legacy_schema")

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest="cmd",required=True)
    cat=sub.add_parser("catalog");cat.add_argument("trace")
    ext=sub.add_parser("legacy");ext.add_argument("trace")
    ext.add_argument("--first",type=int,required=True);ext.add_argument("--last",type=int,required=True)
    ext.add_argument("--body",required=True);ext.add_argument("--lat",type=float,required=True);ext.add_argument("--lon",type=float,required=True)
    ext.add_argument("--extract",help="new file for selected raw wrapped rows; never overwrites")
    ext.add_argument("--max-gap",type=float,default=1.25)
    m1=sub.add_parser("m1");m1.add_argument("capture");m1.add_argument("--radius",type=float,required=True)
    m1.add_argument("--omega",type=float,required=True,help="verified signed rate about +north axis, rad/s")
    m1.add_argument("--mu",type=float,required=True);m1.add_argument("--max-gap",type=float,default=1.25)
    sub.add_parser("self-test")
    a=p.parse_args()
    if a.cmd=="self-test":
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
        return 0 if result.wasSuccessful() else 1
    if hasattr(a,"max_gap") and (not finite(a.max_gap) or a.max_gap<=0): p.error("max-gap must be finite and positive")
    if a.cmd=="catalog": result=catalogue(a.trace)
    elif a.cmd=="legacy":
        result,rows=select_legacy(a.trace,a.first,a.last,(a.body,a.lat,a.lon),a.max_gap)
        if a.extract:
            with pathlib.Path(a.extract).open("x",encoding="utf-8") as f:
                for l,r in rows: f.write(json.dumps({"sourceLine":l,"record":r},allow_nan=False)+"\n")
    else: result=m1_report(a.capture,a.radius,a.omega,a.mu,a.max_gap)
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0
if __name__=="__main__":
    raise SystemExit(main())
