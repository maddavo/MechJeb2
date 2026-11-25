#!/usr/bin/env python3
"""Read-only M0 evidence extraction; standard library, no game/runtime access."""
import argparse
import json
import math
import re
from pathlib import Path

COMMIT_UT = 31155108.390804715
FIELDS = ("dunaProbeUT", "dunaAuthority", "dunaActualThrottleLimit",
          "dunaAppliedThrottle", "dunaProbeMinClearance", "dunaTerrainRadius",
          "dunaEntryResidual", "dunaEntryAngle", "dunaDragAccel")

def number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return float(value)
    return None

def vector(value):
    if isinstance(value, list) and len(value) == 3 and all(number(x) is not None for x in value):
        return tuple(map(float, value))
    return None

def dot(a, b):
    return sum(x*y for x, y in zip(a, b))

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def scale(a, k):
    return tuple(x*k for x in a)

def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))

def norm(a):
    return math.sqrt(dot(a, a))

def unit(a):
    return scale(a, 1/norm(a)) if a is not None and norm(a) > 0 else None

def angle(a, b):
    a, b = unit(a), unit(b)
    return math.degrees(math.atan2(norm(cross(a, b)), dot(a, b))) if a is not None and b is not None else None

def detail_number(detail, key):
    matches = re.findall(r"(?:^|\s)" + re.escape(key) + r"=(\S+)", detail or "")
    if len(matches) != 1:
        return None
    try:
        return number(float(matches[0]))
    except ValueError:
        return None

def solve3(a, b):
    """Small pivoted elimination; reject rank deficient rotation fits."""
    m = [list(row)+[value] for row, value in zip(a, b)]
    reference = max(abs(x) for row in a for x in row)
    if reference == 0:
        return None
    for i in range(3):
        j = max(range(i, 3), key=lambda j: abs(m[j][i]))
        if abs(m[j][i]) <= reference*1e-10:
            return None
        m[i], m[j] = m[j], m[i]
        pivot = m[i][i]
        m[i] = [x/pivot for x in m[i]]
        for j in range(3):
            if j != i:
                k = m[j][i]
                m[j] = [x-k*y for x, y in zip(m[j], m[i])]
    return tuple(row[3] for row in m)

def rotation_fit(samples, radius):
    """Fit omega cross r = v_orb-v_surface; inferred, not runtime metadata."""
    a = [[0.0]*3 for _ in range(3)]
    b = [0.0]*3
    pairs = []
    for s in samples:
        up, v, surf, h = vector(s.get("up")), vector(s.get("velocity")), vector(s.get("surfaceVelocity")), number(s.get("altitudeASL"))
        if up is None or v is None or surf is None or h is None or radius+h <= 0:
            continue
        u = unit(up)
        if u is None or abs(norm(up)-1) > 1e-6:
            continue
        r = scale(u, (radius+h)/radius)
        dv = scale(sub(v, surf), 1/radius)
        pairs.append((r, dv))
        c = cross(r, dv)
        for i in range(3):
            b[i] += c[i]
            for j in range(3):
                a[i][j] += (dot(r, r) if i == j else 0) - r[i]*r[j]
    omega = solve3(a, b) if pairs else None
    if omega is None:
        return {"samples": len(pairs), "status": "rank_deficient_or_missing"}
    residuals = [norm(sub(cross(omega, r), dv))*radius for r, dv in pairs]
    return {"samples": len(pairs), "status": "inferred_only", "angularVelocity_rad_s": omega,
            "rotationPeriod_s": 2*math.pi/norm(omega) if norm(omega) > 0 else None,
            "rmsResidual_m_s": math.sqrt(sum(x*x for x in residuals)/len(residuals)),
            "maxResidual_m_s": max(residuals)}

def state_metrics(s):
    d = s.get("controllerDetail") or ""
    f, v, u = vector(s.get("forward")), vector(s.get("surfaceVelocity")), vector(s.get("up"))
    out = {"ut": number(s.get("ut")), "phase": s.get("phase"),
           "altitudeASL_m": number(s.get("altitudeASL")), "mass_t": number(s.get("mass")),
           "retrogradeAngle_deg": angle(f, scale(v, -1)) if f is not None and v is not None else None,
           "radialVelocity_m_s": dot(unit(u), v) if unit(u) is not None and v is not None else None,
           "attitudeErrorDegrees": number(s.get("attitudeErrorDegrees")),
           "commandedThrottle": number(s.get("commandedThrottle")),
           "thrustAcceleration": number(s.get("thrustAcceleration")),
           "maxThrustAcceleration": number(s.get("maxThrustAcceleration")),
           "limitedMaxThrustAcceleration": number(s.get("limitedMaxThrustAcceleration"))}
    out.update({k: detail_number(d, k) for k in FIELDS})
    for key in ("maxThrustAcceleration", "limitedMaxThrustAcceleration", "dunaAuthority", "thrustAcceleration"):
        a, m = out[key], out["mass_t"]
        out[key+"_equivalentForward_kN"] = a*m if a is not None and m is not None else None
    return out

def stats(rows, keys):
    result = {}
    for key in keys:
        vals = [r[key] for r in rows if r.get(key) is not None]
        result[key] = {"n": len(vals), "min": min(vals), "max": max(vals), "mean": sum(vals)/len(vals)} if vals else {"n": 0}
    return result

def analyse(path, radius_override=None):
    states, count, last_line, transitions = [], 0, None, []
    for line in Path(path).open(encoding="utf-8-sig"):
        outer = json.loads(line)
        source = outer["sourceLine"]
        s = outer["record"]
        if last_line is not None and source != last_line+1:
            raise ValueError("Input is not a contiguous source-line extract")
        last_line = source
        count += 1
        if s.get("recordType") not in ("guidance_state", "guidance_transition"):
            continue
        if number(s.get("ut")) is None:
            continue
        if states and s["ut"] < states[-1][1]["ut"]:
            raise ValueError("State UT regressed: possible reload or mixed session")
        states.append((source, s))
        if s.get("recordType") == "guidance_transition":
            transitions.append({"sourceLine": source, **state_metrics(s)})
    matches = [(l, s) for l, s in states if s.get("recordType") == "guidance_transition"
               and abs(s["ut"]-COMMIT_UT) < 1e-6
               and s.get("phase") == "DecelerationBurn"
               and "dunaBrakeReason=delayed_probe_insufficient_hover_authority" in (s.get("controllerDetail") or "")]
    if len(matches) != 1:
        raise ValueError("Expected exactly one matching commitment transition")
    source, commit = matches[0]
    c = state_metrics(commit)
    if c["dunaProbeUT"] is None or abs(c["dunaProbeUT"]-commit["ut"]) > 1e-6:
        raise ValueError("Commit transition is not same-epoch probe evidence")
    inferred_radius = c["dunaTerrainRadius"] + c["dunaProbeMinClearance"] - c["altitudeASL_m"]
    radius = inferred_radius if radius_override is None else radius_override
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("Radius must be positive")
    radial = radius + c["altitudeASL_m"]
    g = number(commit.get("localGravity"))
    before = []
    after = []
    first_thrust = None
    previous = None
    for l, s in states:
        x = state_metrics(s)
        ut = x["ut"]
        x["sourceLine"] = l
        x["forwardAxisSlew_deg_s"] = None
        if previous is not None:
            p, t = previous
            dt = ut-t
            f0, f1 = vector(p.get("forward")), vector(s.get("forward"))
            if 0 < dt <= 2 and p.get("warpRate") == 1 and s.get("warpRate") == 1 and f0 is not None and f1 is not None:
                theta = angle(f0, f1)
                x["forwardAxisSlew_deg_s"] = theta/dt if theta is not None else None
        # A same-UT transition can precede a guidance_state. Keep the first for
        # the next finite difference, do not divide by zero.
        if previous is None or ut > previous[1]:
            previous = (s, ut)
        if COMMIT_UT-30 <= ut < COMMIT_UT:
            before.append(x)
        if COMMIT_UT <= ut <= COMMIT_UT+60:
            after.append(x)
        if ut >= COMMIT_UT and first_thrust is None and x["thrustAcceleration"] is not None and x["thrustAcceleration"] > 0:
            first_thrust = x
    keys = ("retrogradeAngle_deg", "forwardAxisSlew_deg_s", "attitudeErrorDegrees",
            "maxThrustAcceleration_equivalentForward_kN", "limitedMaxThrustAcceleration_equivalentForward_kN",
            "dunaAuthority_equivalentForward_kN", "dunaActualThrottleLimit", "dunaAppliedThrottle",
            "thrustAcceleration_equivalentForward_kN", "mass_t")
    # dunaAuthority is last stored probe authority, not necessarily same-tick
    # engine state. Summarize it only on same-epoch records.
    def same_epoch(rows):
        return [x for x in rows if x["dunaProbeUT"] is not None and abs(x["dunaProbeUT"]-x["ut"]) <= 1e-6]
    samples = [s for _, s in states]
    return {"schemaVersion": 1, "records": count, "stateRecords": len(states),
            "commit": {"sourceLine": source, **c},
            "bodyInference": {"radiusFromClearanceAndASL_m": inferred_radius, "radiusUsed_m": radius,
                              "radiusOrigin": "trace_inferred" if radius_override is None else "caller_supplied",
                              "muFromLocalGravity_m3_s2": g*radial**2 if g is not None else None,
                              "rotationFitAllStates": rotation_fit(samples, radius),
                              "rotationFitNearCommit": rotation_fit([s for s in samples if abs(s["ut"]-COMMIT_UT) <= 60], radius)},
            "before30s": stats(before, tuple(k for k in keys if not k.startswith("dunaAuthority"))),
            "after60s": stats(after, tuple(k for k in keys if not k.startswith("dunaAuthority"))),
            "sameEpochProbeBefore": stats(same_epoch(before), ("dunaAuthority_equivalentForward_kN",)),
            "sameEpochProbeAfter": stats(same_epoch(after), ("dunaAuthority_equivalentForward_kN",)),
            "firstObservedPositiveForwardThrustAfterCommit": first_thrust,
            "transitions": transitions, "lastState": state_metrics(states[-1][1]),
            "limitations": ["Forward-axis slew is sampled pointing change, not yaw/roll or full angular velocity.",
                            "No per-engine allocator state in legacy trace; no causal attribution.",
                            "Rotation/gravity fits are trace-derived, not independent body metadata.",
                            "No landing outcome inferred from final state."]}

def self_test():
    w = (1e-5, -9e-5, 2e-5)
    radius = 320000.0
    rows = []
    for axis in ([1,0,0], [0,1,0], [0,0,1]):
        r = scale(axis, radius+50000)
        v = (10, 20, 30)
        rows.append({"up": axis, "altitudeASL": 50000, "velocity": list(v),
                     "surfaceVelocity": list(sub(v, cross(w, r)))})
    fit = rotation_fit(rows, radius)
    assert norm(sub(fit["angularVelocity_rad_s"], w)) < 1e-12
    assert fit["maxResidual_m_s"] < 1e-8
    assert rotation_fit(rows[:1], radius)["status"] == "rank_deficient_or_missing"
    assert abs(angle((1,0,0), (0,1,0))-90) < 1e-12
    assert detail_number(" dunaAuthority=3.1", "dunaAuthority") == 3.1
    assert detail_number(" dunaAuthority=NaN", "dunaAuthority") is None
    print("Synthetic math self-tests passed")

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("flight", nargs="?")
    p.add_argument("--radius-m", type=float)
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    if args.self_test:
        self_test()
    elif args.flight:
        print(json.dumps(analyse(args.flight, args.radius_m), indent=2, allow_nan=False))
    else:
        p.error("supply duna-flight.jsonl or --self-test")

if __name__ == "__main__":
    main()
