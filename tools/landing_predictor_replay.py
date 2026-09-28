"""Offline airless V1 predictor replay from a passive capture.

This ports the gravity-only branch of ReentrySimulation's coast, BS34 step,
and virtual speed limiter. Terrain not present in the capture remains unknown;
the script never treats a target-radius crossing as ground contact.
"""

import argparse
import json
import math
from pathlib import Path

from landing_predictor_capture_reader import read_capture


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, factor):
    return tuple(x * factor for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def gravity(position, mu):
    return scale(position, -mu / norm(position) ** 3)


def absolute(position, ut, submission):
    unit = scale(position, 1 / norm(position))
    latitude = math.degrees(math.asin(max(-1, min(1, dot(unit, submission["bodyAxisNorth"])))))
    longitude = math.degrees(math.atan2(dot(unit, submission["bodyAxis90"]),
                                        dot(unit, submission["bodyAxis0"])))
    longitude -= 360 * (ut - submission["captureEpochUT"]) / submission["rotationPeriod"]
    return [latitude, (longitude + 180) % 360 - 180, norm(position), ut]


def unit_from_latlon(latitude, longitude, submission):
    lat, lon = math.radians(latitude), math.radians(longitude)
    return add(add(scale(submission["bodyAxis0"], math.cos(lat) * math.cos(lon)),
                   scale(submission["bodyAxis90"], math.cos(lat) * math.sin(lon))),
               scale(submission["bodyAxisNorth"], math.sin(lat)))


def ballistic_target_radius_crossing(submission):
    """Spherical crossing is a search horizon, never a claim of ground contact."""
    position = tuple(submission["positionBCI"])
    velocity = tuple(submission["velocityBCI"])
    ut = submission["inputUT"]
    target_radius = submission["bodyRadius"] + submission["targetTerrainASL"]
    start_radius = norm(position)
    if start_radius <= target_radius:
        raise ValueError("captured ballistic state already inside target radius")
    period = 2 * math.pi * math.sqrt(start_radius ** 3 / submission["bodyMu"])
    horizon = ut + submission["maxOrbits"] * period
    while ut < horizon:
        step = min(1.0, horizon - ut)
        next_position, next_velocity = rk4(position, velocity, step, submission["bodyMu"])
        if norm(next_position) <= target_radius:
            fraction = (norm(position) - target_radius) / (norm(position) - norm(next_position))
            crossing_ut = ut + fraction * step
            crossing_position, _ = rk4(position, velocity, fraction * step, submission["bodyMu"])
            return absolute(crossing_position, crossing_ut, submission)
        position, velocity, ut = next_position, next_velocity, ut + step
    raise ValueError("no ballistic target-radius crossing in captured horizon")


def approach_frame(submission, ballistic_crossing):
    target = unit_from_latlon(submission["targetLatitude"], submission["targetLongitude"], submission)
    ballistic = unit_from_latlon(ballistic_crossing[0], ballistic_crossing[1], submission)
    tangent = add(ballistic, scale(target, -dot(ballistic, target)))
    if norm(tangent) < 1e-9:
        raise ValueError("ballistic crossing cannot define downrange")
    downrange = scale(tangent, 1 / norm(tangent))
    crossrange = cross(target, downrange)
    return target, downrange, crossrange


def project_target_error(endpoint, submission, frame):
    target, downrange, crossrange = frame
    direction = unit_from_latlon(endpoint[0], endpoint[1], submission)
    radius = submission["bodyRadius"]
    return (radius * math.atan2(dot(direction, downrange), dot(direction, target)),
            radius * math.asin(max(-1, min(1, dot(direction, crossrange)))))


def rk4(position, velocity, dt, mu):
    a1 = gravity(position, mu)
    v2 = add(velocity, scale(a1, dt / 2))
    a2 = gravity(add(position, scale(velocity, dt / 2)), mu)
    v3 = add(velocity, scale(a2, dt / 2))
    a3 = gravity(add(position, scale(v2, dt / 2)), mu)
    v4 = add(velocity, scale(a3, dt))
    a4 = gravity(add(position, scale(v3, dt)), mu)
    next_position = add(position, scale(add(add(velocity, scale(v2, 2)),
                                         add(scale(v3, 2), v4)), dt / 6))
    next_velocity = add(velocity, scale(add(add(a1, scale(a2, 2)),
                                         add(scale(a3, 2), a4)), dt / 6))
    return next_position, next_velocity


def coast(submission, end_ut):
    position = tuple(submission["positionBCI"])
    velocity = tuple(submission["velocityBCI"])
    ut = submission["inputUT"]
    if end_ut < ut:
        raise ValueError("forced brake precedes snapshot")
    while ut < end_ut:
        step = min(1.0, end_ut - ut)
        if step < 1e-8:
            break
        position, velocity = rk4(position, velocity, step, submission["bodyMu"])
        ut += step
    return position, velocity


def bs34_step(position, velocity, dt, min_dt, start_position, submission):
    mu = submission["bodyMu"]
    minimum_radius = submission["bodyRadius"] + submission["probableLandingSiteASL"]
    for attempt in range(1, 101):
        a1 = gravity(position, mu)
        dx1, dv1 = scale(velocity, dt), scale(a1, dt)
        a2 = gravity(add(position, scale(dx1, 0.5)), mu)
        dx2, dv2 = scale(add(velocity, scale(dv1, 0.5)), dt), scale(a2, dt)
        a3 = gravity(add(position, scale(dx2, 0.75)), mu)
        dx3, dv3 = scale(add(velocity, scale(dv2, 0.75)), dt), scale(a3, dt)
        fourth_position = add(position, add(scale(dx1, 2 / 9),
                                            add(scale(dx2, 3 / 9), scale(dx3, 4 / 9))))
        fourth_velocity = add(velocity, add(scale(dv1, 2 / 9),
                                            add(scale(dv2, 3 / 9), scale(dv3, 4 / 9))))
        dv4 = scale(gravity(fourth_position, mu), dt)
        dx = add(scale(dx1, 2 / 9), add(scale(dx2, 3 / 9), scale(dx3, 4 / 9)))
        dv = add(scale(dv1, 2 / 9), add(scale(dv2, 3 / 9), scale(dv3, 4 / 9)))
        zv = add(scale(dv1, 7 / 24), add(scale(dv2, 6 / 24),
                                         add(scale(dv3, 8 / 24), scale(dv4, 3 / 24))))
        error = max(norm(add(zv, scale(dv, -1))), 1e-5)
        will_cross_radius = norm(add(position, dx)) - minimum_radius < 0
        next_dt = dt * (0.5 if will_cross_radius else 0.9 * (0.01 / error) ** (1 / 3))
        next_dt = max(min_dt, min(10.0, next_dt))
        distance_squared = dot(add(position, scale(start_position, -1)),
                               add(position, scale(start_position, -1)))
        if distance_squared < 1000 ** 2:
            next_dt = min(next_dt, 0.02)
        elif distance_squared < 5000 ** 2:
            next_dt = min(next_dt, 0.5)
        elif distance_squared < 10000 ** 2:
            next_dt = min(next_dt, 1.0)
        if (error > 0.01 or will_cross_radius) and dt > min_dt:
            dt = next_dt
            continue
        return add(position, dx), add(velocity, dv), dt, next_dt, attempt
    raise ValueError("BS34 step did not converge")


def replay_forced_candidate(submission, controller_terrain_asl=None):
    if submission["policy"] != "SafeDescentSpeedPolicy" or submission["kind"] != "forced_candidate":
        raise ValueError("replay currently supports forced airless SafeDescentSpeedPolicy cases")
    forced_ut = submission["forcedBrakingStartUT"]
    position, velocity = coast(submission, forced_ut)
    start_position = position
    start = absolute(position, forced_ut, submission)
    ut = forced_ut
    dt = submission["dt"]
    min_dt = submission["minDt"]
    stop_radius = submission["bodyRadius"] + submission["probableLandingSiteASL"]
    maximum_time = submission["maxOrbits"] * 2 * math.pi * math.sqrt(norm(position) ** 3 / submission["bodyMu"])
    steps = 0
    delta_v = 0.0
    controller_transition = None
    while norm(position) >= stop_radius and ut - forced_ut <= maximum_time and steps <= 50000:
        previous_position, previous_velocity, previous_ut = position, velocity, ut
        position, velocity, elapsed, dt, attempts = bs34_step(position, velocity, dt, min_dt,
                                                              start_position, submission)
        ut += elapsed
        steps += attempts
        surface_velocity = add(velocity, scale(cross(submission["angularVelocity"], position), -1))
        speed = norm(surface_velocity)
        altitude = norm(position) - submission["bodyRadius"] - submission["decelEndASL"]
        available_accel = submission["maxThrustAcceleration"] - submission["bodyGeeASL"] * 9.81
        limit_square = 2 * available_accel * altitude
        allowed = 0.9 * math.sqrt(limit_square) if limit_square >= 0 else math.nan
        if speed > allowed:
            change = min(speed - allowed, dt * submission["maxThrustAcceleration"])
            surface_velocity = scale(surface_velocity, 1 - change / speed)
            velocity = add(surface_velocity, cross(submission["angularVelocity"], position))
            delta_v += change
        transition_radius = (submission["bodyRadius"] + controller_terrain_asl + 205
                             if controller_terrain_asl is not None else None)
        if transition_radius is not None and controller_transition is None and \
                norm(previous_position) >= transition_radius > norm(position):
            fraction = ((norm(previous_position) - transition_radius) /
                        (norm(previous_position) - norm(position)))
            transition_position = add(previous_position,
                                      scale(add(position, scale(previous_position, -1)), fraction))
            transition_velocity = add(previous_velocity,
                                      scale(add(velocity, scale(previous_velocity, -1)), fraction))
            transition_surface_velocity = add(transition_velocity,
                                              scale(cross(submission["angularVelocity"], transition_position), -1))
            controller_transition = {
                "ut": previous_ut + fraction * elapsed,
                "clearance": norm(transition_position) - submission["bodyRadius"] - controller_terrain_asl,
                "radialSpeed": dot(transition_velocity, scale(transition_position, 1 / norm(transition_position))),
                "surfaceSpeed": norm(transition_surface_velocity),
            }
    if steps > 50000 or ut - forced_ut > maximum_time:
        raise ValueError("forced candidate did not reach stopping radius")
    return {"start": start, "end": absolute(position, ut, submission),
            "endSurfaceSpeed": norm(add(velocity, scale(cross(submission["angularVelocity"], position), -1))),
            "endRadialSpeed": dot(velocity, scale(position, 1 / norm(position))),
            "virtualDeltaV": delta_v, "steps": steps,
            "controllerTransition": controller_transition}


def refine_spherical_brake_time(cases, target_tolerance, maximum_iterations=24):
    """Refine signed downrange; terrain and terminal gates remain separate.

    The input is a common-snapshot candidate batch. An unqueried refined path
    is never labelled terrain-resolved or publishable by this function.
    """
    if not math.isfinite(target_tolerance) or target_tolerance <= 0:
        raise ValueError("target tolerance must be positive and finite")
    submission = cases[0]["submission"]
    ballistic = ballistic_target_radius_crossing(submission)
    frame = approach_frame(submission, ballistic)
    ordered = sorted(cases, key=lambda case: case["submission"]["forcedBrakingStartUT"])
    points = []
    for case in ordered:
        other = case["submission"]
        if any(other[field] != submission[field] for field in
               ("inputUT", "body", "targetLatitude", "targetLongitude", "targetTerrainASL")):
            raise ValueError("candidate batch mixes captured snapshots")
        if case["worker"]["outcome"] != "LANDED" or not case["worker"]["complete"]:
            points.append(None)
            continue
        down, across = project_target_error(case["worker"]["simulatorEnd"], submission, frame)
        points.append((other["forcedBrakingStartUT"], down, across))
    brackets = []
    for earlier, later in zip(points, points[1:]):
        if earlier and later and earlier[1] * later[1] <= 0 and earlier[0] < later[0]:
            brackets.append((earlier, later))
    if not brackets:
        return {"status": "no_signed_bracket", "ballisticTargetRadiusCrossing": ballistic}
    low, high = min(brackets, key=lambda pair: max(abs(pair[0][1]), abs(pair[1][1])))
    best = min((low, high), key=lambda point: abs(point[1]))
    maximum_observed_slope = abs((high[1] - low[1]) / (high[0] - low[0]))
    iterations = 0
    while iterations < maximum_iterations:
        timing_interval = high[0] - low[0]
        if max(abs(low[1]), abs(high[1])) < target_tolerance and \
                timing_interval * maximum_observed_slope < target_tolerance:
            break
        brake_ut = (low[0] + high[0]) / 2
        candidate = dict(submission, forcedBrakingStartUT=brake_ut)
        replay = replay_forced_candidate(candidate)
        down, across = project_target_error(replay["end"], submission, frame)
        point = (brake_ut, down, across)
        if abs(down) < abs(best[1]):
            best = point
        if low[1] * down <= 0:
            maximum_observed_slope = max(maximum_observed_slope,
                                         abs((high[1] - down) / (high[0] - brake_ut)))
            high = point
        else:
            maximum_observed_slope = max(maximum_observed_slope,
                                         abs((down - low[1]) / (brake_ut - low[0])))
            low = point
        iterations += 1
    timing_interval = high[0] - low[0]
    converged = max(abs(low[1]), abs(high[1])) < target_tolerance and \
        timing_interval * maximum_observed_slope < target_tolerance
    return {"status": "spherical_refined_terrain_unresolved" if converged else "not_converged",
            "ballisticTargetRadiusCrossing": ballistic,
            "brakeUT": best[0], "signedDownrangeError": best[1],
            "crossrangeError": best[2], "brakeTimeInterval": timing_interval,
            "timingDistanceEstimate": timing_interval * maximum_observed_slope,
            "bracketDownrangeErrors": [low[1], high[1]],
            "iterations": iterations, "targetTolerance": target_tolerance}


def sampled_terrain_clearance(resolved, worker):
    """Classify only recorded samples; no samples means unknown, not contact."""
    samples = resolved.get("terrainSamples") if resolved else None
    if not samples or len(samples) != resolved.get("terrainQueryCount"):
        return {"status": "terrain_unresolved"}
    clearances = [sample[2] - sample[3] for sample in samples]
    if not all(map(math.isfinite, clearances)):
        return {"status": "terrain_unresolved"}
    early_contact = any(clearance < 0 for clearance in clearances[:-1])
    if resolved.get("terrainContactConfirmed") and \
            resolved.get("terrainContactIndex", -1) < worker.get("trajectorySamples", 0) - 1:
        early_contact = True
    return {"status": "early_intersection" if early_contact else
            "sampled_clear_to_handoff" if clearances[-1] >= 0 else "handoff_below_terrain",
            "sampleCount": len(samples), "minimumSampledClearance": min(clearances),
            "handoffClearance": clearances[-1], "localTerrainASL": samples[-1][3]}


def compare_capture(capture, submission_ids=None):
    rows = []
    for case in capture["cases"]:
        submission, worker = case["submission"], case["worker"]
        if submission["kind"] != "forced_candidate" or (submission_ids and
                                                                case["submissionId"] not in submission_ids):
            continue
        if worker["recordType"] != "worker_result" or not worker["complete"]:
            continue
        replay = replay_forced_candidate(submission)
        captured_end = worker["simulatorEnd"]
        rows.append({"submissionId": case["submissionId"],
                     "forcedBrakeUT": submission["forcedBrakingStartUT"],
                     "capturedEnd": captured_end,
                     "replayedEnd": replay["end"],
                     "capturedEndSurfaceSpeed": worker["endSurfaceSpeed"],
                     "replayedEndSurfaceSpeed": replay["endSurfaceSpeed"],
                     "capturedSteps": worker["steps"], "replayedSteps": replay["steps"],
                     "terrain": sampled_terrain_clearance(case["resolved"], worker)})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--submission-id", type=int, action="append")
    args = parser.parse_args()
    capture = read_capture(args.capture)
    print(json.dumps(compare_capture(capture, set(args.submission_id or [])), indent=2))


if __name__ == "__main__":
    main()
