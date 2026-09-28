"""Independent airless propagation of V1's brake trigger and throttle law.

Terrain is supplied as a constant for numerical parity; flight clearance is
validated by KSP PQS in the production transaction.
"""

import math

from landing_predictor_replay import absolute, add, cross, dot, gravity, norm, rk4, scale


def _unit(vector):
    return scale(vector, 1 / norm(vector))


def _rk4_thrust(position, velocity, step, mu, thrust):
    a1 = add(gravity(position, mu), thrust)
    v2 = add(velocity, scale(a1, step / 2))
    a2 = add(gravity(add(position, scale(velocity, step / 2)), mu), thrust)
    v3 = add(velocity, scale(a2, step / 2))
    a3 = add(gravity(add(position, scale(v2, step / 2)), mu), thrust)
    v4 = add(velocity, scale(a3, step))
    a4 = add(gravity(add(position, scale(v3, step)), mu), thrust)
    new_position = add(position, scale(add(add(velocity, scale(v2, 2)),
                                       add(scale(v3, 2), v4)), step / 6))
    new_velocity = add(velocity, scale(add(add(a1, scale(a2, 2)),
                                       add(scale(a3, 2), a4)), step / 6))
    return new_position, new_velocity


def replay_controller_braking(submission, brake_reference_ut, initial_mass,
                              maximum_thrust, maximum_mass_flow,
                              policy_terrain_asl, policy_thrust_acceleration):
    """Return the handoff predicted by V1's speed policy with aligned attitude."""
    body_radius = submission["bodyRadius"]
    mu = submission["bodyMu"]
    angular = tuple(submission["angularVelocity"])
    policy_radius = body_radius + policy_terrain_asl + 200
    stop_radius = policy_radius + 5
    policy_gravity = submission["bodyGeeASL"] * 9.81
    position = tuple(submission["positionBCI"])
    velocity = tuple(submission["velocityBCI"])
    ut = submission["inputUT"]
    release_ut = max(ut, brake_reference_ut - 5)

    def speed_limit(at_position, mass):
        return .9 * math.sqrt(2 * (policy_thrust_acceleration *
                                   initial_mass / mass - policy_gravity) *
                                  (norm(at_position) - policy_radius))

    triggered = False
    coast_steps = 0
    while ut < release_ut or not triggered:
        coast_steps += 1
        if coast_steps > 20000 or norm(position) <= stop_radius:
            raise ValueError("no controller braking window")
        surface = add(velocity, scale(cross(angular, position), -1))
        if norm(surface) > .9 * speed_limit(position, initial_mass):
            triggered = True
        if ut >= release_ut and triggered:
            break
        step = min(1., max(.02, release_ut - ut)) if ut < release_ut else .2
        position, velocity = rk4(position, velocity, step, mu)
        ut += step
    burn_start_ut = ut

    mass = initial_mass
    delta_v = 0.
    steps = 0
    step = max(submission["minDt"], min(.2, submission["dt"]))
    while steps < 50000:
        steps += 1
        previous_position, previous_velocity, previous_ut = position, velocity, ut
        up = _unit(position)
        surface = add(velocity, scale(cross(angular, position), -1))
        speed = norm(surface)
        if speed < 1e-8:
            raise ValueError("velocity direction unresolved")
        allowed = speed_limit(position, mass)
        next_position = add(position, scale(velocity, step))
        if norm(next_position) < stop_radius:
            next_position = scale(_unit(next_position), stop_radius)
        next_allowed = speed_limit(next_position, mass)
        local_gravity = norm(gravity(position, mu))
        radial = dot(surface, up)
        radial_fraction = abs(radial / speed)
        minimum_accel = -local_gravity * radial_fraction
        maximum_accel = maximum_thrust / mass - local_gravity * radial_fraction
        controlled_speed = speed if radial > 0 else -speed
        desired_accel = ((-allowed - controlled_speed) / .3 +
                         (-next_allowed + allowed) / step)
        throttle = 0 if radial > 0 else max(0., min(1.,
            (desired_accel - minimum_accel) / (maximum_accel - minimum_accel)))
        thrust_accel = scale(_unit(surface), -throttle * maximum_thrust / mass)
        position, velocity = _rk4_thrust(position, velocity, step, mu, thrust_accel)
        mass = max(.01 * initial_mass, mass - throttle * maximum_mass_flow * step)
        delta_v += throttle * maximum_thrust / mass * step
        ut += step
        if norm(position) <= stop_radius:
            fraction = ((norm(previous_position) - stop_radius) /
                        (norm(previous_position) - norm(position)))
            fraction = max(0., min(1., fraction))
            position = add(previous_position, scale(add(position,
                scale(previous_position, -1)), fraction))
            position = scale(_unit(position), stop_radius)
            velocity = add(previous_velocity, scale(add(velocity,
                scale(previous_velocity, -1)), fraction))
            ut = previous_ut + fraction * step
            break
    else:
        raise ValueError("V1 handoff not reached")
    surface = add(velocity, scale(cross(angular, position), -1))
    return {"burnStartUT": burn_start_ut,
            "end": absolute(position, ut, submission),
            "endSurfaceSpeed": norm(surface), "deltaV": delta_v,
            "steps": steps, "finalMass": mass}
