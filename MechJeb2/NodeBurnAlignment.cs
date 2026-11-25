using System;
using UnityEngine;

namespace MuMech
{
    // One force-axis contract for node steering, readiness and termination.
    // Geometry has no engine-count, symmetry, control-point or CoT assumptions.
    internal static class NodeBurnAlignment
    {
        internal static bool Valid(Vector3d vector) =>
            Finite(vector.x) && Finite(vector.y) && Finite(vector.z) &&
            vector.sqrMagnitude > 1e-18 && Finite(vector.sqrMagnitude);

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        internal static double ErrorRadians(Vector3d force, Vector3d target) =>
            Valid(force) && Valid(target) ? Math.Acos(Math.Max(-1, Math.Min(1,
                Vector3d.Dot(force.normalized, target.normalized)))) : double.PositiveInfinity;

        internal static bool TryAttitude(Vector3d force, Vector3d target,
            QuaternionD currentAttitude, out QuaternionD attitude)
        {
            attitude = currentAttitude;
            if (!Valid(force) || !Valid(target)) return false;
            // Rotate the entire current reference frame. Rotating only its
            // forward vector loses the necessary roll on canted engines.
            attitude = MathExtensions.FromToRotation(force.normalized, target.normalized) * currentAttitude;
            return true;
        }

        internal static Vector3d SelectForce(Vector3d measured, Vector3d available, bool burning) =>
            // Gimbal deflection is the attitude controller's output, not a new
            // airframe alignment target. Measured force is used separately for
            // delivered acceleration, never for steering/readiness.
            available;

        internal static Vector3d NozzleForce(Vector3d direction, double thrust, double multiplier) =>
            Valid(direction) && Finite(thrust) && thrust > 0 && Finite(multiplier) && multiplier > 0 ?
                direction.normalized * (thrust * multiplier) : Vector3d.zero;

        internal static Vector3d RcsForce(Vector3d direction, Vector3d command, double power) =>
            Valid(command) ? NozzleForce(direction, power,
                Math.Max(0, Vector3d.Dot(direction.normalized, command.normalized))) : Vector3d.zero;
    }

    internal static class NodeBurnForceSource
    {
        internal static Vector3d Engines(MechJebCore core, bool burning, out Vector3d available, out Vector3d measured)
        {
            // VesselState computes this resultant with movable gimbals at their
            // neutral rotations, weighting the current engine limiters (which
            // include differential throttle). Locked gimbals retain their axis.
            available = core.VesselState.ThrustVectorMaxThrottle;
            measured = Vector3d.zero;
            foreach (var wrapper in core.VesselState.EngineWrappers)
            {
                ModuleEngines engine = wrapper.Engine;
                if (!engine.isEnabled || !engine.EngineIgnited || !engine.isOperational) continue;
                for (int i = 0; i < engine.thrustTransforms.Count; ++i)
                {
                    // Sample the restored ACTUAL nozzle transforms. VesselState
                    // temporarily neutralizes gimbals for its generic stats.
                    Vector3d direction = -engine.thrustTransforms[i].forward;
                    double multiplier = engine.thrustTransformMultipliers[i];
                    measured += NodeBurnAlignment.NozzleForce(direction, engine.finalThrust, multiplier);
                }
            }
            // Do not chase measured gimbal deflection in either coast or burn.
            // Never reset gimbals here or rewrite throttle allocations.
            return NodeBurnAlignment.SelectForce(measured, available, burning);
        }

        internal static Vector3d DifferentialForceAxis(Vector3d available, Vector3d unallocated,
            bool nodeActive) => nodeActive ?
                (NodeBurnAlignment.Valid(available) ? available.normalized :
                 NodeBurnAlignment.Valid(unallocated) ? unallocated.normalized : Vector3d.up) : Vector3d.up;

        internal static Vector3d Rcs(MechJebCore core)
        {
            Vector3d force = Vector3d.zero;
            if (core.Rcsbal.Enabled)
            {
                // DoRCS commands Z=-1: +Y in KSP's vessel reference frame.
                core.Rcsbal.GetThrottles(Vector3.up, out double[] throttles,
                    out System.Collections.Generic.List<RCSSolver.Thruster> thrusters);
                if (throttles == null || throttles.Length != thrusters.Count) return force;
                for (int i = 0; i < throttles.Length; ++i)
                    force += RcsModule(core, thrusters[i].PartModule, throttles[i]);
                return force;
            }
            foreach (Part part in core.vessel.parts)
                foreach (PartModule module in part.Modules)
                    if (module is ModuleRCS rcs && !part.ShieldedFromAirstream &&
                        rcs.isEnabled && rcs.rcsEnabled && rcs.enableZ && !rcs.isJustForShow && !rcs.flameout)
                        force += RcsModule(core, rcs, rcs.thrusterPower);
            return force;
        }

        private static Vector3d RcsModule(MechJebCore core, ModuleRCS rcs, double selectedPower)
        {
            Vector3d force = Vector3d.zero;
            if (!rcs.isEnabled || !rcs.rcsEnabled || !rcs.enableZ || rcs.isJustForShow ||
                rcs.flameout || rcs.part.ShieldedFromAirstream) return force;
            foreach (Transform nozzle in rcs.thrusterTransforms)
            {
                            if (!nozzle.gameObject.activeInHierarchy) continue;
                            Vector3d direction = rcs.useZaxis ? -nozzle.forward : -nozzle.up;
                            double power = selectedPower * rcs.thrustPercentage / 100;
                            if (FlightInputHandler.fetch.precisionMode)
                            {
                                double lever = rcs.useLever ? rcs.GetLeverDistance(nozzle, direction, core.VesselState.CoM) : 0;
                                power *= rcs.useLever ? (lever > 1 ? 1 / lever : 1) : rcs.precisionFactor;
                            }
                            force += NodeBurnAlignment.RcsForce(direction, core.VesselState.Forward, power);
            }
            return force;
        }
    }
}
