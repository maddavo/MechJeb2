using System;
using System.Linq;

namespace MuMech
{
    internal static class DifferentialAxialTrim
    {
        // Use the same soft objective at every axis angle. The positive
        // projections define capacity without assuming counter-thrust is fired.
        internal const double TransverseWeight = 4;
        internal const double RetainedAxialFraction = .9;

        internal static double[] Solve(Vector3d[] force, double[] upper, Vector3d constant, Vector3d axis,
            Vector3d[] torque = null, Vector3d constantTorque = default)
        {
            int n = force.Length;
            double scale = Math.Max(1, force.Sum(v => v.magnitude) + constant.magnitude);
            var axial = new double[n];
            var transverse = new Vector3d[n];
            var preferred = new double[n];
            double target = 0;
            for (int i = 0; i < n; ++i)
            {
                axial[i] = Vector3d.Dot(force[i], axis) / scale;
                transverse[i] = force[i] / scale - axial[i] * axis;
                if (axial[i] > 0)
                {
                    target += axial[i] * upper[i];
                    preferred[i] = upper[i];
                }
            }

            // There is no useful commanded-axis capacity. Minimize lateral
            // force without rewarding propulsion opposite the command.
            if (target <= 1e-10 && constant.sqrMagnitude == 0) return preferred;
            Vector3d constantSide = (constant - Vector3d.Dot(constant, axis) * axis) / scale;
            // Steady torque is a soft trim objective. A one-metre reference
            // arm prevents a small residual on a short-lever craft dominating force.
            const double steadyTorqueWeight = 32;
            double torqueScale = torque == null ? 1 :
                Math.Max(scale, torque.Sum(v => v.magnitude) + constantTorque.magnitude);
            double torqueWeight = steadyTorqueWeight / (torqueScale * torqueScale);
            var matrix = new double[n, n];
            var linear = new double[n];
            for (int i = 0; i < n; ++i)
            {
                linear[i] = TransverseWeight * Vector3d.Dot(transverse[i], constantSide) - axial[i] * target;
                for (int j = 0; j < n; ++j)
                    matrix[i, j] = TransverseWeight * Vector3d.Dot(transverse[i], transverse[j]) + axial[i] * axial[j];
                if (torque != null)
                {
                    linear[i] += torqueWeight * Vector3d.Dot(torque[i], constantTorque);
                    for (int j = 0; j < n; ++j)
                        matrix[i, j] += torqueWeight * Vector3d.Dot(torque[i], torque[j]);
                }
                matrix[i, i] += 1e-5; // Resolve interchangeable engine mixes consistently.
            }
            alglib.minqpcreate(n, out var qp);
            alglib.minqpsetquadraticterm(qp, matrix, false);
            alglib.minqpsetlinearterm(qp, linear);
            alglib.minqpsetbc(qp, new double[n], upper);
            Protect(qp, force, upper, preferred, axis, null, RetainedAxialFraction);
            alglib.minqpsetalgodensegenipm(qp, 0);
            alglib.minqpoptimize(qp);
            alglib.minqpresults(qp, out var allocation, out var report);
            return report.terminationtype > 0 && allocation.All(x => !double.IsNaN(x) && !double.IsInfinity(x))
                ? allocation : preferred;
        }

        internal static void Protect(alglib.minqpstate state, Vector3d[] force, double[] upper,
            double[] trim, Vector3d axis, double[] braking, double retainedFraction)
        {
            double[,] constraints = ForceConstraints(force, upper, trim, axis, braking, retainedFraction, out int count);
            if (count > 0) alglib.minqpsetlc(state, constraints, new[] { 1, 1 }, count);
        }

        internal static void TorqueHeadroom(Vector3d[] force, Vector3d[] torque, double[] upper,
            double[] trim, Vector3d axis, double[] braking, out Vector3d positive, out Vector3d negative)
        {
            positive = negative = Vector3d.zero;
            double[,] constraints = ForceConstraints(force, upper, trim, axis, braking, RetainedAxialFraction, out int count);
            alglib.minlpcreate(force.Length, out var state);
            alglib.minlpsetbc(state, new double[force.Length], upper);
            if (count > 0) alglib.minlpsetlc(state, constraints, new[] { 1, 1 }, count);
            for (int component = 0; component < 3; component += 2)
            {
                int index = component;
                var cost = torque.Select(value => value[index]).ToArray();
                double scale = cost.Max(Math.Abs);
                if (scale <= 1e-8) continue;
                for (int i = 0; i < cost.Length; i++) cost[i] /= scale;
                negative[component] = Extremum(state, cost, trim) * scale;
                for (int i = 0; i < cost.Length; i++) cost[i] = -cost[i];
                positive[component] = Extremum(state, cost, trim) * scale;
            }
        }

        private static double Extremum(alglib.minlpstate state, double[] cost, double[] trim)
        {
            alglib.minlpsetcost(state, cost);
            alglib.minlpoptimize(state);
            alglib.minlpresults(state, out var allocation, out var report);
            if (report.terminationtype <= 0) return 0;
            double gain = 0;
            for (int i = 0; i < cost.Length; i++) gain += cost[i] * (trim[i] - allocation[i]);
            return double.IsNaN(gain) || double.IsInfinity(gain) ? 0 : Math.Max(0, gain);
        }

        private static double[,] ForceConstraints(Vector3d[] force, double[] upper,
            double[] trim, Vector3d axis, double[] braking, double retainedFraction, out int count)
        {
            var axial = force.Select(v => Vector3d.Dot(v, axis)).ToArray();
            var constraints = new double[2, force.Length + 1];
            count = AddReserve(constraints, 0, axial, upper, trim, retainedFraction, relativeToTrim: true);
            if (braking != null)
                count += AddReserve(constraints, count, braking, upper, trim, Landing.BrakingAllocation.RetainedFraction, relativeToTrim: false);
            return constraints;
        }

        private static int AddReserve(double[,] constraints, int row, double[] projection,
            double[] upper, double[] trim, double fraction, bool relativeToTrim)
        {
            double capacity = 0, nominal = 0;
            for (int i = 0; i < projection.Length; ++i)
            {
                capacity += Math.Max(0, projection[i]) * upper[i];
                nominal += projection[i] * trim[i];
            }
            if (capacity <= 1e-8 || nominal <= 0) return 0;
            for (int i = 0; i < projection.Length; ++i)
                constraints[row, i] = projection[i] / capacity;
            double required = relativeToTrim ? nominal * fraction : Math.Min(nominal, capacity * fraction);
            constraints[row, projection.Length] = required / capacity;
            return 1;
        }
    }
}
