using System;

namespace MuMech
{
    // The controller's normalized output represents a request against the same
    // ordinary-plus-backup budget used to plan the manoeuvre.
    internal static class DifferentialTorquePriority
    {
        internal static double Residual(double demand, double ordinary, double backup,
            double previous, double dt, double responseSeconds, double angularRate = 0)
        {
            if (double.IsNaN(demand) || double.IsInfinity(demand) || !(backup > 0)) return 0;
            if (double.IsNaN(responseSeconds) || double.IsInfinity(responseSeconds)) responseSeconds = 0;
            double target = Math.Sign(demand) * Math.Min(backup, Math.Max(0, Math.Abs(demand) - ordinary));
            if (Math.Sign(previous) != Math.Sign(target) && target != 0) previous = 0;
            // This residual is already beyond ordinary authority. Apply a
            // quicker ramp only while building torque against measured rotation.
            bool braking = target * angularRate < 0 && Math.Abs(target) > Math.Abs(previous);
            double timeConstant = braking ? .04 : Math.Max(.15, responseSeconds);
            double blend = 1 - Math.Exp(-Math.Max(0, dt) / timeConstant);
            double applied = previous + blend * (target - previous);
            return Math.Abs(applied) < 1e-5 ? 0 : Math.Max(-backup, Math.Min(backup, applied));
        }
    }
}
