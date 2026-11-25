namespace MuMech
{
    // No roll-angle target means automatic spin damping while attitude control
    // is engaged. It never means free roll; OFF releases attitude authority.
    internal static class AttitudeRollPolicy
    {
        internal static bool KillRateOnly(double requestedRate) => requestedRate == 0;
        internal static double PositionError(double error, double requestedRate) =>
            KillRateOnly(requestedRate) ? 0 : error;
        internal static double TargetRate(double positionRate, double requestedRate) =>
            KillRateOnly(requestedRate) ? 0 : positionRate;
        internal static double RateWithoutAngle(double requestedRate) =>
            double.IsNaN(requestedRate) ? 0 : requestedRate;
    }
}
