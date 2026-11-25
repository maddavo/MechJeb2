namespace MuMech.Landing
{
    internal static class BrakingAllocation
    {
        // Reserve most of the attainable retrograde force at the applied throttle.
        // The existing objective still chooses torque and thrust within that budget.
        internal const double RetainedFraction = 0.8;

        internal static bool MeetsReserve(double[] projection, double[] allocation, double throttle)
        {
            double capacity = 0, actual = 0;
            for (int i = 0; i < projection.Length; ++i)
            {
                capacity += System.Math.Max(0, projection[i]);
                actual += projection[i] * allocation[i];
            }
            return capacity <= 0 || actual >= capacity * throttle * RetainedFraction;
        }

        internal static void Reserve(alglib.minqpstate state, double[] projection, double throttle, double[] trim = null)
        {
            double capacity = 0;
            foreach (double force in projection)
                if (force > 0) capacity += force;
            if (capacity <= 0 || double.IsInfinity(capacity)) return;

            var constraint = new double[1, projection.Length + 1];
            for (int i = 0; i < projection.Length; ++i)
                constraint[0, i] = projection[i] / capacity;
            double required = RetainedFraction * throttle;
            if (trim != null)
            {
                double trimmedForce = 0;
                for (int i = 0; i < projection.Length; ++i)
                    trimmedForce += constraint[0, i] * trim[i];
                // The reserve must be feasible without overriding a user limit.
                required = System.Math.Min(required, trimmedForce);
            }
            constraint[0, projection.Length] = required;
            alglib.minqpsetlc(state, constraint, new[] { 1 }, 1);
        }
    }
}
