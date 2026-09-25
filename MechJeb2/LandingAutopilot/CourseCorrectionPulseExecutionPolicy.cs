namespace MuMech
{
    namespace Landing
    {
        /// <summary>
        /// Defines the terminal precision for a V1 course-correction pulse.
        /// The thrust controller can resolve a small low-throttle burn over
        /// several physics ticks; treating 0.05 m/s as already complete
        /// discards effect-scaled commands before any throttle is applied.
        /// </summary>
        public static class CourseCorrectionPulseExecutionPolicy
        {
            public const double CompletionDeltaV = 0.005;

            public static bool HasCompleted(double remainingDeltaV) =>
                remainingDeltaV <= CompletionDeltaV;
        }
    }
}
