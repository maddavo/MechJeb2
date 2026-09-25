namespace MuMech
{
    namespace Landing
    {
        /// <summary>
        /// Combines settled post-burn course-correction solutions. The
        /// predictor already rejects incompatible impact endpoints; this
        /// policy prevents modest finite-difference variation in the solver
        /// from holding V1 in its no-throttle confirmation state forever.
        /// </summary>
        public static class CourseCorrectionPredictionConsensus
        {
            public static Vector3d AddSample(Vector3d currentMean, int currentSampleCount, Vector3d sample) =>
                currentSampleCount <= 0 ? sample :
                (currentMean * currentSampleCount + sample) / (currentSampleCount + 1);

            public static bool IsUsable(Vector3d correction, double minimumMagnitude) =>
                correction.sqrMagnitude > minimumMagnitude * minimumMagnitude;
        }
    }
}
