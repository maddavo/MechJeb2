using System.Reflection;
using MuMech;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class LandingPredictorTeardownTests
    {
        [Fact]
        public void SceneTeardownMakesLateUpdateAndMapCallbacksInertWithoutGameState()
        {
            var predictor = new MechJebModuleLandingPredictions(null!);
            predictor.OnDestroy();
            predictor.OnDestroy();
            predictor.OnFixedUpdate();
            Assert.Null(predictor.GetResult());
            typeof(MechJebModuleLandingPredictions).GetMethod("DoMapView",
                BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(predictor, null);
        }
    }
}
