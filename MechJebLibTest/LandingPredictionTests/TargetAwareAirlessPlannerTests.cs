using MuMech.Landing;
using System;
using System.Threading;
using Xunit;
using Xunit.Abstractions;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwareAirlessPlannerTests
    {
        private readonly ITestOutputHelper _output;

        public TargetAwareAirlessPlannerTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void MunSnapshotRefinesSignedDownrangeWithBoundedTerrainQueries()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL, 200, 32, 1024);
            DriveThroughRefinement(planner);

            Assert.True(planner.Stage == TargetAwarePlannerStage.Complete,
                planner.Failure ?? planner.Stage.ToString());
            Assert.True(planner.SelectedTerrain.ClearPath);
            Assert.True(planner.SelectedTerrain.HandoffClearance >= 0);
            Assert.InRange(planner.SignedDownrangeError, -200, 200);
            Assert.InRange(planner.CrossrangeError, 100, 200);
            Assert.True(planner.TimingDistanceEstimate < 200);
            Assert.InRange(planner.TerrainQueryCount, 1, 1024);
            _output.WriteLine($"Mun flat-terrain replay: queries={planner.TerrainQueryCount}, " +
                              $"terrainMs={planner.TerrainQueryMilliseconds:F3}, " +
                              $"downrange={planner.SignedDownrangeError:F2}, " +
                              $"crossrange={planner.CrossrangeError:F2}, " +
                              $"timingInterval={planner.TimingInterval:F3}, " +
                              $"transitionVertical={planner.TerminalHandoff.ControllerTransitionVerticalSpeed:F2}, " +
                              $"idealStopDistance={planner.TerminalHandoff.OptimisticVerticalStoppingDistance:F2}");
        }

        [Fact]
        public void ApprovedTerminalPoliciesCanValidateMunTimingSearch()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL, 200, 32, 1536);
            DriveThroughRefinement(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.TerminalTouchdownValidated);
            Assert.InRange(planner.TerrainQueryCount, 1, 1536);
            _output.WriteLine($"Approved terminal result={planner.PathEndpointKind} " +
                $"queries={planner.TerrainQueryCount}");
        }

        [Fact]
        public void CapturedMunFirstBallisticIntersectionFindsPoweredCandidateOnFlatTerrain()
        {
            // Submission 101 of the 2026-09-30 failed deorbit. PQS samples
            // were not captured; target-height terrain isolates the timing
            // search from unknown off-target terrain.
            const double ut = 24604034.297638249;
            var source = new AirlessTargetAwareSnapshot(ut, ut, 200000,
                65138397520.780693, 0.16605670009835341, 138984.37657447491,
                0.67416666666666658, 23.473055555555554, 492.18766502593644,
                692.18766502593644, 8.1485774582046222, 0.2,
                0.019999999552965164, 0.5, -400, 8300,
                new Vector3d(-110771.32007350624, 2746.8805482437429, 194786.49761048483),
                new Vector3d(-433.08841679591922, -5.882039743143495, -245.25398543228636),
                new Vector3d(0, -4.5207853300063085E-05, 0),
                new Vector3d(-0.25558771502106337, 0, 0.96678587077507072),
                new Vector3d(-0.96678587077507072, 0, -0.25558771502106331),
                new Vector3d(-1.5649716988686863E-17, 1, 5.9196606006842562E-17));
            var snapshot = new AirlessTargetAwareSnapshot(source, 78.541345069119416,
                640.00023396729716, 0, 0.27192431688308716, 0,
                200692.18766502594, 1.629016227964847, 8.1485774582046222,
                false, new Vector3d(0.85455363988876343, 0.013799682259559631,
                    0.51917988061904907), 0, 1, 0, 0, 0.5, 2.9194062313181348, 0);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 101, new object(),
                (latitude, longitude) => source.TargetTerrainASL, 200, 32, 1536);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            var coarse = planner.RunCoarse();
            foreach (var candidate in coarse)
            {
                var site = AirlessTargetAwareSimulation.ToAbsolute(candidate.End.Position,
                    candidate.End.UT, snapshot);
                _output.WriteLine($"candidate brake={candidate.BrakeUT - ut:F1}s " +
                    $"longitude={site.Longitude:F2} speed={candidate.EndSurfaceSpeed:F1} " +
                    $"handoff={candidate.ReachedHandoff}");
            }
            planner.SetCoarseOutputs(coarse);
            DriveUntilSettled(planner);
            _output.WriteLine($"coarse stage={planner.Stage} queries={planner.TerrainQueryCount}");
            Assert.Equal(TargetAwarePlannerStage.ReadyRefinement, planner.Stage);
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            DriveUntilSettled(planner);
            _output.WriteLine($"final stage={planner.Stage} failure={planner.Failure} " +
                $"brake={planner.SelectedOutput?.BrakeUT - ut:F1}s " +
                $"landable={planner.TerminalTouchdownValidated} queries={planner.TerrainQueryCount}");
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.TerminalTouchdownValidated);

            var raised = new TargetAwareAirlessPlanner(snapshot, 1, 102, new object(),
                (latitude, longitude) => longitude > 40 ? 2932 :
                    source.TargetTerrainASL, 200, 32, 1536);
            raised.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(raised);
            raised.BeginCoarseWorker();
            raised.SetCoarseOutputs(raised.RunCoarse());
            DriveUntilSettled(raised);
            if (raised.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                raised.BeginRefinementWorker();
                raised.SetRefinement(raised.RunRefinement());
                DriveUntilSettled(raised);
            }
            _output.WriteLine($"raised stage={raised.Stage} failure={raised.Failure} " +
                $"brake={raised.SelectedOutput?.BrakeUT - ut:F1}s " +
                $"landable={raised.TerminalTouchdownValidated} " +
                $"policyTerrain={raised.SelectedPolicyTerrainASL:F1} " +
                $"queries={raised.TerrainQueryCount}");
            Assert.Equal(TargetAwarePlannerStage.Complete, raised.Stage);
            Assert.True(raised.TerminalTouchdownValidated);
            Assert.NotNull(raised.SelectedOutput);
            Assert.True(raised.SelectedOutput.BrakeUT >= ut + 15);
            Assert.Equal(2932, raised.SelectedPolicyTerrainASL, 0);
            Assert.InRange(raised.TerrainQueryCount, 1, 1536);
        }

        [Fact]
        public void LatestMunFirstIntersectionProducesPoweredForecastAcrossSampledTerrain()
        {
            // Submission 100 of the 30 September flight. The capture contains
            // ballistic contacts, not the full PQS map. Interpolate those
            // measured heights to challenge the endpoint-policy feedback.
            const double ut = 24604031.736792564;
            var source = new AirlessTargetAwareSnapshot(ut, ut, 200000,
                65138397520.78069, 0.1660567000983534, 138984.37657447575,
                0.6741666666666666, 23.473055555555554, 492.18766502593644,
                692.1876650259364, 8.075855316204823, 0.2,
                0.019999999552965164, 0.5, -400, 8300,
                new Vector3d(-70451.54261689488, 2758.4874235945067, 212717.23347697995),
                new Vector3d(-492.9175996172553, -6.319988545222581, -161.79268432363656),
                new Vector3d(0, -4.5207853300062813e-05, 0),
                new Vector3d(-0.06684665250743577, 0, 0.9977632610236509),
                new Vector3d(-0.9977632610236509, 0, -0.06684665250743571),
                new Vector3d(-4.093041769618119e-18, 1, 6.10933614530049e-17));
            var snapshot = new AirlessTargetAwareSnapshot(source, 79.24860520095535,
                640.0002696139525, 0, 0.27192431688308716, 0,
                200692.18766502594, 1.629016227964847, 8.075855316204823,
                false, new Vector3d(0.947510302066803, -0.017922893166542053,
                    0.3192228674888611), 0, 1, 0, 0, 0.5, 2.955644091911381, 0);
            var measured = new[] {
                (23.473, 492.188), (25.873, 1575.939),
                (28.298, 1787.120), (31.078, 1823.541),
                (33.973, 2692.722), (38.115, 2462.647),
                (43.256, 2829.106), (51.574, 2534.990),
                (65.084, 3635.904), (117.667, 4062.097)
            };
            double Terrain(double latitude, double longitude)
            {
                if (longitude <= measured[0].Item1) return measured[0].Item2;
                for (int i = 1; i < measured.Length; ++i)
                    if (longitude <= measured[i].Item1)
                    {
                        double fraction = (longitude - measured[i - 1].Item1) /
                            (measured[i].Item1 - measured[i - 1].Item1);
                        return measured[i - 1].Item2 + fraction *
                            (measured[i].Item2 - measured[i - 1].Item2);
                    }
                return measured[measured.Length - 1].Item2;
            }
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 100,
                new object(), Terrain, 200, 32, 1536);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                planner.BeginRefinementWorker();
                planner.SetRefinement(planner.RunRefinement());
                DriveUntilSettled(planner);
            }
            _output.WriteLine($"first intersection stage={planner.Stage} " +
                $"failure={planner.Failure} queries={planner.TerrainQueryCount} " +
                $"brake={planner.SelectedOutput?.BrakeUT - ut:F1}s");
            Assert.True(planner.Stage == TargetAwarePlannerStage.Complete &&
                planner.TerminalTouchdownValidated, planner.Failure ?? planner.Stage.ToString());
            var endpoint = planner.NominalTerminalTerrain.Contact;
            var site = AirlessTargetAwareSimulation.ToAbsolute(endpoint.Position,
                endpoint.UT, snapshot);
            _output.WriteLine($"powered terminal lon={site.Longitude:F3} " +
                $"target lon={source.TargetLongitude:F3} " +
                $"horizontal={planner.TerminalHandoff.EndHorizontalSpeed:F2}m/s");
            Assert.True(site.Longitude > source.TargetLongitude);
            Assert.InRange(planner.TerrainQueryCount, 1, 1536);
        }

        [Fact]
        public void FailedMunRefresh24CanRevalidateRaisedEndpointTerrain()
        {
            // Submission 24 from the 2026-09-29 failed KSP flight. PQS is
            // unavailable offline, so a raised-ridge oracle tests whether
            // the selected endpoint terrain feeds back into V1's speed policy.
            const double inputUT = 24603579.228602532;
            var source = new AirlessTargetAwareSnapshot(inputUT, inputUT, 200000,
                65138397520.780693, 0.16605670009835341, 138984.37657447575,
                0.67416666666666658, 23.473055555555554, 492.18766502593644,
                692.18766502593644, 8.1128061441219437, 0.2,
                0.019999999552965164, 0.5, -400, 8300,
                new Vector3d(152563.34319155806, 3747.2005581466815, 162815.752504458),
                new Vector3d(-385.34555053115309, 3.243742091594747, 337.63961415603808),
                new Vector3d(0, -4.5207853300062813E-05, 0),
                new Vector3d(-0.079946715521042863, 0, 0.99679913858178948),
                new Vector3d(-0.99679913858178948, 0, -0.079946715521042808),
                new Vector3d(-4.8951627897149581E-18, 1, 6.103432792959659E-17));
            var snapshot = new AirlessTargetAwareSnapshot(source, 78.887671897122345,
                640.0003892624502, 0, 0.27192431688308716, 0,
                200692.18766502594, 1.629016227964847, 8.1128061441219437);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 24, new object(),
                (latitude, longitude) => longitude < 22 ? 1420 : source.TargetTerrainASL,
                200, 32, 1024);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                planner.BeginRefinementWorker();
                planner.SetRefinement(planner.RunRefinement());
                DriveUntilSettled(planner);
            }
            Assert.True(planner.Stage == TargetAwarePlannerStage.Complete,
                planner.Failure ?? planner.Stage.ToString());
            Assert.True(planner.TerminalTouchdownValidated);
            Assert.True(planner.SelectedOutput.BrakeUT > inputUT);
            Assert.True(planner.PolicyEscalations > 0);
        }

        [Fact]
        public void NoReticleMunSnapshotCanPublishAnHonestMissAboveUprangeTerrain()
        {
            // First post-deorbit snapshot from the 2026-09-29 no-reticle
            // flight. The live capture did not record each candidate's PQS
            // sample, so the oracle supplies an explicit raised uprange ridge.
            const double inputUT = 24603464.048182253;
            var source = new AirlessTargetAwareSnapshot(inputUT, inputUT, 200000,
                65138397520.78069, 0.1660567000983534, 138984.37657447497,
                0.6741666666666666, 23.473055555555554, 492.18766502593644,
                692.1876650259364, 8.109494985867846, 0.2,
                0.019999999552965164, 0.5, -400, 8300,
                new Vector3d(168422.503375561, 3231.06776177337, 147997.13795877405),
                new Vector3d(-337.18116223774234, 5.598991540146544, 380.9405218515084),
                new Vector3d(0, -4.520785330006307e-05, 0),
                new Vector3d(-0.23815475457506763, 0, 0.971227220002245),
                new Vector3d(-0.971227220002245, 0, -0.23815475457506757),
                new Vector3d(-1.4582291282281835e-17, 1, 5.946855123099965e-17));
            var snapshot = new AirlessTargetAwareSnapshot(source, 78.919843146512,
                640.0000722821159, 0, 0.27192431688308716, 0,
                200692.18766502594, 1.629016227964847, 8.109494985867846);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => longitude < 20 ? 835 : source.TargetTerrainASL,
                200, 32, 1024);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                planner.BeginRefinementWorker();
                planner.SetRefinement(planner.RunRefinement());
                DriveUntilSettled(planner);
            }
            Assert.True(planner.Stage == TargetAwarePlannerStage.Complete,
                planner.Failure ?? planner.Stage.ToString());
            Assert.True(planner.PolicyEscalations > 0);
            Assert.True(planner.TerminalTouchdownValidated);
            Assert.False(double.IsNaN(planner.SelectedOutput.BrakeUT));
            Assert.InRange(planner.TerrainQueryCount, 1, 1024);
            _output.WriteLine($"Raised-terrain Mun replay: powered brake at " +
                $"{planner.SelectedOutput.BrakeUT:F2}; queries={planner.TerrainQueryCount}");

            var activeBaseline = new TargetAwareAirlessPlanner(snapshot, 1, 3,
                new object(), (latitude, longitude) => longitude < 20 ? 835 :
                    source.TargetTerrainASL, 200, 32, 1024, directForecast: true);
            activeBaseline.SetDirectOutput(AirlessTargetAwareSimulation.RunNominalV1(snapshot));
            DriveUntilSettled(activeBaseline);
            if (activeBaseline.Stage == TargetAwarePlannerStage.ReadyTerminal)
            {
                activeBaseline.BeginTerminalWorker();
                activeBaseline.SetTerminalEnvelope(activeBaseline.RunTerminalEnvelope());
                DriveUntilSettled(activeBaseline);
            }
            Assert.Equal(TargetAwarePlannerStage.Complete, activeBaseline.Stage);
            Assert.True(activeBaseline.SelectedTerrain.HasFirstContact ||
                activeBaseline.TerminalTouchdownValidated ||
                activeBaseline.TerminalImpactForecast);
            _output.WriteLine($"Direct V1 baseline: impact=" +
                $"{(activeBaseline.SelectedTerrain.HasFirstContact || activeBaseline.TerminalImpactForecast)} " +
                $"landable={activeBaseline.TerminalTouchdownValidated} " +
                $"terrainQueries={activeBaseline.TerrainQueryCount}");
        }

        [Fact]
        public void ChangingTargetDoesNotChangeControllerCompatibleTerminalValidity()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var farTarget = new AirlessTargetAwareSnapshot(source.InputUT, source.EpochUT,
                source.BodyRadius, source.BodyMu, source.BodyGeeASL,
                source.RotationPeriod, source.TargetLatitude, 90,
                source.TargetTerrainASL, source.DecelEndASL,
                source.MaximumThrustAcceleration, source.Dt, source.MinDt,
                source.MaxOrbits, source.MinimumTerrainASL, source.MaximumTerrainASL,
                source.Position, source.Velocity, source.AngularVelocity,
                source.Axis0, source.Axis90, source.AxisNorth);
            var snapshot = new AirlessTargetAwareSnapshot(farTarget, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL, 200, 32, 1536);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                planner.BeginRefinementWorker();
                planner.SetRefinement(planner.RunRefinement());
                DriveUntilSettled(planner);
            }
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.TerminalTouchdownValidated);
            var nearSnapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var near = new TargetAwareAirlessPlanner(nearSnapshot, 1, 2, new object(),
                (latitude, longitude) => nearSnapshot.TargetTerrainASL, 200, 32, 1536);
            near.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(nearSnapshot));
            DriveUntilSettled(near);
            near.BeginCoarseWorker();
            near.SetCoarseOutputs(near.RunCoarse());
            DriveUntilSettled(near);
            near.BeginRefinementWorker();
            near.SetRefinement(near.RunRefinement());
            DriveUntilSettled(near);
            Assert.Equal(TargetAwarePlannerStage.Complete, near.Stage);
            Assert.True(near.TerminalTouchdownValidated);
        }

        [Fact]
        public void LiveBrakingForecastReportsFirstRaisedTerrainImpactWithoutChangingV1Policy()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            Vector3d retrograde = -(source.Velocity -
                Vector3d.Cross(source.AngularVelocity, source.Position)).normalized;
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration,
                true, retrograde);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => 1500, 200, 32, 1024,
                directForecast: true);
            planner.SetDirectOutput(AirlessTargetAwareSimulation.Run(snapshot,
                snapshot.InputUT, true));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.SelectedTerrain.HasFirstContact);
            Assert.Equal(TargetAwarePathEndpointKind.BrakingImpact,
                planner.PathEndpointKind);
            Assert.InRange(planner.SelectedTerrain.FirstContact.Position.magnitude -
                snapshot.BodyRadius, 1499, 1501);
            Assert.Equal(source.DecelEndASL - 200, planner.SelectedPolicyTerrainASL);
        }

        [Fact]
        public void ClearHandoffWithLargeHorizontalStopRemainsAnUncertainContinuation()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var position = new Vector3d(source.BodyRadius + 697, 0, 0);
            var state = new AirlessTargetAwareState(position,
                new Vector3d(-5, 100, 0), source.InputUT + 1);
            var output = new AirlessTargetAwareOutput(source.InputUT,
                new System.Collections.Generic.List<AirlessTargetAwareState>(),
                new System.Collections.Generic.List<AirlessTargetAwareState> { state },
                true, 100, 0, 1, true, snapshot.InitialMass);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => 492, 200, 32, 1024,
                directForecast: true);
            planner.SetDirectOutput(output);
            DriveUntilSettled(planner, false);
            Assert.Equal(TargetAwarePlannerStage.ReadyTerminal, planner.Stage);
            Assert.Equal(TargetAwarePathEndpointKind.BrakingHandoff,
                planner.PathEndpointKind);
            Assert.False(planner.SelectedTerrain.HasFirstContact);
            Assert.InRange(planner.TerminalHandoff.EndHorizontalSpeed, 99, 101);
            Assert.True(planner.TerminalHandoff.IdealHoverHorizontalStoppingDistance > 10000);
            Assert.True(planner.TerminalHandoff.IdealHoverHorizontalStoppingTime > 200);
            planner.BeginTerminalWorker();
            planner.SetTerminalEnvelope(planner.RunTerminalEnvelope());
            Assert.False(planner.TerminalTouchdownValidated);
        }

        [Fact]
        public void DirectMunBaselineResolvesTerrainWithApprovedTerminalPolicies()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => source.TargetTerrainASL,
                200, 32, 1024, directForecast: true);
            planner.SetDirectOutput(AirlessTargetAwareSimulation.RunNominalV1(snapshot));
            DriveUntilSettled(planner, false);
            Assert.Equal(TargetAwarePlannerStage.ReadyTerminal, planner.Stage);
            var handoffSite = AirlessTargetAwareSimulation.ToAbsolute(
                planner.SelectedOutput.End.Position, planner.SelectedOutput.End.UT, snapshot);
            _output.WriteLine($"direct handoff longitude={handoffSite.Longitude:F3}");
            planner.BeginTerminalWorker();
            planner.SetTerminalEnvelope(planner.RunTerminalEnvelope());
            var terminalSite = AirlessTargetAwareSimulation.ToAbsolute(
                planner.TerminalEnvelope.Nominal.Trajectory[
                    planner.TerminalEnvelope.Nominal.Trajectory.Count - 1].Position,
                planner.TerminalEnvelope.Nominal.Trajectory[
                    planner.TerminalEnvelope.Nominal.Trajectory.Count - 1].UT, snapshot);
            _output.WriteLine($"direct terminal longitude={terminalSite.Longitude:F3}");
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.NominalTerminalTerrain.Resolved);
            Assert.True(planner.DelayedTerminalTerrain.Resolved);
            Assert.True(planner.TerminalTouchdownValidated);
            Assert.InRange(planner.TerrainQueryCount, 1, 1024);
            Assert.InRange(planner.TerminalEndpointSeparation, 0, 200);
        }

        [Fact]
        public void TerminalPathContinuesToLowerTerrainInsteadOfRejectingNoContact()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            // The braking handoff is west of this boundary; terminal motion
            // crosses it before the modelled touchdown on lower ground.
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => longitude < -16.364 ?
                    source.TargetTerrainASL : source.TargetTerrainASL - 92,
                200, 32, 1024, directForecast: true);
            planner.SetDirectOutput(AirlessTargetAwareSimulation.RunNominalV1(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.NominalTerminalTerrain.HasContact);
            Assert.True(planner.DelayedTerminalTerrain.HasContact);
            Assert.True(planner.TerminalTouchdownValidated);
            Assert.InRange(planner.TerrainQueryCount, 1, 1024);
        }


        [Fact]
        public void DirectForecastRejectsMissingPathBeforeTerrainResolution()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var incomplete = new AirlessTargetAwareOutput(source.InputUT,
                new System.Collections.Generic.List<AirlessTargetAwareState>(),
                new System.Collections.Generic.List<AirlessTargetAwareState>(),
                false, double.NaN, 0, 0, true);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => 492, 200, 32, 1024,
                directForecast: true);
            Assert.Throws<InvalidOperationException>(() =>
                planner.SetDirectOutput(incomplete));
        }

        [Fact]
        public void ChangedTerrainDuringRefinementRejectsTheWholeTransaction()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            bool ridgePresent = false;
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => ridgePresent ? 10000 : snapshot.TargetTerrainASL, 200, 32, 1024);
            DriveUntil(planner, TargetAwarePlannerStage.ReadyRefinement);
            ridgePresent = true;
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            DriveUntilSettled(planner);

            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.Null(planner.SelectedOutput);
        }

        [Fact]
        public void SlowTerrainQueryStopsTheFlightThreadBatchAfterOneSample()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) =>
                {
                    Thread.Sleep(3);
                    return snapshot.TargetTerrainASL;
                }, 200, 32, 1024, 1);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            planner.AdvanceTerrain();
            Assert.Equal(1, planner.TerrainQueryCount);
            Assert.Equal(TargetAwarePlannerStage.ResolveBallistic, planner.Stage);
        }

        [Fact]
        public void TerrainQueryFailureLeavesNoPublishableCandidate()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => throw new InvalidOperationException("PQS unavailable"),
                200, 32, 1024);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            planner.AdvanceTerrain();
            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.StartsWith("TerrainQueryFailed", planner.Failure);
            Assert.Null(planner.SelectedOutput);
        }

        [Fact]
        public void FarDownrangeRidgeDoesNotHideASeparateClearSignedRoot()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => longitude > 30 ? 1000 : snapshot.TargetTerrainASL,
                200, 32, 1024);
            DriveThroughRefinement(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.SelectedTerrain.ClearPath);
            Assert.InRange(planner.SignedDownrangeError, -200, 200);
        }

        [Fact]
        public void SelectedCoastIsRecheckedBeforeCommit()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot(16000);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL,
                200, 32, 4096);
            DriveThroughRefinement(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.NotEmpty(planner.SelectedOutput.CoastSamples);
        }

        [Fact]
        public void TerrainChangeOnSelectedCoastRejectsTheTransaction()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot(16000);
            bool ridgePresent = false;
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => ridgePresent ? 30000 : snapshot.TargetTerrainASL,
                200, 32, 4096);
            DriveUntil(planner, TargetAwarePlannerStage.ReadyRefinement);
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            for (int i = 0; i < 200 && planner.Stage == TargetAwarePlannerStage.ResolveRefinement; ++i)
                planner.AdvanceTerrain();
            Assert.Equal(TargetAwarePlannerStage.ResolveSelectedCoast, planner.Stage);
            ridgePresent = true;
            planner.AdvanceTerrain();
            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.Contains("TerrainChangedDuringTransaction", planner.Failure);
        }

        [Fact]
        public void MunRepeatedRefreshReusesTerrainAndPreservesForecastExactly()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var cache = new TargetAwareTerrainCache();
            int calls = 0;
            double Terrain(double lat, double lon)
            {
                calls++;
                return source.TargetTerrainASL;
            }
            TargetAwareAirlessPlanner Run(long sequence, int budget)
            {
                var p = new TargetAwareAirlessPlanner(snapshot, 1, sequence,
                    this, Terrain, 200, 32, budget, 1, false, cache);
                p.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
                DriveUntilSettled(p);
                Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, p.Stage);
                p.BeginCoarseWorker();
                p.SetCoarseOutputs(p.RunCoarse());
                DriveUntilSettled(p);
                if (p.Stage == TargetAwarePlannerStage.ReadyRefinement)
                {
                    p.BeginRefinementWorker();
                    p.SetRefinement(p.RunRefinement());
                    DriveUntilSettled(p);
                }
                Assert.True(p.Stage == TargetAwarePlannerStage.Complete,
                    p.Failure ?? p.Stage.ToString());
                Assert.True(p.TerminalTouchdownValidated);
                Assert.Equal(p.TerrainSampleCount, p.TerrainCacheHits + p.TerrainQueryCount);
                return p;
            }
            var first = Run(1, 1536);
            Assert.Equal(calls, first.TerrainQueryCount);
            Assert.True(first.TerrainCacheHits > 0);
            int before = calls;
            var second = Run(2, 1536);
            Assert.Equal(calls - before, second.TerrainQueryCount);
            Assert.True(second.TerrainQueryCount < first.TerrainQueryCount);
            Assert.True(second.TerrainCacheHits > first.TerrainCacheHits);
            Assert.Equal(first.SelectedOutput.BrakeUT, second.SelectedOutput.BrakeUT);
            Assert.Equal(first.NominalTerminalTerrain.Contact.UT, second.NominalTerminalTerrain.Contact.UT);
            Assert.Equal(first.NominalTerminalTerrain.Contact.Position.x, second.NominalTerminalTerrain.Contact.Position.x);
            Assert.Equal(first.NominalTerminalTerrain.Contact.Position.y, second.NominalTerminalTerrain.Contact.Position.y);
            Assert.Equal(first.NominalTerminalTerrain.Contact.Position.z, second.NominalTerminalTerrain.Contact.Position.z);
            _output.WriteLine($"PQS calls cold={first.TerrainQueryCount}, warm={second.TerrainQueryCount}; " +
                $"cache hits cold={first.TerrainCacheHits}, warm={second.TerrainCacheHits}; " +
                $"terrain samples={first.TerrainSampleCount}; identical brake and touchdown");
        }

        private void DriveThroughRefinement(TargetAwareAirlessPlanner planner)
        {
            DriveUntil(planner, TargetAwarePlannerStage.ReadyRefinement);
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            DriveUntilSettled(planner);
            _output.WriteLine($"After refinement: queries={planner.TerrainQueryCount}");
        }

        private void DriveUntil(TargetAwareAirlessPlanner planner, TargetAwarePlannerStage destination)
        {
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(planner.Snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            _output.WriteLine($"After ballistic: queries={planner.TerrainQueryCount}");
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            _output.WriteLine($"After coarse: queries={planner.TerrainQueryCount}");
            Assert.True(planner.Stage == destination, planner.Failure ?? planner.Stage.ToString());
        }

        private static void DriveUntilSettled(TargetAwareAirlessPlanner planner,
            bool includeTerminal = true)
        {
            for (int i = 0; i < 200 && (planner.Stage == TargetAwarePlannerStage.ResolveBallistic ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveCoarse ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveRefinement ||
                                          planner.Stage == TargetAwarePlannerStage.ReadyPolicyEscalation ||
                                          planner.Stage == TargetAwarePlannerStage.ResolvePolicyEscalation ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveDirectForecast ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveSelectedCoast ||
                                          (includeTerminal && planner.Stage == TargetAwarePlannerStage.ReadyTerminal) ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveTerminalNominal ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveTerminalDelayed); ++i)
            {
                if (planner.Stage == TargetAwarePlannerStage.ReadyPolicyEscalation)
                {
                    planner.BeginPolicyEscalation();
                    planner.SetPolicyEscalation(planner.RunPolicyEscalation());
                    continue;
                }
                if (planner.Stage == TargetAwarePlannerStage.ReadyTerminal)
                {
                    planner.BeginTerminalWorker();
                    planner.SetTerminalEnvelope(planner.RunTerminalEnvelope());
                    continue;
                }
                int before = planner.TerrainQueryCount;
                int samplesBefore = planner.TerrainSampleCount;
                planner.AdvanceTerrain();
                Assert.InRange(planner.TerrainQueryCount - before, 0, 32);
                Assert.InRange(planner.TerrainSampleCount - samplesBefore, 0, 32);
            }
        }
    }
}
