using System;
using KSP.Localization;
using ModuleWheels;
using MuMech.Landing;
using UnityEngine;

namespace MuMech;

public class MechJebModuleLandingAutopilot : AutopilotModule
{
	private bool _deployedGears;

	public bool LandAtTarget;

	public bool LandingTraceEnabled = true;

	private double _nextLandingTraceUT;

	private long _lastTracedPredictionVersion = -1L;

	private string _lastTracedStep;

	[Persistent(pass = 7)]
	public readonly EditableDouble TouchdownSpeed = 0.5;

	[Persistent(pass = 7)]
	public bool DeployGears = true;

	[Persistent(pass = 7)]
	public readonly EditableInt LimitGearsStage = 0;

	[Persistent(pass = 7)]
	public bool DeployChutes = true;

	[Persistent(pass = 7)]
	public readonly EditableInt LimitChutesStage = 0;

	[Persistent(pass = 7)]
	public bool RCSAdjustment = true;

	private ParachutePlan _parachutePlan;

	private MechJebModuleLandingPredictions _predictor;

	public IDescentSpeedPolicy DescentSpeedPolicy;

	private double _vesselAverageDrag;

	public ReentrySimulation.Result Prediction => _predictor.Result;

	public long PredictionVersion => _predictor.ResultVersion;

	private ReentrySimulation.Result _errorPrediction => _predictor.GetErrorResult();

	public bool PredictionReady
	{
		get
		{
			if (Prediction == null)
			{
				return false;
			}
			return Prediction.Outcome == ReentrySimulation.Outcome.LANDED;
		}
	}

	private bool _errorPredictionReady
	{
		get
		{
			ReentrySimulation.Result errorPrediction = _errorPrediction;
			if (errorPrediction != null)
			{
				return errorPrediction.Outcome == ReentrySimulation.Outcome.LANDED;
			}
			return false;
		}
	}

	private double _landingAltitude
	{
		get
		{
			if (!PredictionReady)
			{
				return 0.0;
			}
			double num = Prediction.Body.TerrainAltitude(Prediction.EndPosition.Latitude, Prediction.EndPosition.Longitude, false);
			if (num != Prediction.EndASL)
			{
				Prediction.EndASL = num;
			}
			return Prediction.EndASL;
		}
	}

	public Vector3d LandingSite => base.MainBody.GetWorldSurfacePosition(Prediction.EndPosition.Latitude, Prediction.EndPosition.Longitude, _landingAltitude) - base.MainBody.position;

	private Vector3d _rotatedLandingSite => Prediction.WorldEndPosition();

	public double LastCourseCorrectionDownrangeError { get; private set; } = double.NaN;


	public double LastCourseCorrectionLongBias { get; private set; }

	public double LastCourseCorrectionHandoffLimit { get; private set; }

	public MechJebModuleLandingAutopilot(MechJebCore core)
		: base(core)
	{
	}

	public override void OnStart(StartState state)
	{
		_predictor = Core.GetComputerModule<MechJebModuleLandingPredictions>();
	}

	public void LandAtPositionTarget(object controller)
	{
		LandAtTarget = true;
		Users.Add(controller);
		_predictor.Users.Add(this);
		base.Vessel.RemoveAllManeuverNodes();
		_deployedGears = false;
		_parachutePlan = new ParachutePlan(this);
		_parachutePlan.StartPlanning();
		if (base.Orbit.PeA < 0.0)
		{
			SetStep(new CourseCorrection(Core));
		}
		else if (UseLowDeorbitStrategy())
		{
			SetStep(new PlaneChange(Core));
		}
		else
		{
			SetStep(new DeorbitBurn(Core));
		}
	}

	public void LandUntargeted(object controller)
	{
		LandAtTarget = false;
		Users.Add(controller);
		_deployedGears = false;
		_parachutePlan = new ParachutePlan(this);
		_parachutePlan.StartPlanning();
		SetStep(new UntargetedDeorbit(Core));
	}

	public void StopLanding()
	{
		Users.Clear();
		Core.Thrust.ThrustOff();
		Core.Thrust.Users.Remove(this);
		if (Core.Landing.RCSAdjustment)
		{
			Core.RCS.Enabled = false;
		}
		SetStep(null);
	}

	public override void Drive(FlightCtrlState s)
	{
		if (base.Active)
		{
			DescentSpeedPolicy = PickDescentSpeedPolicy();
			_predictor.descentSpeedPolicy = PickDescentSpeedPolicy();
			_predictor.decelEndAltitudeASL = DecelerationEndAltitude();
			_predictor.parachuteSemiDeployMultiplier = _parachutePlan.Multiplier;
			double num = Math.Min(base.VesselState.AltitudeBottom, Math.Min(base.VesselState.AltitudeASL, base.VesselState.AltitudeTrue));
			if (DeployGears && !_deployedGears && num < 1000.0)
			{
				DeployLandingGears();
			}
			base.Drive(s);
		}
	}

	public override void OnFixedUpdate()
	{
		_vesselAverageDrag = VesselAverageDrag();
		base.OnFixedUpdate();
		DeployParachutes();
		TraceLandingState();
	}

	private void TraceLandingState()
	{
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		if (!LandingTraceEnabled || !base.Active)
		{
			return;
		}
		string name = base.CurrentStep.GetType().Name;
		if (_lastTracedStep != name)
		{
			TraceLanding("step=" + name + " status=\"" + base.Status + "\"");
			_lastTracedStep = name;
		}
		if (PredictionVersion != _lastTracedPredictionVersion)
		{
			_lastTracedPredictionVersion = PredictionVersion;
			if (Prediction != null)
			{
				double num = (PredictionReady ? Vector3d.Distance(Core.Target.GetPositionTargetPosition(), LandingSite) : double.NaN);
				TraceLanding($"prediction version={PredictionVersion} outcome={Prediction.Outcome} inputUT={Prediction.InputUT:F2} endUT={Prediction.EndUT:F2} " + $"lat={Prediction.EndPosition.Latitude:F6} lon={Prediction.EndPosition.Longitude:F6} endASL={Prediction.EndASL:F1} targetError={num:F1}");
			}
		}
		double num2 = ((name == "DecelerationBurn" || name == "CourseCorrection") ? 0.1 : 1.0);
		if (!(base.VesselState.Time < _nextLandingTraceUT))
		{
			_nextLandingTraceUT = base.VesselState.Time + num2;
			TraceLanding($"state step={name} ut={base.VesselState.Time:F2} warp={TimeWarp.CurrentRate:F1} alt={base.VesselState.AltitudeASL:F1} " + $"surfaceSpeed={base.VesselState.SpeedSurface:F2} verticalSpeed={base.VesselState.SpeedVertical:F2} horizontalSpeed={base.VesselState.SpeedSurfaceHorizontal:F2} " + $"throttle={Core.Thrust.TargetThrottle:F3} thrustAccel={base.VesselState.CurrentThrustAcceleration:F3} maxAccel={base.VesselState.LimitedMaxThrustAcceleration:F3} " + $"apA={base.Orbit.ApA:F1} peA={base.Orbit.PeA:F1} attitudeError={Core.Attitude.attitudeAngleFromTarget():F2} " + $"predictionVersion={PredictionVersion}{base.CurrentStep.TraceDetails}");
		}
	}

	public void TraceLanding(string message)
	{
		if (LandingTraceEnabled)
		{
			Debug.Log((object)("[MechJebLandingTrace] " + message));
		}
	}

	protected override void OnModuleEnabled()
	{
		Core.Attitude.Users.Add(this);
		Core.Thrust.Users.Add(this);
	}

	protected override void OnModuleDisabled()
	{
		Core.Attitude.attitudeDeactivate();
		_predictor.Users.Remove(this);
		_predictor.descentSpeedPolicy = null;
		Core.Thrust.ThrustOff();
		Core.Thrust.Users.Remove(this);
		if (Core.Landing.RCSAdjustment)
		{
			Core.RCS.Enabled = false;
		}
		SetStep(null);
	}

	public Vector3d ComputeCourseCorrection(bool allowPrograde, double downrangeLongBias = 0.0, double downrangeHandoffLimit = 0.0)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		//IL_009f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_00af: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0204: Unknown result type (might be due to invalid IL or missing references)
		//IL_0209: Unknown result type (might be due to invalid IL or missing references)
		//IL_0244: Unknown result type (might be due to invalid IL or missing references)
		//IL_0249: Unknown result type (might be due to invalid IL or missing references)
		//IL_024e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0253: Unknown result type (might be due to invalid IL or missing references)
		//IL_0255: Unknown result type (might be due to invalid IL or missing references)
		//IL_025a: Unknown result type (might be due to invalid IL or missing references)
		//IL_025f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0264: Unknown result type (might be due to invalid IL or missing references)
		//IL_0266: Unknown result type (might be due to invalid IL or missing references)
		//IL_0268: Unknown result type (might be due to invalid IL or missing references)
		//IL_0269: Unknown result type (might be due to invalid IL or missing references)
		//IL_026e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0270: Unknown result type (might be due to invalid IL or missing references)
		//IL_0271: Unknown result type (might be due to invalid IL or missing references)
		//IL_0273: Unknown result type (might be due to invalid IL or missing references)
		//IL_0278: Unknown result type (might be due to invalid IL or missing references)
		//IL_0324: Unknown result type (might be due to invalid IL or missing references)
		//IL_0329: Unknown result type (might be due to invalid IL or missing references)
		//IL_032e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0333: Unknown result type (might be due to invalid IL or missing references)
		//IL_0290: Unknown result type (might be due to invalid IL or missing references)
		//IL_0295: Unknown result type (might be due to invalid IL or missing references)
		//IL_029d: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_02de: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0301: Unknown result type (might be due to invalid IL or missing references)
		//IL_030e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0313: Unknown result type (might be due to invalid IL or missing references)
		//IL_0318: Unknown result type (might be due to invalid IL or missing references)
		//IL_031d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0169: Unknown result type (might be due to invalid IL or missing references)
		//IL_016e: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0174: Unknown result type (might be due to invalid IL or missing references)
		//IL_0176: Unknown result type (might be due to invalid IL or missing references)
		//IL_0177: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		//IL_017e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0183: Unknown result type (might be due to invalid IL or missing references)
		//IL_0188: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018b: Unknown result type (might be due to invalid IL or missing references)
		//IL_018d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0192: Unknown result type (might be due to invalid IL or missing references)
		//IL_0198: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_03e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_03e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_03f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0412: Unknown result type (might be due to invalid IL or missing references)
		//IL_0414: Unknown result type (might be due to invalid IL or missing references)
		//IL_041b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0420: Unknown result type (might be due to invalid IL or missing references)
		//IL_0434: Unknown result type (might be due to invalid IL or missing references)
		//IL_0436: Unknown result type (might be due to invalid IL or missing references)
		//IL_043b: Unknown result type (might be due to invalid IL or missing references)
		//IL_043d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0444: Unknown result type (might be due to invalid IL or missing references)
		//IL_0446: Unknown result type (might be due to invalid IL or missing references)
		//IL_044b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0455: Unknown result type (might be due to invalid IL or missing references)
		//IL_045a: Unknown result type (might be due to invalid IL or missing references)
		//IL_045f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0371: Unknown result type (might be due to invalid IL or missing references)
		//IL_0376: Unknown result type (might be due to invalid IL or missing references)
		//IL_0378: Unknown result type (might be due to invalid IL or missing references)
		//IL_037a: Unknown result type (might be due to invalid IL or missing references)
		//IL_03bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_03cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d0: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = _rotatedLandingSite - base.MainBody.position;
		double num = base.MainBody.Radius + DecelerationEndAltitude() - 100.0;
		if (num > base.Orbit.ApR || base.Vessel.LandedOrSplashed)
		{
			StopLanding();
		}
		Vector3d val2 = base.Orbit.WorldBCIPositionAtUT((base.Orbit.PeR < num) ? base.Orbit.NextTimeOfRadius(base.VesselState.Time, num) : base.Orbit.NextPeriapsisTime(base.VesselState.Time));
		Quaternion val3 = Quaternion.FromToRotation(Vector3d.op_Implicit(val2), Vector3d.op_Implicit(val));
		Vector3d[] array = (Vector3d[])(object)new Vector3d[3]
		{
			((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized,
			base.VesselState.RadialPlusSurface,
			base.VesselState.NormalPlusSurface
		};
		Vector3d[] array2 = (Vector3d[])(object)new Vector3d[3];
		for (int i = 0; i < 3; i++)
		{
			Orbit val4 = base.Orbit.PerturbedOrbit(base.VesselState.Time, 1.0 * array[i]);
			double ut = ((val4.PeR < num) ? val4.NextTimeOfRadius(base.VesselState.Time, num) : val4.NextPeriapsisTime(base.VesselState.Time));
			Vector3d val5 = val4.WorldBCIPositionAtUT(ut) - val2;
			val5 = Vector3d.op_Implicit(val3 * Vector3d.op_Implicit(val5));
			val5 = Vector3d.Exclude(val, val5);
			array2[i] = val5 / 1.0;
		}
		Vector3d val6 = base.MainBody.GetWorldSurfacePosition((double)Core.Target.targetLatitude, (double)Core.Target.targetLongitude, 0.0) - base.MainBody.position;
		val6 = Vector3d.op_Implicit(Quaternion.AngleAxis((float)(360.0 * (Prediction.EndUT - base.VesselState.Time) / base.MainBody.rotationPeriod), Vector3d.op_Implicit(((Vector3d)(ref base.MainBody.angularVelocity)).normalized)) * Vector3d.op_Implicit(val6));
		Vector3d val7 = val6 - val;
		val7 = Vector3d.Exclude(val, val7);
		Vector3d val9;
		Vector3d val10;
		if (allowPrograde)
		{
			Vector3d val8 = ((Vector3d)(ref array2[0])).magnitude * array[0] + (double)Math.Sign(Vector3d.Dot(array2[0], array2[1])) * ((Vector3d)(ref array2[1])).magnitude * array[1];
			val9 = ((Vector3d)(ref val8)).normalized;
			val10 = Vector3d.Dot(val9, array[0]) * array2[0] + Vector3d.Dot(val9, array[1]) * array2[1];
		}
		else
		{
			val9 = array[1];
			val10 = array2[1];
		}
		LastCourseCorrectionDownrangeError = double.NaN;
		LastCourseCorrectionLongBias = 0.0;
		LastCourseCorrectionHandoffLimit = downrangeHandoffLimit;
		if (allowPrograde && ((Vector3d)(ref val10)).sqrMagnitude > 1E-12)
		{
			Vector3d normalized = ((Vector3d)(ref val10)).normalized;
			double num3 = (LastCourseCorrectionDownrangeError = Vector3d.Dot(val7, normalized));
			if (downrangeLongBias > 0.0 && downrangeHandoffLimit > 0.0 && num3 >= 0.0 - downrangeHandoffLimit)
			{
				double num4 = Math.Max(0.0, num3 + downrangeLongBias);
				val7 += (num4 - num3) * normalized;
				LastCourseCorrectionLongBias = downrangeLongBias;
			}
		}
		Matrix2X2 matrix2X = new Matrix2X2(((Vector3d)(ref val10)).sqrMagnitude, Vector3d.Dot(val10, array2[2]), Vector3d.Dot(val10, array2[2]), ((Vector3d)(ref array2[2])).sqrMagnitude);
		Vector2d val11 = default(Vector2d);
		((Vector2d)(ref val11))._002Ector(Vector3d.Dot(val7, val10), Vector3d.Dot(val7, array2[2]));
		Vector2d val12 = matrix2X.Inverse() * val11;
		return val12.x * val9 + val12.y * array[2];
	}

	public void ControlParachutes()
	{
		if (_parachutePlan == null)
		{
			_parachutePlan = new ParachutePlan(this);
		}
		if (!ParachutesDeployable())
		{
			_predictor.runErrorSimulations = false;
			_parachutePlan.ClearData();
			return;
		}
		_predictor.runErrorSimulations = true;
		if (_errorPredictionReady && !double.IsNaN(_errorPrediction.ParachuteMultiplier))
		{
			_parachutePlan.AddResult(_errorPrediction);
		}
		if (PredictionReady && !double.IsNaN(Prediction.ParachuteMultiplier))
		{
			_parachutePlan.AddResult(Prediction);
		}
	}

	private void DeployParachutes()
	{
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		if (!base.VesselState.MainBody.atmosphere || !DeployChutes)
		{
			return;
		}
		for (int i = 0; i < base.VesselState.Parachutes.Count; i++)
		{
			ModuleParachute val = base.VesselState.Parachutes[i];
			double landingAltitude = _landingAltitude;
			double num = (double)val.deployAltitude * _parachutePlan.Multiplier + landingAltitude;
			if (((PartModule)val).part.inverseStage >= (int)LimitChutesStage && (int)val.deploymentState == 0 && num > base.VesselState.AltitudeASL && (int)val.deploymentSafeState == 0)
			{
				val.Deploy();
			}
		}
	}

	public bool ParachutesDeployable()
	{
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		if (!base.VesselState.MainBody.atmosphere)
		{
			return false;
		}
		if (!DeployChutes)
		{
			return false;
		}
		for (int i = 0; i < base.VesselState.Parachutes.Count; i++)
		{
			ModuleParachute val = base.VesselState.Parachutes[i];
			if (Math.Max(((PartModule)val).part.inverseStage, 0) >= (int)LimitChutesStage && (int)val.deploymentState == 0)
			{
				return true;
			}
		}
		return false;
	}

	private bool ParachutesDeployed()
	{
		return base.VesselState.ParachuteDeployed;
	}

	private void DeployLandingGears()
	{
		for (int i = 0; i < base.Vessel.parts.Count; i++)
		{
			Part val = base.Vessel.parts[i];
			if (!val.HasModule<ModuleWheelDeployment>() || Math.Max(val.inverseStage, 0) < (int)LimitGearsStage)
			{
				continue;
			}
			foreach (ModuleWheelDeployment item in val.FindModulesImplementing<ModuleWheelDeployment>())
			{
				if (item.fsm.CurrentState == item.st_retracted || item.fsm.CurrentState == item.st_retracting)
				{
					item.EventToggle();
				}
			}
		}
		_deployedGears = true;
	}

	private IDescentSpeedPolicy PickDescentSpeedPolicy()
	{
		if (UseAtmosphereToBrake())
		{
			return new PoweredCoastDescentSpeedPolicy(base.MainBody.Radius + DecelerationEndAltitude(), base.MainBody.GeeASL * 9.81, base.VesselState.LimitedMaxThrustAcceleration);
		}
		return new SafeDescentSpeedPolicy(base.MainBody.Radius + DecelerationEndAltitude(), base.MainBody.GeeASL * 9.81, base.VesselState.LimitedMaxThrustAcceleration);
	}

	public double DecelerationEndAltitude()
	{
		if (!UseAtmosphereToBrake())
		{
			return 200.0 + _landingAltitude;
		}
		double num = base.MainBody.DragLength(_landingAltitude, _vesselAverageDrag + ParachuteAddedDragCoef(), base.VesselState.Mass);
		return 1.1 * num + _landingAltitude;
	}

	public bool UseAtmosphereToBrake()
	{
		double num = base.MainBody.DragLength(_landingAltitude, _vesselAverageDrag + ParachuteAddedDragCoef(), base.VesselState.Mass);
		if (base.MainBody.RealMaxAtmosphereAltitude() > 0.0)
		{
			return num < 0.7 * base.MainBody.RealMaxAtmosphereAltitude();
		}
		return false;
	}

	private double VesselAverageDrag()
	{
		float num = 0f;
		for (int i = 0; i < base.Vessel.parts.Count; i++)
		{
			Part val = base.Vessel.parts[i];
			if (!val.DragCubes.None && !val.ShieldedFromAirstream)
			{
				float num2 = 0f;
				for (int j = 0; j < 6; j++)
				{
					num2 = val.DragCubes.WeightedDrag[j] * val.DragCubes.AreaOccluded[j];
				}
				num += num2 / 6f;
			}
		}
		return num * PhysicsGlobals.DragCubeMultiplier;
	}

	private double ParachuteAddedDragCoef()
	{
		double num = 0.0;
		if (!base.VesselState.MainBody.atmosphere || !DeployChutes)
		{
			return num * (double)PhysicsGlobals.DragCubeMultiplier;
		}
		for (int i = 0; i < base.VesselState.Parachutes.Count; i++)
		{
			ModuleParachute val = base.VesselState.Parachutes[i];
			if (((PartModule)val).part.inverseStage < (int)LimitChutesStage)
			{
				continue;
			}
			float num2 = 0f;
			for (int j = 0; j < ((PartModule)val).part.DragCubes.Cubes.Count; j++)
			{
				DragCube val2 = ((PartModule)val).part.DragCubes.Cubes[j];
				if (!(val2.Name != "DEPLOYED"))
				{
					for (int k = 0; k < 6; k++)
					{
						num2 = Mathf.Max(num2, ((PartModule)val).part.DragCubes.WeightedDrag[k] - val2.Weight * val2.Drag[k]);
					}
				}
			}
			num += (double)num2;
		}
		return num * (double)PhysicsGlobals.DragCubeMultiplier;
	}

	private bool UseLowDeorbitStrategy()
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		if (base.MainBody.atmosphere)
		{
			return false;
		}
		Vector3d val = base.Orbit.WorldOrbitalVelocityAtUT(base.Orbit.NextPeriapsisTime(base.VesselState.Time));
		double num = Math.Pow(((Vector3d)(ref val)).magnitude, 2.0) / (2.0 * base.VesselState.LimitedMaxThrustAcceleration);
		return base.Orbit.PeA < 2.0 * num + base.MainBody.Radius / 4.0;
	}

	public double MaxAllowedSpeed()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		return DescentSpeedPolicy.MaxAllowedSpeed(base.VesselState.CoM - base.MainBody.position, base.VesselState.SurfaceVelocity);
	}

	public double MaxAllowedSpeedAfterDt(double dt)
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		return DescentSpeedPolicy.MaxAllowedSpeed(base.VesselState.CoM + base.VesselState.OrbitalVelocity * dt - base.MainBody.position, base.VesselState.SurfaceVelocity + dt * base.VesselState.GravityForce);
	}

	[ValueInfoItem("#MechJeb_ParachuteControlInfo", InfoItem.Category.Misc, showInEditor = false)]
	public string ParachuteControlInfo()
	{
		if (!ParachutesDeployable())
		{
			return "N/A";
		}
		return string.Concat(Localizer.Format("#MechJeb_ChuteMultiplier", new string[1] { _parachutePlan.Multiplier.ToString("F7") }) + Localizer.Format("#MechJeb_MultiplierQuality", new string[1] { _parachutePlan.MultiplierQuality.ToString("F1") }), Localizer.Format("#MechJeb_Usingpredictions", new object[1] { _parachutePlan.MultiplierDataAmount }));
	}

	public void SetTargetKSC(MechJebCore controller)
	{
		Users.Add(controller);
		Core.Target.SetPositionTarget(base.MainBody, MechJebModuleLandingGuidance.LandingSites[0].Latitude, MechJebModuleLandingGuidance.LandingSites[0].Longitude);
	}
}
