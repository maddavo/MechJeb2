using System;
using System.Collections.Generic;
using Smooth.Delegates;
using Smooth.Dispose;
using Smooth.Pools;
using UnityEngine;

namespace MuMech;

public class ReentrySimulation
{
	public enum Outcome
	{
		LANDED,
		AEROBRAKED,
		TIMED_OUT,
		NO_REENTRY,
		ERROR
	}

	public struct Prediction
	{
		public double FirstDrag;

		public double FirstLift;

		public double SpeedOfSound;

		public double Mach;

		public double DynamicPressurekPa;
	}

	public class Result
	{
		public double Maxdt;

		public int Steps;

		public double TimeToComplete;

		public ulong ID;

		public Outcome Outcome;

		public Exception Exception;

		public CelestialBody Body;

		public ReferenceFrame ReferenceFrame;

		public double EndUT;

		public AbsoluteVector StartPosition;

		public AbsoluteVector EndPosition;

		public AbsoluteVector EndVelocity;

		public bool AeroBrake;

		public double AeroBrakeUT;

		public AbsoluteVector AeroBrakePosition;

		public AbsoluteVector AeroBrakeVelocity;

		public double EndASL;

		public List<AbsoluteVector> Trajectory;

		public double MaxDragGees;

		public double DeltaVExpended;

		public bool MultiplierHasError;

		public double ParachuteMultiplier;

		public Orbit InputInitialOrbit;

		public double InputUT;

		public List<SimulatedParachute> InputParachuteList;

		public IDescentSpeedPolicy InputDescentSpeedPolicy;

		public double InputDecelEndAltitudeASL;

		public double InputMaxThrustAccel;

		public double InputParachuteSemiDeployMultiplier;

		public double InputProbableLandingSiteASL;

		public bool InputMultiplierHasError;

		public double InputDT;

		public string DebugLog;

		private static readonly Pool<Result> _pool = new Pool<Result>((DelegateFunc<Result>)Create, (DelegateAction<Result>)Reset);

		public Prediction Prediction;

		private static Result Create()
		{
			return new Result();
		}

		public void Release()
		{
			if (Trajectory != null)
			{
				ListPool<AbsoluteVector>.Instance.Release(Trajectory);
			}
			Exception = null;
			_pool.Release(this);
		}

		private static void Reset(Result obj)
		{
			obj.AeroBrake = false;
		}

		public static Result Borrow()
		{
			return _pool.Borrow();
		}

		public Vector3d RelativeEndPosition()
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0011: Unknown result type (might be due to invalid IL or missing references)
			return WorldEndPosition() - Body.position;
		}

		public Vector3d WorldEndPosition()
		{
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			return ReferenceFrame.WorldPositionAtCurrentTime(EndPosition);
		}

		private Vector3d WorldEndVelocity()
		{
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			return ReferenceFrame.WorldVelocityAtCurrentTime(EndVelocity);
		}

		public Orbit EndOrbit()
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_0007: Unknown result type (might be due to invalid IL or missing references)
			return MuUtils.OrbitFromStateVectors(WorldEndPosition(), WorldEndVelocity(), Body, EndUT);
		}

		public Vector3d WorldAeroBrakePosition()
		{
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			return ReferenceFrame.WorldPositionAtCurrentTime(AeroBrakePosition);
		}

		public Vector3d WorldAeroBrakeVelocity()
		{
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			return ReferenceFrame.WorldVelocityAtCurrentTime(AeroBrakeVelocity);
		}

		public Orbit AeroBrakeOrbit()
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_0007: Unknown result type (might be due to invalid IL or missing references)
			return MuUtils.OrbitFromStateVectors(WorldAeroBrakePosition(), WorldAeroBrakeVelocity(), Body, EndUT);
		}

		public Disposable<List<Vector3d>> WorldTrajectory(double timeStep, bool world = true)
		{
			//IL_004e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0035: Unknown result type (might be due to invalid IL or missing references)
			//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
			//IL_0096: Unknown result type (might be due to invalid IL or missing references)
			Disposable<List<Vector3d>> val = ListPool<Vector3d>.Instance.BorrowDisposable();
			if (Trajectory.Count == 0)
			{
				return val;
			}
			val.value.Add(world ? ReferenceFrame.WorldPositionAtCurrentTime(Trajectory[0]) : ReferenceFrame.BodyPositionAtCurrentTime(Trajectory[0]));
			double uT = Trajectory[0].UT;
			for (int i = 0; i < Trajectory.Count; i++)
			{
				AbsoluteVector absolute = Trajectory[i];
				if (absolute.UT > uT + timeStep)
				{
					val.value.Add(world ? ReferenceFrame.WorldPositionAtCurrentTime(absolute) : ReferenceFrame.BodyPositionAtCurrentTime(absolute));
					uT = absolute.UT;
				}
			}
			return val;
		}

		public double GetOvershoot(EditableAngle targetLatitude, EditableAngle targetLongitude)
		{
			//IL_0025: Unknown result type (might be due to invalid IL or missing references)
			//IL_0030: Unknown result type (might be due to invalid IL or missing references)
			//IL_0035: Unknown result type (might be due to invalid IL or missing references)
			//IL_0055: Unknown result type (might be due to invalid IL or missing references)
			//IL_0060: Unknown result type (might be due to invalid IL or missing references)
			//IL_0065: Unknown result type (might be due to invalid IL or missing references)
			//IL_006a: Unknown result type (might be due to invalid IL or missing references)
			//IL_0090: Unknown result type (might be due to invalid IL or missing references)
			//IL_009b: Unknown result type (might be due to invalid IL or missing references)
			//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
			//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
			//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
			//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
			//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
			//IL_00af: Unknown result type (might be due to invalid IL or missing references)
			//IL_00b4: Unknown result type (might be due to invalid IL or missing references)
			//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
			//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
			//IL_00da: Unknown result type (might be due to invalid IL or missing references)
			//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
			//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ef: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f1: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f4: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f9: Unknown result type (might be due to invalid IL or missing references)
			//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
			//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
			//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
			Vector3d val = Body.GetWorldSurfacePosition(EndPosition.Latitude, EndPosition.Longitude, 0.0) - Body.position;
			Vector3d val2 = Body.GetWorldSurfacePosition((double)targetLatitude, (double)targetLongitude, 0.0) - Body.position;
			Vector3d val3 = Body.GetWorldSurfacePosition(StartPosition.Latitude, StartPosition.Longitude, 0.0) - Body.position;
			Vector3d val4 = val2 - val3;
			Vector3d val5 = Vector3d.Cross(val4, Vector3d.up);
			if (val5 == Vector3d.up)
			{
				val5 = Vector3d.Cross(val4, Vector3d.forward);
			}
			Vector3d val6 = MathExtensions.ProjectOnPlane(planeNormal: Vector3d.Cross(val4, val5), vector: (val - val2).ProjectOnPlane(val5));
			Vector3d val7 = val4 + val6;
			return ((Vector3d)(ref val7)).magnitude - ((Vector3d)(ref val4)).magnitude;
		}

		public override string ToString()
		{
			//IL_019e: Unknown result type (might be due to invalid IL or missing references)
			//IL_01a3: Unknown result type (might be due to invalid IL or missing references)
			//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
			//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
			string text = "Simulation result\n{";
			text += "Inputs:\n{";
			if (InputInitialOrbit != null)
			{
				text = text + "\n input_initialOrbit: " + (object)InputInitialOrbit;
			}
			text = text + "\n input_UT: " + InputUT;
			if (InputDescentSpeedPolicy != null)
			{
				text = text + "\n input_descentSpeedPolicy: " + InputDescentSpeedPolicy;
			}
			text = text + "\n input_decelEndAltitudeASL: " + InputDecelEndAltitudeASL;
			text = text + "\n input_maxThrustAccel: " + InputMaxThrustAccel;
			text = text + "\n input_parachuteSemiDeployMultiplier: " + InputParachuteSemiDeployMultiplier;
			text = text + "\n input_probableLandingSiteASL: " + InputProbableLandingSiteASL;
			text = text + "\n input_multiplierHasError: " + InputMultiplierHasError;
			text = text + "\n input_dt: " + InputDT;
			text += "\n}";
			text = text + "\nid: " + ID;
			text = text + "\noutcome: " + Outcome;
			text = text + "\nmaxdt: " + Maxdt;
			text = text + "\ntimeToComplete: " + TimeToComplete;
			text = text + "\nendUT: " + EndUT;
			Vector3d val;
			if (ReferenceFrame != null)
			{
				string text2 = text;
				val = ReferenceFrame.WorldPositionAtCurrentTime(StartPosition);
				text = text2 + "\nstartPosition: " + ((object)(Vector3d)(ref val)).ToString();
			}
			if (ReferenceFrame != null)
			{
				string text3 = text;
				val = ReferenceFrame.WorldPositionAtCurrentTime(EndPosition);
				text = text3 + "\nendPosition: " + ((object)(Vector3d)(ref val)).ToString();
			}
			text = text + "\nendASL: " + EndASL;
			text = text + "\nendVelocity: " + EndVelocity.Longitude + "," + EndVelocity.Latitude + "," + EndVelocity.Radius;
			text = text + "\nmaxDragGees: " + MaxDragGees;
			text = text + "\ndeltaVExpended: " + DeltaVExpended;
			text = text + "\nmultiplierHasError: " + MultiplierHasError;
			text = text + "\nparachuteMultiplier: " + ParachuteMultiplier;
			return text + "\n}";
		}
	}

	public class SimCurves
	{
		private static readonly Pool<SimCurves> _simcurvesPool = new Pool<SimCurves>((DelegateFunc<SimCurves>)CreateSimCurve, (DelegateAction<SimCurves>)ResetSimCurve);

		private bool _loaded;

		private CelestialBody _body;

		private FloatCurve _liftCurve { get; set; }

		public FloatCurve LiftMachCurve { get; private set; }

		private FloatCurve _dragCurve { get; set; }

		private FloatCurve _dragMachCurve { get; set; }

		private FloatCurve _dragCurveTail { get; set; }

		private FloatCurve _dragCurveSurface { get; set; }

		private FloatCurve _dragCurveTip { get; set; }

		private FloatCurve _dragCurveCd { get; set; }

		private FloatCurve _dragCurveCdPower { get; set; }

		private FloatCurve _dragCurveMultiplier { get; set; }

		public FloatCurve AtmospherePressureCurve { get; private set; }

		public FloatCurve AtmosphereTemperatureSunMultCurve { get; private set; }

		public FloatCurve LatitudeTemperatureBiasCurve { get; private set; }

		public FloatCurve LatitudeTemperatureSunMultCurve { get; private set; }

		public FloatCurve AxialTemperatureSunMultCurve { get; private set; }

		public FloatCurve AtmosphereTemperatureCurve { get; private set; }

		public FloatCurve DragCurvePseudoReynolds { get; private set; }

		public double SpaceTemperature { get; private set; }

		private SimCurves()
		{
		}

		private static SimCurves CreateSimCurve()
		{
			return new SimCurves();
		}

		public void Release()
		{
			_simcurvesPool.Release(this);
		}

		private static void ResetSimCurve(SimCurves obj)
		{
		}

		public static SimCurves Borrow(CelestialBody newBody)
		{
			SimCurves simCurves = _simcurvesPool.Borrow();
			simCurves.Setup(newBody);
			return simCurves;
		}

		private void Setup(CelestialBody newBody)
		{
			//IL_001b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0025: Expected O, but got Unknown
			//IL_0035: Unknown result type (might be due to invalid IL or missing references)
			//IL_003f: Expected O, but got Unknown
			//IL_004f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0059: Expected O, but got Unknown
			//IL_006e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0078: Expected O, but got Unknown
			//IL_008d: Unknown result type (might be due to invalid IL or missing references)
			//IL_0097: Expected O, but got Unknown
			//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
			//IL_00b6: Expected O, but got Unknown
			//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d5: Expected O, but got Unknown
			//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f4: Expected O, but got Unknown
			//IL_0109: Unknown result type (might be due to invalid IL or missing references)
			//IL_0113: Expected O, but got Unknown
			//IL_0128: Unknown result type (might be due to invalid IL or missing references)
			//IL_0132: Expected O, but got Unknown
			//IL_0142: Unknown result type (might be due to invalid IL or missing references)
			//IL_014c: Expected O, but got Unknown
			//IL_0187: Unknown result type (might be due to invalid IL or missing references)
			//IL_0191: Expected O, but got Unknown
			//IL_01a2: Unknown result type (might be due to invalid IL or missing references)
			//IL_01ac: Expected O, but got Unknown
			//IL_01bd: Unknown result type (might be due to invalid IL or missing references)
			//IL_01c7: Expected O, but got Unknown
			//IL_01d8: Unknown result type (might be due to invalid IL or missing references)
			//IL_01e2: Expected O, but got Unknown
			//IL_01f3: Unknown result type (might be due to invalid IL or missing references)
			//IL_01fd: Expected O, but got Unknown
			//IL_020e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0218: Expected O, but got Unknown
			if (!_loaded)
			{
				_dragCurveCd = new FloatCurve(PhysicsGlobals.DragCurveCd.Curve.keys);
				_dragCurveCdPower = new FloatCurve(PhysicsGlobals.DragCurveCdPower.Curve.keys);
				_dragCurveMultiplier = new FloatCurve(PhysicsGlobals.DragCurveMultiplier.Curve.keys);
				_dragCurveSurface = new FloatCurve(PhysicsGlobals.SurfaceCurves.dragCurveSurface.Curve.keys);
				_dragCurveTail = new FloatCurve(PhysicsGlobals.SurfaceCurves.dragCurveTail.Curve.keys);
				_dragCurveTip = new FloatCurve(PhysicsGlobals.SurfaceCurves.dragCurveTip.Curve.keys);
				_liftCurve = new FloatCurve(PhysicsGlobals.BodyLiftCurve.liftCurve.Curve.keys);
				LiftMachCurve = new FloatCurve(PhysicsGlobals.BodyLiftCurve.liftMachCurve.Curve.keys);
				_dragCurve = new FloatCurve(PhysicsGlobals.BodyLiftCurve.dragCurve.Curve.keys);
				_dragMachCurve = new FloatCurve(PhysicsGlobals.BodyLiftCurve.dragMachCurve.Curve.keys);
				DragCurvePseudoReynolds = new FloatCurve(PhysicsGlobals.DragCurvePseudoReynolds.Curve.keys);
				SpaceTemperature = PhysicsGlobals.SpaceTemperature;
				_loaded = true;
			}
			if ((Object)(object)newBody != (Object)(object)_body)
			{
				_body = newBody;
				AtmospherePressureCurve = new FloatCurve(newBody.atmospherePressureCurve.Curve.keys);
				AtmosphereTemperatureSunMultCurve = new FloatCurve(newBody.atmosphereTemperatureSunMultCurve.Curve.keys);
				LatitudeTemperatureBiasCurve = new FloatCurve(newBody.latitudeTemperatureBiasCurve.Curve.keys);
				LatitudeTemperatureSunMultCurve = new FloatCurve(newBody.latitudeTemperatureSunMultCurve.Curve.keys);
				AtmosphereTemperatureCurve = new FloatCurve(newBody.atmosphereTemperatureCurve.Curve.keys);
				AxialTemperatureSunMultCurve = new FloatCurve(newBody.axialTemperatureSunMultCurve.Curve.keys);
			}
		}

		public void CopyTo(DragCubeList dest)
		{
			dest.DragCurveCd = _dragCurveCd;
			dest.DragCurveCdPower = _dragCurveCdPower;
			dest.DragCurveMultiplier = _dragCurveMultiplier;
			dest.BodyLiftCurve.liftCurve = _liftCurve;
			dest.BodyLiftCurve.dragCurve = _dragCurve;
			dest.BodyLiftCurve.dragMachCurve = _dragMachCurve;
			dest.BodyLiftCurve.liftMachCurve = LiftMachCurve;
			dest.SurfaceCurves.dragCurveMultiplier = _dragCurveMultiplier;
			dest.SurfaceCurves.dragCurveSurface = _dragCurveSurface;
			dest.SurfaceCurves.dragCurveTail = _dragCurveTail;
			dest.SurfaceCurves.dragCurveTip = _dragCurveTip;
		}
	}

	private Orbit _inputInitialOrbit;

	private double _inputUT;

	private IDescentSpeedPolicy _inputDescentSpeedPolicy;

	private double _inputDecelEndAltitudeASL;

	private double _inputMaxThrustAccel;

	private double _inputParachuteSemiDeployMultiplier;

	private double _inputProbableLandingSiteASL;

	private bool _inputMultiplierHasError;

	private double _inputDT;

	private readonly Orbit _initialOrbit = new Orbit();

	private bool _bodyHasAtmosphere;

	private double _bodyRadius;

	private double _gravParameter;

	private SimulatedVessel _vessel;

	private Vector3d _bodyAngularVelocity;

	private IDescentSpeedPolicy _descentSpeedPolicy;

	private double _decelRadius;

	private double _aerobrakedRadius;

	private double _startUT;

	private CelestialBody _mainBody;

	private double _maxThrustAccel;

	private double _probableLandingSiteASL;

	private double _probableLandingSiteRadius;

	private QuaternionD _attitude;

	private bool _orbitReenters;

	private readonly ReferenceFrame _referenceFrame = new ReferenceFrame();

	private double _dt;

	private double _maxDT;

	public double MinDT;

	private double _maxSimulatedTime;

	private double _maxOrbits;

	private bool _noSKiptoFreefall;

	private double _parachuteSemiDeployMultiplier;

	private bool _multiplierHasError;

	private Vector3d _x;

	private Vector3d _startX;

	private Vector3d _v;

	private double _t;

	private Vector3 _lastRecordedDrag;

	private double _maxDragGees;

	private double _deltaVExpended;

	private List<AbsoluteVector> _trajectory;

	private int _steps;

	public static int ActiveStep;

	public static double ActiveDt;

	private SimCurves _simCurves;

	private static bool _once = true;

	private static readonly Pool<ReentrySimulation> _pool = new Pool<ReentrySimulation>((DelegateFunc<ReentrySimulation>)Create, (DelegateAction<ReentrySimulation>)Reset);

	private Result _result;

	private static ulong _resultId;

	private double _nextLog;

	public static int PoolSize => _pool.Size;

	private static ReentrySimulation Create()
	{
		return new ReentrySimulation();
	}

	public void Release()
	{
		_pool.Release(this);
	}

	private static void Reset(ReentrySimulation obj)
	{
	}

	public static ReentrySimulation Borrow(Orbit initialOrbit, double ut, SimulatedVessel vessel, SimCurves simcurves, IDescentSpeedPolicy descentSpeedPolicy, double decelEndAltitudeASL, double maxThrustAccel, double parachuteSemiDeployMultiplier, double probableLandingSiteASL, bool multiplierHasError, double dt, double minDT, double maxOrbits, bool noSKiptoFreefall)
	{
		ReentrySimulation reentrySimulation = _pool.Borrow();
		reentrySimulation.Init(initialOrbit, ut, vessel, simcurves, descentSpeedPolicy, decelEndAltitudeASL, maxThrustAccel, parachuteSemiDeployMultiplier, probableLandingSiteASL, multiplierHasError, dt, minDT, maxOrbits, noSKiptoFreefall);
		return reentrySimulation;
	}

	private void Init(Orbit initialOrbit, double ut, SimulatedVessel vessel, SimCurves simcurves, IDescentSpeedPolicy descentSpeedPolicy, double decelEndAltitudeASL, double maxThrustAccel, double parachuteSemiDeployMultiplier, double probableLandingSiteASL, bool multiplierHasError, double dt, double minDT, double maxOrbits, bool noSKiptoFreefall)
	{
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0174: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		_inputInitialOrbit = initialOrbit;
		_inputUT = ut;
		_vessel = vessel;
		_inputDescentSpeedPolicy = descentSpeedPolicy;
		_inputDecelEndAltitudeASL = decelEndAltitudeASL;
		_inputMaxThrustAccel = maxThrustAccel;
		_inputParachuteSemiDeployMultiplier = parachuteSemiDeployMultiplier;
		_inputProbableLandingSiteASL = probableLandingSiteASL;
		_inputMultiplierHasError = multiplierHasError;
		_inputDT = dt;
		_attitude = QuaternionD.op_Implicit(Quaternion.Euler(180f, 0f, 0f));
		MinDT = minDT;
		_maxDT = dt;
		_dt = _maxDT;
		_steps = 0;
		_maxOrbits = maxOrbits;
		_noSKiptoFreefall = noSKiptoFreefall;
		_initialOrbit.UpdateFromOrbitAtUT(initialOrbit, ut, initialOrbit.referenceBody);
		CelestialBody referenceBody = initialOrbit.referenceBody;
		_bodyHasAtmosphere = referenceBody.atmosphere;
		_bodyRadius = referenceBody.Radius;
		_gravParameter = referenceBody.gravParameter;
		_parachuteSemiDeployMultiplier = parachuteSemiDeployMultiplier;
		_multiplierHasError = multiplierHasError;
		_bodyAngularVelocity = referenceBody.angularVelocity;
		_descentSpeedPolicy = descentSpeedPolicy;
		_decelRadius = _bodyRadius + decelEndAltitudeASL;
		_aerobrakedRadius = _bodyRadius + referenceBody.RealMaxAtmosphereAltitude();
		_mainBody = referenceBody;
		_maxThrustAccel = maxThrustAccel;
		_probableLandingSiteASL = probableLandingSiteASL;
		_probableLandingSiteRadius = probableLandingSiteASL + _bodyRadius;
		_referenceFrame.UpdateAtCurrentTime(initialOrbit.referenceBody);
		_orbitReenters = OrbitReenters(initialOrbit);
		_startX = _initialOrbit.WorldBCIPositionAtUT(_startUT);
		if (_orbitReenters)
		{
			_startUT = ut;
			_t = _startUT;
			AdvanceToFreefallEnd(_initialOrbit);
		}
		_maxDragGees = 0.0;
		_deltaVExpended = 0.0;
		_trajectory = ListPool<AbsoluteVector>.Instance.Borrow();
		_simCurves = simcurves;
		_once = true;
	}

	public Result RunSimulation()
	{
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_02dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_019d: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		_result = Result.Borrow();
		try
		{
			_result.InputInitialOrbit = _inputInitialOrbit;
			_result.InputUT = _inputUT;
			_result.InputDescentSpeedPolicy = _inputDescentSpeedPolicy;
			_result.InputDecelEndAltitudeASL = _inputDecelEndAltitudeASL;
			_result.InputMaxThrustAccel = _inputMaxThrustAccel;
			_result.InputParachuteSemiDeployMultiplier = _inputParachuteSemiDeployMultiplier;
			_result.InputProbableLandingSiteASL = _inputProbableLandingSiteASL;
			_result.InputMultiplierHasError = _inputMultiplierHasError;
			_result.InputDT = _inputDT;
			if (!_orbitReenters)
			{
				_result.Outcome = Outcome.NO_REENTRY;
				return _result;
			}
			_result.StartPosition = _referenceFrame.ToAbsolute(_x, _t);
			_maxSimulatedTime = _maxOrbits * 2.0 * Math.PI * Math.Sqrt(Math.Pow(Math.Abs(((Vector3d)(ref _x)).magnitude), 3.0) / _gravParameter);
			RecordTrajectory();
			double num = _t + _maxSimulatedTime;
			while (true)
			{
				if (Landed())
				{
					_result.Outcome = Outcome.LANDED;
					break;
				}
				if (!_result.AeroBrake && Aerobraked())
				{
					_result.AeroBrake = true;
					_result.AeroBrakeUT = _t;
					_result.AeroBrakePosition = _referenceFrame.ToAbsolute(_x, _t);
					_result.AeroBrakeVelocity = _referenceFrame.ToAbsolute(_v, _t);
				}
				if (_t > num || Escaping() || _steps > 50000)
				{
					_result.Outcome = (_result.AeroBrake ? Outcome.AEROBRAKED : Outcome.TIMED_OUT);
					break;
				}
				BS34Step();
				LimitSpeed();
				RecordTrajectory();
			}
			_result.ID = _resultId++;
			_result.Body = _mainBody;
			_result.ReferenceFrame = _referenceFrame;
			_result.EndUT = _t;
			_result.TimeToComplete = _t - _inputUT;
			_result.MaxDragGees = _maxDragGees;
			_result.DeltaVExpended = _deltaVExpended;
			_result.EndPosition = _referenceFrame.ToAbsolute(_x, _t);
			_result.EndVelocity = _referenceFrame.ToAbsolute(_v, _t);
			_result.Trajectory = _trajectory;
			_result.ParachuteMultiplier = _parachuteSemiDeployMultiplier;
			_result.MultiplierHasError = _multiplierHasError;
			_result.Maxdt = _maxDT;
			_result.Steps = _steps;
		}
		catch (Exception exception)
		{
			_result.Exception = exception;
			_result.Outcome = Outcome.ERROR;
		}
		finally
		{
			if (_trajectory != _result.Trajectory)
			{
				ListPool<AbsoluteVector>.Instance.Release(_trajectory);
			}
			_vessel.Release();
			_simCurves.Release();
		}
		return _result;
	}

	private bool OrbitReenters(Orbit initialOrbit)
	{
		if (!(initialOrbit.PeR < _decelRadius))
		{
			return initialOrbit.PeR < _aerobrakedRadius;
		}
		return true;
	}

	private bool Landed()
	{
		return ((Vector3d)(ref _x)).magnitude < _probableLandingSiteRadius;
	}

	private bool Aerobraked()
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		if (_bodyHasAtmosphere && ((Vector3d)(ref _x)).magnitude > _aerobrakedRadius)
		{
			return Vector3d.Dot(_x, _v) > 0.0;
		}
		return false;
	}

	private bool Escaping()
	{
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		double num = Math.Sqrt(2.0 * _gravParameter / ((Vector3d)(ref _x)).magnitude);
		if (_bodyHasAtmosphere && ((Vector3d)(ref _v)).magnitude > num)
		{
			return Vector3d.Dot(_x, _v) > 0.0;
		}
		return false;
	}

	private void AdvanceToFreefallEnd(Orbit initialOrbit)
	{
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
		_t = FindFreefallEndTime(initialOrbit);
		_x = initialOrbit.WorldBCIPositionAtUT(_t);
		_v = initialOrbit.WorldOrbitalVelocityAtUT(_t);
		if (double.IsNaN(((Vector3d)(ref _v)).magnitude))
		{
			double gravParameter = initialOrbit.referenceBody.gravParameter;
			double num = (0.0 - gravParameter) / (2.0 * initialOrbit.semiMajorAxis);
			_v = Math.Sqrt(Math.Abs(2.0 * (num + gravParameter / ((Vector3d)(ref _x)).magnitude))) * ((Vector3d)(ref _x)).normalized;
			if (initialOrbit.MeanAnomalyAtUT(_t) > Math.PI)
			{
				_v *= -1.0;
			}
		}
	}

	private double FindFreefallEndTime(Orbit initialOrbit)
	{
		if (_noSKiptoFreefall || FreefallEnded(initialOrbit, _t))
		{
			return _t;
		}
		double num = _t;
		double num2 = initialOrbit.NextPeriapsisTime(_t);
		while (num2 - num > 1.0)
		{
			double num3 = (num2 + num) / 2.0;
			if (FreefallEnded(initialOrbit, num3))
			{
				num2 = num3;
			}
			else
			{
				num = num3;
			}
		}
		return (num2 + num) / 2.0;
	}

	private bool FreefallEnded(Orbit initialOrbit, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		Vector3d pos = initialOrbit.WorldBCIPositionAtUT(ut);
		Vector3d val = SurfaceVelocity(pos, initialOrbit.WorldOrbitalVelocityAtUT(ut));
		if (((Vector3d)(ref pos)).magnitude < _aerobrakedRadius)
		{
			return true;
		}
		if (Vector3d.Dot(val, initialOrbit.Up(ut)) > 0.0)
		{
			return false;
		}
		if (((Vector3d)(ref pos)).magnitude < _decelRadius)
		{
			return true;
		}
		if (_descentSpeedPolicy != null && ((Vector3d)(ref val)).magnitude > _descentSpeedPolicy.MaxAllowedSpeed(pos, val))
		{
			return true;
		}
		return false;
	}

	private void BS34Step()
	{
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0076: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0109: Unknown result type (might be due to invalid IL or missing references)
		//IL_010e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0113: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_012a: Unknown result type (might be due to invalid IL or missing references)
		//IL_012c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0131: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_013b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0145: Unknown result type (might be due to invalid IL or missing references)
		//IL_0153: Unknown result type (might be due to invalid IL or missing references)
		//IL_0155: Unknown result type (might be due to invalid IL or missing references)
		//IL_015a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0168: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		//IL_017d: Unknown result type (might be due to invalid IL or missing references)
		//IL_017f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0184: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0198: Unknown result type (might be due to invalid IL or missing references)
		//IL_019a: Unknown result type (might be due to invalid IL or missing references)
		//IL_019f: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_01af: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0209: Unknown result type (might be due to invalid IL or missing references)
		//IL_020b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0210: Unknown result type (might be due to invalid IL or missing references)
		//IL_021e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0223: Unknown result type (might be due to invalid IL or missing references)
		//IL_022d: Unknown result type (might be due to invalid IL or missing references)
		//IL_022f: Unknown result type (might be due to invalid IL or missing references)
		//IL_023d: Unknown result type (might be due to invalid IL or missing references)
		//IL_023f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0244: Unknown result type (might be due to invalid IL or missing references)
		//IL_0252: Unknown result type (might be due to invalid IL or missing references)
		//IL_0254: Unknown result type (might be due to invalid IL or missing references)
		//IL_0259: Unknown result type (might be due to invalid IL or missing references)
		//IL_0267: Unknown result type (might be due to invalid IL or missing references)
		//IL_026c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0276: Unknown result type (might be due to invalid IL or missing references)
		//IL_0278: Unknown result type (might be due to invalid IL or missing references)
		//IL_0286: Unknown result type (might be due to invalid IL or missing references)
		//IL_0288: Unknown result type (might be due to invalid IL or missing references)
		//IL_028d: Unknown result type (might be due to invalid IL or missing references)
		//IL_029b: Unknown result type (might be due to invalid IL or missing references)
		//IL_029d: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_02dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0301: Unknown result type (might be due to invalid IL or missing references)
		//IL_0303: Unknown result type (might be due to invalid IL or missing references)
		//IL_0310: Unknown result type (might be due to invalid IL or missing references)
		//IL_0312: Unknown result type (might be due to invalid IL or missing references)
		//IL_0318: Unknown result type (might be due to invalid IL or missing references)
		//IL_031d: Unknown result type (might be due to invalid IL or missing references)
		//IL_031e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0323: Unknown result type (might be due to invalid IL or missing references)
		//IL_0328: Unknown result type (might be due to invalid IL or missing references)
		//IL_032b: Unknown result type (might be due to invalid IL or missing references)
		//IL_032d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0342: Unknown result type (might be due to invalid IL or missing references)
		//IL_0344: Unknown result type (might be due to invalid IL or missing references)
		//IL_0444: Unknown result type (might be due to invalid IL or missing references)
		//IL_044a: Unknown result type (might be due to invalid IL or missing references)
		//IL_044f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0454: Unknown result type (might be due to invalid IL or missing references)
		//IL_053b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0540: Unknown result type (might be due to invalid IL or missing references)
		//IL_0541: Unknown result type (might be due to invalid IL or missing references)
		//IL_0546: Unknown result type (might be due to invalid IL or missing references)
		//IL_054d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0552: Unknown result type (might be due to invalid IL or missing references)
		//IL_0553: Unknown result type (might be due to invalid IL or missing references)
		//IL_0558: Unknown result type (might be due to invalid IL or missing references)
		bool flag;
		do
		{
			_steps++;
			ActiveStep = _steps;
			ActiveDt = _dt;
			flag = false;
			Vector3d val = _dt * TotalAccel(_x, _v, record: true);
			Vector3d val2 = _dt * _v;
			Vector3d val3 = _dt * TotalAccel(_x + 0.5 * val2, _v + 0.5 * val);
			Vector3d val4 = _dt * (_v + 0.5 * val);
			Vector3d val5 = _dt * TotalAccel(_x + 0.75 * val4, _v + 0.75 * val3);
			Vector3d val6 = _dt * (_v + 0.75 * val3);
			Vector3d val7 = _dt * TotalAccel(_x + 2.0 / 9.0 * val2 + 1.0 / 3.0 * val4 + 4.0 / 9.0 * val6, _v + 2.0 / 9.0 * val + 1.0 / 3.0 * val3 + 4.0 / 9.0 * val5);
			Vector3d val8 = (2.0 * val2 + 3.0 * val4 + 4.0 * val6) * (1.0 / 9.0);
			Vector3d val9 = (2.0 * val + 3.0 * val3 + 4.0 * val5) * (1.0 / 9.0);
			Vector3d val10 = (7.0 * val + 6.0 * val3 + 8.0 * val5 + 3.0 * val7) * (1.0 / 24.0) - val9;
			Vector3 val11 = Vector3d.op_Implicit(_x + val8);
			double num = (double)((Vector3)(ref val11)).magnitude - _bodyRadius;
			double num2 = num - _probableLandingSiteASL;
			double pressure = Pressure(Vector3d.op_Implicit(val11));
			Vector3d val12 = SurfaceVelocity(Vector3d.op_Implicit(val11), _v + val9);
			double num3 = AirDensity(Vector3d.op_Implicit(val11), num);
			double speedOfSound = _mainBody.GetSpeedOfSound(Pressure(Vector3d.op_Implicit(val11)), num3);
			double magnitude = ((Vector3d)(ref val12)).magnitude;
			double mach = Math.Min(magnitude / speedOfSound, 50.0);
			double shockTemp = ShockTemperature(magnitude, mach);
			bool flag2 = num < _probableLandingSiteASL || _vessel.WillChutesDeploy(num2, num, _probableLandingSiteASL, pressure, shockTemp, _t, _parachuteSemiDeployMultiplier);
			double num4 = Math.Max(((Vector3d)(ref val10)).magnitude, 1E-05);
			double num5 = (flag2 ? (_dt * 0.5) : (_dt * 0.9 * Math.Pow(0.01 / num4, 1.0 / 3.0)));
			if (double.IsNaN(num5))
			{
				num5 = MinDT;
			}
			num5 = Math.Max(num5, MinDT);
			num5 = Math.Min(num5, 10.0);
			Vector3d val13 = _x - _startX;
			double sqrMagnitude = ((Vector3d)(ref val13)).sqrMagnitude;
			if (sqrMagnitude < 1000000.0)
			{
				num5 = Math.Min(num5, 0.02);
			}
			else if (sqrMagnitude < 25000000.0)
			{
				num5 = Math.Min(num5, 0.5);
			}
			else if (sqrMagnitude < 100000000.0)
			{
				num5 = Math.Min(num5, 1.0);
			}
			if ((num4 > 0.01 || flag2) && _dt > MinDT)
			{
				_dt = num5;
				flag = true;
				continue;
			}
			_maxDragGees = Math.Max(_maxDragGees, ((Vector3)(ref _lastRecordedDrag)).magnitude / 9.81f);
			bool flag3 = _vessel.Simulate(num2, num, _probableLandingSiteASL, pressure, shockTemp, _t, _parachuteSemiDeployMultiplier);
			_x += val8;
			_v += val9;
			_t += _dt;
			_dt = (flag3 ? MinDT : num5);
		}
		while (flag);
	}

	private void LimitSpeed()
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_0078: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		if (_descentSpeedPolicy != null)
		{
			Vector3d val = SurfaceVelocity(_x, _v);
			double num = _descentSpeedPolicy.MaxAllowedSpeed(_x, val);
			if (((Vector3d)(ref val)).magnitude > num)
			{
				double num2 = Math.Min(((Vector3d)(ref val)).magnitude - num, _dt * _maxThrustAccel);
				val -= num2 * ((Vector3d)(ref val)).normalized;
				_deltaVExpended += num2;
				_v = val + Vector3d.Cross(_bodyAngularVelocity, _x);
			}
		}
	}

	private void RecordTrajectory()
	{
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		_trajectory.Add(_referenceFrame.ToAbsolute(_x, _t));
	}

	private Vector3d TotalAccel(Vector3d pos, Vector3d vel, bool record = false)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0127: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_012d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0134: Unknown result type (might be due to invalid IL or missing references)
		//IL_0144: Unknown result type (might be due to invalid IL or missing references)
		//IL_0149: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_015d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0163: Unknown result type (might be due to invalid IL or missing references)
		//IL_0164: Unknown result type (might be due to invalid IL or missing references)
		//IL_0169: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		//IL_017e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0180: Unknown result type (might be due to invalid IL or missing references)
		//IL_0182: Unknown result type (might be due to invalid IL or missing references)
		//IL_0187: Unknown result type (might be due to invalid IL or missing references)
		//IL_0189: Unknown result type (might be due to invalid IL or missing references)
		//IL_014f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0151: Unknown result type (might be due to invalid IL or missing references)
		//IL_0156: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = SurfaceVelocity(pos, vel);
		double altitude = ((Vector3d)(ref pos)).magnitude - _bodyRadius;
		double num = AirDensity(pos, altitude);
		double num2 = Pressure(pos);
		double speedOfSound = _mainBody.GetSpeedOfSound(num2, num);
		float num3 = Mathf.Min((float)(((Vector3d)(ref val)).magnitude / speedOfSound), 50f);
		float num4 = (float)(0.0005 * num * ((Vector3d)(ref val)).sqrMagnitude);
		double num5 = num * ((Vector3d)(ref val)).magnitude;
		double num6 = _simCurves.DragCurvePseudoReynolds.Evaluate((float)num5);
		if (_once)
		{
			ref Prediction prediction = ref _result.Prediction;
			Vector3d val2 = DragForce(pos, vel, num4, num3);
			prediction.FirstDrag = ((Vector3d)(ref val2)).magnitude / 9.81;
			ref Prediction prediction2 = ref _result.Prediction;
			val2 = LiftForce(pos, vel, num4, num3);
			prediction2.FirstLift = ((Vector3d)(ref val2)).magnitude / 9.81;
			_result.Prediction.Mach = num3;
			_result.Prediction.SpeedOfSound = speedOfSound;
			_result.Prediction.DynamicPressurekPa = num4;
		}
		Vector3d val3 = DragForce(pos, vel, num4, num3) * num6 / _vessel.totalMass;
		if (record)
		{
			_lastRecordedDrag = Vector3d.op_Implicit(val3);
		}
		Vector3d val4 = GravAccel(pos);
		Vector3d val5 = LiftForce(pos, vel, num4, num3) / _vessel.totalMass;
		Vector3d result = val4 + val3 + val5;
		if (_once)
		{
			_once = false;
		}
		return result;
	}

	private Vector3d GravAccel(Vector3d pos)
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		return (0.0 - _gravParameter / ((Vector3d)(ref pos)).sqrMagnitude) * ((Vector3d)(ref pos)).normalized;
	}

	private Vector3d DragForce(Vector3d pos, Vector3d vel, float dynamicPressurekPa, float mach)
	{
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		if (!_bodyHasAtmosphere)
		{
			return Vector3d.zero;
		}
		Vector3d val = SurfaceVelocity(pos, vel);
		Vector3d localVelocity = _attitude * Vector3d.up * ((Vector3d)(ref val)).magnitude;
		Vector3d val2 = _vessel.Drag(localVelocity, dynamicPressurekPa, mach);
		return -((Vector3d)(ref val)).normalized * ((Vector3d)(ref val2)).magnitude;
	}

	private Vector3d LiftForce(Vector3d pos, Vector3d vel, float dynamicPressurekPa, float mach)
	{
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		if (!_bodyHasAtmosphere)
		{
			return Vector3d.zero;
		}
		Vector3d val = SurfaceVelocity(pos, vel);
		Vector3d val2 = _attitude * Vector3d.up * ((Vector3d)(ref val)).magnitude;
		Vector3d val3 = _vessel.Lift(val2, dynamicPressurekPa, mach);
		return QuaternionD.op_Implicit(Quaternion.FromToRotation(Vector3d.op_Implicit(val2), Vector3d.op_Implicit(val))) * val3;
	}

	private double Pressure(Vector3d pos)
	{
		double altitude = ((Vector3d)(ref pos)).magnitude - _bodyRadius;
		return StaticPressure(altitude);
	}

	private Vector3d SurfaceVelocity(Vector3d pos, Vector3d vel)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		return vel - Vector3d.Cross(_bodyAngularVelocity, pos);
	}

	private double AirDensity(Vector3d pos, double altitude)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		double num = StaticPressure(altitude);
		double temperature = GetTemperature(pos, altitude);
		return FlightGlobals.getAtmDensity(num, temperature, _mainBody);
	}

	private double StaticPressure(double altitude)
	{
		if (!_mainBody.atmosphere)
		{
			return 0.0;
		}
		if (altitude >= _mainBody.atmosphereDepth)
		{
			return 0.0;
		}
		if (_mainBody.atmosphereUsePressureCurve)
		{
			if (_mainBody.atmospherePressureCurveIsNormalized)
			{
				return Mathf.Lerp(0f, (float)_mainBody.atmospherePressureSeaLevel, _simCurves.AtmospherePressureCurve.Evaluate((float)(altitude / _mainBody.atmosphereDepth)));
			}
			return _simCurves.AtmospherePressureCurve.Evaluate((float)altitude);
		}
		return _mainBody.atmospherePressureSeaLevel * Math.Pow(1.0 - _mainBody.atmosphereTemperatureLapseRate * altitude / _mainBody.atmosphereTemperatureSeaLevel, _mainBody.atmosphereGasMassLapseRate);
	}

	private double GetTemperature(Vector3d position, double altitude)
	{
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_012c: Unknown result type (might be due to invalid IL or missing references)
		if (!_mainBody.atmosphere)
		{
			return PhysicsGlobals.SpaceTemperature;
		}
		if (altitude > _mainBody.atmosphereDepth)
		{
			return PhysicsGlobals.SpaceTemperature;
		}
		Vector3 val = Vector3d.op_Implicit(((Vector3d)(ref position)).normalized);
		float num = Mathf.Acos(Vector3.Dot(_mainBody.bodyTransform.up, val));
		if (num > (float)Math.PI / 2f)
		{
			num = (float)Math.PI - num;
		}
		float num2 = ((float)Math.PI / 2f - num) * 57.29578f;
		Vector3d val2 = FlightGlobals.Bodies[0].position - position + _mainBody.position;
		Vector3 val3 = Vector3d.op_Implicit(((Vector3d)(ref val2)).normalized);
		Vector3 up = _mainBody.bodyTransform.up;
		float num3 = Vector3.Dot(val3, up);
		float num4 = Mathf.Acos(Vector3.Dot(up, val));
		float num5 = Mathf.Acos(num3);
		float num6 = (1f + Mathf.Cos(num5 - num4)) * 0.5f;
		float num7 = (1f + Mathf.Cos(num5 + num4)) * 0.5f;
		float num8 = ((1f + Vector3.Dot(val3, Quaternion.AngleAxis(45f * Mathf.Sign((float)_mainBody.rotationPeriod), up) * val)) * 0.5f - num7) / (num6 - num7);
		double num9 = (double)_simCurves.LatitudeTemperatureBiasCurve.Evaluate(num2) + (double)_simCurves.LatitudeTemperatureSunMultCurve.Evaluate(num2) * (double)num8 + (double)_simCurves.AxialTemperatureSunMultCurve.Evaluate(num3);
		double num10 = (_mainBody.atmosphereUseTemperatureCurve ? ((!_mainBody.atmosphereTemperatureCurveIsNormalized) ? ((double)_simCurves.AtmosphereTemperatureCurve.Evaluate((float)altitude)) : UtilMath.Lerp(_simCurves.SpaceTemperature, _mainBody.atmosphereTemperatureSeaLevel, (double)_simCurves.AtmosphereTemperatureCurve.Evaluate((float)(altitude / _mainBody.atmosphereDepth)))) : (_mainBody.atmosphereTemperatureSeaLevel - _mainBody.atmosphereTemperatureLapseRate * altitude));
		return num10 + (double)_simCurves.AtmosphereTemperatureSunMultCurve.Evaluate((float)altitude) * num9;
	}

	private double ShockTemperature(double velocity, double mach)
	{
		double num = velocity * PhysicsGlobals.NewtonianTemperatureFactor;
		double num2 = Math.Pow(UtilMath.Clamp01((mach - PhysicsGlobals.NewtonianMachTempLerpStartMach) / (PhysicsGlobals.NewtonianMachTempLerpEndMach - PhysicsGlobals.NewtonianMachTempLerpStartMach)), PhysicsGlobals.NewtonianMachTempLerpExponent);
		if (num2 > 0.0)
		{
			double num3 = PhysicsGlobals.MachTemperatureScalar * Math.Pow(velocity, PhysicsGlobals.MachTemperatureVelocityExponent);
			num = UtilMath.LerpUnclamped(num, num3, num2);
		}
		return num * (double)HighLogic.CurrentGame.Parameters.Difficulty.ReentryHeatScale * _mainBody.shockTemperatureMultiplier;
	}

	public void Log(Vector3d pos, Vector3d vel)
	{
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
		if (_t > _nextLog)
		{
			_nextLog = _t + 10.0;
			double altitude = ((Vector3d)(ref pos)).magnitude - _bodyRadius;
			Vector3d val = SurfaceVelocity(pos, vel);
			double temperature = GetTemperature(pos, altitude);
			double speedOfSound = _mainBody.GetSpeedOfSound(Pressure(pos), AirDensity(pos, altitude));
			float num = Mathf.Min((float)(((Vector3d)(ref val)).magnitude / speedOfSound), 50f);
			float num2 = (float)(0.0005 * AirDensity(pos, altitude) * ((Vector3d)(ref val)).sqrMagnitude);
			Result result = _result;
			result.DebugLog = result.DebugLog + "\n " + (_t - _startUT).ToString("F2").PadLeft(8) + " Alt:" + altitude.ToString("F0").PadLeft(6) + " Vel:" + ((Vector3d)(ref vel)).magnitude.ToString("F2").PadLeft(8) + " AirVel:" + ((Vector3d)(ref val)).magnitude.ToString("F2").PadLeft(8) + " SoS:" + speedOfSound.ToString("F2").PadLeft(6) + " mach:" + num.ToString("F2").PadLeft(6) + " dynP:" + num2.ToString("F5").PadLeft(9) + " Temp:" + temperature.ToString("F2").PadLeft(8) + " Lat:" + _referenceFrame.Latitude(pos).ToString("F2").PadLeft(6);
		}
	}
}
