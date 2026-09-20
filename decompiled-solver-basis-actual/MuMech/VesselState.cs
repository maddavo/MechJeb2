using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KSP.Localization;
using MechJebLib.Utils;
using MechJebLibBindings;
using Smooth.Delegates;
using Smooth.Pools;
using UnityEngine;

namespace MuMech;

public class VesselState
{
	public delegate void VesselStatePartExtension(Part p);

	public delegate void VesselStatePartModuleExtension(PartModule pm);

	public delegate double DoubleDelegate();

	public delegate double DoubleVesselDelegate(Vessel v);

	public delegate void CalculateVesselAeroForcesDelegate(Vessel vessel, out Vector3 aeroForce, out Vector3 aeroTorque, Vector3 velocityWorldVector, double altitude);

	public class EngineInfo
	{
		public struct FuelRequirement
		{
			public double RequiredLastFrame;

			public double RequiredAtMaxThrottle;
		}

		private readonly Queue _rotSave = new Queue();

		public readonly Dictionary<int, FuelRequirement> ResourceRequired = new Dictionary<int, FuelRequirement>();

		public readonly Vector6 TorqueDifferentialThrottle = new Vector6();

		private float _atmP0;

		private float _atmP1;

		private Vector3d _com;

		public double LowestUllage = 1.0;

		public double MaxResponseTime;

		public Vector3d ThrustCurrent;

		public Vector3d ThrustMax;

		public Vector3d ThrustMin;

		public void Update(Vector3d c, Vessel vessel)
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_0006: Unknown result type (might be due to invalid IL or missing references)
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0011: Unknown result type (might be due to invalid IL or missing references)
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0056: Unknown result type (might be due to invalid IL or missing references)
			//IL_0057: Unknown result type (might be due to invalid IL or missing references)
			ThrustCurrent = Vector3d.zero;
			ThrustMax = Vector3d.zero;
			ThrustMin = Vector3d.zero;
			MaxResponseTime = 0.0;
			TorqueDifferentialThrottle.Reset();
			ResourceRequired.Clear();
			LowestUllage = 1.0;
			_com = c;
			_atmP0 = (float)(vessel.staticPressurekPa * PhysicsGlobals.KpaToAtmospheres);
			float num = (float)(vessel.altitude + (double)TimeWarp.fixedDeltaTime * vessel.verticalSpeed);
			_atmP1 = (float)(FlightGlobals.getStaticPressure((double)num, (CelestialBody)null) * PhysicsGlobals.KpaToAtmospheres);
		}

		public void CheckUllageStatus(ModuleEngines e)
		{
			if (IsRealFuelsCorrectlyInitialized && !e.getFlameoutState && e.EngineIgnited && ((PartModule)e).isEnabled && !(e.requestedThrottle > 0f) && RFModuleEnginesRFType.IsInstance((object)e) && RFullageField.GetValue<bool>((object)e) && RFignitionsField.GetValue<int>((object)e) != 0)
			{
				object value = RFullageSetField.GetValue<object>((object)e);
				double num = (double)RFGetUllageStabilityMethod.Invoke(value, Array.Empty<object>());
				if (num < LowestUllage)
				{
					LowestUllage = num;
				}
			}
		}

		public void AddNewEngine(ModuleEngines e, ModuleGimbal? gimbal, List<EngineWrapper> enginesWrappers, ref Vector3d cot, ref Vector3d dot, ref double coTWeightSum)
		{
			//IL_010a: Unknown result type (might be due to invalid IL or missing references)
			//IL_010f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0111: Unknown result type (might be due to invalid IL or missing references)
			//IL_0116: Unknown result type (might be due to invalid IL or missing references)
			//IL_0118: Unknown result type (might be due to invalid IL or missing references)
			//IL_011d: Unknown result type (might be due to invalid IL or missing references)
			//IL_011f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0124: Unknown result type (might be due to invalid IL or missing references)
			//IL_01c9: Unknown result type (might be due to invalid IL or missing references)
			//IL_01ce: Unknown result type (might be due to invalid IL or missing references)
			//IL_01d3: Unknown result type (might be due to invalid IL or missing references)
			//IL_01d8: Unknown result type (might be due to invalid IL or missing references)
			//IL_01fb: Unknown result type (might be due to invalid IL or missing references)
			//IL_0202: Unknown result type (might be due to invalid IL or missing references)
			//IL_0204: Unknown result type (might be due to invalid IL or missing references)
			//IL_0209: Unknown result type (might be due to invalid IL or missing references)
			//IL_020e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0215: Unknown result type (might be due to invalid IL or missing references)
			//IL_021c: Unknown result type (might be due to invalid IL or missing references)
			//IL_021e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0223: Unknown result type (might be due to invalid IL or missing references)
			//IL_0228: Unknown result type (might be due to invalid IL or missing references)
			//IL_022f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0236: Unknown result type (might be due to invalid IL or missing references)
			//IL_0238: Unknown result type (might be due to invalid IL or missing references)
			//IL_0240: Unknown result type (might be due to invalid IL or missing references)
			//IL_0245: Unknown result type (might be due to invalid IL or missing references)
			//IL_024a: Unknown result type (might be due to invalid IL or missing references)
			//IL_0253: Unknown result type (might be due to invalid IL or missing references)
			//IL_025c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0261: Unknown result type (might be due to invalid IL or missing references)
			//IL_0266: Unknown result type (might be due to invalid IL or missing references)
			//IL_026b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0270: Unknown result type (might be due to invalid IL or missing references)
			//IL_0279: Unknown result type (might be due to invalid IL or missing references)
			//IL_0280: Unknown result type (might be due to invalid IL or missing references)
			//IL_0282: Unknown result type (might be due to invalid IL or missing references)
			//IL_0287: Unknown result type (might be due to invalid IL or missing references)
			//IL_028c: Unknown result type (might be due to invalid IL or missing references)
			//IL_02aa: Unknown result type (might be due to invalid IL or missing references)
			//IL_02af: Unknown result type (might be due to invalid IL or missing references)
			//IL_02b4: Unknown result type (might be due to invalid IL or missing references)
			//IL_02b5: Unknown result type (might be due to invalid IL or missing references)
			//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
			//IL_02bc: Unknown result type (might be due to invalid IL or missing references)
			//IL_02c1: Unknown result type (might be due to invalid IL or missing references)
			//IL_02c6: Unknown result type (might be due to invalid IL or missing references)
			//IL_02ca: Unknown result type (might be due to invalid IL or missing references)
			//IL_02d0: Unknown result type (might be due to invalid IL or missing references)
			//IL_02d5: Unknown result type (might be due to invalid IL or missing references)
			//IL_02da: Unknown result type (might be due to invalid IL or missing references)
			//IL_02df: Unknown result type (might be due to invalid IL or missing references)
			//IL_02e4: Unknown result type (might be due to invalid IL or missing references)
			//IL_02e9: Unknown result type (might be due to invalid IL or missing references)
			//IL_02eb: Unknown result type (might be due to invalid IL or missing references)
			//IL_02f2: Unknown result type (might be due to invalid IL or missing references)
			//IL_02f4: Unknown result type (might be due to invalid IL or missing references)
			//IL_02fc: Unknown result type (might be due to invalid IL or missing references)
			//IL_0301: Unknown result type (might be due to invalid IL or missing references)
			//IL_0306: Unknown result type (might be due to invalid IL or missing references)
			//IL_0308: Unknown result type (might be due to invalid IL or missing references)
			//IL_030c: Unknown result type (might be due to invalid IL or missing references)
			//IL_030e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0316: Unknown result type (might be due to invalid IL or missing references)
			//IL_031b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0320: Unknown result type (might be due to invalid IL or missing references)
			//IL_0322: Unknown result type (might be due to invalid IL or missing references)
			//IL_032d: Unknown result type (might be due to invalid IL or missing references)
			//IL_032f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0331: Unknown result type (might be due to invalid IL or missing references)
			//IL_0336: Unknown result type (might be due to invalid IL or missing references)
			//IL_033b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0340: Unknown result type (might be due to invalid IL or missing references)
			//IL_0342: Unknown result type (might be due to invalid IL or missing references)
			//IL_034a: Unknown result type (might be due to invalid IL or missing references)
			//IL_034c: Unknown result type (might be due to invalid IL or missing references)
			//IL_034e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0353: Unknown result type (might be due to invalid IL or missing references)
			//IL_0358: Unknown result type (might be due to invalid IL or missing references)
			//IL_035d: Unknown result type (might be due to invalid IL or missing references)
			//IL_03a8: Unknown result type (might be due to invalid IL or missing references)
			//IL_03aa: Unknown result type (might be due to invalid IL or missing references)
			//IL_03ac: Unknown result type (might be due to invalid IL or missing references)
			//IL_036d: Unknown result type (might be due to invalid IL or missing references)
			//IL_036f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0371: Unknown result type (might be due to invalid IL or missing references)
			//IL_037c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0384: Unknown result type (might be due to invalid IL or missing references)
			//IL_0178: Unknown result type (might be due to invalid IL or missing references)
			//IL_0191: Unknown result type (might be due to invalid IL or missing references)
			//IL_03ea: Unknown result type (might be due to invalid IL or missing references)
			if (!e.EngineIgnited || !((PartModule)e).isEnabled)
			{
				return;
			}
			float num = e.atmosphereCurve.Evaluate(_atmP0);
			float num2 = e.atmosphereCurve.Evaluate(_atmP1);
			float num3 = Mathf.Min(num, num2);
			foreach (Propellant propellant in e.propellants)
			{
				double max = e.maxFuelFlow * propellant.ratio;
				AddResource(propellant.id, propellant.currentRequirement, max);
			}
			if (!e.isOperational)
			{
				return;
			}
			float num4 = e.thrustPercentage / 100f;
			double num5 = e.maxFuelFlow * e.flowMultiplier * num3 * e.g;
			double num6 = e.minFuelFlow * e.flowMultiplier * num3 * e.g;
			double num7 = num6 + (num5 - num6) * (double)num4;
			double num8 = (e.throttleLocked ? num7 : num6);
			double num9 = e.finalThrust;
			_rotSave.Clear();
			Vector3d val = Vector3d.zero;
			Vector3d val2 = Vector3d.zero;
			Vector3d val3 = Vector3d.zero;
			Vector3d val4 = Vector3d.zero;
			double num10 = num5;
			double num11 = num6;
			if (e.throttleLocked)
			{
				num10 *= (double)num4;
				num11 = num10;
			}
			if ((Object)(object)gimbal != (Object)null && !gimbal.gimbalLock)
			{
				_rotSave.Clear();
				for (int i = 0; i < gimbal.gimbalTransforms.Count; i++)
				{
					Transform val5 = gimbal.gimbalTransforms[i];
					_rotSave.Enqueue(val5.localRotation);
					val5.localRotation = gimbal.initRots[i];
				}
			}
			for (int j = 0; j < e.thrustTransforms.Count; j++)
			{
				Transform val6 = e.thrustTransforms[j];
				Vector3d val7 = Vector3d.op_Implicit(-val6.forward);
				float num12 = e.thrustTransformMultipliers[j];
				double num13 = num9 * (double)num12;
				double num14 = num7 * (double)num12;
				ThrustCurrent += num13 * val7;
				ThrustMax += num14 * val7;
				ThrustMin += num8 * val7 * (double)num12;
				cot += num14 * Vector3d.op_Implicit(val6.position);
				dot -= num13 * val7;
				coTWeightSum += num14;
				Quaternion val8 = QuaternionExtensions.Inverse(((PartModule)e).part.vessel.ReferenceTransform.rotation);
				Vector3d val9 = Vector3d.op_Implicit(val8 * Vector3d.op_Implicit(val7));
				Vector3d val10 = Vector3d.op_Implicit(val8 * Vector3d.op_Implicit(val6.position - _com));
				val2 += (num10 - num11) * val9 * (double)num12;
				val += num11 * val9 * (double)num12;
				val4 += (num10 - num11) * (double)num12 * Vector3d.Cross(val10, val9);
				val3 += num11 * (double)num12 * Vector3d.Cross(val10, val9);
				if (!e.throttleLocked)
				{
					TorqueDifferentialThrottle.Add(Vector3d.Cross(val10, val9) * (double)(float)(num5 - num6) * (double)num12);
				}
			}
			enginesWrappers.Add(new EngineWrapper(e, val2, val3, val4));
			if ((Object)(object)gimbal != (Object)null && !gimbal.gimbalLock)
			{
				foreach (Transform gimbalTransform in gimbal.gimbalTransforms)
				{
					gimbalTransform.localRotation = (Quaternion)_rotSave.Dequeue();
				}
			}
			if (e.useEngineResponseTime)
			{
				double num15 = 1.0 / (double)Math.Min(e.engineAccelerationSpeed, e.engineDecelerationSpeed);
				if (num15 > MaxResponseTime)
				{
					MaxResponseTime = num15;
				}
			}
		}

		private void AddResource(int id, double current, double max)
		{
			FuelRequirement value2;
			if (ResourceRequired.TryGetValue(id, out var value))
			{
				value2 = value;
			}
			else
			{
				value2 = default(FuelRequirement);
				ResourceRequired[id] = value2;
			}
			value2.RequiredLastFrame += current;
			value2.RequiredAtMaxThrottle += max;
		}
	}

	private class IntakeInfo
	{
		private static readonly List<ModuleResourceIntake> _empty = new List<ModuleResourceIntake>();

		private readonly Dictionary<int, List<ModuleResourceIntake>> _allIntakes = new Dictionary<int, List<ModuleResourceIntake>>();

		public void Update()
		{
			foreach (List<ModuleResourceIntake> value in _allIntakes.Values)
			{
				ListPool<ModuleResourceIntake>.Instance.Release(value);
			}
			_allIntakes.Clear();
		}

		public void AddIntake(ModuleResourceIntake intake)
		{
			int id = PartResourceLibrary.Instance.GetDefinition(intake.resourceName).id;
			List<ModuleResourceIntake> list;
			if (_allIntakes.TryGetValue(id, out List<ModuleResourceIntake> value))
			{
				list = value;
			}
			else
			{
				list = ListPool<ModuleResourceIntake>.Instance.Borrow();
				_allIntakes[id] = list;
			}
			list.Add(intake);
		}

		public List<ModuleResourceIntake> GetIntakes(int id)
		{
			if (!_allIntakes.TryGetValue(id, out List<ModuleResourceIntake> value))
			{
				return _empty;
			}
			return value;
		}
	}

	public sealed class ResourceInfo
	{
		public struct IntakeData
		{
			public readonly ModuleResourceIntake Intake;

			public readonly double PredictedMassFlow;

			public IntakeData(ModuleResourceIntake intake, double predictedMassFlow)
			{
				Intake = intake;
				PredictedMassFlow = predictedMassFlow;
			}
		}

		private static readonly Pool<ResourceInfo> _pool = new Pool<ResourceInfo>((DelegateFunc<ResourceInfo>)Create, (DelegateAction<ResourceInfo>)Reset);

		public readonly List<IntakeData> Intakes = new List<IntakeData>();

		public double IntakeAvailable;

		public double Required;

		public double RequiredAtMaxThrottle;

		public double IntakeProvided
		{
			get
			{
				double num = 0.0;
				foreach (IntakeData intake in Intakes)
				{
					if (intake.Intake.intakeEnabled)
					{
						num += intake.PredictedMassFlow;
					}
				}
				return num;
			}
		}

		public static int PoolSize => _pool.Size;

		private ResourceInfo()
		{
		}

		private static ResourceInfo Create()
		{
			return new ResourceInfo();
		}

		public void Release()
		{
			_pool.Release(this);
		}

		public static void Release(Dictionary<int, ResourceInfo>.ValueCollection objList)
		{
			foreach (ResourceInfo obj in objList)
			{
				obj.Release();
			}
		}

		private static void Reset(ResourceInfo obj)
		{
			obj.Required = 0.0;
			obj.RequiredAtMaxThrottle = 0.0;
			obj.IntakeAvailable = 0.0;
			obj.Intakes.Clear();
		}

		public static ResourceInfo Borrow(PartResourceDefinition r, double req, double atMax, List<ModuleResourceIntake> modules, Vessel vessel)
		{
			ResourceInfo resourceInfo = _pool.Borrow();
			resourceInfo.Init(r, req, atMax, modules, vessel);
			return resourceInfo;
		}

		private void Init(PartResourceDefinition r, double req, double atMax, List<ModuleResourceIntake> modules, Vessel vessel)
		{
			//IL_002b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0030: Unknown result type (might be due to invalid IL or missing references)
			//IL_0031: Unknown result type (might be due to invalid IL or missing references)
			//IL_0036: Unknown result type (might be due to invalid IL or missing references)
			//IL_003b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0040: Unknown result type (might be due to invalid IL or missing references)
			//IL_0045: Unknown result type (might be due to invalid IL or missing references)
			//IL_0048: Unknown result type (might be due to invalid IL or missing references)
			//IL_004d: Unknown result type (might be due to invalid IL or missing references)
			//IL_0051: Unknown result type (might be due to invalid IL or missing references)
			//IL_0056: Unknown result type (might be due to invalid IL or missing references)
			//IL_0157: Unknown result type (might be due to invalid IL or missing references)
			//IL_015c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0161: Unknown result type (might be due to invalid IL or missing references)
			//IL_0166: Unknown result type (might be due to invalid IL or missing references)
			//IL_016b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0170: Unknown result type (might be due to invalid IL or missing references)
			//IL_017f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0181: Unknown result type (might be due to invalid IL or missing references)
			//IL_0186: Unknown result type (might be due to invalid IL or missing references)
			//IL_0188: Unknown result type (might be due to invalid IL or missing references)
			//IL_018d: Unknown result type (might be due to invalid IL or missing references)
			//IL_0192: Unknown result type (might be due to invalid IL or missing references)
			//IL_0197: Unknown result type (might be due to invalid IL or missing references)
			//IL_019c: Unknown result type (might be due to invalid IL or missing references)
			//IL_01a6: Unknown result type (might be due to invalid IL or missing references)
			//IL_01b0: Unknown result type (might be due to invalid IL or missing references)
			//IL_01ba: Unknown result type (might be due to invalid IL or missing references)
			double num = r.density * 1000f;
			float fixedDeltaTime = TimeWarp.fixedDeltaTime;
			Required = req * num / (double)fixedDeltaTime;
			RequiredAtMaxThrottle = atMax * num;
			Vector3d srf_velocity = vessel.srf_velocity;
			Vector3d val = srf_velocity + (double)fixedDeltaTime * vessel.acceleration;
			Vector3d normalized = ((Vector3d)(ref srf_velocity)).normalized;
			Vector3d normalized2 = ((Vector3d)(ref val)).normalized;
			double magnitude = ((Vector3d)(ref srf_velocity)).magnitude;
			double magnitude2 = ((Vector3d)(ref val)).magnitude;
			float num2 = (float)(vessel.altitude + (double)fixedDeltaTime * vessel.verticalSpeed);
			double staticPressurekPa = vessel.staticPressurekPa;
			double staticPressure = FlightGlobals.getStaticPressure((double)num2, (CelestialBody)null);
			double atmDensity = FlightGlobals.getAtmDensity(staticPressurekPa, vessel.externalTemperature, (CelestialBody)null);
			double atmDensity2 = FlightGlobals.getAtmDensity(staticPressure, FlightGlobals.getExternalTemperature((double)num2, (CelestialBody)null), (CelestialBody)null);
			double speedOfSound = vessel.mainBody.GetSpeedOfSound(staticPressurekPa, atmDensity);
			double speedOfSound2 = vessel.mainBody.GetSpeedOfSound(staticPressure, atmDensity2);
			float mach = ((speedOfSound > 0.0) ? ((float)(((Vector3d)(ref srf_velocity)).magnitude / speedOfSound)) : 0f);
			float mach2 = ((speedOfSound2 > 0.0) ? ((float)(((Vector3d)(ref val)).magnitude / speedOfSound2)) : 0f);
			Intakes.Clear();
			foreach (ModuleResourceIntake module in modules)
			{
				Transform intakeTransform = module.intakeTransform;
				if ((Object)(object)intakeTransform == (Object)null)
				{
					continue;
				}
				Vector3d val2 = Vector3d.op_Implicit(intakeTransform.forward);
				Vector3 val3 = fixedDeltaTime * vessel.angularVelocity;
				Vector3d intakeFwd = Vector3d.op_Implicit(Quaternion.AngleAxis(57.29578f * ((Vector3)(ref val3)).magnitude, val3) * Vector3d.op_Implicit(val2));
				double val4 = MassProvided(magnitude, normalized, atmDensity, staticPressurekPa, mach, module, val2);
				double val5 = MassProvided(magnitude2, normalized2, atmDensity2, staticPressure, mach2, module, intakeFwd);
				double val6 = Math.Min(val4, val5);
				double num3 = 0.0;
				foreach (PartResource resource in ((PartModule)module).part.Resources)
				{
					if (resource.info.id == r.id)
					{
						num3 += resource.maxAmount;
					}
				}
				num3 = num3 * num / (double)fixedDeltaTime;
				val6 = Math.Min(val6, num3);
				Intakes.Add(new IntakeData(module, val6));
			}
		}

		private double MassProvided(double vesselSpeed, Vector3d normVesselSpeed, double atmDensity, double staticPressure, float mach, ModuleResourceIntake intake, Vector3d intakeFwd)
		{
			//IL_0031: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			if ((intake.checkForOxygen && !FlightGlobals.currentMainBody.atmosphereContainsOxygen) || staticPressure < intake.kPaThreshold)
			{
				return 0.0;
			}
			double intakeSpeed = intake.intakeSpeed;
			double num = Vector3d.Dot(normVesselSpeed, intakeFwd);
			if (num < 0.0)
			{
				num = 0.0;
			}
			else if (num > 1.0)
			{
				num = 1.0;
			}
			double num2 = (intakeSpeed + num * vesselSpeed) * intake.area * intake.unitScalar * (double)intake.machCurve.Evaluate(mach);
			return atmDensity * num2 * 1000.0;
		}
	}

	public class EngineWrapper
	{
		public readonly ModuleEngines Engine;

		public float ThrustRatio
		{
			get
			{
				return Engine.thrustPercentage / 100f;
			}
			set
			{
				Engine.thrustPercentage = value * 100f;
			}
		}

		public Vector3d MaxVariableForce { get; }

		public Vector3d ConstantTorque { get; }

		public Vector3d MaxVariableTorque { get; }

		public EngineWrapper(ModuleEngines module, Vector3d maxVariableForce, Vector3d constantTorque, Vector3d maxVariableTorque)
		{
			//IL_000e: Unknown result type (might be due to invalid IL or missing references)
			//IL_000f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0015: Unknown result type (might be due to invalid IL or missing references)
			//IL_0016: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_001e: Unknown result type (might be due to invalid IL or missing references)
			Engine = module;
			MaxVariableForce = maxVariableForce;
			ConstantTorque = constantTorque;
			MaxVariableTorque = maxVariableTorque;
		}
	}

	private const double D_VELOCITY_SQR_THRESHOLD = 100.0;

	private const double D_VELOCITY_SQR_MIN_THRESHOLD = 1.0;

	private const double D_ALTITUDE_THRESHOLD = 300.0;

	private const float F_AO_A_THRESHOLD = 2f;

	public static readonly ClassContext RFModuleEnginesRFType;

	public static readonly FieldContext RFullageSetField;

	public static readonly FieldContext RFignitionsField;

	public static readonly FieldContext RFignitedField;

	public static readonly FieldContext RFullageField;

	public static readonly MethodContext RFGetUllageStabilityMethod;

	public static bool IsRealFuelsCorrectlyInitialized;

	public static DelegateContext<DoubleVesselDelegate> FARVesselDragCoeff;

	public static DelegateContext<DoubleVesselDelegate> FARVesselRefArea;

	public static DelegateContext<DoubleVesselDelegate> FARVesselTermVelEst;

	public static DelegateContext<DoubleVesselDelegate> FARVesselDynPres;

	public static DelegateContext<CalculateVesselAeroForcesDelegate> FARCalculateVesselAeroForces;

	public static bool IsFARCorrectlyInitialized;

	public static string? Message;

	private readonly EngineInfo _einfo = new EngineInfo();

	private readonly Dictionary<ModuleEngines, ModuleGimbal?> _engines = new Dictionary<ModuleEngines, ModuleGimbal>();

	private readonly IntakeInfo _iinfo = new IntakeInfo();

	public readonly List<EngineWrapper> EngineWrappers = new List<EngineWrapper>();

	public readonly List<ModuleParachute> Parachutes = new List<ModuleParachute>();

	public readonly Vector6 RCSThrustAvailable = new Vector6();

	public readonly Vector6 RCSTorqueAvailable = new Vector6();

	public readonly Dictionary<int, ResourceInfo> Resources = new Dictionary<int, ResourceInfo>();

	public readonly Vector6 TorqueControlSurface = new Vector6();

	public readonly Vector6 TorqueGimbal = new Vector6();

	public readonly Vector6 TorqueOthers = new Vector6();

	public readonly Vector6 TorqueReactionWheel = new Vector6();

	public readonly List<VesselStatePartExtension> VesselStatePartExtensions = new List<VesselStatePartExtension>();

	public readonly List<VesselStatePartModuleExtension> VesselStatePartModuleExtensions = new List<VesselStatePartModuleExtension>();

	private double _altitudeBottom;

	private bool _altitudeBottomIsCurrent;

	private double _lastAltitudeAsl;

	private float _lastAoA;

	private Vector3 _lastFarForce;

	private Vector3d _lastSurfaceVelocity;

	[ValueInfoItem("#MechJeb_Altitude_ASL", InfoItem.Category.Surface, format = "SI", siSigFigs = 6, units = "m")]
	public double AltitudeASL;

	[ValueInfoItem("#MechJeb_Altitude_true", InfoItem.Category.Surface, format = "SI", siSigFigs = 6, units = "m")]
	public double AltitudeTrue;

	[ValueInfoItem("#MechJeb_AngleToPrograde", InfoItem.Category.Orbit, format = "F2", units = "º")]
	public double AngleToPrograde;

	public Vector3d AngularMomentum;

	public Vector3d AngularVelocity;

	[ValueInfoItem("#MechJeb_AngleOfAttack", InfoItem.Category.Misc, format = "F2", units = "º")]
	public double AoA;

	[ValueInfoItem("#MechJeb_DisplacementAngle", InfoItem.Category.Misc, format = "F2", units = "º")]
	public double AoD;

	[ValueInfoItem("#MechJeb_AngleOfSideslip", InfoItem.Category.Misc, format = "F2", units = "º")]
	public double AoS;

	[ValueInfoItem("Area Drag", InfoItem.Category.Vessel, format = "SI", units = "m²")]
	public double AreaDrag;

	public double AtmosphericDensity;

	[ValueInfoItem("#MechJeb_AtmosphereDensity", InfoItem.Category.Misc, format = "SI", units = "g/m³")]
	public double AtmosphericDensityInGrams;

	[ValueInfoItem("Celestial Longitude", InfoItem.Category.Orbit, format = "F3")]
	public double CelestialLongitude;

	public Vector3d CoL;

	public double CoLWeightSum;

	public Vector3d CoM;

	public Vector3d CoT;

	public double CoTWeightSum;

	public double DeltaT;

	public Vector3d DoT;

	[ValueInfoItem("Drag Acceleration", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double DragAcceleration;

	[ValueInfoItem("Drag Force", InfoItem.Category.Vessel, format = "SI", units = "kN")]
	public double DragForce;

	[ValueInfoItem("#MechJeb_DragCoefficient", InfoItem.Category.Vessel, format = "F2")]
	public double DragCoefficient;

	[ValueInfoItem("#MechJeb_DynamicPressure", InfoItem.Category.Misc, format = "SI", units = "Pa")]
	public double DynamicPressure;

	public Vector3d East;

	public Vector3d Forward;

	[ValueInfoItem("#MechJeb_AerothermalFlux", InfoItem.Category.Vessel, format = "SI", units = "W/m²")]
	public double FreeMolecularAerothermalFlux;

	public Vector3d GravityForce;

	[ValueInfoItem("#MechJeb_Heading", InfoItem.Category.Surface, format = "F1", units = "º")]
	public double Heading;

	public Vector3d HorizontalOrbit;

	public Vector3d HorizontalSurface;

	[ValueInfoItem("#MechJeb_IntakeAir", InfoItem.Category.Vessel, format = "SI", units = "kg/s")]
	public double IntakeAir;

	[ValueInfoItem("#MechJeb_IntakeAirAllIntakes", InfoItem.Category.Vessel, format = "SI", units = "kg/s")]
	public double IntakeAirAllIntakes;

	[ValueInfoItem("#MechJeb_intakeAirAtMax", InfoItem.Category.Vessel, format = "SI", units = "kg/s")]
	public double IntakeAirAtMax;

	[ValueInfoItem("#MechJeb_IntakeAirNeeded", InfoItem.Category.Vessel, format = "SI", units = "kg/s")]
	public double IntakeAirNeeded;

	[ValueInfoItem("#MechJeb_Latitude", InfoItem.Category.Surface, format = "ANGLE_NS")]
	public double Latitude;

	[ValueInfoItem("Lift Acceleration", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double LiftAcceleration;

	[ValueInfoItem("Lift Force", InfoItem.Category.Vessel, format = "SI", units = "kN")]
	public double LiftForce;

	[ValueInfoItem("#MechJeb_LocalGravity", InfoItem.Category.Misc, format = "SI", units = "m/s²")]
	public double LocalGravity;

	[ValueInfoItem("#MechJeb_Longitude", InfoItem.Category.Surface, format = "ANGLE_EW")]
	public double Longitude;

	[ValueInfoItem("#MechJeb_Mach", InfoItem.Category.Vessel, format = "F2")]
	public double Mach;

	public CelestialBody? MainBody;

	public double Mass;

	[ValueInfoItem("#MechJeb_MaxDynamicPressure", InfoItem.Category.Misc, format = "SI", units = "Pa")]
	public double MaxDynamicPressure;

	public double MaxEngineResponseTime;

	public Vector3d MoI;

	public Vector3d NormalPlus;

	public Vector3d NormalPlusSurface;

	public Vector3d North;

	public Vector3d OrbitalPosition;

	public Vector3d OrbitalVelocity;

	[ValueInfoItem("#MechJeb_Apoapsis", InfoItem.Category.Orbit, units = "m", format = "SI", siSigFigs = 6, category = InfoItem.Category.Orbit)]
	public double OrbitApA;

	[ValueInfoItem("#MechJeb_ArgumentOfPeriapsis", InfoItem.Category.Orbit, format = "F1", units = "º")]
	public double OrbitArgumentOfPeriapsis;

	[ValueInfoItem("#MechJeb_Eccentricity", InfoItem.Category.Orbit, format = "F3")]
	public double OrbitEccentricity;

	[ValueInfoItem("#MechJeb_Inclination", InfoItem.Category.Orbit, format = "F3", units = "º")]
	public double OrbitInclination;

	[ValueInfoItem("#MechJeb_LAN", InfoItem.Category.Orbit, format = "ANGLE")]
	public double OrbitLAN;

	[ValueInfoItem("#MechJeb_Periapsis", InfoItem.Category.Orbit, units = "m", format = "SI", siSigFigs = 6, category = InfoItem.Category.Orbit)]
	public double OrbitPeA;

	[ValueInfoItem("#MechJeb_OrbitalPeriod", InfoItem.Category.Orbit, format = "TIME", timeDecimalPlaces = 2, category = InfoItem.Category.Orbit)]
	public double OrbitPeriod;

	[ValueInfoItem("#MechJeb_SemiMajorAxis", InfoItem.Category.Orbit, format = "SI", siSigFigs = 6, units = "m")]
	public double OrbitSemiMajorAxis;

	[ValueInfoItem("#MechJeb_TimeToApoapsis", InfoItem.Category.Orbit, format = "TIME", timeDecimalPlaces = 1)]
	public double OrbitTimeToAp;

	[ValueInfoItem("#MechJeb_TimeToPeriapsis", InfoItem.Category.Orbit, format = "TIME", timeDecimalPlaces = 1)]
	public double OrbitTimeToPe;

	public bool ParachuteDeployed;

	[ValueInfoItem("#MechJeb_Pitch", InfoItem.Category.Surface, format = "F1", units = "º")]
	public double Pitch;

	[ValueInfoItem("#MechJeb_PureDrag", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double PureDrag;

	public Vector3d PureDragVector;

	[ValueInfoItem("#MechJeb_PureLift", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double PureLift;

	public Vector3d PureLiftVector;

	public Vector3d RadialPlus;

	public Vector3d RadialPlusSurface;

	public double Radius;

	public bool RCSThrust;

	[ValueInfoItem("#MechJeb_Roll", InfoItem.Category.Surface, format = "F1", units = "º")]
	public double Roll;

	public Vector3d RootPartPosition;

	public Quaternion RotationSurface;

	public Quaternion RotationVesselSurface;

	[ValueInfoItem("#MechJeb_SpeedOfSound", InfoItem.Category.Vessel, format = "SI", units = "m/s")]
	public double SpeedOfSound;

	[ValueInfoItem("#MechJeb_OrbitalSpeed", InfoItem.Category.Orbit, format = "SI", units = "m/s")]
	public double SpeedOrbital;

	[ValueInfoItem("#MechJeb_OrbitHorizontalSpeed", InfoItem.Category.Orbit, format = "SI", units = "m/s")]
	public double SpeedOrbitalHorizontal;

	[ValueInfoItem("#MechJeb_SurfaceSpeed", InfoItem.Category.Surface, format = "SI", units = "m/s")]
	public double SpeedSurface;

	[ValueInfoItem("#MechJeb_SurfaceHorizontalSpeed", InfoItem.Category.Surface, format = "SI", units = "m/s")]
	public double SpeedSurfaceHorizontal;

	[ValueInfoItem("#MechJeb_VerticalSpeed", InfoItem.Category.Surface, format = "SI", units = "m/s")]
	public double SpeedVertical;

	[ValueInfoItem("#MechJeb_SurfaceAltitudeASL", InfoItem.Category.Surface, format = "SI", siSigFigs = 4, units = "m")]
	public double SurfaceAltitudeASL;

	public Vector3d SurfaceVelocity;

	public float ThrottleFixedLimit = 1f;

	public float ThrottleLimit = 1f;

	public Vector3d ThrustForward;

	public Vector3d ThrustVectorLastFrame;

	public Vector3d ThrustVectorMaxThrottle;

	public Vector3d ThrustVectorMinThrottle;

	[ValueInfoItem("#MechJeb_UniversalTime", InfoItem.Category.Recorder, format = "TIME")]
	public double Time;

	public Vector3d TorqueAvailable;

	public Vector3d TorqueDifferentialThrottle;

	public Vector3d TorqueReactionSpeed;

	public Vector3d TorqueResponseSpeed;

	public Vector3d TorqueWeightedExponentialResponseDelay;

	public Vector3d TorqueWeightedLinearResponseDelay;

	public Vector3d Up;

	public Vector3d VelocityMainBodySurface;

	private readonly MechJebCore _core;

	private Vessel _vessel => ((PartModule)_core).vessel;

	public double LowestUllage => _einfo.LowestUllage;

	public double ThrustAvailable => Vector3d.Dot(ThrustVectorMaxThrottle, Forward);

	public double ThrustMinimum => Vector3d.Dot(ThrustVectorMinThrottle, Forward);

	public double ThrustCurrent => Vector3d.Dot(ThrustVectorLastFrame, Forward);

	public double MaxThrustAcceleration => ThrustAvailable / Mass;

	public double MinThrustAcceleration => ThrustMinimum / Mass;

	public double CurrentThrustAcceleration => ThrustCurrent / Mass;

	public double LimitedMaxThrustAcceleration => MaxThrustAcceleration * (double)ThrottleFixedLimit + MinThrustAcceleration * (double)(1f - ThrottleFixedLimit);

	[ValueInfoItem("#MechJeb_Altitude_bottom", InfoItem.Category.Surface, format = "SI", siSigFigs = 6, units = "m")]
	public double AltitudeBottom
	{
		get
		{
			if (_altitudeBottomIsCurrent)
			{
				return _altitudeBottom;
			}
			_altitudeBottom = ComputeVesselBottomAltitude();
			_altitudeBottomIsCurrent = true;
			return _altitudeBottom;
		}
	}

	public VesselState(MechJebCore core)
	{
		_core = core;
	}

	static VesselState()
	{
		RFModuleEnginesRFType = ReflectionUtils.Assembly("RealFuels").Class("RealFuels.ModuleEnginesRF");
		RFullageSetField = ReflectionUtils.Assembly("RealFuels").Class("RealFuels.ModuleEnginesRF").Field("ullageSet", BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public);
		RFignitionsField = ReflectionUtils.Assembly("RealFuels").Class("RealFuels.ModuleEnginesRF").Field("ignitions", BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public);
		RFignitedField = ReflectionUtils.Assembly("RealFuels").Class("RealFuels.ModuleEnginesRF").Field("ignited", BindingFlags.Instance | BindingFlags.NonPublic);
		RFullageField = ReflectionUtils.Assembly("RealFuels").Class("RealFuels.ModuleEnginesRF").Field("ullage", BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public);
		RFGetUllageStabilityMethod = ReflectionUtils.Assembly("RealFuels").Class("RealFuels.Ullage.UllageSet").Method("GetUllageStability", BindingFlags.Instance | BindingFlags.Public, (Type[])null);
		ValidateRealFuels();
		FARVesselDragCoeff = ReflectionUtils.Assembly("FerramAerospaceResearch").Class("FerramAerospaceResearch.FARAPI").Method("VesselDragCoeff", BindingFlags.Static | BindingFlags.Public, new Type[1] { typeof(Vessel) })
			.StaticDelegate<DoubleVesselDelegate>();
		FARVesselRefArea = ReflectionUtils.Assembly("FerramAerospaceResearch").Class("FerramAerospaceResearch.FARAPI").Method("VesselRefArea", BindingFlags.Static | BindingFlags.Public, new Type[1] { typeof(Vessel) })
			.StaticDelegate<DoubleVesselDelegate>();
		FARVesselTermVelEst = ReflectionUtils.Assembly("FerramAerospaceResearch").Class("FerramAerospaceResearch.FARAPI").Method("VesselTermVelEst", BindingFlags.Static | BindingFlags.Public, new Type[1] { typeof(Vessel) })
			.StaticDelegate<DoubleVesselDelegate>();
		FARVesselDynPres = ReflectionUtils.Assembly("FerramAerospaceResearch").Class("FerramAerospaceResearch.FARAPI").Method("VesselDynPres", BindingFlags.Static | BindingFlags.Public, new Type[1] { typeof(Vessel) })
			.StaticDelegate<DoubleVesselDelegate>();
		FARCalculateVesselAeroForces = ReflectionUtils.Assembly("FerramAerospaceResearch").Class("FerramAerospaceResearch.FARAPI").Method("CalculateVesselAeroForces", BindingFlags.Static | BindingFlags.Public, new Type[5]
		{
			typeof(Vessel),
			typeof(Vector3).MakeByRefType(),
			typeof(Vector3).MakeByRefType(),
			typeof(Vector3),
			typeof(double)
		})
			.StaticDelegate<CalculateVesselAeroForcesDelegate>();
		ValidateFAR();
	}

	private static void ValidateRealFuels()
	{
		IsRealFuelsCorrectlyInitialized = ReflectionUtils.IsLoadedRealFuels && RFModuleEnginesRFType.IsValid && RFullageSetField.IsValid && RFGetUllageStabilityMethod.IsValid && RFignitionsField.IsValid && RFignitedField.IsValid && RFullageField.IsValid;
		if (!ReflectionUtils.IsLoadedRealFuels)
		{
			Debug.Log((object)"MechJeb: RealFuels Assembly is not available, prior messages are ignorable.");
		}
		else if (IsRealFuelsCorrectlyInitialized)
		{
			Debug.Log((object)"MechJeb: RealFuels Assembly is wired up properly.");
		}
		else
		{
			Debug.Log((object)"MechJeb ERROR: RealFuels integration in VesselState failed to initialize correctly, THIS IS A BUG.");
		}
	}

	private static void ValidateFAR()
	{
		IsFARCorrectlyInitialized = ReflectionUtils.IsLoadedFAR && FARVesselDragCoeff.IsValid && FARVesselRefArea.IsValid && FARVesselTermVelEst.IsValid && FARVesselDynPres.IsValid && FARCalculateVesselAeroForces.IsValid;
		if (!ReflectionUtils.IsLoadedFAR)
		{
			Debug.Log((object)"MechJeb: FAR Assembly is not available, prior messages are ignorable.");
		}
		else if (IsFARCorrectlyInitialized)
		{
			Debug.Log((object)"MechJeb: FAR Assembly is wired up properly.");
		}
		else
		{
			Debug.Log((object)"MechJeb ERROR: FAR integration in VesselState failed to initialize correctly, THIS IS A BUG.");
		}
	}

	[GeneralInfoItem("#MechJeb_DebugString", InfoItem.Category.Misc, showInEditor = true)]
	public void DebugString()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Message, Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	public bool Update()
	{
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		if (Time == Planetarium.GetUniversalTime())
		{
			return true;
		}
		if ((Object)(object)_vessel.rootPart.rb == (Object)null)
		{
			return false;
		}
		UpdateVelocityAndCoM();
		UpdateBasicInfo();
		UpdateRCSThrustAndTorque();
		EngineWrappers.Clear();
		_einfo.Update(CoM, _vessel);
		_iinfo.Update();
		AnalyzeParts(_einfo, _iinfo);
		UpdateResourceRequirements(_einfo, _iinfo);
		ToggleRCSThrust();
		UpdateMoIAndAngularMom();
		return true;
	}

	private void UpdateVelocityAndCoM()
	{
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		Mass = _vessel.totalMass;
		CoM = _vessel.CoMD;
		OrbitalVelocity = _vessel.obt_velocity;
		OrbitalPosition = CoM - _vessel.mainBody.position;
	}

	private void UpdateBasicInfo()
	{
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0070: Unknown result type (might be due to invalid IL or missing references)
		//IL_0081: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00db: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0108: Unknown result type (might be due to invalid IL or missing references)
		//IL_010d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0112: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Unknown result type (might be due to invalid IL or missing references)
		//IL_011e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_0129: Unknown result type (might be due to invalid IL or missing references)
		//IL_012e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0133: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0145: Unknown result type (might be due to invalid IL or missing references)
		//IL_014a: Unknown result type (might be due to invalid IL or missing references)
		//IL_014f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0153: Unknown result type (might be due to invalid IL or missing references)
		//IL_0158: Unknown result type (might be due to invalid IL or missing references)
		//IL_015f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0173: Unknown result type (might be due to invalid IL or missing references)
		//IL_0178: Unknown result type (might be due to invalid IL or missing references)
		//IL_0184: Unknown result type (might be due to invalid IL or missing references)
		//IL_0189: Unknown result type (might be due to invalid IL or missing references)
		//IL_018e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0195: Unknown result type (might be due to invalid IL or missing references)
		//IL_019b: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0201: Unknown result type (might be due to invalid IL or missing references)
		//IL_0206: Unknown result type (might be due to invalid IL or missing references)
		//IL_020b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0210: Unknown result type (might be due to invalid IL or missing references)
		//IL_0228: Unknown result type (might be due to invalid IL or missing references)
		//IL_022d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0232: Unknown result type (might be due to invalid IL or missing references)
		//IL_026c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0272: Unknown result type (might be due to invalid IL or missing references)
		//IL_0283: Unknown result type (might be due to invalid IL or missing references)
		//IL_0289: Unknown result type (might be due to invalid IL or missing references)
		//IL_028e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0293: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_02af: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_02be: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_030d: Unknown result type (might be due to invalid IL or missing references)
		//IL_031a: Unknown result type (might be due to invalid IL or missing references)
		//IL_032a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_036d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0372: Unknown result type (might be due to invalid IL or missing references)
		//IL_0382: Unknown result type (might be due to invalid IL or missing references)
		//IL_0387: Unknown result type (might be due to invalid IL or missing references)
		//IL_038c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0398: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_03b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0416: Unknown result type (might be due to invalid IL or missing references)
		//IL_0421: Unknown result type (might be due to invalid IL or missing references)
		//IL_0426: Unknown result type (might be due to invalid IL or missing references)
		//IL_0484: Unknown result type (might be due to invalid IL or missing references)
		//IL_049b: Unknown result type (might be due to invalid IL or missing references)
		//IL_04cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0511: Unknown result type (might be due to invalid IL or missing references)
		//IL_04fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0538: Unknown result type (might be due to invalid IL or missing references)
		//IL_0752: Unknown result type (might be due to invalid IL or missing references)
		//IL_0757: Unknown result type (might be due to invalid IL or missing references)
		//IL_075c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0762: Unknown result type (might be due to invalid IL or missing references)
		//IL_077e: Unknown result type (might be due to invalid IL or missing references)
		//IL_079b: Unknown result type (might be due to invalid IL or missing references)
		//IL_07e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_07ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_07ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_07f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_0805: Unknown result type (might be due to invalid IL or missing references)
		//IL_080a: Unknown result type (might be due to invalid IL or missing references)
		//IL_080e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0813: Unknown result type (might be due to invalid IL or missing references)
		//IL_0852: Unknown result type (might be due to invalid IL or missing references)
		//IL_0857: Unknown result type (might be due to invalid IL or missing references)
		//IL_0859: Unknown result type (might be due to invalid IL or missing references)
		Time = Planetarium.GetUniversalTime();
		DeltaT = TimeWarp.fixedDeltaTime;
		Up = ((Vector3d)(ref OrbitalPosition)).normalized;
		Rigidbody rb = _vessel.rootPart.rb;
		if ((Object)(object)rb != (Object)null)
		{
			RootPartPosition = Vector3d.op_Implicit(rb.position);
		}
		North = _vessel.north;
		East = _vessel.east;
		Forward = Vector3d.op_Implicit(_vessel.GetTransform().up);
		RotationSurface = Quaternion.LookRotation(Vector3d.op_Implicit(North), Vector3d.op_Implicit(Up));
		RotationVesselSurface = Quaternion.Inverse(Quaternion.Euler(90f, 0f, 0f) * Quaternion.Inverse(_vessel.GetTransform().rotation) * RotationSurface);
		SurfaceVelocity = OrbitalVelocity - _vessel.mainBody.getRFrmVel(CoM);
		VelocityMainBodySurface = Vector3d.op_Implicit(RotationSurface * Vector3d.op_Implicit(SurfaceVelocity));
		Vector3d val = Vector3d.Exclude(Up, OrbitalVelocity);
		HorizontalOrbit = ((Vector3d)(ref val)).normalized;
		val = Vector3d.Exclude(Up, SurfaceVelocity);
		HorizontalSurface = ((Vector3d)(ref val)).normalized;
		AngularVelocity = Vector3d.op_Implicit(_vessel.angularVelocity);
		val = Vector3d.Exclude(SurfaceVelocity, Up);
		RadialPlusSurface = ((Vector3d)(ref val)).normalized;
		val = Vector3d.Exclude(OrbitalVelocity, Up);
		RadialPlus = ((Vector3d)(ref val)).normalized;
		NormalPlusSurface = -Vector3d.Cross(RadialPlusSurface, ((Vector3d)(ref SurfaceVelocity)).normalized);
		NormalPlus = -Vector3d.Cross(RadialPlus, ((Vector3d)(ref OrbitalVelocity)).normalized);
		Mach = _vessel.mach;
		GravityForce = FlightGlobals.getGeeForceAtPosition(CoM);
		LocalGravity = ((Vector3d)(ref GravityForce)).magnitude;
		SpeedOrbital = ((Vector3d)(ref OrbitalVelocity)).magnitude;
		SpeedSurface = ((Vector3d)(ref SurfaceVelocity)).magnitude;
		SpeedVertical = Vector3d.Dot(SurfaceVelocity, Up);
		val = Vector3d.Exclude(Up, SurfaceVelocity);
		SpeedSurfaceHorizontal = ((Vector3d)(ref val)).magnitude;
		val = OrbitalVelocity - SpeedVertical * Up;
		SpeedOrbitalHorizontal = ((Vector3d)(ref val)).magnitude;
		Vector3 val2 = Vector3.ProjectOnPlane(Vector3d.op_Implicit(((Vector3d)(ref SurfaceVelocity)).normalized), _vessel.ReferenceTransform.right);
		double num = 180.0 / Math.PI * Math.Atan2(Vector3.Dot(((Vector3)(ref val2)).normalized, _vessel.ReferenceTransform.forward), Vector3.Dot(((Vector3)(ref val2)).normalized, _vessel.ReferenceTransform.up));
		AoA = ((double.IsNaN(num) || SpeedSurface < 0.01) ? 0.0 : num);
		val2 = Vector3.ProjectOnPlane(Vector3d.op_Implicit(((Vector3d)(ref SurfaceVelocity)).normalized), _vessel.ReferenceTransform.forward);
		double num2 = 180.0 / Math.PI * Math.Atan2(Vector3.Dot(((Vector3)(ref val2)).normalized, _vessel.ReferenceTransform.right), Vector3.Dot(((Vector3)(ref val2)).normalized, _vessel.ReferenceTransform.up));
		AoS = ((double.IsNaN(num2) || SpeedSurface < 0.01) ? 0.0 : num2);
		double num3 = 180.0 / Math.PI * Math.Acos(Statics.Clamp((double)Vector3.Dot(_vessel.ReferenceTransform.up, Vector3d.op_Implicit(((Vector3d)(ref SurfaceVelocity)).normalized)), -1.0, 1.0));
		AoD = ((double.IsNaN(num3) || SpeedSurface < 0.01) ? 0.0 : num3);
		Heading = ((Quaternion)(ref RotationVesselSurface)).eulerAngles.y;
		Pitch = ((((Quaternion)(ref RotationVesselSurface)).eulerAngles.x > 180f) ? (360.0 - (double)((Quaternion)(ref RotationVesselSurface)).eulerAngles.x) : ((double)(0f - ((Quaternion)(ref RotationVesselSurface)).eulerAngles.x)));
		Roll = ((((Quaternion)(ref RotationVesselSurface)).eulerAngles.z > 180f) ? ((double)((Quaternion)(ref RotationVesselSurface)).eulerAngles.z - 360.0) : ((double)((Quaternion)(ref RotationVesselSurface)).eulerAngles.z));
		AltitudeASL = _vessel.mainBody.GetAltitude(CoM);
		SurfaceAltitudeASL = (((Object)(object)_vessel.mainBody.pqsController != (Object)null) ? _vessel.pqsAltitude : 0.0);
		if (_vessel.mainBody.ocean && SurfaceAltitudeASL < 0.0)
		{
			SurfaceAltitudeASL = 0.0;
		}
		AltitudeTrue = AltitudeASL - SurfaceAltitudeASL;
		_altitudeBottomIsCurrent = false;
		double staticPressure = FlightGlobals.getStaticPressure(AltitudeASL, _vessel.mainBody);
		double externalTemperature = FlightGlobals.getExternalTemperature(AltitudeASL, (CelestialBody)null);
		AtmosphericDensity = FlightGlobals.getAtmDensity(staticPressure, externalTemperature, (CelestialBody)null);
		AtmosphericDensityInGrams = AtmosphericDensity * 1000.0;
		DynamicPressure = GetDynamicPressure();
		if (DynamicPressure > MaxDynamicPressure)
		{
			MaxDynamicPressure = DynamicPressure;
		}
		FreeMolecularAerothermalFlux = 0.5 * AtmosphericDensity * SpeedSurface * SpeedSurface * SpeedSurface;
		SpeedOfSound = _vessel.speedOfSound;
		OrbitApA = _vessel.orbit.ApA;
		OrbitPeA = _vessel.orbit.PeA;
		OrbitPeriod = _vessel.orbit.period;
		OrbitTimeToAp = _vessel.orbit.timeToAp;
		OrbitTimeToPe = _vessel.orbit.timeToPe;
		OrbitLAN = _vessel.orbit.LAN;
		OrbitArgumentOfPeriapsis = _vessel.orbit.argumentOfPeriapsis;
		OrbitInclination = _vessel.orbit.inclination;
		OrbitEccentricity = _vessel.orbit.eccentricity;
		OrbitSemiMajorAxis = _vessel.orbit.semiMajorAxis;
		CelestialLongitude = Planetarium.right.AngleInPlane(-Planetarium.up, OrbitalPosition);
		Latitude = _vessel.mainBody.GetLatitude(CoM, false);
		Longitude = MuUtils.ClampDegrees180(_vessel.mainBody.GetLongitude(CoM, false));
		if ((Object)(object)_vessel.mainBody != (Object)(object)Planetarium.fetch.Sun)
		{
			val = _vessel.mainBody.orbit.getOrbitalVelocityAtUT(Time);
			Vector3d xzy = ((Vector3d)(ref val)).xzy;
			val = _vessel.mainBody.orbit.GetOrbitNormal();
			Vector3d xzy2 = ((Vector3d)(ref val)).xzy;
			AngleToPrograde = MuUtils.ClampDegrees360((double)((_vessel.orbit.inclination > 90.0 || _vessel.orbit.inclination < -90.0) ? 1 : (-1)) * OrbitalPosition.AngleInPlane(xzy2, xzy));
		}
		else
		{
			AngleToPrograde = 0.0;
		}
		MainBody = _vessel.mainBody;
		Radius = ((Vector3d)(ref OrbitalPosition)).magnitude;
	}

	private double GetDynamicPressure()
	{
		if (ReflectionUtils.IsLoadedFAR)
		{
			return FARVesselDynPres.Call(_vessel) * 1000.0;
		}
		return _vessel.dynamicPressurekPa * 1000.0;
	}

	private void UpdateRCSThrustAndTorque()
	{
		//IL_0120: Unknown result type (might be due to invalid IL or missing references)
		//IL_0125: Unknown result type (might be due to invalid IL or missing references)
		//IL_012a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_024a: Unknown result type (might be due to invalid IL or missing references)
		//IL_024f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0250: Unknown result type (might be due to invalid IL or missing references)
		//IL_026e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0273: Unknown result type (might be due to invalid IL or missing references)
		//IL_0260: Unknown result type (might be due to invalid IL or missing references)
		//IL_0265: Unknown result type (might be due to invalid IL or missing references)
		//IL_0278: Unknown result type (might be due to invalid IL or missing references)
		//IL_027d: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0323: Unknown result type (might be due to invalid IL or missing references)
		//IL_0328: Unknown result type (might be due to invalid IL or missing references)
		//IL_032a: Unknown result type (might be due to invalid IL or missing references)
		//IL_032f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0334: Unknown result type (might be due to invalid IL or missing references)
		//IL_0339: Unknown result type (might be due to invalid IL or missing references)
		//IL_034c: Unknown result type (might be due to invalid IL or missing references)
		//IL_034e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0353: Unknown result type (might be due to invalid IL or missing references)
		//IL_0358: Unknown result type (might be due to invalid IL or missing references)
		//IL_035a: Unknown result type (might be due to invalid IL or missing references)
		//IL_035f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0306: Unknown result type (might be due to invalid IL or missing references)
		//IL_0308: Unknown result type (might be due to invalid IL or missing references)
		//IL_030d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0312: Unknown result type (might be due to invalid IL or missing references)
		//IL_0314: Unknown result type (might be due to invalid IL or missing references)
		//IL_0319: Unknown result type (might be due to invalid IL or missing references)
		//IL_02af: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
		RCSThrustAvailable.Reset();
		RCSTorqueAvailable.Reset();
		if (!_vessel.ActionGroups[(KSPActionGroup)8])
		{
			return;
		}
		MechJebModuleRCSBalancer rcsbal = _vessel.GetMasterMechJeb().Rcsbal;
		if (rcsbal.Enabled)
		{
			Vector3d zero = Vector3d.zero;
			Vector6.Direction[] values = Vector6.Values;
			foreach (Vector6.Direction direction in values)
			{
				Vector3d val = Vector6.Directions[(int)direction];
				rcsbal.GetThrottles(Vector3d.op_Implicit(val), out var throttles, out var thrusters);
				if (throttles == null)
				{
					continue;
				}
				for (int j = 0; j < throttles.Length; j++)
				{
					if (!(throttles[j] <= 0.0))
					{
						Vector3d val2 = Vector3d.op_Implicit(thrusters[j].GetThrust(Vector3d.op_Implicit(val), Vector3d.op_Implicit(zero)));
						RCSThrustAvailable.Add(Vector3d.op_Implicit(_vessel.GetTransform().InverseTransformDirection(Vector3d.op_Implicit(val * Vector3d.Dot(val2 * throttles[j], val)))));
					}
				}
			}
		}
		Vector3d val3 = Vector3d.op_Implicit(_vessel.CurrentCoM);
		Vector3 val5 = default(Vector3);
		Vector3 val6 = default(Vector3);
		foreach (Part part in _vessel.parts)
		{
			foreach (PartModule module in part.Modules)
			{
				ModuleRCS val4 = (ModuleRCS)(object)((module is ModuleRCS) ? module : null);
				if ((Object)(object)val4 == (Object)null || part.ShieldedFromAirstream || !val4.rcsEnabled || !((PartModule)val4).isEnabled || val4.isJustForShow || val4.flameout || !val4.rcs_active)
				{
					continue;
				}
				((Vector3)(ref val5))._002Ector((float)(val4.enablePitch ? 1 : 0), (float)(val4.enableRoll ? 1 : 0), (float)(val4.enableYaw ? 1 : 0));
				((Vector3)(ref val6))._002Ector(val4.enableX ? 1f : 0f, (float)(val4.enableZ ? 1 : 0), (float)(val4.enableY ? 1 : 0));
				foreach (Transform thrusterTransform in val4.thrusterTransforms)
				{
					if (!((Component)thrusterTransform).gameObject.activeInHierarchy)
					{
						continue;
					}
					Vector3d val7 = thrusterTransform.position - val3;
					Vector3d val8 = Vector3d.op_Implicit(val4.useZaxis ? (-thrusterTransform.forward) : (-thrusterTransform.up));
					float num = val4.thrusterPower * val4.thrustPercentage * 0.01f;
					if (FlightInputHandler.fetch.precisionMode)
					{
						if (val4.useLever)
						{
							float leverDistance = val4.GetLeverDistance(thrusterTransform, Vector3d.op_Implicit(val8), Vector3d.op_Implicit(val3));
							if (leverDistance > 1f)
							{
								num /= leverDistance;
							}
						}
						else
						{
							num *= val4.precisionFactor;
						}
					}
					Vector3d val9 = val8 * (double)num;
					if (!rcsbal.Enabled)
					{
						RCSThrustAvailable.Add(Vector3d.op_Implicit(Vector3.Scale(_vessel.GetTransform().InverseTransformDirection(Vector3d.op_Implicit(val9)), val6)));
					}
					Vector3d val10 = Vector3d.op_Implicit(Vector3.Cross(Vector3d.op_Implicit(val7), Vector3d.op_Implicit(val9)));
					RCSTorqueAvailable.Add(Vector3d.op_Implicit(Vector3.Scale(_vessel.GetTransform().InverseTransformDirection(Vector3d.op_Implicit(val10)), val5)));
				}
			}
		}
	}

	[GeneralInfoItem("#MechJeb_RCSTranslation", InfoItem.Category.Vessel, showInEditor = true)]
	public void RCSTranslation()
	{
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_RCSTranslation"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Pos", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(RCSThrustAvailable.Positive), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Neg", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(RCSThrustAvailable.Negative), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_RCSTorque", InfoItem.Category.Vessel, showInEditor = true)]
	public void RCSTorque()
	{
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_RCSTorque"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Pos", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(RCSTorqueAvailable.Positive), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Neg", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(RCSTorqueAvailable.Negative), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}

	private void CalculateVesselAeroForcesWithCache(Vessel v, out Vector3 farForce, Vector3d surfaceVelocity, double altitudeASL)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		float num = Vector3.Angle(((Component)v.rootPart).transform.TransformDirection(Vector3.up), Vector3d.op_Implicit(surfaceVelocity));
		Vector3d val = _lastSurfaceVelocity - surfaceVelocity;
		if (((Vector3d)(ref val)).sqrMagnitude > 100.0 || Math.Abs(altitudeASL - _lastAltitudeAsl) > 300.0 || (((Vector3d)(ref surfaceVelocity)).sqrMagnitude > 1.0 && num - _lastAoA > 2f))
		{
			FARCalculateVesselAeroForces.Call(v, out farForce, out var _, Vector3d.op_Implicit(surfaceVelocity), altitudeASL);
			_lastSurfaceVelocity = surfaceVelocity;
			_lastAltitudeAsl = altitudeASL;
			_lastAoA = num;
			_lastFarForce = farForce;
		}
		else
		{
			farForce = _lastFarForce;
		}
	}

	private void AnalyzeParts(EngineInfo einfo, IntakeInfo iinfo)
	{
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0102: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_0146: Unknown result type (might be due to invalid IL or missing references)
		//IL_014b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_0154: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0181: Unknown result type (might be due to invalid IL or missing references)
		//IL_0194: Unknown result type (might be due to invalid IL or missing references)
		//IL_0199: Unknown result type (might be due to invalid IL or missing references)
		//IL_019e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_01af: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0831: Unknown result type (might be due to invalid IL or missing references)
		//IL_083c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0847: Unknown result type (might be due to invalid IL or missing references)
		//IL_084c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0851: Unknown result type (might be due to invalid IL or missing references)
		//IL_0856: Unknown result type (might be due to invalid IL or missing references)
		//IL_085d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0868: Unknown result type (might be due to invalid IL or missing references)
		//IL_0873: Unknown result type (might be due to invalid IL or missing references)
		//IL_0878: Unknown result type (might be due to invalid IL or missing references)
		//IL_087d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0882: Unknown result type (might be due to invalid IL or missing references)
		//IL_0889: Unknown result type (might be due to invalid IL or missing references)
		//IL_0894: Unknown result type (might be due to invalid IL or missing references)
		//IL_089f: Unknown result type (might be due to invalid IL or missing references)
		//IL_08a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_08a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_08ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_08b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_08c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_08cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_08d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_08d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_08da: Unknown result type (might be due to invalid IL or missing references)
		//IL_08e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_08ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_08f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_08fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0901: Unknown result type (might be due to invalid IL or missing references)
		//IL_0906: Unknown result type (might be due to invalid IL or missing references)
		//IL_0912: Unknown result type (might be due to invalid IL or missing references)
		//IL_091d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0922: Unknown result type (might be due to invalid IL or missing references)
		//IL_0927: Unknown result type (might be due to invalid IL or missing references)
		//IL_09fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a02: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a17: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a1c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a21: Unknown result type (might be due to invalid IL or missing references)
		//IL_095b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0961: Unknown result type (might be due to invalid IL or missing references)
		//IL_0966: Unknown result type (might be due to invalid IL or missing references)
		//IL_096b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0977: Unknown result type (might be due to invalid IL or missing references)
		//IL_097c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0988: Unknown result type (might be due to invalid IL or missing references)
		//IL_098e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0993: Unknown result type (might be due to invalid IL or missing references)
		//IL_0998: Unknown result type (might be due to invalid IL or missing references)
		//IL_099f: Unknown result type (might be due to invalid IL or missing references)
		//IL_09a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_09aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_09af: Unknown result type (might be due to invalid IL or missing references)
		//IL_09b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_09bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_09c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_09c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_09ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_09cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_09e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_09ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_09f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_09f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a28: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a2d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a34: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a39: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a40: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a45: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ac1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ac6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a5d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a68: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a6d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a74: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a7a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a7f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a84: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a88: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a8d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a93: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a99: Unknown result type (might be due to invalid IL or missing references)
		//IL_0afe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b03: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b0e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b13: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b18: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b1d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b22: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ade: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ae9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0aee: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ab0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ab5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c3a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c40: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c45: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c4a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c4e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c59: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c5e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c65: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c70: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c75: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c9d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ca5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0caa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ccd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ccf: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b3e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b43: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b5c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b6c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b6e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b79: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b7e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b8e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b93: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b98: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b9d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bc1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bc9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bce: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bdf: Unknown result type (might be due to invalid IL or missing references)
		//IL_0be1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0be6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bec: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bed: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bf2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c16: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c1e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c23: Unknown result type (might be due to invalid IL or missing references)
		//IL_027e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0282: Unknown result type (might be due to invalid IL or missing references)
		//IL_0287: Unknown result type (might be due to invalid IL or missing references)
		//IL_028c: Unknown result type (might be due to invalid IL or missing references)
		//IL_028e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0292: Unknown result type (might be due to invalid IL or missing references)
		//IL_0297: Unknown result type (might be due to invalid IL or missing references)
		//IL_029c: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02db: Unknown result type (might be due to invalid IL or missing references)
		//IL_074b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0750: Unknown result type (might be due to invalid IL or missing references)
		//IL_0752: Unknown result type (might be due to invalid IL or missing references)
		//IL_0757: Unknown result type (might be due to invalid IL or missing references)
		//IL_075e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0763: Unknown result type (might be due to invalid IL or missing references)
		//IL_0765: Unknown result type (might be due to invalid IL or missing references)
		//IL_076a: Unknown result type (might be due to invalid IL or missing references)
		//IL_076f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0771: Unknown result type (might be due to invalid IL or missing references)
		//IL_0773: Unknown result type (might be due to invalid IL or missing references)
		//IL_0778: Unknown result type (might be due to invalid IL or missing references)
		//IL_077a: Unknown result type (might be due to invalid IL or missing references)
		//IL_077f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0784: Unknown result type (might be due to invalid IL or missing references)
		//IL_0789: Unknown result type (might be due to invalid IL or missing references)
		//IL_078b: Unknown result type (might be due to invalid IL or missing references)
		//IL_078d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0792: Unknown result type (might be due to invalid IL or missing references)
		//IL_0393: Unknown result type (might be due to invalid IL or missing references)
		//IL_0399: Invalid comparison between Unknown and I4
		//IL_039d: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a3: Invalid comparison between Unknown and I4
		//IL_03c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_07ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_07d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_07db: Unknown result type (might be due to invalid IL or missing references)
		//IL_07e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_07ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_07f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_07f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_07fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0804: Unknown result type (might be due to invalid IL or missing references)
		//IL_0809: Unknown result type (might be due to invalid IL or missing references)
		//IL_080e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0489: Unknown result type (might be due to invalid IL or missing references)
		//IL_048b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0490: Unknown result type (might be due to invalid IL or missing references)
		//IL_0495: Unknown result type (might be due to invalid IL or missing references)
		//IL_049a: Unknown result type (might be due to invalid IL or missing references)
		//IL_04a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_04a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_04a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_041f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0421: Unknown result type (might be due to invalid IL or missing references)
		//IL_0426: Unknown result type (might be due to invalid IL or missing references)
		//IL_042b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0430: Unknown result type (might be due to invalid IL or missing references)
		//IL_0438: Unknown result type (might be due to invalid IL or missing references)
		//IL_043a: Unknown result type (might be due to invalid IL or missing references)
		//IL_043f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0444: Unknown result type (might be due to invalid IL or missing references)
		//IL_0449: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_04d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_04d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_04da: Unknown result type (might be due to invalid IL or missing references)
		//IL_04dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_04eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_066f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0671: Unknown result type (might be due to invalid IL or missing references)
		//IL_0681: Unknown result type (might be due to invalid IL or missing references)
		//IL_0683: Unknown result type (might be due to invalid IL or missing references)
		//IL_058a: Unknown result type (might be due to invalid IL or missing references)
		//IL_058c: Unknown result type (might be due to invalid IL or missing references)
		//IL_059c: Unknown result type (might be due to invalid IL or missing references)
		//IL_059e: Unknown result type (might be due to invalid IL or missing references)
		//IL_05a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_05e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_05ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_05ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_05f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_05f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_05fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0602: Unknown result type (might be due to invalid IL or missing references)
		//IL_0609: Unknown result type (might be due to invalid IL or missing references)
		//IL_060b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0610: Unknown result type (might be due to invalid IL or missing references)
		//IL_0615: Unknown result type (might be due to invalid IL or missing references)
		//IL_061a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0635: Unknown result type (might be due to invalid IL or missing references)
		//IL_0637: Unknown result type (might be due to invalid IL or missing references)
		//IL_063c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0641: Unknown result type (might be due to invalid IL or missing references)
		//IL_0643: Unknown result type (might be due to invalid IL or missing references)
		//IL_0648: Unknown result type (might be due to invalid IL or missing references)
		//IL_064d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0652: Unknown result type (might be due to invalid IL or missing references)
		Parachutes.Clear();
		ParachuteDeployed = false;
		TorqueAvailable = Vector3d.zero;
		Vector6 vector = new Vector6();
		Vector6 vector2 = new Vector6();
		Vector6 vector3 = new Vector6();
		TorqueReactionWheel.Reset();
		TorqueControlSurface.Reset();
		TorqueGimbal.Reset();
		TorqueOthers.Reset();
		PureDragVector = Vector3d.zero;
		PureLiftVector = Vector3d.zero;
		if (ReflectionUtils.IsLoadedFAR)
		{
			DragCoefficient = FARVesselDragCoeff.Call(_vessel);
			AreaDrag = FARVesselRefArea.Call(_vessel) * DragCoefficient * (double)PhysicsGlobals.DragMultiplier;
		}
		else
		{
			DragCoefficient = 0.0;
			AreaDrag = 0.0;
		}
		CoL = Vector3d.zero;
		CoLWeightSum = 0.0;
		CoT = Vector3d.zero;
		DoT = Vector3d.zero;
		CoTWeightSum = 0.0;
		ThrustForward = Vector3d.zero;
		Vector3 val5 = default(Vector3);
		Vector3 val6 = default(Vector3);
		Vector3 val13 = default(Vector3);
		Vector3 val14 = default(Vector3);
		Vector3 val15 = default(Vector3);
		Vector3 val16 = default(Vector3);
		Vector3 val17 = default(Vector3);
		Vector3 val18 = default(Vector3);
		foreach (Part part in _vessel.parts)
		{
			Vector3d val = Vector3d.op_Implicit(Vector3.zero);
			Vector3d val2 = Vector3d.op_Implicit(-part.dragVectorDir * part.dragScalar);
			if (!part.hasLiftModule)
			{
				val = Vector3d.op_Implicit(Vector3.ProjectOnPlane(((Component)part).transform.rotation * (part.bodyLiftScalar * part.DragCubes.LiftForce), -part.dragVectorDir));
			}
			if (!ReflectionUtils.IsLoadedFAR)
			{
				DragCoefficient += part.DragCubes.DragCoeff;
				AreaDrag += part.DragCubes.AreaDrag * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragMultiplier;
			}
			foreach (VesselStatePartExtension vesselStatePartExtension in VesselStatePartExtensions)
			{
				vesselStatePartExtension(part);
			}
			_engines.Clear();
			foreach (PartModule module in part.Modules)
			{
				if (!module.isEnabled)
				{
					continue;
				}
				ModuleLiftingSurface val3 = (ModuleLiftingSurface)(object)((module is ModuleLiftingSurface) ? module : null);
				if ((Object)(object)val3 != (Object)null)
				{
					val += val3.liftForce;
					val2 += val3.dragForce;
				}
				ModuleReactionWheel val4 = (ModuleReactionWheel)(object)((module is ModuleReactionWheel) ? module : null);
				if ((Object)(object)val4 != (Object)null)
				{
					val4.GetPotentialTorque(ref val5, ref val6);
					TorqueReactionWheel.Add(Vector3d.op_Implicit(val5));
					TorqueReactionWheel.Add(Vector3d.op_Implicit(-val6));
				}
				else
				{
					ModuleEngines val7 = (ModuleEngines)(object)((module is ModuleEngines) ? module : null);
					if (val7 == null)
					{
						ModuleResourceIntake val8 = (ModuleResourceIntake)(object)((module is ModuleResourceIntake) ? module : null);
						if (val8 == null)
						{
							ModuleParachute val9 = (ModuleParachute)(object)((module is ModuleParachute) ? module : null);
							if (val9 == null)
							{
								ModuleControlSurface val10 = (ModuleControlSurface)(object)((module is ModuleControlSurface) ? module : null);
								if (val10 == null)
								{
									ModuleGimbal val11 = (ModuleGimbal)(object)((module is ModuleGimbal) ? module : null);
									if (val11 == null)
									{
										if (!(module is ModuleRCS))
										{
											ITorqueProvider val12 = (ITorqueProvider)(object)((module is ITorqueProvider) ? module : null);
											if (val12 != null)
											{
												val12.GetPotentialTorque(ref val13, ref val14);
												TorqueOthers.Add(Vector3d.op_Implicit(val13));
												TorqueOthers.Add(Vector3d.op_Implicit(val14));
											}
										}
									}
									else
									{
										if (val11.engineMultsList == null)
										{
											val11.CreateEngineList();
										}
										foreach (List<KeyValuePair<ModuleEngines, float>> engineMults in val11.engineMultsList)
										{
											foreach (KeyValuePair<ModuleEngines, float> item in engineMults)
											{
												_engines[item.Key] = val11;
											}
										}
										val11.GetPotentialTorque(ref val15, ref val16);
										TorqueGimbal.Add(Vector3d.op_Implicit(val15));
										TorqueGimbal.Add(Vector3d.op_Implicit(-val16));
										float num = (float)Statics.Clamp((double)(50f - val11.gimbalResponseSpeed), 0.0, 50.0);
										if (val11.useGimbalResponseSpeed)
										{
											vector.Positive += num * val15.Abs();
											vector.Negative += num * val16.Abs();
											vector3.Add((double)(Mathf.Abs(val11.gimbalRange) / val11.gimbalResponseSpeed) * Vector3d.Max(Vector3d.op_Implicit(val15.Abs()), Vector3d.op_Implicit(val16.Abs())));
										}
									}
								}
								else
								{
									val10.GetPotentialTorque(ref val17, ref val18);
									TorqueControlSurface.Add(Vector3d.op_Implicit(val17));
									TorqueControlSurface.Add(Vector3d.op_Implicit(val18));
									if (val10.useExponentialSpeed)
									{
										float num2 = (float)Statics.Clamp((double)(50f - val10.actuatorSpeed / val10.ctrlSurfaceRange), 0.0, 50.0);
										vector.Positive = Vector3d.op_Implicit(num2 * val17.Abs());
										vector.Negative = Vector3d.op_Implicit(num2 * val18.Abs());
									}
									else
									{
										float num3 = (float)Statics.Clamp((double)(50f - 10f * val10.actuatorSpeed / val10.ctrlSurfaceRange), 0.0, 50.0);
										vector2.Positive = Vector3d.op_Implicit(num3 * val17.Abs());
										vector2.Negative = Vector3d.op_Implicit(num3 * val18.Abs());
									}
									vector3.Add((double)(Mathf.Abs(val10.ctrlSurfaceRange) / val10.actuatorSpeed) * Vector3d.Max(Vector3d.op_Implicit(val17.Abs()), Vector3d.op_Implicit(val18.Abs())));
								}
							}
							else
							{
								Parachutes.Add(val9);
								if ((int)val9.deploymentState == 3 || (int)val9.deploymentState == 2)
								{
									ParachuteDeployed = true;
								}
							}
						}
						else
						{
							iinfo.AddIntake(val8);
						}
					}
					else if (!_engines.ContainsKey(val7))
					{
						_engines.Add(val7, null);
					}
				}
				foreach (VesselStatePartModuleExtension vesselStatePartModuleExtension in VesselStatePartModuleExtensions)
				{
					vesselStatePartModuleExtension(module);
				}
			}
			foreach (KeyValuePair<ModuleEngines, ModuleGimbal> engine in _engines)
			{
				einfo.AddNewEngine(engine.Key, engine.Value, EngineWrappers, ref CoT, ref DoT, ref CoTWeightSum);
				einfo.CheckUllageStatus(engine.Key);
			}
			PureDragVector += val2;
			PureLiftVector += val;
			Vector3d val19 = val2 + val;
			Vector3d val20 = Vector3d.Project(val19, -SurfaceVelocity);
			Vector3d val21 = val19 - val20;
			double magnitude = ((Vector3d)(ref val21)).magnitude;
			if ((Object)(object)part.rb != (Object)null && magnitude > 0.01)
			{
				CoLWeightSum += magnitude;
				CoL += (Vector3d.op_Implicit(part.rb.worldCenterOfMass) + Vector3d.op_Implicit(part.partTransform.rotation * part.CoLOffset)) * magnitude;
			}
		}
		TorqueAvailable += Vector3d.Max(TorqueReactionWheel.Positive, TorqueReactionWheel.Negative);
		TorqueAvailable += Vector3d.Max(RCSTorqueAvailable.Positive, RCSTorqueAvailable.Negative);
		TorqueAvailable += Vector3d.Max(TorqueControlSurface.Positive, TorqueControlSurface.Negative);
		TorqueAvailable += Vector3d.Max(TorqueGimbal.Positive, TorqueGimbal.Negative);
		TorqueAvailable += Vector3d.Max(TorqueOthers.Positive, TorqueOthers.Negative);
		TorqueDifferentialThrottle = Vector3d.Max(einfo.TorqueDifferentialThrottle.Positive, einfo.TorqueDifferentialThrottle.Negative);
		TorqueDifferentialThrottle.y = 0.0;
		if (((Vector3d)(ref TorqueAvailable)).sqrMagnitude > 0.0)
		{
			TorqueReactionSpeed = Vector3d.Max(vector3.Positive, vector3.Negative);
			((Vector3d)(ref TorqueReactionSpeed)).Scale(TorqueAvailable.InvertNoNaN());
			TorqueWeightedExponentialResponseDelay = Vector3d.Max(vector.Positive, vector.Negative);
			TorqueWeightedLinearResponseDelay = Vector3d.Max(vector2.Positive, vector2.Negative);
			Vector3d val22 = TorqueWeightedExponentialResponseDelay + TorqueWeightedLinearResponseDelay;
			((Vector3d)(ref val22)).Scale(TorqueAvailable.InvertNoNaN());
			TorqueResponseSpeed = new Vector3(50f, 50f, 50f) - val22;
		}
		else
		{
			TorqueReactionSpeed = Vector3d.zero;
			TorqueResponseSpeed = Vector3d.op_Implicit(new Vector3(50f, 50f, 50f));
		}
		ThrustVectorMaxThrottle = einfo.ThrustMax;
		ThrustVectorMinThrottle = einfo.ThrustMin;
		ThrustVectorLastFrame = einfo.ThrustCurrent;
		if (CoTWeightSum > 0.0)
		{
			CoT /= CoTWeightSum;
			Vector3d val23 = CoM - CoT;
			ThrustForward = ((Vector3d)(ref val23)).normalized;
			if (Vector3d.Dot(ThrustForward, Forward) < 0.0)
			{
				ThrustForward = Forward;
			}
		}
		DoT = ((Vector3d)(ref DoT)).normalized;
		if (CoLWeightSum > 0.0)
		{
			CoL /= CoLWeightSum;
		}
		Vector3d val24 = -Vector3d.Cross(Vector3d.op_Implicit(((Component)_vessel).transform.right), -((Vector3d)(ref SurfaceVelocity)).normalized);
		if (ReflectionUtils.IsLoadedFAR && !_vessel.packed && SurfaceVelocity != Vector3d.zero)
		{
			CalculateVesselAeroForcesWithCache(_vessel, out var farForce, SurfaceVelocity, AltitudeASL);
			Vector3d val25 = Vector3d.Dot(Vector3d.op_Implicit(farForce), -((Vector3d)(ref SurfaceVelocity)).normalized) * -((Vector3d)(ref SurfaceVelocity)).normalized;
			DragForce = ((Vector3d)(ref val25)).magnitude;
			DragAcceleration = ((Vector3d)(ref val25)).magnitude / Mass;
			PureDragVector = val25 / Mass;
			PureDrag = DragAcceleration;
			Vector3d val26 = Vector3d.Dot(Vector3d.op_Implicit(farForce), val24) * val24;
			LiftForce = ((Vector3d)(ref val26)).magnitude;
			LiftAcceleration = ((Vector3d)(ref val26)).magnitude / Mass;
			PureLiftVector = val26 / Mass;
			PureLift = LiftAcceleration;
		}
		else
		{
			Vector3d val27 = PureDragVector + PureLiftVector;
			PureDragVector /= Mass;
			PureLiftVector /= Mass;
			PureDrag = ((Vector3d)(ref PureDragVector)).magnitude;
			PureLift = ((Vector3d)(ref PureLiftVector)).magnitude;
			DragForce = Vector3d.Dot(val27, -((Vector3d)(ref SurfaceVelocity)).normalized);
			DragAcceleration = DragForce / Mass;
			LiftForce = Vector3d.Dot(val27, val24);
			LiftAcceleration = LiftForce / Mass;
		}
		MaxEngineResponseTime = einfo.MaxResponseTime;
	}

	[GeneralInfoItem("#MechJeb_Torque", InfoItem.Category.Vessel, showInEditor = true)]
	public void TorqueCompare()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0193: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_021f: Unknown result type (might be due to invalid IL or missing references)
		Vector3d vector = Vector3d.Max(TorqueReactionWheel.Positive, TorqueReactionWheel.Negative);
		Vector3d vector2 = Vector3d.Max(RCSTorqueAvailable.Positive, RCSTorqueAvailable.Negative);
		Vector3d vector3 = Vector3d.Max(TorqueControlSurface.Positive, TorqueControlSurface.Negative);
		Vector3d vector4 = Vector3d.Max(TorqueGimbal.Positive, TorqueGimbal.Negative);
		Vector3d vector5 = Vector3d.Max(_einfo.TorqueDifferentialThrottle.Positive, _einfo.TorqueDifferentialThrottle.Negative);
		vector5.y = 0.0;
		Vector3d vector6 = Vector3d.Max(TorqueOthers.Positive, TorqueOthers.Negative);
		GUILayout.Label("Torque sources", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label("ReactionWheel", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.Label("RCS", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.Label("ControlSurface", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.Label("Gimbal", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.Label("Diff Throttle", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.Label("Others (FAR)", GuiUtils.LabelNoWrap, Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(MuUtils.PrettyPrint(vector), GuiUtils.LabelNoWrap, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(vector2), GuiUtils.LabelNoWrap, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(vector3), GuiUtils.LabelNoWrap, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(vector4), GuiUtils.LabelNoWrap, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(vector5), GuiUtils.LabelNoWrap, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(vector6), GuiUtils.LabelNoWrap, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndVertical();
		GUILayout.EndHorizontal();
	}

	private void UpdateResourceRequirements(EngineInfo einfo, IntakeInfo iinfo)
	{
		ResourceInfo.Release(Resources.Values);
		Resources.Clear();
		foreach (KeyValuePair<int, EngineInfo.FuelRequirement> item in einfo.ResourceRequired)
		{
			int key = item.Key;
			EngineInfo.FuelRequirement value = item.Value;
			Resources[key] = ResourceInfo.Borrow(PartResourceLibrary.Instance.GetDefinition(key), value.RequiredLastFrame, value.RequiredAtMaxThrottle, iinfo.GetIntakes(key), _vessel);
		}
		int id = PartResourceLibrary.Instance.GetDefinition("IntakeAir").id;
		IntakeAir = 0.0;
		IntakeAirNeeded = 0.0;
		IntakeAirAtMax = 0.0;
		IntakeAirAllIntakes = 0.0;
		if (Resources.ContainsKey(id))
		{
			IntakeAir = Resources[id].IntakeProvided;
			IntakeAirAllIntakes = Resources[id].IntakeAvailable;
			IntakeAirNeeded = Resources[id].Required;
			IntakeAirAtMax = Resources[id].RequiredAtMaxThrottle;
		}
	}

	private void ToggleRCSThrust()
	{
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		if (((Vector3d)(ref ThrustVectorMaxThrottle)).magnitude == 0.0 && _vessel.ActionGroups[(KSPActionGroup)8])
		{
			RCSThrust = true;
			ThrustVectorMaxThrottle += Forward * RCSThrustAvailable.Down;
		}
		else
		{
			RCSThrust = false;
		}
	}

	private void UpdateMoIAndAngularMom()
	{
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		MoI = Vector3d.op_Implicit(_vessel.MOI);
		AngularMomentum = Vector3d.op_Implicit(_vessel.angularMomentum);
	}

	[ValueInfoItem("#MechJeb_TerminalVelocity", InfoItem.Category.Vessel, format = "SI", units = "m/s")]
	public double TerminalVelocity()
	{
		if (ReflectionUtils.IsLoadedFAR)
		{
			return FARVesselTermVelEst.Call(_vessel);
		}
		if ((Object)(object)MainBody == (Object)null || AltitudeASL > MainBody.RealMaxAtmosphereAltitude())
		{
			return double.PositiveInfinity;
		}
		return Math.Sqrt(2000.0 * Mass * LocalGravity / (AreaDrag * _vessel.atmDensity));
	}

	public double ThrustAccel(double throttle)
	{
		return (1.0 - throttle) * MinThrustAcceleration + throttle * MaxThrustAcceleration;
	}

	public double HeadingFromDirection(Vector3d dir)
	{
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		return MuUtils.ClampDegrees360(180.0 / Math.PI * Math.Atan2(Vector3d.Dot(dir, East), Vector3d.Dot(dir, North)));
	}

	private double ComputeVesselBottomAltitude()
	{
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0076: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		if ((Object)(object)_vessel == (Object)null || (Object)(object)_vessel.rootPart.rb == (Object)null)
		{
			return 0.0;
		}
		double num = AltitudeTrue;
		foreach (Part part in _vessel.parts)
		{
			if (!((Object)(object)part.collider == (Object)null))
			{
				Bounds bounds = part.collider.bounds;
				Vector3 extents = ((Bounds)(ref bounds)).extents;
				float num2 = Mathf.Max(((Vector3)(ref extents))[0], Mathf.Max(((Vector3)(ref extents))[1], ((Vector3)(ref extents))[2]));
				double val = _vessel.mainBody.GetAltitude(Vector3d.op_Implicit(((Bounds)(ref bounds)).center)) - (double)num2 - SurfaceAltitudeASL;
				val = Math.Max(0.0, val);
				if (val < num)
				{
					num = val;
				}
			}
		}
		return num;
	}
}
