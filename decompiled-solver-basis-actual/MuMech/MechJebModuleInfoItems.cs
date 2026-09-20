using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using CompoundParts;
using KSP.Localization;
using KSP.UI.Screens;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.Utils;
using Smooth.Pools;
using UniLinq;
using UnityEngine;

namespace MuMech;

public class MechJebModuleInfoItems : ComputerModule
{
	private readonly MovingAverage rcsThrustAvg = new MovingAverage();

	private readonly MovingAverage _rcsTranslationEfficiencyAvg = new MovingAverage();

	[Persistent(pass = 4)]
	public bool showStagedMass;

	[Persistent(pass = 4)]
	public bool showBurnedMass;

	[Persistent(pass = 4)]
	public bool showInitialMass;

	[Persistent(pass = 4)]
	public bool showFinalMass;

	[Persistent(pass = 4)]
	public bool showThrust;

	public bool showRcs;

	[Persistent(pass = 4)]
	public bool showVacInitialTWR = true;

	[Persistent(pass = 4)]
	public bool showAtmoInitialTWR;

	[Persistent(pass = 4)]
	public bool showAtmoMaxTWR;

	[Persistent(pass = 4)]
	public bool showVacMaxTWR;

	[Persistent(pass = 4)]
	public bool showVacDeltaV = true;

	[Persistent(pass = 4)]
	public bool showAtmoCumulativeDeltaV;

	[Persistent(pass = 4)]
	public bool showVacCumulativeDeltaV;

	[Persistent(pass = 4)]
	public bool showControllableMass;

	[Persistent(pass = 4)]
	public bool showRcsUllageTime;

	[Persistent(pass = 4)]
	public bool showTime = true;

	[Persistent(pass = 4)]
	public bool showAtmoDeltaV = true;

	[Persistent(pass = 4)]
	public bool showISP = true;

	[Persistent(pass = 4)]
	public bool liveSLT = true;

	[Persistent(pass = 4)]
	public float altSLTScale;

	[Persistent(pass = 4)]
	public float machScale;

	[Persistent(pass = 4)]
	public int TWRBody = 1;

	[Persistent(pass = 4)]
	public int StageDisplayState;

	[Persistent(pass = 4)]
	public bool showEmpty;

	[Persistent(pass = 4)]
	public bool timeSeconds;

	private MechJebStageStatsHelper stageStatsHelper;

	private static GUIStyle _separatorStyleField;

	private List<Part> parts
	{
		get
		{
			if (!HighLogic.LoadedSceneIsEditor)
			{
				if (!((Object)(object)base.Vessel == (Object)null))
				{
					return base.Vessel.Parts;
				}
				return new List<Part>();
			}
			return EditorLogic.fetch.ship.parts;
		}
	}

	private static GUIStyle _separatorStyle
	{
		get
		{
			//IL_0020: Unknown result type (might be due to invalid IL or missing references)
			//IL_0026: Expected O, but got Unknown
			//IL_0038: Unknown result type (might be due to invalid IL or missing references)
			//IL_0048: Unknown result type (might be due to invalid IL or missing references)
			//IL_004d: Unknown result type (might be due to invalid IL or missing references)
			//IL_0059: Unknown result type (might be due to invalid IL or missing references)
			//IL_006b: Expected O, but got Unknown
			if (_separatorStyleField == null || (Object)(object)_separatorStyleField.normal.background == (Object)null)
			{
				Texture2D val = new Texture2D(1, 1);
				val.SetPixel(0, 0, new Color(0.5f, 0.5f, 0.5f));
				val.Apply();
				GUIStyle val2 = new GUIStyle();
				val2.normal.background = val;
				val2.padding.left = 50;
				_separatorStyleField = val2;
			}
			return _separatorStyleField;
		}
	}

	public MechJebModuleInfoItems(MechJebCore core)
		: base(core)
	{
	}

	[ValueInfoItem("#MechJeb_NodeBurnTime", InfoItem.Category.Misc)]
	public string NextManeuverNodeBurnTime()
	{
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Vessel.patchedConicsUnlocked() || !Enumerable.Any<ManeuverNode>((IEnumerable<ManeuverNode>)base.Vessel.patchedConicSolver.maneuverNodes))
		{
			return "N/A";
		}
		ManeuverNode obj = Enumerable.First<ManeuverNode>((IEnumerable<ManeuverNode>)base.Vessel.patchedConicSolver.maneuverNodes);
		Vector3d burnVector = obj.GetBurnVector(obj.patch);
		return GuiUtils.TimeToDHMS(((Vector3d)(ref burnVector)).magnitude / base.VesselState.LimitedMaxThrustAcceleration);
	}

	[ValueInfoItem("#MechJeb_TimeToNode", InfoItem.Category.Misc)]
	public string TimeToManeuverNode()
	{
		if (!base.Vessel.patchedConicsUnlocked() || !Enumerable.Any<ManeuverNode>((IEnumerable<ManeuverNode>)base.Vessel.patchedConicSolver.maneuverNodes))
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Vessel.patchedConicSolver.maneuverNodes[0].UT - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_NodedV", InfoItem.Category.Misc)]
	public string NextManeuverNodeDeltaV()
	{
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Vessel.patchedConicsUnlocked() || !Enumerable.Any<ManeuverNode>((IEnumerable<ManeuverNode>)base.Vessel.patchedConicSolver.maneuverNodes))
		{
			return "N/A";
		}
		Vector3d burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
		return Statics.ToSI(((Vector3d)(ref burnVector)).magnitude, 4, int.MaxValue) + "m/s";
	}

	[ValueInfoItem("#MechJeb_SurfaceTWR", InfoItem.Category.Vessel, format = "F2", showInEditor = true)]
	public double SurfaceTWR()
	{
		if (!HighLogic.LoadedSceneIsEditor)
		{
			return base.VesselState.ThrustAvailable / (base.VesselState.Mass * base.MainBody.GeeASL * 9.81);
		}
		return MaxAcceleration() / 9.81;
	}

	[ValueInfoItem("#MechJeb_LocalTWR", InfoItem.Category.Vessel, format = "F2", showInEditor = false)]
	public double LocalTWR()
	{
		return base.VesselState.ThrustAvailable / (base.VesselState.Mass * ((Vector3d)(ref base.VesselState.GravityForce)).magnitude);
	}

	[ValueInfoItem("#MechJeb_ThrottleTWR", InfoItem.Category.Vessel, format = "F2", showInEditor = false)]
	public double ThrottleTWR()
	{
		return base.VesselState.ThrustCurrent / (base.VesselState.Mass * ((Vector3d)(ref base.VesselState.GravityForce)).magnitude);
	}

	[ValueInfoItem("#MechJeb_AtmosphericPressurePa", InfoItem.Category.Misc, format = "SI", units = "Pa")]
	public double AtmosphericPressurekPa()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		return FlightGlobals.getStaticPressure(base.VesselState.CoM) * 1000.0;
	}

	[ValueInfoItem("#MechJeb_AtmosphericPressure", InfoItem.Category.Misc, format = "F3", units = "atm")]
	public double AtmosphericPressure()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		return FlightGlobals.getStaticPressure(base.VesselState.CoM) * PhysicsGlobals.KpaToAtmospheres;
	}

	[ValueInfoItem("#MechJeb_Coordinates", InfoItem.Category.Surface)]
	public string GetCoordinateString()
	{
		return Coordinates.ToStringDMS(base.VesselState.Latitude, base.VesselState.Longitude, newline: true);
	}

	private static string OrbitSummary(Orbit o)
	{
		if (!(o.eccentricity > 1.0))
		{
			return Statics.ToSI(o.PeA, 4, int.MaxValue) + "m x " + Statics.ToSI(o.ApA, 4, int.MaxValue) + "m";
		}
		return "hyperbolic, Pe = " + Statics.ToSI(o.PeA, 4, int.MaxValue) + "m";
	}

	private static string OrbitSummaryWithInclination(Orbit o)
	{
		return OrbitSummary(o) + ", inc. " + o.inclination.ToString("F1") + "º";
	}

	[ValueInfoItem("#MechJeb_MeanAnomaly", InfoItem.Category.Orbit, format = "ANGLE")]
	public double MeanAnomaly()
	{
		return base.Orbit.meanAnomaly * (180.0 / Math.PI);
	}

	[ValueInfoItem("#MechJeb_Orbit", InfoItem.Category.Orbit)]
	public string CurrentOrbitSummary()
	{
		return OrbitSummary(base.Orbit);
	}

	[ValueInfoItem("#MechJeb_TargetOrbit", InfoItem.Category.Target)]
	public string TargetOrbitSummary()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return OrbitSummary(Core.Target.TargetOrbit);
	}

	[ValueInfoItem("#MechJeb_OrbitWithInc", InfoItem.Category.Orbit, description = "#MechJeb_OrbitWithInc_desc")]
	public string CurrentOrbitSummaryWithInclination()
	{
		return OrbitSummaryWithInclination(base.Orbit);
	}

	[ValueInfoItem("#MechJeb_TargetOrbitWithInc", InfoItem.Category.Target, description = "#MechJeb_TargetOrbitWithInc_desc")]
	public string TargetOrbitSummaryWithInclination()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return OrbitSummaryWithInclination(Core.Target.TargetOrbit);
	}

	[ValueInfoItem("#MechJeb_OrbitalEnergy", InfoItem.Category.Orbit, description = "#MechJeb_OrbitalEnergy_desc", format = "SI", units = "J/kg")]
	public double OrbitalEnergy()
	{
		return base.Orbit.orbitalEnergy;
	}

	[ValueInfoItem("#MechJeb_PotentialEnergy", InfoItem.Category.Orbit, description = "#MechJeb_PotentialEnergy_desc", format = "SI", units = "J/kg")]
	public double PotentialEnergy()
	{
		return (0.0 - base.Orbit.referenceBody.gravParameter) / base.Orbit.radius;
	}

	[ValueInfoItem("#MechJeb_KineticEnergy", InfoItem.Category.Orbit, description = "#MechJeb_KineticEnergy_desc", format = "SI", units = "J/kg")]
	public double KineticEnergy()
	{
		return base.Orbit.orbitalEnergy + base.Orbit.referenceBody.gravParameter / base.Orbit.radius;
	}

	[ValueInfoItem("#MechJeb_RCSthrust", InfoItem.Category.Misc, format = "SI", units = "N")]
	public double RCSThrust()
	{
		rcsThrustAvg.Value = RCSThrustNow();
		return rcsThrustAvg.Value * 1000.0;
	}

	private double RCSThrustNow()
	{
		double num = 0.0;
		for (int i = 0; i < base.Vessel.parts.Count; i++)
		{
			Part val = base.Vessel.parts[i];
			foreach (ModuleRCS item in Enumerable.OfType<ModuleRCS>((IEnumerable)val.Modules))
			{
				if (!((Object)(object)val.Rigidbody == (Object)null) && ((PartModule)item).isEnabled && !item.isJustForShow)
				{
					for (int j = 0; j < item.thrustForces.Length; j++)
					{
						num += (double)(item.thrustForces[j] * item.thrusterPower);
					}
				}
			}
		}
		return num;
	}

	[ValueInfoItem("#MechJeb_RCSTranslationEfficiency", InfoItem.Category.Misc)]
	public string RCSTranslationEfficiency()
	{
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0132: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_013e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0143: Unknown result type (might be due to invalid IL or missing references)
		//IL_0148: Unknown result type (might be due to invalid IL or missing references)
		//IL_014d: Unknown result type (might be due to invalid IL or missing references)
		//IL_014f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		double num = RCSThrustNow();
		double num2 = 0.0;
		FlightCtrlState state = FlightInputHandler.state;
		Vector3 val = default(Vector3);
		((Vector3)(ref val))._002Ector(0f - state.X, 0f - state.Z, 0f - state.Y);
		if (num == 0.0 || ((Vector3)(ref val)).magnitude == 0f)
		{
			return "--";
		}
		((Vector3)(ref val)).Normalize();
		foreach (Part part in base.Vessel.parts)
		{
			foreach (ModuleRCS item in Enumerable.OfType<ModuleRCS>((IEnumerable)part.Modules))
			{
				if (!((Object)(object)part.Rigidbody == (Object)null) && ((PartModule)item).isEnabled && !item.isJustForShow)
				{
					Vector3 val2 = Vector3d.op_Implicit(part.Rigidbody.worldCenterOfMass - base.VesselState.CoM);
					val2 = Quaternion.Inverse(base.Vessel.GetTransform().rotation) * val2;
					for (int i = 0; i < item.thrustForces.Length; i++)
					{
						float num3 = item.thrustForces[i];
						Transform val3 = item.thrusterTransforms[i];
						Vector3 val4 = Quaternion.Inverse(base.Vessel.GetTransform().rotation) * -val3.up;
						double num4 = Vector3.Dot(val, ((Vector3)(ref val4)).normalized);
						num2 += num4 * (double)item.thrusterPower * (double)num3;
					}
				}
			}
		}
		_rcsTranslationEfficiencyAvg.Value = num2 / num;
		return (_rcsTranslationEfficiencyAvg.Value * 100.0).ToString("F2") + "%";
	}

	[ValueInfoItem("#MechJeb_RCSdV", InfoItem.Category.Vessel, format = "F1", units = "m/s", showInEditor = true)]
	public double RCSDeltaVVacuum()
	{
		double num = 0.0;
		int num2 = 0;
		double num3 = 9.81;
		double num4 = base.Vessel.TotalResourceMass("MonoPropellant");
		foreach (ModuleRCS module in base.Vessel.GetModules<ModuleRCS>())
		{
			num += (double)module.atmosphereCurve.Evaluate(0f);
			num2++;
			num3 = module.G;
		}
		double num5 = VesselMass();
		double num6 = num5 - num4;
		if (num2 == 0 || num6 <= 0.0)
		{
			return 0.0;
		}
		return num / (double)num2 * num3 * Math.Log(num5 / num6);
	}

	[ValueInfoItem("#MechJeb_AngularVelocity", InfoItem.Category.Vessel, showInEditor = false, showInFlight = true)]
	public string AngularVelocity()
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		return MuUtils.PrettyPrint(((Vector3d)(ref base.VesselState.AngularVelocity)).xzy * (180.0 / Math.PI)) + "°/s";
	}

	[ValueInfoItem("#MechJeb_CurrentAcceleration", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double CurrentAcceleration()
	{
		return CurrentThrust() / (1000.0 * VesselMass());
	}

	[ValueInfoItem("#MechJeb_CurrentThrust", InfoItem.Category.Vessel, format = "SI", units = "N")]
	public double CurrentThrust()
	{
		return base.VesselState.ThrustCurrent * 1000.0;
	}

	[ValueInfoItem("#MechJeb_TimeToSoIWwitch", InfoItem.Category.Orbit)]
	public string TimeToSOITransition()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Invalid comparison between Unknown and I4
		if ((int)base.Orbit.patchEndTransition == 1)
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Orbit.EndUT - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_SurfaceGravity", InfoItem.Category.Surface, format = "SI", units = "m/s²")]
	public double SurfaceGravity()
	{
		return base.MainBody.GeeASL * 9.81;
	}

	[ValueInfoItem("#MechJeb_EscapeVelocity", InfoItem.Category.Orbit, format = "SI", siSigFigs = 3, units = "m/s")]
	public double EscapeVelocity()
	{
		return Math.Sqrt(2.0 * base.MainBody.gravParameter / base.VesselState.Radius);
	}

	[ValueInfoItem("#MechJeb_VesselName", InfoItem.Category.Vessel, showInEditor = false)]
	public string VesselName()
	{
		return base.Vessel.vesselName;
	}

	[ValueInfoItem("#MechJeb_VesselType", InfoItem.Category.Vessel, showInEditor = false)]
	public string VesselType()
	{
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		if (!((Object)(object)base.Vessel != (Object)null))
		{
			return "-";
		}
		return EnumExtensions.displayDescription((Enum)(object)base.Vessel.vesselType);
	}

	[ValueInfoItem("#MechJeb_VesselMass", InfoItem.Category.Vessel, format = "F3", units = "t", showInEditor = true)]
	public double VesselMass()
	{
		if (HighLogic.LoadedSceneIsEditor)
		{
			return Enumerable.Sum<Part>((IEnumerable<Part>)EditorLogic.fetch.ship.parts, (Func<Part, float>)((Part p) => p.mass + p.GetResourceMass()));
		}
		return base.VesselState.Mass;
	}

	[ValueInfoItem("#MechJeb_MaxVesselMass", InfoItem.Category.Vessel, showInEditor = true, showInFlight = false)]
	public string MaximumVesselMass()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Invalid comparison between Unknown and I4
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Invalid comparison between Unknown and I4
		SpaceCenterFacility val = (SpaceCenterFacility)(((int)EditorDriver.editorFacility == 1) ? 2 : 5);
		float craftMassLimit = GameVariables.Instance.GetCraftMassLimit(ScenarioUpgradeableFacilities.GetFacilityLevel(val), (int)EditorDriver.editorFacility == 1);
		if (!(craftMassLimit < float.MaxValue))
		{
			return CachedLocalizer.Instance.MechJebInfoItemsUnlimitedText;
		}
		return $"{craftMassLimit:F3} t";
	}

	[ValueInfoItem("#MechJeb_DryMass", InfoItem.Category.Vessel, showInEditor = true, format = "F3", units = "t")]
	public double DryMass()
	{
		return Enumerable.Sum<Part>(Enumerable.Where<Part>((IEnumerable<Part>)parts, (Func<Part, bool>)((Part p) => p.IsPhysicallySignificant())), (Func<Part, float>)((Part p) => p.mass + p.GetPhysicslessChildMass()));
	}

	[ValueInfoItem("#MechJeb_LiquidFuelandOxidizerMass", InfoItem.Category.Vessel, showInEditor = true, format = "F2", units = "t")]
	public double LiquidFuelAndOxidizerMass()
	{
		return base.Vessel.TotalResourceMass("LiquidFuel") + base.Vessel.TotalResourceMass("Oxidizer");
	}

	[ValueInfoItem("#MechJeb_MonopropellantMass", InfoItem.Category.Vessel, showInEditor = true, format = "F2", units = "kg")]
	public double MonoPropellantMass()
	{
		return base.Vessel.TotalResourceMass("MonoPropellant");
	}

	[ValueInfoItem("#MechJeb_TotalElectricCharge", InfoItem.Category.Vessel, showInEditor = true, format = "SI", units = "Ah")]
	public double TotalElectricCharge()
	{
		return base.Vessel.TotalResourceAmount(PartResourceLibrary.ElectricityHashcode);
	}

	[ValueInfoItem("#MechJeb_MaxThrust", InfoItem.Category.Vessel, format = "SI", units = "N", showInEditor = true)]
	public double MaxThrust()
	{
		if (HighLogic.LoadedSceneIsEditor)
		{
			IEnumerable<ModuleEngines> enumerable = Enumerable.SelectMany<Part, ModuleEngines, ModuleEngines>(Enumerable.Where<Part>((IEnumerable<Part>)EditorLogic.fetch.ship.parts, (Func<Part, bool>)((Part part) => part.inverseStage == StageManager.LastStage)), (Func<Part, IEnumerable<ModuleEngines>>)((Part part) => Enumerable.OfType<ModuleEngines>((IEnumerable)part.Modules)), (Func<Part, ModuleEngines, ModuleEngines>)((Part part, ModuleEngines engine) => engine));
			return 1000f * Enumerable.Sum<ModuleEngines>(enumerable, (Func<ModuleEngines, float>)((ModuleEngines e) => e.minThrust + e.thrustPercentage / 100f * (e.maxThrust - e.minThrust)));
		}
		return 1000.0 * base.VesselState.ThrustAvailable;
	}

	[ValueInfoItem("#MechJeb_MinThrust", InfoItem.Category.Vessel, format = "SI", units = "N", showInEditor = true)]
	public double MinThrust()
	{
		if (HighLogic.LoadedSceneIsEditor)
		{
			IEnumerable<ModuleEngines> enumerable = Enumerable.SelectMany<Part, ModuleEngines, ModuleEngines>(Enumerable.Where<Part>((IEnumerable<Part>)EditorLogic.fetch.ship.parts, (Func<Part, bool>)((Part part) => part.inverseStage == StageManager.LastStage)), (Func<Part, IEnumerable<ModuleEngines>>)((Part part) => Enumerable.OfType<ModuleEngines>((IEnumerable)part.Modules)), (Func<Part, ModuleEngines, ModuleEngines>)((Part part, ModuleEngines engine) => engine));
			return 1000f * Enumerable.Sum<ModuleEngines>(enumerable, (Func<ModuleEngines, float>)((ModuleEngines e) => (!e.throttleLocked) ? e.minThrust : (e.minThrust + e.thrustPercentage / 100f * (e.maxThrust - e.minThrust))));
		}
		return base.VesselState.ThrustMinimum;
	}

	[ValueInfoItem("#MechJeb_MaxAcceleration", InfoItem.Category.Vessel, format = "SI", units = "m/s²", showInEditor = true)]
	public double MaxAcceleration()
	{
		return MaxThrust() / (1000.0 * VesselMass());
	}

	[ValueInfoItem("#MechJeb_MinAcceleration", InfoItem.Category.Vessel, format = "SI", units = "m/s²", showInEditor = true)]
	public double MinAcceleration()
	{
		return MinThrust() / (1000.0 * VesselMass());
	}

	[ValueInfoItem("#MechJeb_Gforce", InfoItem.Category.Vessel, format = "F4", units = "g", showInEditor = true)]
	public double Acceleration()
	{
		if (!((Object)(object)base.Vessel != (Object)null))
		{
			return 0.0;
		}
		return base.Vessel.geeForce;
	}

	[ValueInfoItem("#MechJeb_PartCount", InfoItem.Category.Vessel, showInEditor = true)]
	public int PartCount()
	{
		return parts.Count;
	}

	[ValueInfoItem("#MechJeb_MaxPartCount", InfoItem.Category.Vessel, showInEditor = true)]
	public string MaxPartCount()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Invalid comparison between Unknown and I4
		float facilityLevel = ScenarioUpgradeableFacilities.GetFacilityLevel(EditorEnumExtensions.ToFacility(EditorDriver.editorFacility));
		int partCountLimit = GameVariables.Instance.GetPartCountLimit(facilityLevel, (int)EditorDriver.editorFacility == 1);
		if (partCountLimit < int.MaxValue)
		{
			return partCountLimit.ToString();
		}
		return Localizer.Format("#MechJeb_InfoItems_UnlimitedText");
	}

	[ValueInfoItem("#MechJeb_PartCountDivideMaxParts", InfoItem.Category.Vessel, showInEditor = true)]
	public string PartCountAndMaxPartCount()
	{
		return PartCount() + " / " + MaxPartCount();
	}

	[ValueInfoItem("#MechJeb_StrutCount", InfoItem.Category.Vessel, showInEditor = true)]
	public int StrutCount()
	{
		return Enumerable.Count<Part>((IEnumerable<Part>)parts, (Func<Part, bool>)((Part p) => p is CompoundPart && Object.op_Implicit((Object)(object)p.Modules.GetModule<CModuleStrut>(0))));
	}

	[ValueInfoItem("#MechJeb_FuelLinesCount", InfoItem.Category.Vessel, showInEditor = true)]
	public int FuelLinesCount()
	{
		return Enumerable.Count<Part>((IEnumerable<Part>)parts, (Func<Part, bool>)((Part p) => p is CompoundPart && Object.op_Implicit((Object)(object)p.Modules.GetModule<CModuleFuelLine>(0))));
	}

	[ValueInfoItem("#MechJeb_VesselCost", InfoItem.Category.Vessel, showInEditor = true, format = "SI", units = "$")]
	public double VesselCost()
	{
		return Enumerable.Sum<Part>((IEnumerable<Part>)parts, (Func<Part, float>)((Part p) => p.partInfo.cost)) * 1000f;
	}

	[ValueInfoItem("#MechJeb_CrewCount", InfoItem.Category.Vessel)]
	public int CrewCount()
	{
		return base.Vessel.GetCrewCount();
	}

	[ValueInfoItem("#MechJeb_CrewCapacity", InfoItem.Category.Vessel, showInEditor = true)]
	public int CrewCapacity()
	{
		return Enumerable.Sum<Part>((IEnumerable<Part>)parts, (Func<Part, int>)((Part p) => p.CrewCapacity));
	}

	[ValueInfoItem("#MechJeb_DistanceToTarget", InfoItem.Category.Target)]
	public string TargetDistance()
	{
		if (Core.Target.Target == null)
		{
			return "N/A";
		}
		return Statics.ToSI(Core.Target.Distance, 4, int.MaxValue) + "m";
	}

	[ValueInfoItem("#MechJeb_HeadingToTarget", InfoItem.Category.Target)]
	public string HeadingToTarget()
	{
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		if (Core.Target.Target == null)
		{
			return "N/A";
		}
		return base.VesselState.HeadingFromDirection(-Core.Target.RelativePosition).ToString("F1") + "º";
	}

	[ValueInfoItem("#MechJeb_RelativeVelocity", InfoItem.Category.Target)]
	public string TargetRelativeVelocity()
	{
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		Vector3d relativeVelocity = Core.Target.RelativeVelocity;
		return Statics.ToSI(((Vector3d)(ref relativeVelocity)).magnitude, 4, int.MaxValue) + "m/s";
	}

	[ValueInfoItem("#MechJeb_TimeToClosestApproach", InfoItem.Category.Target)]
	public string TargetTimeToClosestApproach()
	{
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0102: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_013b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0140: Unknown result type (might be due to invalid IL or missing references)
		//IL_0155: Unknown result type (might be due to invalid IL or missing references)
		//IL_015a: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		if (Core.Target.Target != null && base.VesselState.AltitudeTrue < 1000.0)
		{
			return GuiUtils.TimeToDHMS(GuiUtils.FromToETA(base.Vessel.CoM, Core.Target.Transform.position));
		}
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		if (double.IsNaN(Core.Target.TargetOrbit.semiMajorAxis))
		{
			return "N/A";
		}
		if (base.VesselState.AltitudeTrue < 1000.0)
		{
			Vector3 val = ((Component)base.Vessel.mainBody).transform.position - ((Component)base.Vessel).transform.position;
			double num = ((Vector3)(ref val)).magnitude;
			val = ((Component)base.Vessel.mainBody).transform.position - Core.Target.Transform.position;
			double num2 = ((Vector3)(ref val)).magnitude;
			double num3 = Vector3d.Distance(Vector3d.op_Implicit(((Component)base.Vessel).transform.position), Vector3d.op_Implicit(Core.Target.Position));
			return GuiUtils.TimeToDHMS(Math.Acos((num * num + num2 * num2 - num3 * num3) / (2.0 * num * num2)) * base.Vessel.mainBody.Radius / base.VesselState.SpeedSurfaceHorizontal);
		}
		return GuiUtils.TimeToDHMS(base.Orbit.NextClosestApproachTime(Core.Target.TargetOrbit, base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_ClosestApproachDistance", InfoItem.Category.Target)]
	public string TargetClosestApproachDistance()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		if (base.VesselState.AltitudeTrue < 1000.0)
		{
			return "N/A";
		}
		if (double.IsNaN(Core.Target.TargetOrbit.semiMajorAxis))
		{
			return "N/A";
		}
		return Statics.ToSI(base.Orbit.NextClosestApproachDistance(Core.Target.TargetOrbit, base.VesselState.Time), 4, int.MaxValue) + "m";
	}

	[ValueInfoItem("#MechJeb_RelativeVelocityAtClosestApproach", InfoItem.Category.Target)]
	public string TargetClosestApproachRelativeVelocity()
	{
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		if (base.VesselState.AltitudeTrue < 1000.0)
		{
			return "N/A";
		}
		if (double.IsNaN(Core.Target.TargetOrbit.semiMajorAxis))
		{
			return "N/A";
		}
		try
		{
			double num = base.Orbit.NextClosestApproachTime(Core.Target.TargetOrbit, base.VesselState.Time);
			if (double.IsNaN(num))
			{
				return "N/A";
			}
			Vector3d val = base.Orbit.WorldOrbitalVelocityAtUT(num) - Core.Target.TargetOrbit.WorldOrbitalVelocityAtUT(num);
			return Statics.ToSI(((Vector3d)(ref val)).magnitude, 4, int.MaxValue) + "m/s";
		}
		catch
		{
			return "N/A";
		}
	}

	[ValueInfoItem("#MechJeb_PeriapsisInTargetSoI", InfoItem.Category.Misc)]
	public string PeriapsisInTargetSOI()
	{
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_008c: Expected O, but got Unknown
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		Orbit val = ((!base.Vessel.patchedConicsUnlocked() || !Enumerable.Any<ManeuverNode>((IEnumerable<ManeuverNode>)base.Vessel.patchedConicSolver.maneuverNodes)) ? base.Vessel.orbit : Enumerable.Last<ManeuverNode>((IEnumerable<ManeuverNode>)base.Vessel.patchedConicSolver.maneuverNodes).nextPatch);
		while (val != null && (Object)(object)val.referenceBody != (Object)(CelestialBody)base.Vessel.targetObject)
		{
			val = val.nextPatch;
		}
		if (val == null)
		{
			return "N/A";
		}
		return Statics.ToSI(val.PeA, 4, int.MaxValue) + "m";
	}

	[ValueInfoItem("#MechJeb_TargetCaptureDV", InfoItem.Category.Misc)]
	public string TargetCaptureDV()
	{
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Expected O, but got Unknown
		if (!Core.Target.NormalTargetExists || !(base.Vessel.targetObject is CelestialBody))
		{
			return "N/A";
		}
		Orbit val = base.Vessel.orbit;
		while (val != null && (Object)(object)val.referenceBody != (Object)(CelestialBody)base.Vessel.targetObject)
		{
			val = val.nextPatch;
		}
		if (val == null)
		{
			return "N/A";
		}
		double num = (val.PeR + val.referenceBody.sphereOfInfluence) / 2.0;
		double num2 = Math.Sqrt(val.referenceBody.gravParameter * (2.0 / val.PeR - 1.0 / val.semiMajorAxis));
		double num3 = Math.Sqrt(val.referenceBody.gravParameter * (2.0 / val.PeR - 1.0 / num));
		return Statics.ToSI(num2 - num3, 4, int.MaxValue) + "m/s";
	}

	[ValueInfoItem("#MechJeb_TargetApoapsis", InfoItem.Category.Target)]
	public string TargetApoapsis()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return Statics.ToSI(Core.Target.TargetOrbit.ApA, 4, int.MaxValue) + "m";
	}

	[ValueInfoItem("#MechJeb_TargetPeriapsis", InfoItem.Category.Target)]
	public string TargetPeriapsis()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return Statics.ToSI(Core.Target.TargetOrbit.PeA, 4, int.MaxValue) + "m";
	}

	[ValueInfoItem("#MechJeb_TargetInclination", InfoItem.Category.Target)]
	public string TargetInclination()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return Core.Target.TargetOrbit.inclination.ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TargetOrbitPeriod", InfoItem.Category.Target)]
	public string TargetOrbitPeriod()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(Core.Target.TargetOrbit.period);
	}

	[ValueInfoItem("#MechJeb_TargetOrbitSpeed", InfoItem.Category.Target)]
	public string TargetOrbitSpeed()
	{
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		Vector3d vel = Core.Target.TargetOrbit.GetVel();
		return Statics.ToSI(((Vector3d)(ref vel)).magnitude, 4, int.MaxValue) + "m/s";
	}

	[ValueInfoItem("#MechJeb_TargetTimeToAp", InfoItem.Category.Target)]
	public string TargetOrbitTimeToAp()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(Core.Target.TargetOrbit.timeToAp);
	}

	[ValueInfoItem("#MechJeb_TargetTimeToPe", InfoItem.Category.Target)]
	public string TargetOrbitTimeToPe()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(Core.Target.TargetOrbit.timeToPe);
	}

	[ValueInfoItem("#MechJeb_TargetLAN", InfoItem.Category.Target)]
	public string TargetLAN()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return Core.Target.TargetOrbit.LAN.ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TargetLDN", InfoItem.Category.Target)]
	public string TargetLDN()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return MuUtils.ClampDegrees360(Core.Target.TargetOrbit.LAN + 180.0).ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TargetTimeToAN", InfoItem.Category.Target)]
	public string TargetTimeToAscendingNode()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if (!Core.Target.TargetOrbit.AscendingNodeEquatorialExists())
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(Core.Target.TargetOrbit.TimeOfAscendingNodeEquatorial(base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_TargetTimeToDN", InfoItem.Category.Target)]
	public string TargetTimeToDescendingNode()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if (!Core.Target.TargetOrbit.DescendingNodeEquatorialExists())
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(Core.Target.TargetOrbit.TimeOfDescendingNodeEquatorial(base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_TargetAoP", InfoItem.Category.Target)]
	public string TargetAoP()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return Core.Target.TargetOrbit.argumentOfPeriapsis.ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TargetEccentricity", InfoItem.Category.Target)]
	public string TargetEccentricity()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(Core.Target.TargetOrbit.eccentricity);
	}

	[ValueInfoItem("#MechJeb_TargetSMA", InfoItem.Category.Target)]
	public string TargetSMA()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return Statics.ToSI(Core.Target.TargetOrbit.semiMajorAxis, 4, int.MaxValue) + "m";
	}

	[ValueInfoItem("#MechJeb_TargetMeanAnomaly", InfoItem.Category.Target, format = "ANGLE")]
	public string TargetMeanAnomaly()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		return MuUtils.ClampDegrees360(Core.Target.TargetOrbit.meanAnomaly * (180.0 / Math.PI)).ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TargetTrueLongitude", InfoItem.Category.Target)]
	public string TargetTrueLongitude()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		double num = Core.Target.TargetOrbit.LAN + Core.Target.TargetOrbit.argumentOfPeriapsis;
		return MuUtils.ClampDegrees360(Core.Target.TargetOrbit.trueAnomaly * (180.0 / Math.PI) + num).ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_SynodicPeriod", InfoItem.Category.Target)]
	public string SynodicPeriod()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Orbit.SynodicPeriod(Core.Target.TargetOrbit));
	}

	[ValueInfoItem("#MechJeb_PhaseAngleToTarget", InfoItem.Category.Target)]
	public string PhaseAngle()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		if (double.IsNaN(Core.Target.TargetOrbit.semiMajorAxis))
		{
			return "N/A";
		}
		return (360.0 - Core.Target.TargetOrbit.PhaseAngle(base.Orbit, base.VesselState.Time)).ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TargetPlanetPhaseAngle", InfoItem.Category.Target)]
	public string TargetPlanetPhaseAngle()
	{
		if (!(Core.Target.Target is CelestialBody))
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody.referenceBody)
		{
			return "N/A";
		}
		return base.MainBody.orbit.PhaseAngle(Core.Target.TargetOrbit, base.VesselState.Time).ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_RelativeInclination", InfoItem.Category.Target)]
	public string RelativeInclinationToTarget()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		return base.Orbit.RelativeInclination(Core.Target.TargetOrbit).ToString("F2") + "º";
	}

	[ValueInfoItem("#MechJeb_TimeToAN", InfoItem.Category.Target)]
	public string TimeToAscendingNodeWithTarget()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		if (!base.Orbit.AscendingNodeExists(Core.Target.TargetOrbit))
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Orbit.TimeOfAscendingNode(Core.Target.TargetOrbit, base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_TimeToDN", InfoItem.Category.Target)]
	public string TimeToDescendingNodeWithTarget()
	{
		if (!Core.Target.NormalTargetExists)
		{
			return "N/A";
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			return "N/A";
		}
		if (!base.Orbit.DescendingNodeExists(Core.Target.TargetOrbit))
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Orbit.TimeOfDescendingNode(Core.Target.TargetOrbit, base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_TimeToEquatorialAN", InfoItem.Category.Orbit)]
	public string TimeToEquatorialAscendingNode()
	{
		if (!base.Orbit.AscendingNodeEquatorialExists())
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Orbit.TimeOfAscendingNodeEquatorial(base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_TimeToEquatorialDN", InfoItem.Category.Orbit)]
	public string TimeToEquatorialDescendingNode()
	{
		if (!base.Orbit.DescendingNodeEquatorialExists())
		{
			return "N/A";
		}
		return GuiUtils.TimeToDHMS(base.Orbit.TimeOfDescendingNodeEquatorial(base.VesselState.Time) - base.VesselState.Time);
	}

	[ValueInfoItem("#MechJeb_CircularOrbitSpeed", InfoItem.Category.Orbit, format = "SI", units = "m/s")]
	public double CircularOrbitSpeed()
	{
		return base.Orbit.CircularOrbitSpeed();
	}

	[GeneralInfoItem("#MechJeb_StageStatsAll", InfoItem.Category.Vessel, showInEditor = true)]
	public void AllStageStats()
	{
		if (stageStatsHelper == null)
		{
			stageStatsHelper = new MechJebStageStatsHelper(this);
		}
		stageStatsHelper.AllStageStats();
	}

	public void UpdateItems()
	{
		stageStatsHelper?.UpdateStageStats();
	}

	[ValueInfoItem("#MechJeb_StageDv_vac", InfoItem.Category.Vessel, format = "F0", units = "m/s", showInEditor = true)]
	public double StageDeltaVVacuum()
	{
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		if (computerModule.VacStats.Count == 0)
		{
			return 0.0;
		}
		return computerModule.VacStats[computerModule.VacStats.Count - 1].DeltaV;
	}

	[ValueInfoItem("#MechJeb_StageDV_atmo", InfoItem.Category.Vessel, format = "F0", units = "m/s", showInEditor = true)]
	public double StageDeltaVAtmosphere()
	{
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		if (computerModule.AtmoStats.Count == 0)
		{
			return 0.0;
		}
		return computerModule.AtmoStats[computerModule.AtmoStats.Count - 1].DeltaV;
	}

	[ValueInfoItem("#MechJeb_StageDV_atmo_vac", InfoItem.Category.Vessel, units = "m/s", showInEditor = true)]
	public string StageDeltaVAtmosphereAndVac()
	{
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		double num = ((computerModule.AtmoStats.Count == 0) ? 0.0 : computerModule.AtmoStats[computerModule.AtmoStats.Count - 1].DeltaV);
		double num2 = ((computerModule.VacStats.Count == 0) ? 0.0 : computerModule.VacStats[computerModule.VacStats.Count - 1].DeltaV);
		return $"{num:F0}, {num2:F0}";
	}

	[ValueInfoItem("#MechJeb_StageTimeFullThrottle", InfoItem.Category.Vessel, format = "TIME", showInEditor = true)]
	public float StageTimeLeftFullThrottle()
	{
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		if (computerModule.VacStats.Count == 0 || computerModule.AtmoStats.Count == 0)
		{
			return 0f;
		}
		float num = (float)computerModule.VacStats[computerModule.VacStats.Count - 1].DeltaTime;
		float num2 = (float)computerModule.AtmoStats[computerModule.AtmoStats.Count - 1].DeltaTime;
		return Mathf.Lerp(num, num2, Mathf.Clamp01((float)FlightGlobals.getStaticPressure()));
	}

	[ValueInfoItem("#MechJeb_StageTimeCurrentThrottle", InfoItem.Category.Vessel, format = "TIME")]
	public float StageTimeLeftCurrentThrottle()
	{
		float num = StageTimeLeftFullThrottle();
		if (num == 0f)
		{
			return 0f;
		}
		return num / base.Vessel.ctrlState.mainThrottle;
	}

	[ValueInfoItem("#MechJeb_StageTimeHover", InfoItem.Category.Vessel, format = "TIME")]
	public float StageTimeLeftHover()
	{
		float num = StageTimeLeftFullThrottle();
		if (num == 0f)
		{
			return 0f;
		}
		double num2 = base.VesselState.LocalGravity / base.VesselState.MaxThrustAcceleration;
		return num / (float)num2;
	}

	[ValueInfoItem("#MechJeb_TotalDV_vacuum", InfoItem.Category.Vessel, format = "F0", units = "m/s", showInEditor = true)]
	public double TotalDeltaVVacuum()
	{
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		return Enumerable.Sum<FuelStats>((IEnumerable<FuelStats>)computerModule.VacStats, (Func<FuelStats, double>)((FuelStats s) => s.DeltaV));
	}

	[ValueInfoItem("#MechJeb_TotalDV_atmo", InfoItem.Category.Vessel, format = "F0", units = "m/s", showInEditor = true)]
	public double TotalDeltaVAtmosphere()
	{
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		return Enumerable.Sum<FuelStats>((IEnumerable<FuelStats>)computerModule.AtmoStats, (Func<FuelStats, double>)((FuelStats s) => s.DeltaV));
	}

	[ValueInfoItem("#MechJeb_TotalDV_atmo_vac", InfoItem.Category.Vessel, units = "m/s", showInEditor = true)]
	public string TotalDeltaVAtmosphereAndVac()
	{
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		double num = Enumerable.Sum<FuelStats>((IEnumerable<FuelStats>)computerModule.AtmoStats, (Func<FuelStats, double>)((FuelStats s) => s.DeltaV));
		double num2 = Enumerable.Sum<FuelStats>((IEnumerable<FuelStats>)computerModule.VacStats, (Func<FuelStats, double>)((FuelStats s) => s.DeltaV));
		return $"{num:F0}, {num2:F0}";
	}

	[GeneralInfoItem("#MechJeb_DockingGuidance_velocity", InfoItem.Category.Target)]
	public void DockingGuidanceVelocity()
	{
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_velocityNA"), Array.Empty<GUILayoutOption>());
			return;
		}
		Vector3d relativeVelocity = Core.Target.RelativeVelocity;
		double x = Vector3d.Dot(relativeVelocity, Vector3d.op_Implicit(base.Vessel.GetTransform().right));
		double x2 = Vector3d.Dot(relativeVelocity, Vector3d.op_Implicit(base.Vessel.GetTransform().forward));
		double x3 = Vector3d.Dot(relativeVelocity, Vector3d.op_Implicit(base.Vessel.GetTransform().up));
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_velocity"), Array.Empty<GUILayoutOption>());
		GUILayout.Label("X: " + MuUtils.PadPositive(x, "F2") + " m/s  [L/J]", Array.Empty<GUILayoutOption>());
		GUILayout.Label("Y: " + MuUtils.PadPositive(x2, "F2") + " m/s  [I/K]", Array.Empty<GUILayoutOption>());
		GUILayout.Label("Z: " + MuUtils.PadPositive(x3, "F2") + " m/s  [H/N]", Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_DockingGuidanceAngularVelocity", InfoItem.Category.Target)]
	public void DockingGuidanceAngularVelocity()
	{
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Expected O, but got Unknown
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0081: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f7: Unknown result type (might be due to invalid IL or missing references)
		if (!(Core.Target.Target is Vessel))
		{
			GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_label2"), Array.Empty<GUILayoutOption>());
			return;
		}
		Vessel val = (Vessel)Core.Target.Target;
		Vector3d val2 = Vector3d.op_Implicit(Quaternion.Inverse(base.Vessel.ReferenceTransform.rotation) * (val.angularVelocity - base.Vessel.angularVelocity) * 57.29578f);
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_label3"), Array.Empty<GUILayoutOption>());
		GUILayout.Label("P: " + MuUtils.PadPositive(val2.x, "F2") + " °/s", Array.Empty<GUILayoutOption>());
		GUILayout.Label("Y: " + MuUtils.PadPositive(val2.z, "F2") + " °/s", Array.Empty<GUILayoutOption>());
		GUILayout.Label("R: " + MuUtils.PadPositive(val2.y, "F2") + " °/s", Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_DockingGuidancePosition", InfoItem.Category.Target)]
	public void DockingGuidancePosition()
	{
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_label4"), Array.Empty<GUILayoutOption>());
			return;
		}
		Vector3d relativePosition = Core.Target.RelativePosition;
		double x = Vector3d.Dot(relativePosition, Vector3d.op_Implicit(base.Vessel.GetTransform().right));
		double x2 = Vector3d.Dot(relativePosition, Vector3d.op_Implicit(base.Vessel.GetTransform().forward));
		double x3 = Vector3d.Dot(relativePosition, Vector3d.op_Implicit(base.Vessel.GetTransform().up));
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_label5"), Array.Empty<GUILayoutOption>());
		GUILayout.Label("X: " + MuUtils.PadPositive(x, "F2") + " m  [L/J]", Array.Empty<GUILayoutOption>());
		GUILayout.Label("Y: " + MuUtils.PadPositive(x2, "F2") + " m  [I/K]", Array.Empty<GUILayoutOption>());
		GUILayout.Label("Z: " + MuUtils.PadPositive(x3, "F2") + " m  [H/N]", Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_AllPlanetPhaseAngles", InfoItem.Category.Orbit)]
	public void AllPlanetPhaseAngles()
	{
		Orbit orbit = base.Orbit;
		while ((Object)(object)orbit.referenceBody != (Object)(object)Planetarium.fetch.Sun)
		{
			orbit = orbit.referenceBody.orbit;
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_label6"), GuiUtils.MiddleCenterLabel, Array.Empty<GUILayoutOption>());
		for (int i = 0; i < FlightGlobals.Bodies.Count; i++)
		{
			CelestialBody val = FlightGlobals.Bodies[i];
			if (!((Object)(object)val == (Object)(object)Planetarium.fetch.Sun) && !((Object)(object)val.referenceBody != (Object)(object)Planetarium.fetch.Sun) && val.orbit != orbit)
			{
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				GUILayout.Label(val.bodyName, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
				GUILayout.Label(orbit.PhaseAngle(val.orbit, base.VesselState.Time).ToString("F2") + "º", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
				GUILayout.EndHorizontal();
			}
		}
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_AllMoonPhaseAngles", InfoItem.Category.Orbit)]
	public void AllMoonPhaseAngles()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_InfoItems_label7"), GuiUtils.MiddleCenterLabel, Array.Empty<GUILayoutOption>());
		if ((Object)(object)base.Orbit.referenceBody != (Object)(object)Planetarium.fetch.Sun)
		{
			Orbit orbit = base.Orbit;
			while ((Object)(object)orbit.referenceBody.referenceBody != (Object)(object)Planetarium.fetch.Sun)
			{
				orbit = orbit.referenceBody.orbit;
			}
			for (int i = 0; i < orbit.referenceBody.orbitingBodies.Count; i++)
			{
				CelestialBody val = orbit.referenceBody.orbitingBodies[i];
				if (val.orbit != orbit)
				{
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					GUILayout.Label(val.bodyName, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
					GUILayout.Label(orbit.PhaseAngle(val.orbit, base.VesselState.Time).ToString("F2") + "º", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GUILayout.EndHorizontal();
				}
			}
		}
		GUILayout.EndVertical();
	}

	[ValueInfoItem("#MechJeb_SurfaceBiome", InfoItem.Category.Misc, showInEditor = false)]
	public string CurrentRawBiome()
	{
		if (base.Vessel.landedAt != string.Empty)
		{
			return base.Vessel.landedAt;
		}
		return base.MainBody.GetExperimentBiomeSafe(base.Vessel.latitude, base.Vessel.longitude);
	}

	[ValueInfoItem("#MechJeb_CurrentBiome", InfoItem.Category.Misc, showInEditor = false)]
	public string CurrentBiome()
	{
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Expected I4, but got Unknown
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Invalid comparison between Unknown and I4
		if (base.Vessel.landedAt != string.Empty)
		{
			return base.Vessel.landedAt;
		}
		if ((Object)(object)base.MainBody.BiomeMap == (Object)null)
		{
			return "N/A";
		}
		string text = base.MainBody.BiomeMap.GetAtt(base.Vessel.latitude * (Math.PI / 180.0), base.Vessel.longitude * (Math.PI / 180.0)).displayname;
		if (text != "")
		{
			text = "'s " + text;
		}
		Situations situation = base.Vessel.situation;
		switch (situation - 1)
		{
		default:
			if ((int)situation != 8)
			{
				break;
			}
			if (base.Vessel.altitude < (double)base.MainBody.scienceValues.flyingAltitudeThreshold)
			{
				return Localizer.Format("#MechJeb_InfoItems_VesselSituation1", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) + text });
			}
			return Localizer.Format("#MechJeb_InfoItems_VesselSituation2", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) + text });
		case 0:
		case 3:
			return LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) + ((text == "") ? Localizer.Format("#MechJeb_InfoItems_VesselSituation5") : text);
		case 1:
			return LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) + ((text == "") ? Localizer.Format("#MechJeb_InfoItems_VesselSituation6") : text);
		case 2:
			break;
		}
		if (base.Vessel.altitude < (double)base.MainBody.scienceValues.spaceAltitudeThreshold)
		{
			return Localizer.Format("#MechJeb_InfoItems_VesselSituation3", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) + text });
		}
		return Localizer.Format("#MechJeb_InfoItems_VesselSituation4", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) + text });
	}

	[GeneralInfoItem("#MechJeb_LatLonClipbardCopy", InfoItem.Category.Misc, showInEditor = false)]
	public void LatLonClipbardCopy()
	{
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Expected O, but got Unknown
		if (GUILayout.Button(Localizer.Format("#MechJeb_InfoItems_CopytoClipboard"), Array.Empty<GUILayoutOption>()))
		{
			TextEditor val = new TextEditor();
			string text = "latitude =  " + base.VesselState.Latitude.ToString("F6") + "\nlongitude = " + base.VesselState.Longitude.ToString("F6") + "\naltitude = " + base.Vessel.altitude.ToString("F2") + "\n";
			val.text = text;
			val.SelectAll();
			val.Copy();
		}
	}

	[GeneralInfoItem("#MechJeb_PoolsStatus", InfoItem.Category.Misc, showInEditor = true)]
	public void DebugString()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		foreach (KeyValuePair<Type, PoolsStatus> item in PoolsStatus.poolsInfo)
		{
			Type type = item.Key;
			if (typeof(IDisposable).IsAssignableFrom(type))
			{
				type = type.GetGenericArguments()[0];
			}
			StringBuilder stringBuilder = StringBuilderCache.Acquire(256);
			stringBuilder.Append(type.Name);
			Type[] genericArguments = type.GetGenericArguments();
			for (int i = 0; i < genericArguments.Length; i++)
			{
				if (i == 0)
				{
					stringBuilder.Append("<");
				}
				if (i > 0)
				{
					stringBuilder.Append(",");
				}
				stringBuilder.Append(type.GetGenericArguments()[i].Name);
				if (i == genericArguments.Length - 1)
				{
					stringBuilder.Append(">");
				}
			}
			GuiUtils.SimpleLabel(StringBuilderCache.ToStringAndRelease(stringBuilder), item.Value.allocated + "/" + item.Value.maxSize);
		}
		GUILayout.EndHorizontal();
	}

	[GeneralInfoItem("#MechJeb_Separator", InfoItem.Category.Misc, showInEditor = true)]
	public void HorizontalSeparator()
	{
		GUILayout.Label("", _separatorStyle, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GUILayout.Height(2f)
		});
	}
}
