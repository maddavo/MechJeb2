using System;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public static class PartExtensions
{
	public struct Vector3Pair
	{
		public Vector3 P1;

		public Vector3 P2;

		public Vector3Pair(Vector3 point1, Vector3 point2)
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_0002: Unknown result type (might be due to invalid IL or missing references)
			//IL_0008: Unknown result type (might be due to invalid IL or missing references)
			//IL_0009: Unknown result type (might be due to invalid IL or missing references)
			P1 = point1;
			P2 = point2;
		}
	}

	public static bool HasModule<T>(this Part part) where T : PartModule
	{
		return (Object)(object)part.FindModuleImplementing<T>() != (Object)null;
	}

	public static T GetModule<T>(this Part part) where T : PartModule
	{
		return part.FindModuleImplementing<T>();
	}

	public static bool EngineHasFuel(this ModuleEngines me)
	{
		if (!me.getFlameoutState)
		{
			return !me.engineShutdown;
		}
		return false;
	}

	public static bool EngineHasFuel(this Part p)
	{
		ModuleEngines val = p.FindModuleImplementing<ModuleEngines>();
		if ((Object)(object)val != (Object)null)
		{
			return val.EngineHasFuel();
		}
		return false;
	}

	public static bool UnstableUllage(this Part p)
	{
		if (!VesselState.IsRealFuelsCorrectlyInitialized)
		{
			return false;
		}
		ModuleEngines val = p.FindModuleImplementing<ModuleEngines>();
		if (val == null)
		{
			return false;
		}
		if (val.finalThrust > 0f || val.requestedThrottle > 0f || val.getFlameoutState || val.EngineIgnited)
		{
			return false;
		}
		if (!VesselState.RFModuleEnginesRFType.IsInstance((object)val))
		{
			return false;
		}
		if (VesselState.RFignitedField.GetValue<bool>((object)val))
		{
			return false;
		}
		if (VesselState.RFignitionsField.GetValue<int>((object)val) == 0)
		{
			return false;
		}
		if (!VesselState.RFullageField.GetValue<bool>((object)val))
		{
			return false;
		}
		object value = VesselState.RFullageSetField.GetValue<object>((object)val);
		return (double)VesselState.RFGetUllageStabilityMethod.Invoke(value, Array.Empty<object>()) < 0.996;
	}

	public static bool IsDecoupler(this Part p)
	{
		if ((Object)(object)p != (Object)null)
		{
			if (!((Object)(object)p.FindModuleImplementing<ModuleDecouplerBase>() != (Object)null) && !((Object)(object)p.FindModuleImplementing<ModuleDockingNode>() != (Object)null))
			{
				return p.Modules.Contains("ProceduralFairingDecoupler");
			}
			return true;
		}
		return false;
	}

	private static bool IsUnfiredDecoupler(this ModuleDecouplerBase decoupler, out Part decoupledPart)
	{
		if (!decoupler.isDecoupled && ((PartModule)decoupler).stagingEnabled && ((PartModule)decoupler).part.stagingOn)
		{
			decoupledPart = decoupler.ExplosiveNode.attachedPart;
			if ((Object)(object)decoupledPart == (Object)(object)((PartModule)decoupler).part.parent)
			{
				decoupledPart = ((PartModule)decoupler).part;
			}
			return true;
		}
		decoupledPart = null;
		return false;
	}

	private static bool IsUnfiredDecoupler(this ModuleDockingNode mDockingNode, out Part decoupledPart)
	{
		if (mDockingNode.staged && ((PartModule)mDockingNode).stagingEnabled && ((PartModule)mDockingNode).part.stagingOn)
		{
			decoupledPart = mDockingNode.referenceNode.attachedPart;
			if ((Object)(object)decoupledPart == (Object)(object)((PartModule)mDockingNode).part.parent)
			{
				decoupledPart = ((PartModule)mDockingNode).part;
			}
			return true;
		}
		decoupledPart = null;
		return false;
	}

	private static bool IsUnfiredProceduralFairingDecoupler(this PartModule decoupler, out Part decoupledPart)
	{
		if (ReflectionUtils.IsLoadedProceduralFairing && decoupler.moduleName == "ProceduralFairingDecoupler" && !((BaseField<KSPField>)(object)((BaseFieldList<BaseField, KSPField>)(object)decoupler.Fields)["decoupled"]).GetValue<bool>((object)decoupler) && decoupler.part.stagingOn)
		{
			decoupledPart = decoupler.part;
			return true;
		}
		decoupledPart = null;
		return false;
	}

	public static bool IsUnfiredDecoupler(this PartModule m, out Part decoupledPart)
	{
		ModuleDecouplerBase val = (ModuleDecouplerBase)(object)((m is ModuleDecouplerBase) ? m : null);
		if (val != null && val.IsUnfiredDecoupler(out decoupledPart))
		{
			return true;
		}
		ModuleDockingNode val2 = (ModuleDockingNode)(object)((m is ModuleDockingNode) ? m : null);
		if (val2 != null && val2.IsUnfiredDecoupler(out decoupledPart))
		{
			return true;
		}
		if (ReflectionUtils.IsLoadedProceduralFairing && m.moduleName == "ProceduralFairingDecoupler" && m.IsUnfiredProceduralFairingDecoupler(out decoupledPart))
		{
			return true;
		}
		decoupledPart = null;
		return false;
	}

	public static bool IsProceduralFairing(this Part p)
	{
		if (!ReflectionUtils.IsLoadedProceduralFairing)
		{
			return false;
		}
		return p.Modules.Contains("ProceduralFairingDecoupler");
	}

	public static bool IsProceduralFairingPayloadFairing(this Part p)
	{
		if (!p.IsProceduralFairing())
		{
			return false;
		}
		PartModule module = (p.parent ?? throw new Exception("ProceduralFairingDecoupler parent is null--fix your root staging?")).Modules.GetModule("ProceduralFairingBase");
		if (module == null)
		{
			throw new Exception("ProceduralFairingBase not found in parent part, weird.");
		}
		return ((BaseField<KSPField>)(object)((BaseFieldList<BaseField, KSPField>)(object)module.Fields)["mode"]).GetValue<string>((object)module) == "Payload";
	}

	public static bool IsUnfiredDecoupler(this Part p, out Part decoupledPart)
	{
		foreach (PartModule module in p.Modules)
		{
			if (module.IsUnfiredDecoupler(out decoupledPart))
			{
				return true;
			}
		}
		decoupledPart = null;
		return false;
	}

	public static bool IsSepratron(this Part p)
	{
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Invalid comparison between Unknown and I4
		if (p.ActivatesEvenIfDisconnected && p.IsThrottleLockedEngine() && p.IsDecoupledInStage(p.inverseStage))
		{
			return (int)p.isControlSource == 0;
		}
		return false;
	}

	public static bool IsEngine(this Part p)
	{
		return (Object)(object)p.FindModuleImplementing<ModuleEngines>() != (Object)null;
	}

	public static bool IsThrottleLockedEngine(this Part p)
	{
		ModuleEngines val = p.FindModuleImplementing<ModuleEngines>();
		if ((Object)(object)val != (Object)null)
		{
			return val.throttleLocked;
		}
		return false;
	}

	public static bool IsParachute(this Part p)
	{
		return (Object)(object)p.FindModuleImplementing<ModuleParachute>() != (Object)null;
	}

	public static bool IsLaunchClamp(this Part p)
	{
		return (Object)(object)p.FindModuleImplementing<LaunchClamp>() != (Object)null;
	}

	public static bool IsDecoupledInStage(this Part p, int stage)
	{
		if (((p.IsUnfiredDecoupler(out var decoupledPart) && (Object)(object)p == (Object)(object)decoupledPart) || p.IsLaunchClamp()) && p.inverseStage == stage)
		{
			return true;
		}
		if (p.parent == null)
		{
			return false;
		}
		if (p.parent.IsUnfiredDecoupler(out decoupledPart) && (Object)(object)p == (Object)(object)decoupledPart && p.parent.inverseStage == stage)
		{
			return true;
		}
		return p.parent.IsDecoupledInStage(stage);
	}

	public static bool IsPhysicallySignificant(this Part p)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Invalid comparison between Unknown and I4
		bool flag = (int)p.physicalSignificance != 1;
		if (HighLogic.LoadedSceneIsEditor)
		{
			flag &= p.PhysicsSignificance != 1 && !p.IsLaunchClamp();
		}
		return flag;
	}

	public static Vector3Pair GetBoundingBox(this Part part)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_015b: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Unknown result type (might be due to invalid IL or missing references)
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		Vector3 val = default(Vector3);
		Vector3 val2 = default(Vector3);
		foreach (Transform item in part.FindModelComponents<Transform>())
		{
			MeshFilter component = ((Component)item).GetComponent<MeshFilter>();
			if ((Object)(object)component == (Object)null)
			{
				continue;
			}
			Mesh mesh = component.mesh;
			if (!((Object)(object)mesh == (Object)null))
			{
				Matrix4x4 val3 = ((Component)part.vessel).transform.worldToLocalMatrix * item.localToWorldMatrix;
				Vector3[] vertices = mesh.vertices;
				foreach (Vector3 val4 in vertices)
				{
					Vector3 val5 = ((Matrix4x4)(ref val3)).MultiplyPoint3x4(val4);
					val2.x = Mathf.Max(val2.x, val5.x);
					val.x = Mathf.Min(val.x, val5.x);
					val2.y = Mathf.Max(val2.y, val5.y);
					val.y = Mathf.Min(val.y, val5.y);
					val2.z = Mathf.Max(val2.z, val5.z);
					val.z = Mathf.Min(val.z, val5.z);
				}
			}
		}
		return new Vector3Pair(val2, val);
	}
}
