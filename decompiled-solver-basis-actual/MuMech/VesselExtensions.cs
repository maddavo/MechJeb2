using System;
using System.Collections;
using System.Collections.Generic;
using Smooth.Delegates;
using Smooth.Slinq;
using Smooth.Slinq.Context;
using UniLinq;
using UnityEngine;

namespace MuMech;

public static class VesselExtensions
{
	private static float lastFixedTime;

	private static readonly Dictionary<Guid, MechJebCore> masterMechJeb = new Dictionary<Guid, MechJebCore>();

	public static bool VesselOffGround(this Vessel vessel)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Invalid comparison between Unknown and I4
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Invalid comparison between Unknown and I4
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Invalid comparison between Unknown and I4
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Invalid comparison between Unknown and I4
		if ((int)vessel.situation != 8 && (int)vessel.situation != 64 && (int)vessel.situation != 32)
		{
			return (int)vessel.situation == 16;
		}
		return true;
	}

	public static List<ITargetable> GetTargetables(this Vessel vessel)
	{
		List<Part> parts;
		if (HighLogic.LoadedSceneIsEditor)
		{
			parts = EditorLogic.fetch.ship.parts;
		}
		else
		{
			if ((Object)(object)vessel == (Object)null)
			{
				return new List<ITargetable>();
			}
			parts = vessel.Parts;
		}
		return Enumerable.ToList<ITargetable>(Enumerable.SelectMany<Part, ITargetable>((IEnumerable<Part>)parts, (Func<Part, IEnumerable<ITargetable>>)((Part part) => Enumerable.OfType<ITargetable>((IEnumerable)part.Modules))));
	}

	public static List<T> GetModules<T>(this Vessel vessel) where T : PartModule
	{
		List<Part> parts;
		if (HighLogic.LoadedSceneIsEditor && (Object)(object)EditorLogic.fetch != (Object)null)
		{
			parts = EditorLogic.fetch.ship.parts;
		}
		else
		{
			if ((Object)(object)vessel == (Object)null || vessel.Parts == null)
			{
				return new List<T>();
			}
			parts = vessel.Parts;
		}
		List<T> list = new List<T>();
		for (int i = 0; i < parts.Count; i++)
		{
			Part val = parts[i];
			if (val.Modules == null)
			{
				continue;
			}
			int count = val.Modules.Count;
			for (int j = 0; j < count; j++)
			{
				PartModule obj = val.Modules[j];
				T val2 = (T)(object)((obj is T) ? obj : null);
				if (val2 != null)
				{
					list.Add(val2);
				}
			}
		}
		return list;
	}

	public static T GetModule<T>(this Vessel vessel, Predicate<T> predicate) where T : PartModule
	{
		List<Part> parts;
		if (HighLogic.LoadedSceneIsEditor && (Object)(object)EditorLogic.fetch != (Object)null)
		{
			parts = EditorLogic.fetch.ship.parts;
		}
		else
		{
			if ((Object)(object)vessel == (Object)null || vessel.Parts == null)
			{
				return default(T);
			}
			parts = vessel.Parts;
		}
		for (int i = 0; i < parts.Count; i++)
		{
			Part val = parts[i];
			if (val.Modules == null)
			{
				continue;
			}
			int count = val.Modules.Count;
			for (int j = 0; j < count; j++)
			{
				PartModule obj = val.Modules[j];
				T val2 = (T)(object)((obj is T) ? obj : null);
				if (val2 != null && predicate(val2))
				{
					return val2;
				}
			}
		}
		return default(T);
	}

	public static MechJebCore GetMasterMechJeb(this Vessel vessel)
	{
		if (lastFixedTime != Time.fixedTime)
		{
			masterMechJeb.Clear();
			lastFixedTime = Time.fixedTime;
		}
		Guid key = (((Object)(object)vessel == (Object)null) ? Guid.Empty : vessel.id);
		if (!masterMechJeb.TryGetValue(key, out var value))
		{
			value = vessel.GetModule((MechJebCore p) => p.running);
			if ((Object)(object)value != (Object)null)
			{
				masterMechJeb.Add(key, value);
			}
			return value;
		}
		return value;
	}

	public static double TotalResourceAmount(this Vessel vessel, PartResourceDefinition definition)
	{
		if (definition == null)
		{
			return 0.0;
		}
		List<Part> list = (HighLogic.LoadedSceneIsEditor ? EditorLogic.fetch.ship.parts : vessel.parts);
		double num = 0.0;
		for (int i = 0; i < list.Count; i++)
		{
			Part val = list[i];
			for (int j = 0; j < val.Resources.Count; j++)
			{
				PartResource val2 = val.Resources[j];
				if (val2.info.id == definition.id)
				{
					num += val2.amount;
				}
			}
		}
		return num;
	}

	public static double TotalResourceAmount(this Vessel vessel, string resourceName)
	{
		return vessel.TotalResourceAmount(PartResourceLibrary.Instance.GetDefinition(resourceName));
	}

	public static double TotalResourceAmount(this Vessel vessel, int resourceId)
	{
		return vessel.TotalResourceAmount(PartResourceLibrary.Instance.GetDefinition(resourceId));
	}

	public static double TotalResourceMass(this Vessel vessel, string resourceName)
	{
		PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(resourceName);
		return vessel.TotalResourceAmount(definition) * (double)definition.density;
	}

	public static double TotalResourceMass(this Vessel vessel, int resourceId)
	{
		PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(resourceId);
		return vessel.TotalResourceAmount(definition) * (double)definition.density;
	}

	public static double MaxResourceAmount(this Vessel vessel, PartResourceDefinition definition)
	{
		if (definition == null)
		{
			return 0.0;
		}
		List<Part> list = (HighLogic.LoadedSceneIsEditor ? EditorLogic.fetch.ship.parts : vessel.parts);
		double num = 0.0;
		for (int i = 0; i < list.Count; i++)
		{
			Part val = list[i];
			for (int j = 0; j < val.Resources.Count; j++)
			{
				PartResource val2 = val.Resources[j];
				if (val2.info.id == definition.id)
				{
					num += val2.maxAmount;
				}
			}
		}
		return num;
	}

	public static double MaxResourceAmount(this Vessel vessel, int id)
	{
		PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(id);
		return vessel.MaxResourceAmount(definition);
	}

	public static double MaxResourceAmount(this Vessel vessel, string resourceName)
	{
		return vessel.MaxResourceAmount(PartResourceLibrary.Instance.GetDefinition(resourceName));
	}

	public static bool HasElectricCharge(this Vessel vessel)
	{
		if ((Object)(object)vessel == (Object)null)
		{
			return false;
		}
		List<Part> list = (HighLogic.LoadedSceneIsEditor ? EditorLogic.fetch.ship.parts : vessel.parts);
		PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(PartResourceLibrary.ElectricityHashcode);
		if (definition == null)
		{
			return false;
		}
		if ((Object)(object)vessel.GetReferenceTransformPart() != (Object)null)
		{
			PartResource val = vessel.GetReferenceTransformPart().Resources.Get(definition.id);
			if (val != null && val.amount > 0.0)
			{
				return true;
			}
		}
		for (int i = 0; i < list.Count; i++)
		{
			PartResource val = list[i].Resources.Get(definition.id);
			if (val != null && val.amount > 0.0)
			{
				return true;
			}
		}
		return false;
	}

	public static bool LiftedOff(this Vessel vessel)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Invalid comparison between Unknown and I4
		return (int)vessel.situation != 4;
	}

	public static Orbit GetPatchAtUT(this Vessel vessel, double UT)
	{
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Invalid comparison between Unknown and I4
		IEnumerable<ManeuverNode> enumerable = Enumerable.Where<ManeuverNode>((IEnumerable<ManeuverNode>)vessel.patchedConicSolver.maneuverNodes, (Func<ManeuverNode, bool>)((ManeuverNode n) => n.UT <= UT));
		Orbit val = vessel.orbit;
		if (Enumerable.Any<ManeuverNode>(enumerable))
		{
			val = Enumerable.First<ManeuverNode>((IEnumerable<ManeuverNode>)Enumerable.OrderByDescending<ManeuverNode, double>(enumerable, (Func<ManeuverNode, double>)((ManeuverNode n) => n.UT))).nextPatch;
		}
		while ((int)val.patchEndTransition != 1 && val.nextPatch.StartUT <= UT)
		{
			val = val.nextPatch;
		}
		return val;
	}

	public static Orbit GetNextPatch(this Vessel vessel, Orbit patch, ManeuverNode ignoreNode = null)
	{
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Invalid comparison between Unknown and I4
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		if (patch == null)
		{
			return null;
		}
		bool flag = (int)patch.patchEndTransition == 1;
		if ((Object)(object)vessel.patchedConicSolver == (Object)null)
		{
			vessel.patchedConicSolver = ((Component)vessel).gameObject.AddComponent<PatchedConicSolver>();
			vessel.patchedConicSolver.Load(vessel.flightPlanNode);
		}
		ManeuverNode val = Slinq.Where<ManeuverNode, IListContext<ManeuverNode>, Orbit>(Slinqable.Slinq<ManeuverNode>((IList<ManeuverNode>)vessel.patchedConicSolver.maneuverNodes), (DelegateFunc<ManeuverNode, Orbit, bool>)((ManeuverNode n, Orbit p) => n.patch == p && n != ignoreNode), patch).FirstOrDefault();
		if (val != null)
		{
			return val.nextPatch;
		}
		if (!flag)
		{
			return patch.nextPatch;
		}
		return null;
	}

	public static ManeuverNode PlaceManeuverNode(this Vessel vessel, Orbit ignoredParameterThatNeedsDeleting, Vector3d dV, double UT)
	{
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0087: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		Orbit patchAtUT = vessel.GetPatchAtUT(UT);
		for (int i = 0; i < 3; i++)
		{
			if (double.IsNaN(((Vector3d)(ref dV))[i]) || double.IsInfinity(((Vector3d)(ref dV))[i]))
			{
				Vector3d val = dV;
				throw new Exception("MechJeb VesselExtensions.PlaceManeuverNode: bad dV: " + ((object)(Vector3d)(ref val)).ToString());
			}
		}
		if (double.IsNaN(UT) || double.IsInfinity(UT))
		{
			throw new Exception("MechJeb VesselExtensions.PlaceManeuverNode: bad UT: " + UT);
		}
		UT = Math.Max(UT, Planetarium.GetUniversalTime());
		Vector3d deltaV = patchAtUT.DeltaVToManeuverNodeCoordinates(UT, dV);
		ManeuverNode obj = vessel.patchedConicSolver.AddManeuverNode(UT);
		obj.DeltaV = deltaV;
		vessel.patchedConicSolver.UpdateFlightPlan();
		return obj;
	}

	public static void RemoveAllManeuverNodes(this Vessel vessel)
	{
		if (vessel.patchedConicsUnlocked())
		{
			while (vessel.patchedConicSolver.maneuverNodes.Count > 0)
			{
				Enumerable.Last<ManeuverNode>((IEnumerable<ManeuverNode>)vessel.patchedConicSolver.maneuverNodes).RemoveSelf();
			}
		}
	}

	public static MechJebModuleDockingAutopilot.Box3d GetBoundingBox(this Vessel vessel, bool debug = false)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0108: Unknown result type (might be due to invalid IL or missing references)
		//IL_0110: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_012e: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		//IL_017f: Unknown result type (might be due to invalid IL or missing references)
		//IL_018d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0193: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01df: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0201: Unknown result type (might be due to invalid IL or missing references)
		//IL_020e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0213: Unknown result type (might be due to invalid IL or missing references)
		//IL_0218: Unknown result type (might be due to invalid IL or missing references)
		Vector3 val = default(Vector3);
		Vector3 val2 = default(Vector3);
		if (debug)
		{
			MonoBehaviour.print((object)("[GetBoundingBox] Start " + vessel.vesselName));
		}
		for (int i = 0; i < vessel.parts.Count; i++)
		{
			Part val3 = vessel.parts[i];
			PartExtensions.Vector3Pair boundingBox = val3.GetBoundingBox();
			if (debug)
			{
				string name = ((Object)val3).name;
				Vector3 val4 = boundingBox.P1 - boundingBox.P2;
				MonoBehaviour.print((object)("[GetBoundingBox] " + name + " " + ((Vector3)(ref val4)).magnitude.ToString("F3")));
			}
			val2.x = Mathf.Max(val2.x, boundingBox.P1.x);
			val.x = Mathf.Min(val.x, boundingBox.P2.x);
			val2.y = Mathf.Max(val2.y, boundingBox.P1.y);
			val.y = Mathf.Min(val.y, boundingBox.P2.y);
			val2.z = Mathf.Max(val2.z, boundingBox.P1.z);
			val.z = Mathf.Min(val.z, boundingBox.P2.z);
		}
		if (debug)
		{
			MonoBehaviour.print((object)("[GetBoundingBox] End " + vessel.vesselName));
		}
		MechJebModuleDockingAutopilot.Box3d result = default(MechJebModuleDockingAutopilot.Box3d);
		result.center = Vector3d.op_Implicit(new Vector3d((double)((val2.x + val.x) / 2f), (double)((val2.y + val.y) / 2f), (double)((val2.z + val.z) / 2f)));
		result.size = Vector3d.op_Implicit(new Vector3d((double)Math.Abs(result.center.x - val2.x), (double)Math.Abs(result.center.y - val2.y), (double)Math.Abs(result.center.z - val2.z)));
		return result;
	}

	public static bool patchedConicsUnlocked(this Vessel vessel)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Invalid comparison between Unknown and I4
		return (int)GameVariables.Instance.GetOrbitDisplayMode(ScenarioUpgradeableFacilities.GetFacilityLevel((SpaceCenterFacility)6)) == 3;
	}

	public static void UpdateNode(this ManeuverNode node, Vector3d dV, double ut)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		node.DeltaV = dV;
		node.UT = ut;
		node.solver.UpdateFlightPlan();
		if (!((Object)(object)node.attachedGizmo == (Object)null))
		{
			node.attachedGizmo.patchBefore = node.patch;
			node.attachedGizmo.patchAhead = node.nextPatch;
		}
	}

	public static Vector3d WorldDeltaV(this ManeuverNode node)
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		return node.patch.Prograde(node.UT) * node.DeltaV.z + node.patch.RadialPlus(node.UT) * node.DeltaV.x + -node.patch.NormalPlus(node.UT) * node.DeltaV.y;
	}

	public static bool hasEnabledRCSModules(this Vessel vessel)
	{
		List<ModuleRCS> list = vessel.FindPartModulesImplementing<ModuleRCS>();
		for (int i = 0; i < list.Count; i++)
		{
			ModuleRCS val = list[i];
			if (!((Object)(object)val == (Object)null) && val.rcsEnabled && ((PartModule)val).isEnabled && !val.isJustForShow)
			{
				return true;
			}
		}
		return false;
	}
}
