using System.Collections.Generic;
using Smooth.Delegates;
using Smooth.Pools;
using UnityEngine;

namespace MuMech;

public class SimulatedPart
{
	public static class DragCubePool
	{
		public static Pool<DragCube> Instance { get; } = new Pool<DragCube>((DelegateFunc<DragCube>)(() => new DragCube()), (DelegateAction<DragCube>)delegate
		{
		});

	}

	protected readonly DragCubeList cubes = new DragCubeList();

	public double totalMass;

	public bool shieldedFromAirstream;

	public bool noDrag;

	public bool hasLiftModule;

	private double bodyLiftMultiplier;

	private ReentrySimulation.SimCurves simCurves;

	private QuaternionD vesselToPart;

	private QuaternionD partToVessel;

	private static readonly Pool<SimulatedPart> pool = new Pool<SimulatedPart>((DelegateFunc<SimulatedPart>)Create, (DelegateAction<SimulatedPart>)Reset);

	public static int PoolSize => pool.Size;

	private static SimulatedPart Create()
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Expected O, but got Unknown
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		SimulatedPart simulatedPart = new SimulatedPart();
		simulatedPart.cubes.BodyLiftCurve = new LiftingSurfaceCurve();
		simulatedPart.cubes.SurfaceCurves = default(SurfaceCurvesList);
		return simulatedPart;
	}

	public virtual void Release()
	{
		foreach (DragCube cube in cubes.Cubes)
		{
			DragCubePool.Instance.Release(cube);
		}
		pool.Release(this);
	}

	public static void Release(List<SimulatedPart> objList)
	{
		for (int i = 0; i < objList.Count; i++)
		{
			objList[i].Release();
		}
	}

	private static void Reset(SimulatedPart obj)
	{
	}

	public static SimulatedPart Borrow(Part p, ReentrySimulation.SimCurves simCurve)
	{
		SimulatedPart simulatedPart = pool.Borrow();
		simulatedPart.Init(p, simCurve);
		return simulatedPart;
	}

	protected void Init(Part p, ReentrySimulation.SimCurves _simCurves)
	{
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		Rigidbody rb = p.rb;
		totalMass = (((Object)(object)rb == (Object)null) ? 0f : rb.mass);
		shieldedFromAirstream = p.ShieldedFromAirstream;
		noDrag = (Object)(object)rb == (Object)null && !PhysicsGlobals.ApplyDragToNonPhysicsParts;
		hasLiftModule = p.hasLiftModule;
		bodyLiftMultiplier = p.bodyLiftMultiplier * PhysicsGlobals.BodyLiftMultiplier;
		simCurves = _simCurves;
		CopyDragCubesList(p.DragCubes, cubes);
		cubes.ForceUpdate(true, true, false);
		partToVessel = QuaternionD.op_Implicit(Quaternion.LookRotation(p.vessel.GetTransform().InverseTransformDirection(((Component)p).transform.forward), p.vessel.GetTransform().InverseTransformDirection(((Component)p).transform.up)));
		vesselToPart = QuaternionD.op_Implicit(Quaternion.Inverse(QuaternionD.op_Implicit(partToVessel)));
	}

	public virtual Vector3d Drag(Vector3d vesselVelocity, double dragFactor, float mach)
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0060: Unknown result type (might be due to invalid IL or missing references)
		if (shieldedFromAirstream || noDrag)
		{
			return Vector3d.zero;
		}
		Vector3d val = vesselToPart * vesselVelocity;
		Vector3d val2 = -((Vector3d)(ref val)).normalized;
		cubes.SetDrag(Vector3d.op_Implicit(val2), mach);
		return -((Vector3d)(ref vesselVelocity)).normalized * (double)cubes.AreaDrag * dragFactor;
	}

	public virtual Vector3d Lift(Vector3d vesselVelocity, double liftFactor)
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		if (shieldedFromAirstream || hasLiftModule)
		{
			return Vector3d.zero;
		}
		return (partToVessel * (Vector3d.op_Implicit(cubes.LiftForce) * bodyLiftMultiplier * liftFactor)).ProjectOnPlane(vesselVelocity);
	}

	public virtual bool SimulateAndRollback(double altATGL, double altASL, double endASL, double pressure, double shockTemp, double time, double semiDeployMultiplier)
	{
		return false;
	}

	public virtual bool Simulate(double altATGL, double altASL, double endASL, double pressure, double shockTemp, double time, double semiDeployMultiplier)
	{
		return false;
	}

	protected void CopyDragCubesList(DragCubeList source, DragCubeList dest)
	{
		dest.ClearCubes();
		dest.SetPart(source.Part);
		dest.None = source.None;
		dest.Procedural = false;
		for (int i = 0; i < source.Cubes.Count; i++)
		{
			DragCube val = DragCubePool.Instance.Borrow();
			CopyDragCube(source.Cubes[i], val);
			dest.Cubes.Add(val);
		}
		dest.SetDragWeights();
		for (int j = 0; j < 6; j++)
		{
			dest.WeightedArea[j] = source.WeightedArea[j];
			dest.WeightedDrag[j] = source.WeightedDrag[j];
			dest.AreaOccluded[j] = source.AreaOccluded[j];
			dest.WeightedDepth[j] = source.WeightedDepth[j];
		}
		dest.SetDragWeights();
		simCurves.CopyTo(dest);
	}

	protected static void CopyDragCube(DragCube source, DragCube dest)
	{
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		dest.Name = source.Name;
		dest.Weight = source.Weight;
		dest.Center = source.Center;
		dest.Size = source.Size;
		for (int i = 0; i < source.Drag.Length; i++)
		{
			dest.Drag[i] = source.Drag[i];
			dest.Area[i] = source.Area[i];
			dest.Depth[i] = source.Depth[i];
			dest.DragModifiers[i] = source.DragModifiers[i];
		}
	}

	protected void SetCubeWeight(string name, float newWeight)
	{
		int count = cubes.Cubes.Count;
		if (count == 0)
		{
			return;
		}
		bool flag = true;
		for (int num = count - 1; num >= 0; num--)
		{
			if (cubes.Cubes[num].Name == name && cubes.Cubes[num].Weight != newWeight)
			{
				cubes.Cubes[num].Weight = newWeight;
				flag = false;
			}
		}
		if (!flag)
		{
			cubes.SetDragWeights();
		}
	}
}
