using System.Collections.Generic;
using Smooth.Delegates;
using Smooth.Pools;
using UnityEngine;

namespace MuMech;

public class SimulatedVessel
{
	public readonly List<SimulatedPart> parts = new List<SimulatedPart>();

	private int count;

	public double totalMass;

	private ReentrySimulation.SimCurves simCurves;

	private static readonly Pool<SimulatedVessel> pool = new Pool<SimulatedVessel>((DelegateFunc<SimulatedVessel>)Create, (DelegateAction<SimulatedVessel>)Reset);

	public static int PoolSize => pool.Size;

	private static SimulatedVessel Create()
	{
		return new SimulatedVessel();
	}

	public void Release()
	{
		pool.Release(this);
	}

	private static void Reset(SimulatedVessel obj)
	{
		SimulatedPart.Release(obj.parts);
		obj.parts.Clear();
	}

	public static SimulatedVessel Borrow(Vessel v, ReentrySimulation.SimCurves simCurves, double startTime, int limitChutesStage)
	{
		SimulatedVessel simulatedVessel = pool.Borrow();
		simulatedVessel.Init(v, simCurves, startTime, limitChutesStage);
		return simulatedVessel;
	}

	private void Init(Vessel v, ReentrySimulation.SimCurves _simCurves, double startTime, int limitChutesStage)
	{
		totalMass = 0.0;
		List<Part> list = v.Parts;
		count = list.Count;
		simCurves = _simCurves;
		if (parts.Capacity < count)
		{
			parts.Capacity = count;
		}
		for (int i = 0; i < count; i++)
		{
			SimulatedPart simulatedPart = null;
			bool flag = false;
			for (int j = 0; j < list[i].Modules.Count; j++)
			{
				PartModule obj = list[i].Modules[j];
				ModuleParachute val = (ModuleParachute)(object)((obj is ModuleParachute) ? obj : null);
				if ((Object)(object)val != (Object)null && v.mainBody.atmosphere)
				{
					flag = true;
					simulatedPart = SimulatedParachute.Borrow(val, simCurves, startTime, limitChutesStage);
				}
			}
			if (!flag)
			{
				simulatedPart = SimulatedPart.Borrow(list[i], simCurves);
			}
			parts.Add(simulatedPart);
			totalMass += simulatedPart.totalMass;
		}
	}

	public Vector3d Drag(Vector3d localVelocity, double dynamicPressurekPa, float mach)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.zero;
		double dragFactor = dynamicPressurekPa * (double)PhysicsGlobals.DragCubeMultiplier * (double)PhysicsGlobals.DragMultiplier;
		for (int i = 0; i < count; i++)
		{
			val += parts[i].Drag(localVelocity, dragFactor, mach);
		}
		return -((Vector3d)(ref localVelocity)).normalized * ((Vector3d)(ref val)).magnitude;
	}

	public Vector3d Lift(Vector3d localVelocity, float dynamicPressurekPa, float mach)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.zero;
		double liftFactor = dynamicPressurekPa * simCurves.LiftMachCurve.Evaluate(mach);
		for (int i = 0; i < count; i++)
		{
			val += parts[i].Lift(localVelocity, liftFactor);
		}
		return val;
	}

	public bool WillChutesDeploy(double altAGL, double altASL, double probableLandingSiteASL, double pressure, double shockTemp, double t, double parachuteSemiDeployMultiplier)
	{
		for (int i = 0; i < count; i++)
		{
			if (parts[i].SimulateAndRollback(altAGL, altASL, probableLandingSiteASL, pressure, shockTemp, t, parachuteSemiDeployMultiplier))
			{
				return true;
			}
		}
		return false;
	}

	public bool Simulate(double altATGL, double altASL, double endASL, double pressure, double shockTemp, double time, double semiDeployMultiplier)
	{
		bool flag = false;
		for (int i = 0; i < count; i++)
		{
			flag |= parts[i].Simulate(altATGL, altASL, endASL, pressure, shockTemp, time, semiDeployMultiplier);
		}
		return flag;
	}
}
