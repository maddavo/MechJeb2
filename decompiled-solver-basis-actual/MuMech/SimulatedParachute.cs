using System;
using Smooth.Delegates;
using Smooth.Pools;
using UnityEngine;

namespace MuMech;

public class SimulatedParachute : SimulatedPart
{
	private ModuleParachute para;

	private deploymentStates state;

	private double openningTime;

	private float deployLevel;

	public bool deploying;

	private bool willDeploy;

	private static readonly Pool<SimulatedParachute> pool = new Pool<SimulatedParachute>((DelegateFunc<SimulatedParachute>)Create, (DelegateAction<SimulatedParachute>)Reset);

	public new static int PoolSize => pool.Size;

	private static SimulatedParachute Create()
	{
		return new SimulatedParachute();
	}

	public override void Release()
	{
		foreach (DragCube cube in cubes.Cubes)
		{
			DragCubePool.Instance.Release(cube);
		}
		pool.Release(this);
	}

	private static void Reset(SimulatedParachute obj)
	{
	}

	public static SimulatedParachute Borrow(ModuleParachute mp, ReentrySimulation.SimCurves simCurve, double startTime, int limitChutesStage)
	{
		SimulatedParachute simulatedParachute = pool.Borrow();
		simulatedParachute.Init(((PartModule)mp).part, simCurve);
		simulatedParachute.Init(mp, startTime, limitChutesStage);
		return simulatedParachute;
	}

	private void Init(ModuleParachute mp, double startTime, int limitChutesStage)
	{
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Expected I4, but got Unknown
		para = mp;
		state = mp.deploymentState;
		willDeploy = limitChutesStage != -1 && ((PartModule)para).part.inverseStage >= limitChutesStage;
		double num = 0.0;
		deploymentStates deploymentState = mp.deploymentState;
		switch ((int)deploymentState)
		{
		case 2:
			num = ((!mp.Anim.isPlaying) ? 10000000.0 : ((double)mp.Anim[mp.semiDeployedAnimation].time));
			break;
		case 3:
			num = ((!mp.Anim.isPlaying) ? 10000000.0 : ((double)mp.Anim[mp.fullyDeployedAnimation].time));
			break;
		case 0:
		case 1:
			num = 10000000.0;
			break;
		default:
			num = 10000000.0;
			break;
		}
		openningTime = startTime - num;
	}

	public override Vector3d Drag(Vector3d vesselVelocity, double dragFactor, float mach)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Invalid comparison between Unknown and I4
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Invalid comparison between Unknown and I4
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		if ((int)state != 2 && (int)state != 3)
		{
			return base.Drag(vesselVelocity, dragFactor, mach);
		}
		return Vector3d.zero;
	}

	public override bool SimulateAndRollback(double altATGL, double altASL, double endASL, double pressure, double shockTemp, double time, double semiDeployMultiplier)
	{
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Expected I4, but got Unknown
		if (!willDeploy)
		{
			return false;
		}
		bool result = false;
		deploymentStates val = state;
		switch ((int)val)
		{
		case 0:
			if (altATGL < semiDeployMultiplier * (double)para.deployAltitude && shockTemp * para.machHeatMult < para.chuteMaxTemp * para.safeMult)
			{
				result = true;
			}
			break;
		case 1:
			if (pressure >= (double)para.minAirPressureToOpen)
			{
				result = true;
			}
			break;
		case 2:
			if (altATGL < (double)para.deployAltitude)
			{
				result = true;
			}
			break;
		}
		return result;
	}

	public override bool Simulate(double altATGL, double altASL, double endASL, double pressure, double shockTemp, double time, double semiDeployMultiplier)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Expected I4, but got Unknown
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d2: Invalid comparison between Unknown and I4
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Invalid comparison between Unknown and I4
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0154: Unknown result type (might be due to invalid IL or missing references)
		//IL_015a: Invalid comparison between Unknown and I4
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0190: Unknown result type (might be due to invalid IL or missing references)
		//IL_01aa: Expected I4, but got Unknown
		//IL_015d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0163: Invalid comparison between Unknown and I4
		if (!willDeploy)
		{
			return false;
		}
		deploymentStates val = state;
		switch ((int)val)
		{
		case 0:
			if (altATGL < semiDeployMultiplier * (double)para.deployAltitude && shockTemp * para.machHeatMult < para.chuteMaxTemp * para.safeMult)
			{
				state = (deploymentStates)1;
				if (pressure >= (double)para.minAirPressureToOpen)
				{
					state = (deploymentStates)2;
					openningTime = time;
				}
			}
			break;
		case 1:
			if (pressure >= (double)para.minAirPressureToOpen)
			{
				state = (deploymentStates)2;
				openningTime = time;
			}
			break;
		case 2:
			if (altATGL < (double)para.deployAltitude)
			{
				state = (deploymentStates)3;
				openningTime = time;
			}
			break;
		}
		float num = (((int)state == 2) ? ((float)Math.Min((time - openningTime) / (double)para.semiDeploymentSpeed, 1.0)) : (((int)state != 3) ? 1f : ((float)Math.Min((time - openningTime) / (double)para.deploymentSpeed, 1.0))));
		if (num < 1f)
		{
			deploying = true;
		}
		else
		{
			deploying = false;
		}
		if (deploying && ((int)state == 2 || (int)state == 3))
		{
			deployLevel = Mathf.Pow(num, para.deploymentCurve);
		}
		else
		{
			deployLevel = 1f;
		}
		val = state;
		switch ((int)val)
		{
		case 0:
		case 1:
		case 4:
			SetCubeWeight("PACKED", 1f);
			SetCubeWeight("SEMIDEPLOYED", 0f);
			SetCubeWeight("DEPLOYED", 0f);
			break;
		case 2:
			SetCubeWeight("PACKED", 1f - deployLevel);
			SetCubeWeight("SEMIDEPLOYED", deployLevel);
			SetCubeWeight("DEPLOYED", 0f);
			break;
		case 3:
			SetCubeWeight("PACKED", 0f);
			SetCubeWeight("SEMIDEPLOYED", 1f - deployLevel);
			SetCubeWeight("DEPLOYED", deployLevel);
			break;
		}
		return deploying;
	}
}
