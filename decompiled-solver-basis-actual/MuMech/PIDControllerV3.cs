using System;

namespace MuMech;

public class PIDControllerV3 : IConfigNode
{
	public Vector3d Kp;

	public Vector3d Ki;

	public Vector3d Kd;

	public Vector3d INTAccum;

	public Vector3d DerivativeAct;

	public Vector3d PropAct;

	private readonly double _max;

	private readonly double _min;

	public PIDControllerV3(Vector3d kp, Vector3d ki, Vector3d kd, double max = double.MaxValue, double min = double.MinValue)
	{
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		Kp = kp;
		Ki = ki;
		Kd = kd;
		_max = max;
		_min = min;
		Reset();
	}

	public Vector3d Compute(Vector3d error, Vector3d omega, Vector3d wlimit)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_015e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0163: Unknown result type (might be due to invalid IL or missing references)
		//IL_0168: Unknown result type (might be due to invalid IL or missing references)
		//IL_016e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0174: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		//IL_017e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0181: Unknown result type (might be due to invalid IL or missing references)
		//IL_0188: Unknown result type (might be due to invalid IL or missing references)
		//IL_018e: Unknown result type (might be due to invalid IL or missing references)
		//IL_019e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01df: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0214: Unknown result type (might be due to invalid IL or missing references)
		//IL_0230: Unknown result type (might be due to invalid IL or missing references)
		//IL_0245: Unknown result type (might be due to invalid IL or missing references)
		DerivativeAct = Vector3d.Scale(omega, Kd);
		wlimit = Vector3d.Scale(wlimit, Kd);
		INTAccum.x = ((Math.Abs(DerivativeAct.x) < 0.6 * _max) ? (INTAccum.x + error.x * Ki.x * (double)TimeWarp.fixedDeltaTime) : (0.9 * INTAccum.x));
		INTAccum.y = ((Math.Abs(DerivativeAct.y) < 0.6 * _max) ? (INTAccum.y + error.y * Ki.y * (double)TimeWarp.fixedDeltaTime) : (0.9 * INTAccum.y));
		INTAccum.z = ((Math.Abs(DerivativeAct.z) < 0.6 * _max) ? (INTAccum.z + error.z * Ki.z * (double)TimeWarp.fixedDeltaTime) : (0.9 * INTAccum.z));
		PropAct = Vector3d.Scale(error, Kp);
		Vector3d val = PropAct + INTAccum;
		((Vector3d)(ref val))._002Ector(Math.Max(0.0 - wlimit.x, Math.Min(wlimit.x, val.x)), Math.Max(0.0 - wlimit.y, Math.Min(wlimit.y, val.y)), Math.Max(0.0 - wlimit.z, Math.Min(wlimit.z, val.z)));
		val += DerivativeAct;
		((Vector3d)(ref val))._002Ector(Math.Max(_min, Math.Min(_max, val.x)), Math.Max(_min, Math.Min(_max, val.y)), Math.Max(_min, Math.Min(_max, val.z)));
		return val;
	}

	public void Reset()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		INTAccum = Vector3d.zero;
	}

	public void Load(ConfigNode node)
	{
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		if (node.HasValue("Kp"))
		{
			Kp = ConfigNode.ParseVector3D(node.GetValue("Kp"));
		}
		if (node.HasValue("Ki"))
		{
			Ki = ConfigNode.ParseVector3D(node.GetValue("Ki"));
		}
		if (node.HasValue("Kd"))
		{
			Kd = ConfigNode.ParseVector3D(node.GetValue("Kd"));
		}
	}

	public void Save(ConfigNode node)
	{
		node.SetValue("Kp", ((object)(Vector3d)(ref Kp)).ToString(), false);
		node.SetValue("Ki", ((object)(Vector3d)(ref Ki)).ToString(), false);
		node.SetValue("Kd", ((object)(Vector3d)(ref Kd)).ToString(), false);
	}
}
