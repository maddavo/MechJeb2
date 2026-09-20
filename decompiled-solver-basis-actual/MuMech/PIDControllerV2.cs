using System;

namespace MuMech;

public class PIDControllerV2 : IConfigNode
{
	private Vector3d _intAccum;

	private Vector3d _derivativeAct;

	private Vector3d _propAct;

	public double Kp;

	public double Ki;

	public double Kd;

	private readonly double _max;

	private readonly double _min;

	public PIDControllerV2(double kp = 0.0, double ki = 0.0, double kd = 0.0, double max = double.MaxValue, double min = double.MinValue)
	{
		Kp = kp;
		Ki = ki;
		Kd = kd;
		_max = max;
		_min = min;
		Reset();
	}

	public Vector3d Compute(Vector3d error, Vector3d omega)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0146: Unknown result type (might be due to invalid IL or missing references)
		//IL_014b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0151: Unknown result type (might be due to invalid IL or missing references)
		//IL_0157: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_0167: Unknown result type (might be due to invalid IL or missing references)
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_017b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0197: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c8: Unknown result type (might be due to invalid IL or missing references)
		_derivativeAct = omega * Kd;
		_intAccum.x = ((Math.Abs(_derivativeAct.x) < 0.6 * _max) ? (_intAccum.x + error.x * Ki * (double)TimeWarp.fixedDeltaTime) : (0.9 * _intAccum.x));
		_intAccum.y = ((Math.Abs(_derivativeAct.y) < 0.6 * _max) ? (_intAccum.y + error.y * Ki * (double)TimeWarp.fixedDeltaTime) : (0.9 * _intAccum.y));
		_intAccum.z = ((Math.Abs(_derivativeAct.z) < 0.6 * _max) ? (_intAccum.z + error.z * Ki * (double)TimeWarp.fixedDeltaTime) : (0.9 * _intAccum.z));
		_propAct = error * Kp;
		Vector3d val = _propAct + _derivativeAct + _intAccum;
		((Vector3d)(ref val))._002Ector(Math.Max(_min, Math.Min(_max, val.x)), Math.Max(_min, Math.Min(_max, val.y)), Math.Max(_min, Math.Min(_max, val.z)));
		return val;
	}

	public Vector3d Compute(Vector3d error, Vector3d omega, Vector3d wlimit)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_0132: Unknown result type (might be due to invalid IL or missing references)
		//IL_014d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0154: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_015f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0172: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		//IL_017f: Unknown result type (might be due to invalid IL or missing references)
		//IL_018f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0196: Unknown result type (might be due to invalid IL or missing references)
		//IL_019c: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01da: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0205: Unknown result type (might be due to invalid IL or missing references)
		//IL_0221: Unknown result type (might be due to invalid IL or missing references)
		//IL_0236: Unknown result type (might be due to invalid IL or missing references)
		_derivativeAct = omega * Kd;
		wlimit *= Kd;
		_intAccum.x = ((Math.Abs(_derivativeAct.x) < 0.6 * _max) ? (_intAccum.x + error.x * Ki * (double)TimeWarp.fixedDeltaTime) : (0.9 * _intAccum.x));
		_intAccum.y = ((Math.Abs(_derivativeAct.y) < 0.6 * _max) ? (_intAccum.y + error.y * Ki * (double)TimeWarp.fixedDeltaTime) : (0.9 * _intAccum.y));
		_intAccum.z = ((Math.Abs(_derivativeAct.z) < 0.6 * _max) ? (_intAccum.z + error.z * Ki * (double)TimeWarp.fixedDeltaTime) : (0.9 * _intAccum.z));
		_propAct = error * Kp;
		Vector3d val = _propAct + _intAccum;
		((Vector3d)(ref val))._002Ector(Math.Max(0.0 - wlimit.x, Math.Min(wlimit.x, val.x)), Math.Max(0.0 - wlimit.y, Math.Min(wlimit.y, val.y)), Math.Max(0.0 - wlimit.z, Math.Min(wlimit.z, val.z)));
		val += _derivativeAct;
		((Vector3d)(ref val))._002Ector(Math.Max(_min, Math.Min(_max, val.x)), Math.Max(_min, Math.Min(_max, val.y)), Math.Max(_min, Math.Min(_max, val.z)));
		return val;
	}

	public void Reset()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		_intAccum = Vector3d.zero;
	}

	public void Load(ConfigNode node)
	{
		if (node.HasValue("Kp"))
		{
			Kp = Convert.ToDouble(node.GetValue("Kp"));
		}
		if (node.HasValue("Ki"))
		{
			Ki = Convert.ToDouble(node.GetValue("Ki"));
		}
		if (node.HasValue("Kd"))
		{
			Kd = Convert.ToDouble(node.GetValue("Kd"));
		}
	}

	public void Save(ConfigNode node)
	{
		node.SetValue("Kp", Kp.ToString(), false);
		node.SetValue("Ki", Ki.ToString(), false);
		node.SetValue("Kd", Kd.ToString(), false);
	}
}
