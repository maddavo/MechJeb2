using System;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech.AttitudeControllers;

public class DirectionTracker
{
	private QuaternionD _previousRotation;

	public Vector3d TrackedRotation;

	private bool NeedsInitialization()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		return _previousRotation == new QuaternionD(0.0, 0.0, 0.0, 0.0);
	}

	public DirectionTracker()
	{
		Reset();
	}

	public Vector3d Update(QuaternionD currentRotation)
	{
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		if (NeedsInitialization())
		{
			_previousRotation = currentRotation;
			TrackedRotation = Vector3d.zero;
			return TrackedRotation;
		}
		Vector3d val = MathExtensions.EulerAngles(QuaternionD.Inverse(_previousRotation) * currentRotation);
		double num = Statics.ClampPi(Statics.Deg2Rad(val.x));
		double num2 = 0.0 - Statics.ClampPi(Statics.Deg2Rad(val.y));
		double num3 = Statics.ClampPi(Statics.Deg2Rad(val.z));
		TrackedRotation.x += num;
		TrackedRotation.y += num3;
		TrackedRotation.z += num2;
		_previousRotation = currentRotation;
		return TrackedRotation;
	}

	public (Vector3d desired, Vector3d error, double distance) Desired(QuaternionD desiredRotation)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = MathExtensions.EulerAngles(QuaternionD.Inverse(_previousRotation) * desiredRotation);
		double num = Statics.ClampPi(Statics.Deg2Rad(val.x));
		double num2 = 0.0 - Statics.ClampPi(Statics.Deg2Rad(val.y));
		double num3 = Statics.ClampPi(Statics.Deg2Rad(val.z));
		Vector3d val2 = default(Vector3d);
		((Vector3d)(ref val2))._002Ector(num, num3, num2);
		double item = Statics.SafeAcos(Math.Cos(num) * Math.Cos(num2));
		return (desired: TrackedRotation + val2, error: val2, distance: item);
	}

	public void Reset()
	{
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		_previousRotation = new QuaternionD(0.0, 0.0, 0.0, 0.0);
		TrackedRotation = Vector3d.zero;
	}

	public void Reset(int i)
	{
		((Vector3d)(ref TrackedRotation))[i] = 0.0;
	}
}
