using System;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public static class MathExtensions
{
	public static Vector3d Sign(this Vector3d vector)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d((double)Math.Sign(vector.x), (double)Math.Sign(vector.y), (double)Math.Sign(vector.z));
	}

	public static Vector3d Abs(this Vector3d vector)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d(Math.Abs(vector.x), Math.Abs(vector.y), Math.Abs(vector.z));
	}

	public static Vector3 Abs(this Vector3 vector)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3(Math.Abs(vector.x), Math.Abs(vector.y), Math.Abs(vector.z));
	}

	public static Vector3d Sqrt(this Vector3d vector)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d(Math.Sqrt(vector.x), Math.Sqrt(vector.y), Math.Sqrt(vector.z));
	}

	public static double MaxMagnitude(this Vector3d vector)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		return Math.Max(Math.Max(Math.Abs(vector.x), Math.Abs(vector.y)), Math.Abs(vector.z));
	}

	public static Vector3d Invert(this Vector3d vector)
	{
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d(1.0 / vector.x, 1.0 / vector.y, 1.0 / vector.z);
	}

	public static Vector3d InvertNoNaN(this Vector3d vector)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d((vector.x != 0.0) ? (1.0 / vector.x) : 0.0, (vector.y != 0.0) ? (1.0 / vector.y) : 0.0, (vector.z != 0.0) ? (1.0 / vector.z) : 0.0);
	}

	public static Vector3d ProjectOnPlane(this Vector3d vector, Vector3d planeNormal)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		return vector - Vector3d.Project(vector, planeNormal);
	}

	public static Vector3d DeltaEuler(this Quaternion delta)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_008c: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d((double)((((Quaternion)(ref delta)).eulerAngles.x > 180f) ? (((Quaternion)(ref delta)).eulerAngles.x - 360f) : ((Quaternion)(ref delta)).eulerAngles.x), (double)(0f - ((((Quaternion)(ref delta)).eulerAngles.y > 180f) ? (((Quaternion)(ref delta)).eulerAngles.y - 360f) : ((Quaternion)(ref delta)).eulerAngles.y)), (double)((((Quaternion)(ref delta)).eulerAngles.z > 180f) ? (((Quaternion)(ref delta)).eulerAngles.z - 360f) : ((Quaternion)(ref delta)).eulerAngles.z));
	}

	public static Vector3d Clamp(this Vector3d value, double min, double max)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d(Clamp(value.x, min, max), Clamp(value.y, min, max), Clamp(value.z, min, max));
	}

	public static double Clamp(double val, double min, double max)
	{
		if (val <= min)
		{
			return min;
		}
		if (val >= max)
		{
			return max;
		}
		return val;
	}

	public static double AngleInPlane(this Vector3d vector, Vector3d planeNormal, Vector3d other)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = vector.ProjectOnPlane(planeNormal);
		Vector3d val2 = other.ProjectOnPlane(planeNormal);
		if (((Vector3d)(ref val)).magnitude == 0.0 || ((Vector3d)(ref val2)).magnitude == 0.0)
		{
			return double.NaN;
		}
		double num = MuUtils.ClampDegrees360(Math.Acos(Vector3d.Dot(((Vector3d)(ref val)).normalized, ((Vector3d)(ref val2)).normalized)) * (180.0 / Math.PI));
		if (Vector3d.Dot(Vector3d.Cross(val, val2), planeNormal) < 0.0)
		{
			return 0.0 - num;
		}
		return num;
	}

	public static Quaternion Add(this Quaternion left, Quaternion right)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		return new Quaternion(left.x + right.x, left.y + right.y, left.z + right.z, left.w + right.w);
	}

	public static Quaternion Mult(this Quaternion left, float lambda)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		return new Quaternion(left.x * lambda, left.y * lambda, left.z * lambda, left.w * lambda);
	}

	public static Quaternion Conj(this Quaternion left)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		return new Quaternion(0f - left.x, 0f - left.y, 0f - left.z, left.w);
	}

	public static Vector3d Project(this Vector3d vector, Vector3d onNormal)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		Vector3d normalized = ((Vector3d)(ref onNormal)).normalized;
		return normalized * Vector3d.Dot(vector, normalized);
	}

	public static bool IsFinite(this Vector3d vector)
	{
		if (((Vector3d)(ref vector))[0].IsFinite() && ((Vector3d)(ref vector))[1].IsFinite())
		{
			return ((Vector3d)(ref vector))[2].IsFinite();
		}
		return false;
	}

	public static bool IsFinite(this double v)
	{
		if (!double.IsNaN(v))
		{
			return !double.IsInfinity(v);
		}
		return false;
	}

	public static double NextGaussian(this Random r, double mu = 0.0, double sigma = 1.0)
	{
		double d = r.NextDouble();
		double num = r.NextDouble();
		double num2 = Math.Sqrt(-2.0 * Math.Log(d)) * Math.Sin(Math.PI * 2.0 * num);
		return mu + sigma * num2;
	}

	public static QuaternionD FromToRotation(Vector3d fromDirection, Vector3d toDirection)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0087: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		if (fromDirection == Vector3d.zero || toDirection == Vector3d.zero)
		{
			return QuaternionD.identity;
		}
		((Vector3d)(ref fromDirection)).Normalize();
		((Vector3d)(ref toDirection)).Normalize();
		double num = Vector3d.Dot(fromDirection, toDirection);
		if (num < -0.9999999999)
		{
			Vector3d val = Vector3d.Cross(fromDirection, Vector3d.op_Implicit((Math.Abs(fromDirection.x) < 1.0 / Math.Sqrt(2.0)) ? Vector3.right : Vector3.up));
			((Vector3d)(ref val)).Normalize();
			return new QuaternionD(val.x, val.y, val.z, 0.0);
		}
		Vector3d val2 = Vector3d.Cross(fromDirection, toDirection);
		double num2 = Math.Sqrt((1.0 + num) * 2.0);
		double num3 = 1.0 / num2;
		return new QuaternionD(val2.x * num3, val2.y * num3, val2.z * num3, num2 * 0.5);
	}

	public static Vector3d EulerAngles(QuaternionD q)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0113: Unknown result type (might be due to invalid IL or missing references)
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_0145: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0200: Unknown result type (might be due to invalid IL or missing references)
		//IL_023a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0164: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0196: Unknown result type (might be due to invalid IL or missing references)
		double num = Math.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
		if (num < 2.220446049250313E-16)
		{
			return Vector3d.zero;
		}
		if (Math.Abs(num - 1.0) > 1E-10)
		{
			q.x /= num;
			q.y /= num;
			q.z /= num;
			q.w /= num;
		}
		double num2 = q.w * q.w;
		double num3 = q.x * q.x;
		double num4 = q.y * q.y;
		double num5 = q.z * q.z;
		double num6 = num3 + num4 + num5 + num2;
		double num7 = q.x * q.w - q.y * q.z;
		if (num7 > 0.499999999 * num6)
		{
			double num8 = 2.0 * Math.Atan2(q.y, q.w);
			return new Vector3d(90.0, Statics.Rad2Deg(Statics.Clamp2Pi(num8)), 0.0);
		}
		if (num7 < -0.499999999 * num6)
		{
			double num9 = -2.0 * Math.Atan2(q.y, q.w);
			return new Vector3d(270.0, Statics.Rad2Deg(Statics.Clamp2Pi(num9)), 0.0);
		}
		double num10 = Math.Asin(2.0 * num7 / num6);
		double num11 = Math.Atan2(2.0 * (q.x * q.z + q.w * q.y), num2 - num3 - num4 + num5);
		double num12 = Math.Atan2(2.0 * (q.x * q.y + q.w * q.z), num2 - num3 + num4 - num5);
		return new Vector3d(Statics.Rad2Deg(Statics.Clamp2Pi(num10)), Statics.Rad2Deg(Statics.Clamp2Pi(num11)), Statics.Rad2Deg(Statics.Clamp2Pi(num12)));
	}

	public static QuaternionD Euler(double x, double y, double z)
	{
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		x = Statics.Deg2Rad(x);
		y = Statics.Deg2Rad(y);
		z = Statics.Deg2Rad(z);
		double num = Math.Cos(x * 0.5);
		double num2 = Math.Sin(x * 0.5);
		double num3 = Math.Cos(y * 0.5);
		double num4 = Math.Sin(y * 0.5);
		double num5 = Math.Cos(z * 0.5);
		double num6 = Math.Sin(z * 0.5);
		QuaternionD result = default(QuaternionD);
		result.w = num5 * num * num3 + num6 * num2 * num4;
		result.x = num5 * num2 * num3 - num6 * num * num4;
		result.y = num5 * num * num4 + num6 * num2 * num3;
		result.z = num6 * num * num3 - num5 * num2 * num4;
		return result;
	}

	public static Vector3d RotateTowards(Vector3d current, Vector3d target, double maxRadiansDelta, double maxMagnitudeDelta)
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		if (((Vector3d)(ref current)).sqrMagnitude < 2.220446049250313E-16)
		{
			return ((Vector3d)(ref target)).normalized * Math.Min(maxMagnitudeDelta, ((Vector3d)(ref target)).magnitude);
		}
		if (((Vector3d)(ref target)).sqrMagnitude < 2.220446049250313E-16)
		{
			return ((Vector3d)(ref current)).normalized * Math.Max(0.0, ((Vector3d)(ref current)).magnitude - maxMagnitudeDelta);
		}
		double magnitude = ((Vector3d)(ref current)).magnitude;
		double magnitude2 = ((Vector3d)(ref target)).magnitude;
		double num = Statics.Deg2Rad(Vector3d.Angle(current, target));
		double num2 = Math.Min(maxRadiansDelta, num);
		double t = 0.0;
		if (num > (double)Mathf.Epsilon)
		{
			t = num2 / num;
		}
		Vector3d val = Slerp(((Vector3d)(ref current)).normalized, ((Vector3d)(ref target)).normalized, t);
		double num3 = magnitude;
		if (Math.Abs(magnitude2 - magnitude) <= 2.220446049250313E-16)
		{
			return val * num3;
		}
		double num4 = Math.Min(maxMagnitudeDelta, Math.Abs(magnitude2 - magnitude));
		num3 = ((magnitude2 > magnitude) ? Math.Min(magnitude + num4, magnitude2) : Math.Max(magnitude - num4, magnitude2));
		return val * num3;
	}

	public static Vector3d Slerp(Vector3d a, Vector3d b, double t)
	{
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_01af: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01db: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0200: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_011a: Unknown result type (might be due to invalid IL or missing references)
		//IL_011f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0151: Unknown result type (might be due to invalid IL or missing references)
		//IL_0154: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_015d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_0167: Unknown result type (might be due to invalid IL or missing references)
		//IL_016b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0184: Unknown result type (might be due to invalid IL or missing references)
		t = Statics.Clamp01(t);
		double num = Vector3d.Dot(((Vector3d)(ref a)).normalized, ((Vector3d)(ref b)).normalized);
		Vector3d val;
		if (num > 0.9999999999)
		{
			val = Vector3d.Lerp(a, b, t);
			return ((Vector3d)(ref val)).normalized * Statics.Lerp(((Vector3d)(ref a)).magnitude, ((Vector3d)(ref b)).magnitude, t);
		}
		if (num < -0.9999999999)
		{
			Vector3d val2 = ((Math.Abs(a.x) < Math.Abs(a.y) && Math.Abs(a.x) < Math.Abs(a.z)) ? Vector3d.Cross(a, new Vector3d(1.0, 0.0, 0.0)) : ((!(Math.Abs(a.y) < Math.Abs(a.z))) ? Vector3d.Cross(a, new Vector3d(0.0, 0.0, 1.0)) : Vector3d.Cross(a, new Vector3d(0.0, 1.0, 0.0))));
			((Vector3d)(ref val2)).Normalize();
			double num2 = Math.Sin(3.1415927410125732 * t);
			double num3 = Math.Cos(3.1415927410125732 * t);
			Vector3d val3 = a * num3 + val2 * num2;
			return ((Vector3d)(ref val3)).normalized * Statics.Lerp(((Vector3d)(ref a)).magnitude, ((Vector3d)(ref b)).magnitude, t);
		}
		num = Clamp(num, -1.0, 1.0);
		double num4 = Math.Acos(num) * t;
		val = b - a * num;
		Vector3d normalized = ((Vector3d)(ref val)).normalized;
		Vector3d val4 = a * Math.Cos(num4) + normalized * Math.Sin(num4);
		double num5 = Statics.Lerp(((Vector3d)(ref a)).magnitude, ((Vector3d)(ref b)).magnitude, t);
		return ((Vector3d)(ref val4)).normalized * num5;
	}
}
