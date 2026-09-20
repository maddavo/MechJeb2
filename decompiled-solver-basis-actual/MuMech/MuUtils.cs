using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace MuMech;

public static class MuUtils
{
	private static readonly string _cfgPath = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/MechJeb2/Plugins/PluginData/MechJeb2");

	public static string SystemClipboard
	{
		get
		{
			return GUIUtility.systemCopyBuffer;
		}
		set
		{
			GUIUtility.systemCopyBuffer = value;
		}
	}

	public static string GetCfgPath(string file)
	{
		return Path.Combine(_cfgPath, file);
	}

	public static bool FileExistsCreateDirectory(string path)
	{
		string directoryName = Path.GetDirectoryName(path);
		if (directoryName != null && !Directory.Exists(directoryName))
		{
			Directory.CreateDirectory(directoryName);
		}
		return File.Exists(path);
	}

	public static string PadPositive(double x, string format = "F3")
	{
		string text = x.ToString(format);
		if (text[0] != '-')
		{
			return " " + text;
		}
		return text;
	}

	public static string PadPositiveSci(double x, string format = "F3")
	{
		string text = ((x > 1000000.0) ? x.ToString("G3") : x.ToString(format));
		if (text[0] != '-')
		{
			return " " + text;
		}
		return text;
	}

	public static string PrettyPrint(Vector3d vector, string format = "F3")
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		return "[" + PadPositive(vector.x, format) + ", " + PadPositive(vector.y, format) + ", " + PadPositive(vector.z, format) + " ]";
	}

	public static string PrettyPrintSci(Vector3d vector, string format = "F3")
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		return "[" + PadPositiveSci(vector.x, format) + ", " + PadPositiveSci(vector.y, format) + ", " + PadPositiveSci(vector.z, format) + " ]";
	}

	public static string PrettyPrint(Quaternion quaternion, string format = "F3")
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		return "[" + PadPositive(quaternion.x, format) + ", " + PadPositive(quaternion.y, format) + ", " + PadPositive(quaternion.z, format) + ", " + PadPositive(quaternion.w, format) + "]";
	}

	public static double ClampDegrees360(double angle)
	{
		angle %= 360.0;
		if (angle < 0.0)
		{
			return angle + 360.0;
		}
		return angle;
	}

	public static double ClampDegrees180(double angle)
	{
		angle = ClampDegrees360(angle);
		if (angle > 180.0)
		{
			angle -= 360.0;
		}
		return angle;
	}

	public static Orbit OrbitFromStateVectors(Vector3d pos, Vector3d vel, CelestialBody body, double ut)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Expected O, but got Unknown
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0070: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		Orbit val = new Orbit();
		Vector3d val2 = pos - body.position;
		val.UpdateFromStateVectors(((Vector3d)(ref val2)).xzy, ((Vector3d)(ref vel)).xzy, body, ut);
		if (double.IsNaN(val.argumentOfPeriapsis))
		{
			Vector3d val3 = Vector3d.op_Implicit(Quaternion.AngleAxis(0f - (float)val.LAN, Vector3d.op_Implicit(Planetarium.up)) * Vector3d.op_Implicit(Planetarium.right));
			Vector3d xzy = ((Vector3d)(ref val.eccVec)).xzy;
			double num = Vector3d.Dot(val3, xzy) / (((Vector3d)(ref val3)).magnitude * ((Vector3d)(ref xzy)).magnitude);
			if (num > 1.0)
			{
				val.argumentOfPeriapsis = 0.0;
			}
			else if (num < -1.0)
			{
				val.argumentOfPeriapsis = 180.0;
			}
			else
			{
				val.argumentOfPeriapsis = Math.Acos(num);
			}
		}
		return val;
	}

	public static void Swap<T>(this IList<T> list, int indexA, int indexB)
	{
		T value = list[indexB];
		T value2 = list[indexA];
		list[indexA] = value;
		list[indexB] = value2;
	}

	public static bool PhysicsRunning()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Invalid comparison between Unknown and I4
		if ((int)TimeWarp.WarpMode != 1)
		{
			return TimeWarp.CurrentRateIndex == 0;
		}
		return true;
	}

	public static Color HSVtoRGB(float hue, float saturation, float value, float alpha)
	{
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		int num = Convert.ToInt32(Math.Floor(hue / 60f)) % 6;
		float num2 = hue / 60f - Mathf.Floor(hue / 60f);
		float num3 = value * (1f - saturation);
		float num4 = value * (1f - num2 * saturation);
		float num5 = value * (1f - (1f - num2) * saturation);
		return (Color)(num switch
		{
			0 => new Color(value, num5, num3, alpha), 
			1 => new Color(num4, value, num3, alpha), 
			2 => new Color(num3, value, num5, alpha), 
			3 => new Color(num3, num4, value, alpha), 
			4 => new Color(num5, num3, value, alpha), 
			_ => new Color(value, num3, num4, alpha), 
		});
	}
}
