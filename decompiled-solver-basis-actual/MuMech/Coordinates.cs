using System;

namespace MuMech;

public class Coordinates
{
	public readonly double Latitude;

	public readonly double Longitude;

	public Coordinates(double latitude, double longitude)
	{
		Latitude = latitude;
		Longitude = longitude;
	}

	public static string ToStringDecimal(double latitude, double longitude, bool newline = false, int precision = 3)
	{
		double num = MuUtils.ClampDegrees180(longitude);
		double num2 = Math.Abs(latitude);
		double num3 = Math.Abs(num);
		return num2.ToString("F" + precision) + "° " + ((latitude > 0.0) ? "N" : "S") + (newline ? "\n" : ", ") + num3.ToString("F" + precision) + "° " + ((num > 0.0) ? "E" : "W");
	}

	public string ToStringDecimal(bool newline = false, int precision = 3)
	{
		return ToStringDecimal(Latitude, Longitude, newline, precision);
	}

	public static string ToStringDMS(double latitude, double longitude, bool newline = false)
	{
		double num = MuUtils.ClampDegrees180(longitude);
		return AngleToDMS(latitude) + ((latitude > 0.0) ? " N" : " S") + (newline ? "\n" : ", ") + AngleToDMS(num) + ((num > 0.0) ? " E" : " W");
	}

	public string ToStringDMS(bool newline = false)
	{
		return ToStringDMS(Latitude, Longitude, newline);
	}

	public static string AngleToDMS(double angle)
	{
		int num = (int)Math.Floor(Math.Abs(angle));
		int num2 = (int)Math.Floor(60.0 * (Math.Abs(angle) - (double)num));
		int num3 = (int)Math.Floor(3600.0 * (Math.Abs(angle) - (double)num - (double)num2 / 60.0));
		return $"{num:0}° {num2:00}' {num3:00}\"";
	}
}
