using System;

namespace MuMech;

public class PIDController : IConfigNode
{
	private double _prevError;

	public double INTAccum;

	public double Kp;

	public double Ki;

	public double Kd;

	private readonly double _max;

	private readonly double _min;

	public PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0, double max = double.MaxValue, double min = double.MinValue)
	{
		Kp = kp;
		Ki = ki;
		Kd = kd;
		_max = max;
		_min = min;
		Reset();
	}

	public double Compute(double error)
	{
		INTAccum += error * (double)TimeWarp.fixedDeltaTime;
		double num = Kp * error + Ki * INTAccum + Kd * (error - _prevError) / (double)TimeWarp.fixedDeltaTime;
		if (Math.Max(_min, Math.Min(_max, num)) != num)
		{
			INTAccum -= error * (double)TimeWarp.fixedDeltaTime;
		}
		_prevError = error;
		return num;
	}

	public void Reset()
	{
		_prevError = (INTAccum = 0.0);
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
