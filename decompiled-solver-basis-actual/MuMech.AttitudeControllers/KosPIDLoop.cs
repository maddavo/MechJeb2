using System;

namespace MuMech.AttitudeControllers;

public class KosPIDLoop
{
	private double _ki;

	private double _loopKi;

	public double Kp { get; set; }

	public double Ki
	{
		get
		{
			return _ki;
		}
		set
		{
			_ki = value;
			_loopKi = value;
		}
	}

	public double Kd { get; set; }

	public double Input { get; set; }

	public double Setpoint { get; set; }

	public double Error { get; set; }

	public double Output { get; set; }

	public double MinOutput { get; set; }

	public double MaxOutput { get; set; }

	public double ErrorSum { get; set; }

	public double PTerm { get; set; }

	public double ITerm { get; set; }

	public double DTerm { get; set; }

	public bool ExtraUnwind { get; set; }

	public double ChangeRate { get; set; }

	public bool UnWinding { get; set; }

	public KosPIDLoop(double maxoutput = double.MaxValue, double minoutput = double.MinValue, bool extraUnwind = false)
		: this(1.0, 0.0, 0.0, maxoutput, minoutput, extraUnwind)
	{
	}

	public KosPIDLoop(double kp, double ki, double kd, double maxoutput = double.MaxValue, double minoutput = double.MinValue, bool extraUnwind = false)
	{
		Kp = kp;
		Ki = ki;
		Kd = kd;
		Input = 0.0;
		Setpoint = 0.0;
		Error = 0.0;
		Output = 0.0;
		MaxOutput = maxoutput;
		MinOutput = minoutput;
		ErrorSum = 0.0;
		PTerm = 0.0;
		ITerm = 0.0;
		DTerm = 0.0;
		ExtraUnwind = extraUnwind;
	}

	public double Update(double input, double setpoint, double minOutput, double maxOutput)
	{
		MaxOutput = maxOutput;
		MinOutput = minOutput;
		Setpoint = setpoint;
		return Update(input);
	}

	public double Update(double input, double setpoint, double maxOutput)
	{
		return Update(input, setpoint, 0.0 - maxOutput, maxOutput);
	}

	public double Update(double input)
	{
		double num = Setpoint - input;
		double num2 = num * Kp;
		double num3 = 0.0;
		double num4 = 0.0;
		double num5 = TimeWarp.fixedDeltaTime;
		if (_loopKi != 0.0)
		{
			if (ExtraUnwind)
			{
				if (Math.Sign(num) != Math.Sign(ErrorSum))
				{
					if (!UnWinding)
					{
						_loopKi *= 2.0;
						UnWinding = true;
					}
				}
				else if (UnWinding)
				{
					_loopKi = _ki;
					UnWinding = false;
				}
			}
			num3 = ITerm + num * num5 * _loopKi;
		}
		ChangeRate = (input - Input) / num5;
		if (Kd != 0.0)
		{
			num4 = (0.0 - ChangeRate) * Kd;
		}
		Output = num2 + num3 + num4;
		if (Output > MaxOutput)
		{
			Output = MaxOutput;
			if (_loopKi != 0.0)
			{
				num3 = Output - Math.Min(num2 + num4, MaxOutput);
			}
		}
		if (Output < MinOutput)
		{
			Output = MinOutput;
			if (_loopKi != 0.0)
			{
				num3 = Output - Math.Max(num2 + num4, MinOutput);
			}
		}
		Input = input;
		Error = num;
		PTerm = num2;
		ITerm = num3;
		DTerm = num4;
		if (_loopKi != 0.0)
		{
			ErrorSum = num3 / _loopKi;
		}
		else
		{
			ErrorSum = 0.0;
		}
		return Output;
	}

	public void ResetI()
	{
		ErrorSum = 0.0;
		ITerm = 0.0;
	}
}
