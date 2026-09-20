using System;

namespace MuMech;

internal class ParachutePlan
{
	private LinearRegression _regression;

	private ReentrySimulation.Result _lastResult;

	private ReentrySimulation.Result _lastErrorResult;

	private readonly CelestialBody _body;

	private readonly MechJebModuleLandingAutopilot _autoPilot;

	private bool _parachutePresent;

	private double _maxSemiDeployHeight;

	private double _minSemiDeployHeight;

	private double _maxMultiplier;

	private double _correlation;

	private const int DATA_SET_SIZE = 40;

	public double Multiplier { get; private set; }

	public int MultiplierDataAmount => _regression.DataSetSize;

	public double MultiplierQuality
	{
		get
		{
			double correlation = _correlation;
			if (double.IsNaN(correlation))
			{
				return 0.0;
			}
			return Math.Max(0.0, correlation * -100.0);
		}
	}

	public void AddResult(ReentrySimulation.Result newResult)
	{
		if (newResult.MultiplierHasError)
		{
			if (_lastErrorResult != null && newResult.ID == _lastErrorResult.ID)
			{
				return;
			}
			_lastErrorResult = newResult;
		}
		else
		{
			if (_lastResult != null && newResult.ID == _lastResult.ID)
			{
				return;
			}
			_lastResult = newResult;
		}
		double overshoot = newResult.GetOvershoot(_autoPilot.Core.Target.targetLatitude, _autoPilot.Core.Target.targetLongitude);
		_regression.Add(overshoot, newResult.ParachuteMultiplier);
		_correlation = _regression.CorrelationCoefficient;
		if (_correlation > -0.2)
		{
			if (_correlation > 0.0 && _regression.DataSetSize > 5)
			{
				ClearData();
			}
			return;
		}
		switch (_regression.DataSetSize)
		{
		case 1:
			Multiplier *= 0.99999;
			break;
		default:
			try
			{
				Multiplier = _regression.YIntercept;
			}
			catch (Exception)
			{
				Multiplier *= 0.99999;
			}
			break;
		case 2:
			break;
		}
		if (Multiplier < 1.0 || double.IsNaN(Multiplier))
		{
			Multiplier = 1.0;
		}
		if (Multiplier > _maxMultiplier)
		{
			Multiplier = _maxMultiplier;
		}
	}

	public ParachutePlan(MechJebModuleLandingAutopilot autopliot)
	{
		_regression = new LinearRegression(40);
		_autoPilot = autopliot;
		_body = autopliot.Vessel.orbit.referenceBody;
	}

	public void ClearData()
	{
		_regression = new LinearRegression(40);
	}

	public void StartPlanning()
	{
		float num = 0f;
		float num2 = 0f;
		_parachutePresent = false;
		for (int i = 0; i < _autoPilot.VesselState.Parachutes.Count; i++)
		{
			ModuleParachute val = _autoPilot.VesselState.Parachutes[i];
			if (val.minAirPressureToOpen > num)
			{
				num = val.minAirPressureToOpen;
			}
			if (val.deployAltitude > num2)
			{
				num2 = val.deployAltitude;
			}
			_parachutePresent = true;
		}
		if (_parachutePresent)
		{
			_maxSemiDeployHeight = _body.AltitudeForPressure(num);
			_minSemiDeployHeight = num2;
			_maxMultiplier = _maxSemiDeployHeight / _minSemiDeployHeight;
			Multiplier = _maxMultiplier / 2.0;
		}
	}
}
