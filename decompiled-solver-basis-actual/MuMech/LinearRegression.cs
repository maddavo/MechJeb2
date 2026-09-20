using System;

namespace MuMech;

internal class LinearRegression
{
	private readonly double[] _x;

	private readonly double[] _y;

	private readonly int _maxDataPoints;

	private int _currentDataPoint;

	private double _sumX;

	private double _sumY;

	private double _sumXx;

	private double _sumXY;

	private double _sumYy;

	private double _slope
	{
		get
		{
			if (DataSetSize < 2)
			{
				throw new Exception("Not enough data to calculate trend line");
			}
			return (_sumXY - _sumX * _sumY / (double)DataSetSize) / (_sumXx - _sumX * _sumX / (double)DataSetSize);
		}
	}

	public double YIntercept => _sumY / (double)DataSetSize - _slope * (_sumX / (double)DataSetSize);

	public int DataSetSize { get; private set; }

	public double CorrelationCoefficient
	{
		get
		{
			double num = Math.Sqrt(_sumXx / (double)DataSetSize - _sumX * _sumX / (double)DataSetSize / (double)DataSetSize);
			double num2 = Math.Sqrt(_sumYy / (double)DataSetSize - _sumY * _sumY / (double)DataSetSize / (double)DataSetSize);
			return (_sumXY / (double)DataSetSize - _sumX * _sumY / (double)DataSetSize / (double)DataSetSize) / num / num2;
		}
	}

	public LinearRegression(int maxDataPoints)
	{
		_maxDataPoints = maxDataPoints;
		DataSetSize = 0;
		_currentDataPoint = -1;
		_x = new double[maxDataPoints];
		_y = new double[maxDataPoints];
	}

	public void Add(double x, double y)
	{
		_currentDataPoint++;
		DataSetSize++;
		if (_currentDataPoint >= _maxDataPoints)
		{
			_currentDataPoint = 0;
		}
		if (DataSetSize > _maxDataPoints)
		{
			DataSetSize = _maxDataPoints;
		}
		_x[_currentDataPoint] = x;
		_y[_currentDataPoint] = y;
		_sumX = 0.0;
		_sumY = 0.0;
		_sumXx = 0.0;
		_sumXY = 0.0;
		_sumYy = 0.0;
		for (int i = 0; i < DataSetSize; i++)
		{
			double num = _x[i];
			double num2 = _y[i];
			_sumX += num;
			_sumXx += num * num;
			_sumY += num2;
			_sumYy += num2 * num2;
			_sumXY += num * num2;
		}
	}
}
