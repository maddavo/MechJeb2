using System;

namespace MuMech;

public class AllGraphTransferCalculator : TransferCalculator
{
	public override int Progress => Math.Min(100, (int)(100.0 * (1.0 - (double)NextDateIndex / (double)DateSamples)));

	public AllGraphTransferCalculator(Orbit o, Orbit target, double minDepartureTime, double maxDepartureTime, double minTransferTime, double maxTransferTime, int width, int height, bool includeCaptureBurn)
		: base(o, target, minDepartureTime, maxDepartureTime, minTransferTime, maxTransferTime, width, height, includeCaptureBurn)
	{
		StartThreads();
	}

	protected override int DurationSamplesForDate(int dateIndex)
	{
		return MaxDurationSamples;
	}
}
