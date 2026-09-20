using System;
using System.Threading;
using MechJebLib.Functions;
using MechJebLib.Lambert;
using MechJebLib.Primitives;
using MechJebLibBindings;
using UnityEngine;
using UnityToolbag;

namespace MuMech;

public class TransferCalculator
{
	public int BestDate;

	public int BestDuration;

	public bool Stop;

	private int _pendingJobs;

	public readonly Orbit OriginOrbit;

	public readonly Orbit DestinationOrbit;

	private readonly Orbit _origin;

	private readonly Orbit _destination;

	protected int NextDateIndex;

	public readonly int DateSamples;

	public readonly double MinDepartureTime;

	public readonly double MaxDepartureTime;

	public readonly double MinTransferTime;

	public readonly double MaxTransferTime;

	protected readonly int MaxDurationSamples;

	public readonly double[,] Computed;

	public double ArrivalDate = -1.0;

	private readonly bool _includeCaptureBurn;

	public bool Finished => _pendingJobs == -1;

	public virtual int Progress => (int)(100.0 * (1.0 - Math.Sqrt((double)Math.Max(0, NextDateIndex) / (double)DateSamples)));

	public TransferCalculator(Orbit o, Orbit target, double minDepartureTime, double maxTransferTime, double minSamplingStep, bool includeCaptureBurn)
		: this(o, target, minDepartureTime, minDepartureTime + maxTransferTime, 3600.0, maxTransferTime, Math.Min(1000, Math.Max(200, (int)(maxTransferTime / Math.Max(minSamplingStep, 60.0)))), Math.Min(1000, Math.Max(200, (int)(maxTransferTime / Math.Max(minSamplingStep, 60.0)))), includeCaptureBurn)
	{
		StartThreads();
	}

	protected TransferCalculator(Orbit o, Orbit target, double minDepartureTime, double maxDepartureTime, double minTransferTime, double maxTransferTime, int width, int height, bool includeCaptureBurn)
	{
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Expected O, but got Unknown
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Expected O, but got Unknown
		OriginOrbit = o;
		DestinationOrbit = target;
		_origin = new Orbit();
		_origin.UpdateFromOrbitAtUT(o, minDepartureTime, o.referenceBody);
		_destination = new Orbit();
		_destination.UpdateFromOrbitAtUT(target, minDepartureTime, target.referenceBody);
		MaxDurationSamples = height;
		DateSamples = width;
		NextDateIndex = DateSamples;
		MinDepartureTime = minDepartureTime;
		MaxDepartureTime = maxDepartureTime;
		MinTransferTime = minTransferTime;
		MaxTransferTime = maxTransferTime;
		_includeCaptureBurn = includeCaptureBurn;
		Computed = new double[DateSamples, MaxDurationSamples];
		_pendingJobs = 0;
	}

	protected void StartThreads()
	{
		if (_pendingJobs != 0)
		{
			throw new Exception("Computation threads have already been started");
		}
		_pendingJobs = Math.Max(1, Environment.ProcessorCount - 1);
		for (int i = 0; i < _pendingJobs; i++)
		{
			ThreadPool.QueueUserWorkItem(ComputeDeltaV);
		}
	}

	private bool IsBetter(int dateIndex1, int durationIndex1, int dateIndex2, int durationIndex2)
	{
		return Computed[dateIndex1, durationIndex1] > Computed[dateIndex2, durationIndex2];
	}

	private void CalcLambertDVs(double t0, double dt, out Vector3d exitDV, out Vector3d captureDV)
	{
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		double num = t0 + dt;
		CelestialBody referenceBody = _origin.referenceBody;
		V3 val = MathExtensions.ToV3(referenceBody.orbit.getOrbitalVelocityAtUT(t0));
		V3 val2 = MathExtensions.ToV3(referenceBody.orbit.getRelativePositionAtUT(t0));
		V3 val3 = MathExtensions.ToV3(_destination.getRelativePositionAtUT(num));
		V3 val4 = MathExtensions.ToV3(_destination.getOrbitalVelocityAtUT(num));
		V3 val5;
		V3 val6;
		try
		{
			(val5, val6) = Gooding.Solve(referenceBody.referenceBody.gravParameter, val2, val3, dt, (TransferGeometry)2, 0, (V3?)V3.Cross(val2, val));
		}
		catch
		{
			val5 = val;
			val6 = val4;
		}
		exitDV = MathExtensions.ToVector3d(val5 - val);
		captureDV = MathExtensions.ToVector3d(val4 - val6);
	}

	private void ComputeDeltaV(object args)
	{
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		for (int num = TakeDateIndex(); num >= 0; num = TakeDateIndex())
		{
			double num2 = DateFromIndex(num);
			if (!double.IsInfinity(num2))
			{
				int num3 = DurationSamplesForDate(num);
				for (int i = 0; i < num3; i++)
				{
					if (Stop)
					{
						break;
					}
					double dt = DurationFromIndex(i);
					CalcLambertDVs(num2, dt, out var exitDV, out var captureDV);
					ManeuverParameters maneuverParameters = ComputeEjectionManeuver(exitDV, _origin, num2);
					Computed[num, i] = ((Vector3d)(ref maneuverParameters.dV)).magnitude;
					if (_includeCaptureBurn)
					{
						Computed[num, i] += ((Vector3d)(ref captureDV)).magnitude;
					}
				}
			}
		}
		JobFinished();
	}

	private void JobFinished()
	{
		if (Interlocked.Decrement(ref _pendingJobs) != 0)
		{
			return;
		}
		for (int i = 0; i < DateSamples; i++)
		{
			int num = DurationSamplesForDate(i);
			for (int j = 0; j < num; j++)
			{
				if (IsBetter(BestDate, BestDuration, i, j))
				{
					BestDate = i;
					BestDuration = j;
				}
			}
		}
		ArrivalDate = DateFromIndex(BestDate) + DurationFromIndex(BestDuration);
		_pendingJobs = -1;
	}

	private int TakeDateIndex()
	{
		return Interlocked.Decrement(ref NextDateIndex);
	}

	protected virtual int DurationSamplesForDate(int dateIndex)
	{
		return (int)((double)MaxDurationSamples * (MaxDepartureTime - DateFromIndex(dateIndex)) / MaxTransferTime);
	}

	public double DurationFromIndex(int index)
	{
		return MinTransferTime + (double)index * (MaxTransferTime - MinTransferTime) / (double)MaxDurationSamples;
	}

	public double DateFromIndex(int index)
	{
		return MinDepartureTime + (double)index * (MaxDepartureTime - MinDepartureTime) / (double)DateSamples;
	}

	private static ManeuverParameters ComputeEjectionManeuver(Vector3d exitVelocity, Orbit initialOrbit, double ut0, bool debug = false)
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0070: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		Vector3d r0 = initialOrbit.getRelativePositionAtUT(ut0);
		Vector3d v0 = initialOrbit.getOrbitalVelocityAtUT(ut0);
		var (val, val2, val3, num) = Astro.SingleImpulseHyperbolicBurn(initialOrbit.referenceBody.gravParameter, MathExtensions.ToV3(r0), MathExtensions.ToV3(v0), MathExtensions.ToV3(exitVelocity), debug);
		if (!num.IsFinite() || !((V3)(ref val3)).magnitude.IsFinite() || !((V3)(ref val2)).magnitude.IsFinite() || !((V3)(ref val)).magnitude.IsFinite())
		{
			Dispatcher.InvokeAsync(delegate
			{
				//IL_0026: Unknown result type (might be due to invalid IL or missing references)
				//IL_0034: Unknown result type (might be due to invalid IL or missing references)
				//IL_0042: Unknown result type (might be due to invalid IL or missing references)
				Debug.Log((object)$"[MechJeb TransferCalculator] BUG mu = {initialOrbit.referenceBody.gravParameter} r0 = {r0} v0 = {v0} vinf = {exitVelocity}");
			});
		}
		return new ManeuverParameters(MathExtensions.V3ToWorld(val2 - val), ut0 + num);
	}
}
