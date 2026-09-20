using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Smooth.Dispose;
using UnityEngine;
using UnityToolbag;

namespace MuMech;

public class MechJebModuleLandingPredictions : ComputerModule
{
	[ValueInfoItem("#MechJeb_SimDragScalar", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double simDragScalar;

	[ValueInfoItem("#MechJeb_SimLiftScalar", InfoItem.Category.Vessel, format = "SI", units = "m/s²")]
	public double simLiftScalar;

	[ValueInfoItem("#MechJeb_SimDynaPressPa", InfoItem.Category.Vessel, format = "SI", units = "Pa")]
	public double simDynamicPressurePa;

	[ValueInfoItem("#MechJeb_SimsimMach", InfoItem.Category.Vessel, format = "F2")]
	public double simMach;

	[ValueInfoItem("#MechJeb_SimSpdOfSnd", InfoItem.Category.Vessel, format = "SI", units = "m/s")]
	public double simSpeedOfSound;

	[Persistent(pass = 4)]
	public bool makeAerobrakeNodes;

	[Persistent(pass = 4)]
	public bool showTrajectory;

	[Persistent(pass = 4)]
	public bool worldTrajectory = true;

	[Persistent(pass = 4)]
	public bool camTrajectory;

	public bool deployChutes;

	public int limitChutesStage;

	public double decelEndAltitudeASL;

	public IDescentSpeedPolicy descentSpeedPolicy;

	public double parachuteSemiDeployMultiplier = 3.0;

	public bool runErrorSimulations;

	protected bool errorSimulationRunning;

	protected readonly Stopwatch errorStopwatch = new Stopwatch();

	protected long millisecondsBetweenErrorSimulations;

	protected readonly Stopwatch stopwatch = new Stopwatch();

	protected long millisecondsBetweenSimulations;

	protected ReentrySimulation.Result result;

	protected ReentrySimulation.Result errorResult;

	private ReentrySimulation.Result candidateResult;

	private const double MinimumResultAcceptanceDistance = 35.0;

	private const double MaximumResultAcceptanceDistance = 200.0;

	public ManeuverNode aerobrakeNode;

	protected const int interationsPerSecond = 5;

	protected double dt = 0.2;

	protected readonly bool variabledt;

	public bool noSkipToFreefall;

	private readonly Queue readyResults = new Queue();

	private Random random;

	public double maxOrbits = 1.0;

	private double lastSimTime;

	private double lastSimSteps;

	private double lastErrorSimTime;

	private double lastErrorSimSteps;

	public ReentrySimulation.Result Result => result;

	public bool SimulationRunning { get; private set; }

	public double SimulationRunningTime
	{
		get
		{
			if (!SimulationRunning)
			{
				return 0.0;
			}
			return (double)stopwatch.ElapsedMilliseconds / 1000.0;
		}
	}

	public long ResultVersion { get; private set; }

	public ReentrySimulation.Result GetResult()
	{
		return Result;
	}

	public ReentrySimulation.Result GetErrorResult()
	{
		if (errorResult != null && (Object)null != (Object)(object)errorResult.Body)
		{
			errorResult.EndASL = errorResult.Body.TerrainAltitude(errorResult.EndPosition.Latitude, errorResult.EndPosition.Longitude, false);
		}
		return errorResult;
	}

	[ValueInfoItem("#MechJeb_LandingSim", InfoItem.Category.Misc, showInEditor = false)]
	public string LandingSimTime()
	{
		return ((double)stopwatch.ElapsedMilliseconds / 1000.0).ToString("F1") + "/" + lastSimTime.ToString("F2") + " (" + lastSimSteps + ")\n" + ((double)errorStopwatch.ElapsedMilliseconds / 1000.0).ToString("F1") + "/" + lastErrorSimTime.ToString("F2") + " (" + lastErrorSimSteps + ")\n" + ReentrySimulation.ActiveDt.ToString("F2") + " " + ReentrySimulation.ActiveStep + "\n" + dt.ToString("F2") + " " + Time.fixedDeltaTime.ToString("F2") + " " + parachuteSemiDeployMultiplier.ToString("F3");
	}

	public override void OnStart(StartState state)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Invalid comparison between Unknown and I4
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Expected O, but got Unknown
		random = new Random();
		if ((int)state != 0 && (int)state != 1)
		{
			Core.AddToPostDrawQueue(new Callback(DoMapView));
		}
	}

	protected override void OnModuleEnabled()
	{
		TryStartSimulation(doErrorSim: false);
	}

	protected override void OnModuleDisabled()
	{
		stopwatch.Stop();
		stopwatch.Reset();
		errorStopwatch.Stop();
		errorStopwatch.Reset();
		if (candidateResult != null)
		{
			candidateResult.Release();
			candidateResult = null;
		}
	}

	public override void OnFixedUpdate()
	{
		CheckForResult();
		TryStartSimulation(doErrorSim: true);
	}

	private void TryStartSimulation(bool doErrorSim)
	{
		try
		{
			if (!base.Vessel.LandedOrSplashed)
			{
				if (!SimulationRunning && (stopwatch.ElapsedMilliseconds > millisecondsBetweenSimulations || !stopwatch.IsRunning))
				{
					stopwatch.Stop();
					stopwatch.Reset();
					StartSimulation(addParachuteError: false);
				}
				if (doErrorSim && runErrorSimulations && !errorSimulationRunning && (errorStopwatch.ElapsedMilliseconds >= millisecondsBetweenErrorSimulations || !errorStopwatch.IsRunning))
				{
					errorStopwatch.Stop();
					errorStopwatch.Reset();
					StartSimulation(addParachuteError: true);
				}
			}
		}
		catch (Exception ex)
		{
			Debug.LogException(ex);
		}
	}

	public override void OnUpdate()
	{
		MaintainAerobrakeNode();
	}

	protected void StartSimulation(bool addParachuteError)
	{
		double probableLandingSiteASL = 0.0;
		double num = parachuteSemiDeployMultiplier;
		if (addParachuteError)
		{
			errorSimulationRunning = true;
			errorStopwatch.Start();
		}
		else
		{
			SimulationRunning = true;
			stopwatch.Start();
		}
		Orbit val = GetReenteringPatch() ?? base.Orbit;
		if (result != null && result.Outcome == ReentrySimulation.Outcome.LANDED && (Object)(object)result.Body != (Object)null)
		{
			probableLandingSiteASL = result.EndASL;
		}
		if (addParachuteError)
		{
			num *= 1.0 + ((double)random.Next(1000000) - 500000.0) / 10000000.0;
		}
		ReentrySimulation.SimCurves simCurves = ReentrySimulation.SimCurves.Borrow(val.referenceBody);
		SimulatedVessel vessel = SimulatedVessel.Borrow(base.Vessel, simCurves, val.StartUT, (Core.Landing.Enabled && deployChutes) ? limitChutesStage : (-1));
		ReentrySimulation state = ReentrySimulation.Borrow(val, val.StartUT, vessel, simCurves, descentSpeedPolicy, decelEndAltitudeASL, base.VesselState.LimitedMaxThrustAcceleration, num, probableLandingSiteASL, addParachuteError, dt, Time.fixedDeltaTime, maxOrbits, noSkipToFreefall);
		ThreadPool.QueueUserWorkItem(RunSimulation, state);
	}

	private void RunSimulation(object o)
	{
		ReentrySimulation reentrySimulation = (ReentrySimulation)o;
		try
		{
			ReentrySimulation.Result result = reentrySimulation.RunSimulation();
			lock (readyResults)
			{
				readyResults.Enqueue(result);
			}
			if (result.MultiplierHasError)
			{
				errorStopwatch.Stop();
				long elapsedMilliseconds = errorStopwatch.ElapsedMilliseconds;
				lastErrorSimTime = (double)elapsedMilliseconds * 0.001;
				lastErrorSimSteps = result.Steps;
				errorStopwatch.Reset();
				millisecondsBetweenErrorSimulations = Math.Max(400 - elapsedMilliseconds, 5L);
				errorStopwatch.Start();
				errorSimulationRunning = false;
				return;
			}
			stopwatch.Stop();
			long elapsedMilliseconds2 = stopwatch.ElapsedMilliseconds;
			stopwatch.Reset();
			millisecondsBetweenSimulations = Math.Max(200 - elapsedMilliseconds2, 5L);
			lastSimTime = (double)elapsedMilliseconds2 * 0.001;
			lastSimSteps = result.Steps;
			if ((result.Outcome == ReentrySimulation.Outcome.AEROBRAKED || result.Outcome == ReentrySimulation.Outcome.LANDED) && variabledt)
			{
				dt = result.Maxdt * ((double)elapsedMilliseconds2 / 1000.0) / (1.0 / 15.0);
				dt = Math.Max(dt, reentrySimulation.MinDT);
				dt = Math.Min(dt, 10.0);
			}
			stopwatch.Start();
			SimulationRunning = false;
		}
		catch (Exception ex2)
		{
			Exception ex3 = ex2;
			Exception ex = ex3;
			Dispatcher.InvokeAsync(delegate
			{
				Debug.LogException(ex);
			});
		}
		finally
		{
			reentrySimulation.Release();
		}
	}

	private void CheckForResult()
	{
		lock (readyResults)
		{
			while (readyResults.Count > 0)
			{
				ReentrySimulation.Result result = (ReentrySimulation.Result)readyResults.Dequeue();
				if (result.Outcome != ReentrySimulation.Outcome.ERROR)
				{
					if ((Object)(object)result.Body != (Object)null)
					{
						result.EndASL = result.Body.TerrainAltitude(result.EndPosition.Latitude, result.EndPosition.Longitude, false);
					}
					if (result.MultiplierHasError)
					{
						if (errorResult != null)
						{
							errorResult.Release();
						}
						errorResult = result;
					}
					else
					{
						AcceptNormalResult(result);
					}
				}
				else
				{
					if (result.Exception != null)
					{
						ComputerModule.Print("Exception in the last simulation\n" + result.Exception.Message + "\n" + result.Exception.StackTrace);
					}
					result.Release();
				}
			}
		}
	}

	private double ResultAcceptanceDistance(ReentrySimulation.Result first, ReentrySimulation.Result second)
	{
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		if ((Object)(object)first.Body == (Object)null || (Object)(object)first.Body != (Object)(object)second.Body)
		{
			return double.PositiveInfinity;
		}
		Vector3d worldSurfacePosition = first.Body.GetWorldSurfacePosition(first.EndPosition.Latitude, first.EndPosition.Longitude, 0.0);
		Vector3d worldSurfacePosition2 = second.Body.GetWorldSurfacePosition(second.EndPosition.Latitude, second.EndPosition.Longitude, 0.0);
		return Vector3d.Distance(worldSurfacePosition, worldSurfacePosition2);
	}

	private bool ResultsAgree(ReentrySimulation.Result first, ReentrySimulation.Result second)
	{
		double num = Math.Abs(second.InputUT - first.InputUT);
		double num2 = base.VesselState.SpeedSurface * num;
		double num3 = Math.Min(200.0, Math.Max(35.0, 25.0 + 0.5 * num2));
		return ResultAcceptanceDistance(first, second) <= num3;
	}

	private void PublishNormalResult(ReentrySimulation.Result newResult)
	{
		if (result != null)
		{
			result.Release();
		}
		result = newResult;
		ResultVersion++;
	}

	private void AcceptNormalResult(ReentrySimulation.Result newResult)
	{
		if (result == null || ResultsAgree(result, newResult))
		{
			if (candidateResult != null)
			{
				candidateResult.Release();
				candidateResult = null;
			}
			PublishNormalResult(newResult);
		}
		else if (candidateResult != null && ResultsAgree(candidateResult, newResult))
		{
			candidateResult.Release();
			candidateResult = null;
			PublishNormalResult(newResult);
		}
		else
		{
			if (candidateResult != null)
			{
				candidateResult.Release();
			}
			candidateResult = newResult;
		}
	}

	protected Orbit GetReenteringPatch()
	{
		Orbit val = base.Orbit;
		int num = 0;
		do
		{
			num++;
			double num2 = val.referenceBody.Radius + val.referenceBody.RealMaxAtmosphereAltitude();
			Orbit nextPatch = base.Vessel.GetNextPatch(val, aerobrakeNode);
			if (val.PeR < num2)
			{
				if (val.Radius(val.StartUT) < num2)
				{
					return val;
				}
				double num3 = val.NextTimeOfRadius(val.StartUT, num2);
				if (val.StartUT < num3 && (nextPatch == null || num3 < nextPatch.StartUT))
				{
					return val;
				}
			}
			val = nextPatch;
		}
		while (val != null);
		return null;
	}

	protected void MaintainAerobrakeNode()
	{
		//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_010a: Unknown result type (might be due to invalid IL or missing references)
		//IL_010f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0158: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0147: Unknown result type (might be due to invalid IL or missing references)
		if (makeAerobrakeNodes)
		{
			if (aerobrakeNode != null && base.Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode) && aerobrakeNode.UT < base.VesselState.Time && base.VesselState.AltitudeASL > base.MainBody.RealMaxAtmosphereAltitude())
			{
				aerobrakeNode.RemoveSelf();
				aerobrakeNode = null;
			}
			ReentrySimulation.Result result = Result;
			if (result != null && result.Outcome == ReentrySimulation.Outcome.AEROBRAKED)
			{
				Orbit reenteringPatch = GetReenteringPatch();
				double num = ((reenteringPatch != base.Orbit || !(base.VesselState.AltitudeASL < base.MainBody.RealMaxAtmosphereAltitude()) || !(base.VesselState.SpeedVertical > 0.0)) ? reenteringPatch.NextPeriapsisTime(reenteringPatch.StartUT) : base.VesselState.Time);
				Orbit val = MuUtils.OrbitFromStateVectors(result.WorldAeroBrakePosition(), result.WorldAeroBrakeVelocity(), result.Body, result.AeroBrakeUT);
				Vector3d dV = OrbitalManeuverCalculator.DeltaVToChangeApoapsis(reenteringPatch, num, val.ApR);
				if (aerobrakeNode != null && base.Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
				{
					Vector3d dV2 = reenteringPatch.DeltaVToManeuverNodeCoordinates(num, dV);
					aerobrakeNode.UpdateNode(dV2, num);
				}
				else
				{
					aerobrakeNode = base.Vessel.PlaceManeuverNode(reenteringPatch, dV, num);
				}
			}
			else if (aerobrakeNode != null && base.Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
			{
				aerobrakeNode.RemoveSelf();
			}
		}
		else if (aerobrakeNode != null && base.Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
		{
			aerobrakeNode.RemoveSelf();
		}
	}

	private void DoMapView()
	{
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_012c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		if ((!MapView.MapIsEnabled && !camTrajectory) || base.Vessel.LandedOrSplashed || !base.Enabled)
		{
			return;
		}
		ReentrySimulation.Result result = Result;
		if (result == null)
		{
			return;
		}
		if (result.Outcome == ReentrySimulation.Outcome.LANDED)
		{
			GLUtils.DrawGroundMarker(result.Body, result.EndPosition.Latitude, result.EndPosition.Longitude, Color.blue, MapView.MapIsEnabled, 60.0);
		}
		if (!showTrajectory || result.Outcome == ReentrySimulation.Outcome.ERROR || result.Outcome == ReentrySimulation.Outcome.NO_REENTRY)
		{
			return;
		}
		double timeStep = Math.Max(Math.Min((result.EndUT - result.InputUT) / 1000.0, 10.0), 0.1);
		Disposable<List<Vector3d>> val = result.WorldTrajectory(timeStep, worldTrajectory);
		try
		{
			if (!MapView.MapIsEnabled && (noSkipToFreefall || base.Vessel.staticPressurekPa > 0.0))
			{
				val.value[0] = base.VesselState.CoM;
			}
			GLUtils.DrawPath(result.Body, val.value, Color.red, MapView.MapIsEnabled);
		}
		finally
		{
			((IDisposable)val)?.Dispose();
		}
	}

	public MechJebModuleLandingPredictions(MechJebCore core)
		: base(core)
	{
	}
}
