using System;
using System.Collections.Generic;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.Functions;
using MechJebLib.HoverslamSimulation;
using MechJebLib.Primitives;
using MechJebLib.Utils;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebModuleHoverslamSimulation : ComputerModule
{
	[ValueInfoItem("#MechJeb_HoverslamSlope", InfoItem.Category.Hoverslam, format = "F2", units = "º", tooltip = "#MechJeb_HoverslamSlope_tooltip")]
	public double Slope;

	[ValueInfoItem("#MechJeb_HoverslamTerrainAltitude", InfoItem.Category.Hoverslam, format = "F1", units = "m", tooltip = "#MechJeb_HoverslamTerrainAltitude_tooltip")]
	public double TerrainAltitude;

	[ToggleInfoItem("#MechJeb_HoverslamMapLandingPrediction", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamMapLandingPrediction_tooltip")]
	[Persistent(pass = 6)]
	public bool MapLandingPrediction = true;

	[EditableInfoItem("#MechJeb_HoverslamSimRecalcInterval", InfoItem.Category.Hoverslam, width = 50f, rightLabel = "s", expandWidth = true, tooltip = "#MechJeb_HoverslamSimRecalcInterval_tooltip")]
	[Persistent(pass = 6)]
	public readonly EditableDouble SimRecalcInterval = new EditableDouble(1.0);

	[EditableInfoItem("#MechJeb_HoverslamVerticalAuthority", InfoItem.Category.Hoverslam, width = 50f, rightLabel = "%", expandWidth = true, tooltip = "#MechJeb_HoverslamVerticalAuthority_tooltip")]
	[Persistent(pass = 6)]
	public readonly EditableDoubleMult VerticalAuthority = new EditableDoubleMult(0.5, 0.01);

	[EditableInfoItem("#MechJeb_HoverslamVerticalAltitude", InfoItem.Category.Hoverslam, width = 50f, rightLabel = "m", expandWidth = true, tooltip = "#MechJeb_HoverslamVerticalAltitude_tooltip")]
	[Persistent(pass = 6)]
	public readonly EditableDouble VerticalAltitude = new EditableDouble(100.0);

	public Vector3d LandingPosition;

	public double IgnitionUT;

	public double LandingUT;

	public double FinalThrustAccel;

	public double Lat;

	public double Lng;

	public double DeltaV;

	public double FinalDescentSpeed;

	public Vector3d IgnitionAttitude;

	private double _lastCycleUT;

	private readonly HoverslamSimulationManager _manager = new HoverslamSimulationManager(true);

	private readonly HoverslamSimulation _hoverslam = new HoverslamSimulation();

	private List<FuelStats> _vacStats => Core.StageStats.VacStats;

	public double IgnitionCountdown => IgnitionUT - base.VesselState.Time;

	public double LandingCountdown => LandingUT - base.VesselState.Time;

	[ValueInfoItem("#MechJeb_HoverslamImpact", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamImpact_tooltip")]
	public string Impact()
	{
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		if (base.Orbit.PeA > 0.0 || base.Vessel.Landed)
		{
			return "N/A";
		}
		double num = base.VesselState.Time;
		try
		{
			for (int i = 0; i < 10; i++)
			{
				Vector3d worldPosition = base.Orbit.WorldPositionAtUT(num);
				double radius = base.MainBody.Radius + base.MainBody.TerrainAltitude(worldPosition);
				num = base.Orbit.NextTimeOfRadius(base.VesselState.Time, radius);
			}
		}
		catch (ArgumentException)
		{
			return GuiUtils.TimeToDHMS(0.0, 1);
		}
		catch (ArithmeticException)
		{
			return GuiUtils.TimeToDHMS(0.0, 1);
		}
		return GuiUtils.TimeToDHMS(num - base.VesselState.Time, 1);
	}

	[ValueInfoItem("#MechJeb_HoverslamIgnition", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamIgnition_tooltip")]
	public string Ignition()
	{
		if (!(base.Orbit.PeA > 0.0) && !base.Vessel.Landed)
		{
			return GuiUtils.TimeToDHMS(IgnitionCountdown, 1);
		}
		return "N/A";
	}

	[ValueInfoItem("#MechJeb_HoverslamTouchdown", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamTouchdown_tooltip")]
	public string Touchdown()
	{
		if (!(base.Orbit.PeA > 0.0) && !base.Vessel.Landed)
		{
			return GuiUtils.TimeToDHMS(LandingCountdown, 1);
		}
		return "N/A";
	}

	[ValueInfoItem("#MechJeb_HoverslamDeltaV", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamDeltaV_tooltip")]
	public string HoverslamDeltaV()
	{
		if (!Statics.IsFinite(DeltaV))
		{
			return "N/A";
		}
		return Statics.ToSI(DeltaV, 4, int.MaxValue) + "m/s";
	}

	[ValueInfoItem("#MechJeb_HoverslamCoordinates", InfoItem.Category.Hoverslam, width = 90f, tooltip = "#MechJeb_HoverslamCoordinates_tooltip")]
	public string HoverslamCoordinates()
	{
		return Coordinates.ToStringDMS(Lat, Lng, newline: true);
	}

	[ValueInfoItem("#MechJeb_HoverslamBiome", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamBiome_tooltip")]
	public string Biome()
	{
		if (!Statics.IsFinite(LandingUT))
		{
			return "N/A";
		}
		return base.MainBody.GetExperimentBiomeSafe(Lat, Lng);
	}

	public MechJebModuleHoverslamSimulation(MechJebCore core)
		: base(core)
	{
	}//IL_004e: Unknown result type (might be due to invalid IL or missing references)
	//IL_0058: Expected O, but got Unknown
	//IL_0059: Unknown result type (might be due to invalid IL or missing references)
	//IL_0063: Expected O, but got Unknown


	public override void OnStart(StartState state)
	{
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Expected O, but got Unknown
		base.Enabled = HighLogic.LoadedSceneIsFlight;
		Core.AddToPostDrawQueue(new Callback(DrawMapViewLanding));
	}

	private void DrawMapViewLanding()
	{
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		if (!HighLogic.LoadedSceneIsEditor && MapView.MapIsEnabled && MapLandingPrediction && base.Vessel.isActiveVessel && !((Object)(object)base.Vessel.GetMasterMechJeb() != (Object)(object)Core) && Statics.IsFinite(LandingUT))
		{
			GLUtils.DrawGroundMarker(base.MainBody, Lat, Lng, Color.magenta, map: true);
		}
	}

	protected override void OnModuleEnabled()
	{
		Reset();
	}

	protected override void OnModuleDisabled()
	{
		Reset();
	}

	private void Reset()
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		LandingPosition = new Vector3d(double.NaN, double.NaN, double.NaN);
		IgnitionUT = double.NaN;
		LandingUT = double.NaN;
		DeltaV = double.NaN;
		IgnitionAttitude = new Vector3d(double.NaN, double.NaN, double.NaN);
		Lat = 0.0;
		Lng = 0.0;
		FinalThrustAccel = -1.0;
		((AsyncJob)_hoverslam).Cancel();
		_lastCycleUT = 0.0;
	}

	private double CalculateDeltaV(double burnTime)
	{
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		Core.StageStats.RequestUpdate();
		int num = -1;
		double num2 = 0.0;
		int num3 = _vacStats.Count - 1;
		while (num3 >= 0 && burnTime > 0.0)
		{
			FuelStats val = _vacStats[num3];
			if (!(val.DeltaV <= 0.0))
			{
				num = num3;
				double num4 = ((val.DeltaTime < burnTime) ? val.DeltaTime : burnTime);
				num2 += Astro.DeltaVFromMassThrustIspBurntime(val.StartMass, val.Thrust, val.Isp, num4);
				burnTime -= num4;
			}
			num3--;
		}
		if (burnTime > 0.0 && num > -1)
		{
			FuelStats val2 = _vacStats[num];
			num2 += Astro.DeltaVFromMassThrustIspBurntime(val2.StartMass, val2.Thrust, val2.Isp, burnTime);
		}
		return num2;
	}

	private double GetGroundRadius()
	{
		if (!Statics.IsFinite(LandingUT))
		{
			return base.MainBody.Radius;
		}
		return base.MainBody.Radius + TerrainAltitude;
	}

	public override void OnFixedUpdate()
	{
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_021f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0224: Unknown result type (might be due to invalid IL or missing references)
		//IL_0229: Unknown result type (might be due to invalid IL or missing references)
		//IL_026c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0271: Unknown result type (might be due to invalid IL or missing references)
		//IL_0273: Unknown result type (might be due to invalid IL or missing references)
		//IL_0309: Unknown result type (might be due to invalid IL or missing references)
		//IL_028b: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_03e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_03f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_040b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0299: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_04c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0389: Unknown result type (might be due to invalid IL or missing references)
		//IL_039a: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_02be: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f6: Unknown result type (might be due to invalid IL or missing references)
		double groundRadius = GetGroundRadius();
		if (base.VesselState.MainBody == null || !base.Vessel.VesselOffGround() || base.Orbit.PeA > 0.0 || base.Orbit.ApR < groundRadius + (double)VerticalAltitude)
		{
			Reset();
		}
		else
		{
			if (base.VesselState.Time < _lastCycleUT + (double)SimRecalcInterval * (double)TimeWarp.CurrentRate)
			{
				return;
			}
			Core.StageStats.RequestUpdate();
			if (_vacStats.Count <= 0 || ((AsyncJob)_hoverslam).IsRunning)
			{
				return;
			}
			if (((AsyncJob)_hoverslam).IsCompleted)
			{
				LandingPosition = MathExtensions.V3ToWorldRotated(_hoverslam.Rf);
				IgnitionUT = _hoverslam.IgnitionUT;
				IgnitionAttitude = MathExtensions.V3ToWorldRotated(_hoverslam.IgnitionAttitude);
				LandingUT = _hoverslam.LandingUT;
				FinalThrustAccel = _hoverslam.FinalThrustAccel;
				base.MainBody.GetLatLngAltAtUT(LandingUT, LandingPosition, out Lat, out Lng, out var _);
				TerrainAltitude = base.MainBody.TerrainAltitude(Lat, Lng, true);
				Slope = base.MainBody.GetPQSSlopeDegrees(Lat, Lng);
				DeltaV = CalculateDeltaV(LandingCountdown - ((IgnitionUT < base.VesselState.Time) ? 0.0 : IgnitionCountdown));
			}
			if (((AsyncJob)_hoverslam).IsFaulted && ((AsyncJob)_hoverslam).Exception != null)
			{
				ComputerModule.Print($"[MechJebModuleHoverslamSimulation] {((AsyncJob)_hoverslam).Exception}");
			}
			if (!((AsyncJob)_hoverslam).TryMarkReady())
			{
				return;
			}
			_manager.Reset();
			V3 val = Math.PI * 2.0 / base.MainBody.rotationPeriod * V3.northpole;
			bool flag = true;
			bool flag2 = ReflectionUtils.IsLoadedRealFuels && base.VesselState.LowestUllage < 1.0;
			int num = -1;
			for (int num2 = _vacStats.Count - 1; num2 >= 0; num2--)
			{
				FuelStats val2 = _vacStats[num2];
				if (!(val2.DeltaV <= 0.0))
				{
					if (flag2 && Statics.IsFinite(val2.RcsUllageTime) && val2.RcsUllageTime > 0.0)
					{
						double num3 = val2.StartMass * 1000.0;
						double rcsThrust = val2.RcsThrust;
						double rcsISP = val2.RcsISP;
						double rcsUllageTime = val2.RcsUllageTime;
						double num4 = Astro.MassFromMassThrustIspBurntime(num3, rcsThrust, rcsISP, rcsUllageTime);
						_manager.AddStage(num3, num4, rcsThrust, rcsISP, val2.KSPStage, num2);
						flag2 = false;
					}
					int num5 = num - val2.KSPStage;
					if (num5 > 0)
					{
						double num6 = Core.Staging.AutostagePreDelay;
						if (num5 > 1)
						{
							double num7 = (double)Core.Staging.AutostagePreDelay + (double)Core.Staging.AutostagePostDelay;
							if (num7 < (double)PhysicsGlobals.StagingCooldownTimer)
							{
								num7 = PhysicsGlobals.StagingCooldownTimer;
							}
							num6 += num7 * (double)(num5 - 1);
						}
						_manager.AddCoast(val2.StartMass * 1000.0, val2.StartMass * 1000.0, num6, val2.KSPStage, num2);
					}
					_manager.AddStage(val2.StartMass * 1000.0, val2.EndMass * 1000.0, val2.Thrust * 1000.0, val2.Isp, val2.KSPStage, num2);
					num = val2.KSPStage;
					flag = false;
				}
			}
			if (!flag)
			{
				double num8 = base.MainBody.gravParameter / (groundRadius * groundRadius);
				double num9 = num8 + Statics.Clamp01((double)VerticalAuthority) * (FinalThrustAccel - num8);
				FinalDescentSpeed = ((FinalThrustAccel < 0.0) ? 0.0 : Math.Sqrt(Math.Max(2.0 * (num9 - num8) * (double)VerticalAltitude, 0.0)));
				_manager.Initial(Core.StageStats.VacR, Core.StageStats.VacV, Core.StageStats.VacT, base.MainBody.gravParameter, val);
				_manager.TargetConditions(groundRadius + (double)VerticalAltitude, FinalDescentSpeed);
				_manager.Reconfigure(_hoverslam);
				if (!((AsyncJob)_hoverslam).TryStartJob((object)null))
				{
					throw new Exception("[MechJebModuleHoverslamSimulation] could not start job");
				}
				_lastCycleUT = base.VesselState.Time;
			}
		}
	}
}
