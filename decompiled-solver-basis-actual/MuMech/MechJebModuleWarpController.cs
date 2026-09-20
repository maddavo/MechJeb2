using System;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleWarpController : ComputerModule
{
	private double warpIncreaseAttemptTime;

	private int lastAskedIndex;

	[Persistent(pass = 4)]
	public bool activateSASOnWarp = true;

	[Persistent(pass = 4)]
	public bool useQuickWarp;

	public double warpToUT { get; private set; }

	public bool WarpPaused { get; private set; }

	public MechJebModuleWarpController(MechJebCore core)
		: base(core)
	{
		WarpPaused = false;
		Priority = 100;
		base.Enabled = true;
	}

	public void useQuickWarpInfoItem()
	{
		useQuickWarp = GUILayout.Toggle(useQuickWarp, Localizer.Format("#MechJeb_WarpHelper_checkbox1"), Array.Empty<GUILayoutOption>());
	}

	[GeneralInfoItem("#MechJeb_MJWarpControl", InfoItem.Category.Misc)]
	public void ControlWarpButton()
	{
		if (WarpPaused && GUILayout.Button(Localizer.Format("#MechJeb_WarpHelper_button3"), Array.Empty<GUILayoutOption>()))
		{
			ResumeWarp();
		}
		if (!WarpPaused && GUILayout.Button(Localizer.Format("#MechJeb_WarpHelper_button4"), Array.Empty<GUILayoutOption>()))
		{
			PauseWarp();
		}
	}

	public override void OnUpdate()
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		if (!WarpPaused && lastAskedIndex > 0 && lastAskedIndex != TimeWarp.CurrentRateIndex && (base.Vessel.LandedOrSplashed || (int)TimeWarp.WarpMode != 0 || TimeWarp.CurrentRateIndex != TimeWarp.fetch.GetMaxRateForAltitude(base.Vessel.altitude, base.Vessel.mainBody)))
		{
			WarpPaused = false;
		}
	}

	public override void OnFixedUpdate()
	{
		if (warpToUT > 0.0)
		{
			WarpToUT(warpToUT);
		}
	}

	private void PauseWarp()
	{
		WarpPaused = true;
		if (activateSASOnWarp && TimeWarp.CurrentRateIndex == 0)
		{
			base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)16, false);
		}
	}

	private void ResumeWarp()
	{
		if (WarpPaused)
		{
			WarpPaused = false;
			SetTimeWarpRate(lastAskedIndex, instant: false);
		}
	}

	private void SetTimeWarpRate(int rateIndex, bool instant)
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		if (rateIndex != TimeWarp.CurrentRateIndex)
		{
			if (activateSASOnWarp && (int)TimeWarp.WarpMode == 0 && TimeWarp.CurrentRateIndex == 0)
			{
				base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)16, true);
			}
			lastAskedIndex = rateIndex;
			if (WarpPaused)
			{
				ScreenMessages.PostScreenMessage(Localizer.Format("#MechJeb_WarpHelper_scrmsg"));
			}
			else
			{
				TimeWarp.SetRate(rateIndex, instant, true);
			}
			if (activateSASOnWarp && rateIndex == 0)
			{
				base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)16, false);
			}
		}
	}

	public void WarpToUT(double UT, double maxRate = -1.0)
	{
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Invalid comparison between Unknown and I4
		if (UT <= base.VesselState.Time)
		{
			warpToUT = 0.0;
			return;
		}
		if (maxRate < 0.0)
		{
			maxRate = TimeWarp.fetch.warpRates[TimeWarp.fetch.warpRates.Length - 1];
		}
		double num;
		if (useQuickWarp)
		{
			num = 1.0;
			if ((int)base.Orbit.patchEndTransition != 1 && base.Orbit.EndUT < UT)
			{
				for (int i = 0; i < TimeWarp.fetch.warpRates.Length && (double)((float)i * Time.fixedDeltaTime * TimeWarp.fetch.warpRates[i]) <= base.Orbit.EndUT - base.VesselState.Time; i++)
				{
					num = (double)TimeWarp.fetch.warpRates[i] + 0.1;
				}
			}
			else
			{
				for (int j = 0; j < TimeWarp.fetch.warpRates.Length && (double)((float)j * Time.fixedDeltaTime * TimeWarp.fetch.warpRates[j]) <= UT - base.VesselState.Time; j++)
				{
					num = (double)TimeWarp.fetch.warpRates[j] + 0.1;
				}
			}
		}
		else
		{
			num = 1.0 * (UT - (base.VesselState.Time + (double)(Time.fixedDeltaTime * (float)TimeWarp.CurrentRateIndex)));
		}
		num = Statics.Clamp(num, 1.0, maxRate);
		if (!base.Vessel.LandedOrSplashed && base.VesselState.AltitudeASL < TimeWarp.fetch.GetAltitudeLimit(1, base.MainBody))
		{
			WarpPhysicsAtRate((float)Math.Min(num, 2.0));
		}
		else
		{
			WarpRegularAtRate((float)num, useQuickWarp, useQuickWarp);
		}
		warpToUT = UT;
	}

	public void WarpRegularAtRate(float maxRate, bool instantOnIncrease = false, bool instantOnDecrease = true)
	{
		if (CheckRegularWarp())
		{
			if (TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex] > maxRate)
			{
				DecreaseRegularWarp(instantOnDecrease);
			}
			else if (TimeWarp.CurrentRateIndex + 1 < TimeWarp.fetch.warpRates.Length && TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex + 1] <= maxRate)
			{
				IncreaseRegularWarp(instantOnIncrease);
			}
		}
	}

	public void WarpPhysicsAtRate(float maxRate, bool instantOnIncrease = false, bool instantOnDecrease = true)
	{
		if (CheckPhysicsWarp())
		{
			if (TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex] > maxRate)
			{
				DecreasePhysicsWarp(instantOnDecrease);
			}
			else if (TimeWarp.CurrentRateIndex + 1 < TimeWarp.fetch.physicsWarpRates.Length && TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex + 1] <= maxRate)
			{
				IncreasePhysicsWarp(instantOnIncrease);
			}
		}
	}

	private bool CheckRegularWarp()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		if ((int)TimeWarp.WarpMode != 0)
		{
			Vector3d val = base.VesselState.CoM - base.MainBody.position;
			if (((Vector3d)(ref val)).magnitude - base.MainBody.Radius > base.MainBody.RealMaxAtmosphereAltitude())
			{
				TimeWarp.fetch.Mode = (Modes)0;
				SetTimeWarpRate(0, instant: true);
			}
			return false;
		}
		return true;
	}

	private bool CheckPhysicsWarp()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Invalid comparison between Unknown and I4
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		if ((int)TimeWarp.WarpMode != 1)
		{
			TimeWarp.fetch.Mode = (Modes)1;
			SetTimeWarpRate(0, instant: true);
			return false;
		}
		return true;
	}

	private bool IncreaseRegularWarp(bool instant = false)
	{
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		if (!CheckRegularWarp())
		{
			return false;
		}
		if (TimeWarp.CurrentRateIndex + 1 == TimeWarp.fetch.warpRates.Length)
		{
			return false;
		}
		if (!base.Vessel.LandedOrSplashed)
		{
			Vector3d val = base.VesselState.CoM - base.MainBody.position;
			double num = ((Vector3d)(ref val)).magnitude - base.MainBody.Radius;
			if (TimeWarp.fetch.GetAltitudeLimit(TimeWarp.CurrentRateIndex + 1, base.MainBody) > num)
			{
				return false;
			}
		}
		if (TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex] != TimeWarp.CurrentRate)
		{
			return false;
		}
		if (base.VesselState.Time - warpIncreaseAttemptTime < 2.0)
		{
			return false;
		}
		warpIncreaseAttemptTime = base.VesselState.Time;
		SetTimeWarpRate(TimeWarp.CurrentRateIndex + 1, instant);
		return true;
	}

	private bool IncreasePhysicsWarp(bool instant = false)
	{
		if (!CheckPhysicsWarp())
		{
			return false;
		}
		if (TimeWarp.CurrentRateIndex + 1 == TimeWarp.fetch.physicsWarpRates.Length)
		{
			return false;
		}
		if (TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex] != TimeWarp.CurrentRate)
		{
			return false;
		}
		if (base.VesselState.Time - warpIncreaseAttemptTime < 2.0)
		{
			return false;
		}
		warpIncreaseAttemptTime = base.VesselState.Time;
		SetTimeWarpRate(TimeWarp.CurrentRateIndex + 1, instant);
		return true;
	}

	private bool DecreaseRegularWarp(bool instant = false)
	{
		if (!CheckRegularWarp())
		{
			return false;
		}
		if (TimeWarp.CurrentRateIndex == 0)
		{
			return false;
		}
		SetTimeWarpRate(TimeWarp.CurrentRateIndex - 1, instant);
		return true;
	}

	private bool DecreasePhysicsWarp(bool instant = false)
	{
		if (!CheckPhysicsWarp())
		{
			return false;
		}
		if (TimeWarp.CurrentRateIndex == 0)
		{
			return false;
		}
		SetTimeWarpRate(TimeWarp.CurrentRateIndex - 1, instant);
		return true;
	}

	public bool MinimumWarp(bool instant = false)
	{
		warpToUT = 0.0;
		if (TimeWarp.CurrentRateIndex == 0)
		{
			return false;
		}
		SetTimeWarpRate(0, instant);
		return true;
	}
}
