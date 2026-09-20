using System;
using System.Collections.Generic;
using UnityEngine;

namespace MuMech;

public abstract class MechJebModuleDeployableController : ComputerModule
{
	protected enum DeployablePartState
	{
		RETRACTED,
		EXTENDED
	}

	protected string ButtonText;

	protected bool Extended;

	[Persistent(pass = 4)]
	public bool AutoDeploy;

	[Persistent(pass = 1)]
	public bool PrevShouldDeploy;

	public bool PrevAutoDeploy = true;

	protected readonly List<ModuleDeployablePart> CachedPartModules = new List<ModuleDeployablePart>(16);

	protected MechJebModuleDeployableController(MechJebCore core)
		: base(core)
	{
		Priority = 200;
		base.Enabled = true;
	}

	protected void DiscoverDeployablePartModules()
	{
		CachedPartModules.Clear();
		foreach (Part part in base.Vessel.Parts)
		{
			foreach (PartModule module in part.Modules)
			{
				if ((Object)(object)module != (Object)null)
				{
					ModuleDeployablePart val = (ModuleDeployablePart)(object)((module is ModuleDeployablePart) ? module : null);
					if (val != null && IsModules(val))
					{
						CachedPartModules.Add(val);
					}
				}
			}
		}
	}

	protected bool IsDeployable(ModuleDeployablePart sa)
	{
		if (!((PartModule)sa).Events["Extend"].active)
		{
			return ((PartModule)sa).Events["Retract"].active;
		}
		return true;
	}

	public void ExtendAll()
	{
		foreach (ModuleDeployablePart cachedPartModule in CachedPartModules)
		{
			if (cachedPartModule != null && IsDeployable(cachedPartModule) && !((PartModule)cachedPartModule).part.ShieldedFromAirstream)
			{
				cachedPartModule.Extend();
			}
		}
	}

	public void RetractAll()
	{
		foreach (ModuleDeployablePart cachedPartModule in CachedPartModules)
		{
			if (cachedPartModule != null && IsDeployable(cachedPartModule) && !((PartModule)cachedPartModule).part.ShieldedFromAirstream)
			{
				cachedPartModule.Retract();
			}
		}
	}

	public bool AllRetracted()
	{
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		foreach (ModuleDeployablePart cachedPartModule in CachedPartModules)
		{
			if (cachedPartModule != null && IsDeployable(cachedPartModule) && (int)cachedPartModule.deployState != 0)
			{
				return false;
			}
		}
		return true;
	}

	private bool ShouldDeploy()
	{
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		if (!base.MainBody.atmosphere)
		{
			return true;
		}
		if (!base.Vessel.LiftedOff())
		{
			return false;
		}
		if (base.Vessel.LandedOrSplashed)
		{
			return false;
		}
		double universalTime = Planetarium.GetUniversalTime();
		double num = base.Orbit.NextPeriapsisTime(universalTime) - universalTime;
		double num2;
		if (num > 0.0 && num < 10.0)
		{
			num2 = base.Orbit.PeA;
		}
		else
		{
			Vector3d relativePositionAtUT = base.Orbit.getRelativePositionAtUT(universalTime);
			double sqrMagnitude = ((Vector3d)(ref relativePositionAtUT)).sqrMagnitude;
			relativePositionAtUT = base.Orbit.getRelativePositionAtUT(universalTime + 10.0);
			num2 = Math.Sqrt(Math.Min(sqrMagnitude, ((Vector3d)(ref relativePositionAtUT)).sqrMagnitude)) - base.MainBody.Radius;
		}
		return num2 > base.MainBody.RealMaxAtmosphereAltitude();
	}

	public override void OnFixedUpdate()
	{
		if (AutoDeploy && !Core.Ascent.Enabled)
		{
			bool flag = ShouldDeploy();
			if (flag)
			{
				if (!PrevShouldDeploy || AutoDeploy != PrevAutoDeploy)
				{
					ExtendAll();
				}
			}
			else if (PrevShouldDeploy || AutoDeploy != PrevAutoDeploy)
			{
				RetractAll();
			}
			PrevShouldDeploy = flag;
			PrevAutoDeploy = true;
		}
		else
		{
			PrevAutoDeploy = false;
		}
		bool flag2 = !AllRetracted();
		if (Extended != flag2)
		{
			ButtonText = GetButtonText(flag2 ? DeployablePartState.EXTENDED : DeployablePartState.RETRACTED);
		}
		Extended = flag2;
	}

	protected bool ExtendingOrRetracting()
	{
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Invalid comparison between Unknown and I4
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Invalid comparison between Unknown and I4
		foreach (ModuleDeployablePart cachedPartModule in CachedPartModules)
		{
			if ((Object)(object)cachedPartModule != (Object)null && IsDeployable(cachedPartModule) && ((int)cachedPartModule.deployState == 3 || (int)cachedPartModule.deployState == 2))
			{
				return true;
			}
		}
		return false;
	}

	protected abstract bool IsModules(ModuleDeployablePart p);

	protected abstract string GetButtonText(DeployablePartState deployablePartState);

	public override void OnStart(StartState state)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		base.OnStart(state);
		if (HighLogic.LoadedSceneIsFlight)
		{
			DiscoverDeployablePartModules();
		}
	}

	public override void OnVesselWasModified(Vessel v)
	{
		DiscoverDeployablePartModules();
	}
}
