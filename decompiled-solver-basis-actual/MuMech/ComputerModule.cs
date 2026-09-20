using System;
using System.Linq;
using UnityEngine;

namespace MuMech;

public class ComputerModule : IComparable<ComputerModule>
{
	public readonly MechJebCore Core;

	public int Priority;

	[Persistent(pass = 1)]
	public string unlockParts = "";

	[Persistent(pass = 1)]
	public string unlockTechs = "";

	public bool UnlockChecked;

	private bool _enabled;

	public readonly ModuleEvent ModuleDisabledEvents = new ModuleEvent();

	public readonly string ProfilerName;

	public bool Dirty;

	public readonly UserPool Users;

	public Vessel Vessel => Part.vessel;

	public CelestialBody MainBody => Part.vessel.mainBody;

	public VesselState VesselState => Core.VesselState;

	public Part Part => ((PartModule)Core).part;

	public Orbit Orbit => Part.vessel.orbit;

	public bool Enabled
	{
		get
		{
			return _enabled;
		}
		set
		{
			if (value != _enabled)
			{
				Dirty = true;
				_enabled = value;
				if (_enabled)
				{
					OnModuleEnabled();
					return;
				}
				OnModuleDisabled();
				ModuleDisabledEvents.Fire(reverse: true);
				ModuleDisabledEvents.Clear();
			}
		}
	}

	public int CompareTo(ComputerModule other)
	{
		if (other != null)
		{
			return Priority.CompareTo(other.Priority);
		}
		return 1;
	}

	protected ComputerModule(MechJebCore core)
	{
		Core = core;
		ProfilerName = GetType().Name;
		Users = new UserPool(this);
	}

	protected virtual void OnModuleEnabled()
	{
	}

	protected virtual void OnModuleDisabled()
	{
	}

	public virtual void OnControlLost()
	{
	}

	public virtual void Drive(FlightCtrlState s)
	{
	}

	public virtual void OnVesselWasModified(Vessel v)
	{
	}

	public virtual void OnVesselStandardModification(Vessel v)
	{
	}

	public virtual void OnStart(StartState state)
	{
	}

	public virtual void OnActive()
	{
	}

	public virtual void OnInactive()
	{
	}

	public virtual void OnAwake()
	{
	}

	public virtual void OnFixedUpdate()
	{
	}

	public virtual void OnUpdate()
	{
	}

	public virtual void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		try
		{
			if (global != null)
			{
				ConfigNode.LoadObjectFromConfig((object)this, global, 4);
			}
			if (type != null)
			{
				ConfigNode.LoadObjectFromConfig((object)this, type, 2);
			}
			if (local != null)
			{
				ConfigNode.LoadObjectFromConfig((object)this, local, 1);
			}
		}
		catch (Exception ex)
		{
			Debug.Log((object)("MechJeb caught exception in OnLoad for " + GetType().Name + ": " + ex));
		}
	}

	public virtual void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		try
		{
			if (global != null)
			{
				ConfigNode.CreateConfigFromObject((object)this, 4, global);
			}
			if (type != null)
			{
				ConfigNode.CreateConfigFromObject((object)this, 2, type);
			}
			if (local != null)
			{
				ConfigNode.CreateConfigFromObject((object)this, 1, local);
			}
			Dirty = false;
		}
		catch (Exception ex)
		{
			Debug.Log((object)("MechJeb caught exception in OnSave for " + GetType().Name + ": " + ex));
		}
	}

	public virtual void OnDestroy()
	{
	}

	protected virtual bool IsSpaceCenterUpgradeUnlocked()
	{
		return true;
	}

	public virtual void UnlockCheck()
	{
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Invalid comparison between Unknown and I4
		if (UnlockChecked)
		{
			return;
		}
		if (HighLogic.CurrentGame != null && (int)HighLogic.CurrentGame.Mode == 0)
		{
			UnlockChecked = true;
		}
		else
		{
			if ((Object)(object)ResearchAndDevelopment.Instance == (Object)null)
			{
				return;
			}
			bool flag = true;
			string[] array = unlockParts.Split(new char[6] { ' ', ',', ';', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
			if (array.Length != 0)
			{
				flag = false;
				string[] array2 = array;
				foreach (string p in array2)
				{
					if (PartLoader.LoadedPartsList.Count((AvailablePart a) => a.name == p) > 0 && ResearchAndDevelopment.PartModelPurchased(PartLoader.LoadedPartsList.First((AvailablePart a) => a.name == p)))
					{
						flag = true;
						break;
					}
				}
			}
			string[] array3 = unlockTechs.Split(new char[6] { ' ', ',', ';', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
			if (array3.Length != 0)
			{
				if (array.Length == 0)
				{
					flag = false;
				}
				string[] array2 = array3;
				for (int i = 0; i < array2.Length; i++)
				{
					if ((int)ResearchAndDevelopment.GetTechnologyState(array2[i]) == 1)
					{
						flag = true;
						break;
					}
				}
			}
			flag = flag && IsSpaceCenterUpgradeUnlocked();
			UnlockChecked = true;
			if (!flag)
			{
				Enabled = false;
				Core.someModuleAreLocked = true;
			}
		}
	}

	protected static void Print(object message)
	{
		MonoBehaviour.print((object)("[MechJeb2] " + message));
	}

	public void Disable()
	{
		Enabled = false;
	}

	public void Enable()
	{
		Enabled = true;
	}

	public void CascadeDisable(ComputerModule m)
	{
		ModuleDisabledEvents.Add(m.Disable);
	}
}
