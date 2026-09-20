using System;
using UnityEngine;

namespace MuMech;

public class AutopilotModule : ComputerModule
{
	public string Status
	{
		get
		{
			if (CurrentStep != null)
			{
				return CurrentStep.Status;
			}
			return "Off";
		}
	}

	protected bool Active => CurrentStep != null;

	public AutopilotStep CurrentStep { get; private set; }

	protected AutopilotModule(MechJebCore core)
		: base(core)
	{
	}

	public override void Drive(FlightCtrlState s)
	{
		if (CurrentStep == null)
		{
			return;
		}
		try
		{
			CurrentStep = CurrentStep.Drive(s);
		}
		catch (Exception ex)
		{
			Debug.LogException(ex);
		}
	}

	public override void OnFixedUpdate()
	{
		if (CurrentStep == null)
		{
			return;
		}
		try
		{
			CurrentStep = CurrentStep.OnFixedUpdate();
		}
		catch (Exception ex)
		{
			Debug.LogException(ex);
		}
	}

	protected void SetStep(AutopilotStep step)
	{
		CurrentStep = step;
	}
}
