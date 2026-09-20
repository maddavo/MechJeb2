using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationLambert : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_intercept_title");

	[Persistent(pass = 4)]
	public EditableTime InterceptInterval = 3600.0;

	private static readonly TimeReference[] _timeReferences = new TimeReference[1] { TimeReference.X_FROM_NOW };

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_intercept_label"), InterceptInterval);
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		if (!target.NormalTargetExists)
		{
			throw new OperationException(Localizer.Format("#MechJeb_intercept_Exception1"));
		}
		if ((Object)(object)o.referenceBody != (Object)(object)target.TargetOrbit.referenceBody)
		{
			throw new OperationException(Localizer.Format("#MechJeb_intercept_Exception2"));
		}
		double num = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d item = OrbitalManeuverCalculator.DeltaVToInterceptAtTime(o, num, target.TargetOrbit, num + (double)InterceptInterval).v1;
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(item, num)
		};
	}
}
