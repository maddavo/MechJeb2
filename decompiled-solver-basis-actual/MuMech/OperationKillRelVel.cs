using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationKillRelVel : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_match_v_title");

	private static readonly TimeReference[] _timeReferences = new TimeReference[2]
	{
		TimeReference.CLOSEST_APPROACH,
		TimeReference.X_FROM_NOW
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		if (!target.NormalTargetExists)
		{
			throw new OperationException(Localizer.Format("#MechJeb_match_v_Exception1"));
		}
		if ((Object)(object)o.referenceBody != (Object)(object)target.TargetOrbit.referenceBody)
		{
			throw new OperationException(Localizer.Format("#MechJeb_match_v_Exception2"));
		}
		double num = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToMatchVelocities(o, num, target.TargetOrbit);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, num)
		};
	}
}
