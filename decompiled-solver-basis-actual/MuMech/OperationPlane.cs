using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationPlane : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_match_planes_title");

	private static readonly TimeReference[] _timeReferences = new TimeReference[4]
	{
		TimeReference.REL_HIGHEST_AD,
		TimeReference.REL_NEAREST_AD,
		TimeReference.REL_ASCENDING,
		TimeReference.REL_DESCENDING
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
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0101: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0180: Unknown result type (might be due to invalid IL or missing references)
		//IL_0182: Unknown result type (might be due to invalid IL or missing references)
		//IL_0141: Unknown result type (might be due to invalid IL or missing references)
		//IL_0143: Unknown result type (might be due to invalid IL or missing references)
		//IL_0189: Unknown result type (might be due to invalid IL or missing references)
		//IL_018b: Unknown result type (might be due to invalid IL or missing references)
		double uT = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		if (!target.NormalTargetExists)
		{
			throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception1"));
		}
		if ((Object)(object)o.referenceBody != (Object)(object)target.TargetOrbit.referenceBody)
		{
			throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception2"));
		}
		bool flag = o.AscendingNodeExists(target.TargetOrbit);
		bool flag2 = o.DescendingNodeExists(target.TargetOrbit);
		double burnUT = 0.0;
		double burnUT2 = 0.0;
		Vector3d val = (flag ? OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesAscending(o, target.TargetOrbit, uT, out burnUT) : Vector3d.zero);
		Vector3d val2 = (flag ? OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesDescending(o, target.TargetOrbit, uT, out burnUT2) : Vector3d.zero);
		Vector3d dV;
		if (_timeSelector.TimeReference == TimeReference.REL_ASCENDING)
		{
			if (!flag)
			{
				throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception3"));
			}
			uT = burnUT;
			dV = val;
		}
		else if (_timeSelector.TimeReference == TimeReference.REL_DESCENDING)
		{
			if (!flag2)
			{
				throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception4"));
			}
			uT = burnUT2;
			dV = val2;
		}
		else if (_timeSelector.TimeReference == TimeReference.REL_NEAREST_AD)
		{
			if (!flag && !flag2)
			{
				throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception5"));
			}
			if (!flag2 || burnUT <= burnUT2)
			{
				uT = burnUT;
				dV = val;
			}
			else
			{
				uT = burnUT2;
				dV = val2;
			}
		}
		else
		{
			if (_timeSelector.TimeReference != TimeReference.REL_HIGHEST_AD)
			{
				throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception6"));
			}
			if (!flag && !flag2)
			{
				throw new OperationException(Localizer.Format("#MechJeb_match_planes_Exception5"));
			}
			if (!flag2 || ((Vector3d)(ref val)).magnitude <= ((Vector3d)(ref val2)).magnitude)
			{
				uT = burnUT;
				dV = val;
			}
			else
			{
				uT = burnUT2;
				dV = val2;
			}
		}
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, uT)
		};
	}
}
