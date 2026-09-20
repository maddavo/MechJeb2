using System.Collections.Generic;
using KSP.Localization;

namespace MuMech;

public class OperationCircularize : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_Maneu_circularize_title");

	private static readonly TimeReference[] _timeReferences = new TimeReference[4]
	{
		TimeReference.APOAPSIS,
		TimeReference.PERIAPSIS,
		TimeReference.X_FROM_NOW,
		TimeReference.ALTITUDE
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
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		double ut = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToCircularize(o, ut);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut)
		};
	}
}
