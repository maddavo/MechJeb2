using System.Collections.Generic;
using KSP.Localization;

namespace MuMech;

public class OperationApoapsis : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_Ap_title");

	[Persistent(pass = 4)]
	public readonly EditableDoubleMult NewApA = new EditableDoubleMult(200000.0, 1000.0);

	private static readonly TimeReference[] _timeReferences = new TimeReference[6]
	{
		TimeReference.PERIAPSIS,
		TimeReference.APOAPSIS,
		TimeReference.X_FROM_NOW,
		TimeReference.ALTITUDE,
		TimeReference.EQ_DESCENDING,
		TimeReference.EQ_ASCENDING
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Ap_label1"), NewApA, "km");
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		double ut = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToChangeApoapsis(o, ut, (double)NewApA + o.referenceBody.Radius);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut)
		};
	}
}
