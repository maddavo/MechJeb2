using System.Collections.Generic;
using KSP.Localization;

namespace MuMech;

public class OperationInclination : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_inclination_title");

	[Persistent(pass = 4)]
	public EditableDouble NewInc = 0.0;

	private static readonly TimeReference[] _timeReferences = new TimeReference[5]
	{
		TimeReference.EQ_HIGHEST_AD,
		TimeReference.EQ_NEAREST_AD,
		TimeReference.EQ_ASCENDING,
		TimeReference.EQ_DESCENDING,
		TimeReference.X_FROM_NOW
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_inclination_label"), NewInc, "º");
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		double ut = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(OrbitalManeuverCalculator.DeltaVToChangeInclination(o, ut, NewInc), ut)
		};
	}
}
