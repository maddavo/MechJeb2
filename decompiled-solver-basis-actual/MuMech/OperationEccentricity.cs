using System.Collections.Generic;

namespace MuMech;

public class OperationEccentricity : Operation
{
	private static readonly string _name = "change eccentricity";

	[Persistent(pass = 4)]
	public readonly EditableDoubleMult NewEcc = new EditableDouble(0.0);

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
		GuiUtils.SimpleTextBox("New eccentricity:", NewEcc);
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		double ut = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToChangeEccentricity(o, ut, NewEcc);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut)
		};
	}
}
