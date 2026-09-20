namespace MuMech;

public class AutopilotStep
{
	protected readonly MechJebCore Core;

	protected VesselState VesselState => Core.VesselState;

	protected Vessel Vessel => ((PartModule)Core).part.vessel;

	protected CelestialBody MainBody => ((PartModule)Core).part.vessel.mainBody;

	protected Orbit Orbit => ((PartModule)Core).part.vessel.orbit;

	public virtual string TraceDetails => string.Empty;

	public string Status { get; protected set; }

	protected AutopilotStep(MechJebCore core)
	{
		Core = core;
	}

	public virtual AutopilotStep Drive(FlightCtrlState s)
	{
		return this;
	}

	public virtual AutopilotStep OnFixedUpdate()
	{
		return this;
	}
}
