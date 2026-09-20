using KSP.Localization;

namespace MuMech;

public static class CustomWindowPresets
{
	public struct Preset
	{
		public string name;

		public string sharingString;
	}

	public static readonly Preset[] presets = new Preset[13]
	{
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname1"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname1") + "\nShow in: flight\nValue:VesselState.SpeedOrbital\nValue:VesselState.OrbitApA\nValue:VesselState.OrbitPeA\nValue:VesselState.OrbitPeriod\nValue:VesselState.OrbitTimeToAp\nValue:VesselState.OrbitTimeToPe\nValue:VesselState.OrbitSemiMajorAxis\nValue:VesselState.OrbitInclination\nValue:VesselState.OrbitEccentricity\nValue:VesselState.OrbitLAN\nValue:VesselState.OrbitArgumentOfPeriapsis\nValue:VesselState.AngleToPrograde\nValue:InfoItems.RelativeInclinationToTarget\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname2"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname2") + "\nShow in: flight\nValue:VesselState.AltitudeASL\nValue:VesselState.AltitudeTrue\nValue:VesselState.Pitch\nValue:VesselState.Heading\nValue:VesselState.Roll\nValue:VesselState.SpeedSurface\nValue:VesselState.SpeedVertical\nValue:VesselState.SpeedSurfaceHorizontal\nValue:InfoItems.GetCoordinateString\nValue:InfoItems.CurrentBiome\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname3"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname3") + "\nShow in: flight editor\nValue:InfoItems.MaxAcceleration\nValue:InfoItems.CurrentAcceleration\nValue:InfoItems.MaxThrust\nValue:InfoItems.VesselMass\nValue:InfoItems.SurfaceTWR\nValue:InfoItems.CrewCapacity\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname4"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname4") + "\nShow in: flight editor\nToggle:StageStats.DVLinearThrust\nValue:InfoItems.StageDeltaVAtmosphereAndVac\nValue:InfoItems.TotalDeltaVAtmosphereAndVac\nGeneral:InfoItems.AllStageStats\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname5"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname5") + "\nShow in: flight\nValue:FlightRecorder.DeltaVExpended\nValue:FlightRecorder.GravityLosses\nValue:FlightRecorder.DragLosses\nValue:FlightRecorder.SteeringLosses\nValue:FlightRecorder.TimeSinceMark\nValue:FlightRecorder.PhaseAngleFromMark\nValue:FlightRecorder.GroundDistanceFromMark\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname6"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname6") + "\nShow in: flight\nValue:InfoItems.TargetTimeToClosestApproach\nValue:InfoItems.TargetClosestApproachDistance\nValue:InfoItems.TargetClosestApproachRelativeVelocity\nValue:InfoItems.TargetDistance\nValue:InfoItems.TargetRelativeVelocity\nValue:InfoItems.RelativeInclinationToTarget\nValue:InfoItems.PhaseAngle\nValue:InfoItems.SynodicPeriod\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname7"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname7") + "\nShow in: flight\nValue:VesselState.AltitudeTrue\nValue:VesselState.SpeedVertical\nValue:VesselState.SpeedSurfaceHorizontal\nValue:InfoItems.SurfaceTWR\nAction:TargetController.PickPositionTargetOnMap\nValue:InfoItems.TargetDistance\nValue:InfoItems.CurrentBiome\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname8"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname8") + "\nShow in: flight\nValue:InfoItems.TargetOrbitSpeed\nValue:InfoItems.TargetApoapsis\nValue:InfoItems.TargetPeriapsis\nValue:InfoItems.TargetOrbitPeriod\nValue:InfoItems.TargetOrbitTimeToAp\nValue:InfoItems.TargetOrbitTimeToPe\nValue:InfoItems.TargetSMA\nValue:InfoItems.TargetInclination\nValue:InfoItems.TargetEccentricity\nValue:InfoItems.TargetLAN\nValue:InfoItems.TargetAoP\nValue:InfoItems.RelativeInclinationToTarget\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname9"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname9") + "\nShow in: flight\nAction:FlightRecorder.Mark\nValue:FlightRecorder.TimeSinceMark\nValue:VesselState.Time\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname10"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname10") + "\nShow in: flight\nAction:TargetController.PickPositionTargetOnMap\nValue:InfoItems.TargetDistance\nValue:InfoItems.HeadingToTarget\nValue:TargetController.GetPositionTargetString\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname11"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname11") + "\nShow in: flight\nValue:VesselState.AoA\nValue:VesselState.AoS\nValue:VesselState.AoD\nValue:VesselState.Mach\nValue:VesselState.DynamicPressure\nValue:VesselState.MaxDynamicPressure\nValue:VesselState.DragForce\nValue:VesselState.LiftForce\nValue:VesselState.DragCoefficient\nValue:VesselState.AreaDrag\nValue:VesselState.IntakeAir\nValue:VesselState.IntakeAirAllIntakes\nValue:VesselState.IntakeAirNeeded\nValue:VesselState.AtmosphericDensityInGrams\nValue:InfoItems.AtmosphericPressure\nValue:VesselState.TerminalVelocity\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname12"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname12") + "\nShow in: flight\nValue:InfoItems.TimeToManeuverNode\nValue:InfoItems.NextManeuverNodeDeltaV\nValue:InfoItems.NextManeuverNodeBurnTime\n-----------------------------"
		},
		new Preset
		{
			name = Localizer.Format("#MechJeb_WindowEd_Presetname13"),
			sharingString = "--- MechJeb Custom Window ---\nName: " + Localizer.Format("#MechJeb_WindowEd_Presetname13") + "\nShow in: flight\nValue:HoverslamSimulation.Impact\nValue:HoverslamSimulation.Ignition\nValue:HoverslamSimulation.Touchdown\nValue:HoverslamSimulation.HoverslamDeltaV\nValue:HoverslamSimulation.HoverslamCoordinates\nValue:HoverslamSimulation.Biome\nValue:HoverslamSimulation.Slope\nValue:HoverslamSimulation.TerrainAltitude\nToggle:HoverslamSimulation.MapLandingPrediction\nEditable:HoverslamSimulation.VerticalAltitude\nEditable:HoverslamSimulation.VerticalAuthority\nEditable:HoverslamAutopilot.TouchdownSpeed\nEditable:HoverslamAutopilot.PWMPulseWidth\nEditable:HoverslamAutopilot.IgnitionLead\nEditable:HoverslamSimulation.SimRecalcInterval\nToggle:HoverslamAutopilot.AutoWarp\nToggle:HoverslamAutopilot.HoldUpright\nAction:HoverslamAutopilot.ToggleEnabled\nValue:HoverslamAutopilot.HoverslamState\n-----------------------------"
		}
	};
}
