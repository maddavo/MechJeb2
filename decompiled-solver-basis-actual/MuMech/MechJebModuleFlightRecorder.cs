using System;
using System.IO;
using UnityEngine;

namespace MuMech;

public class MechJebModuleFlightRecorder : ComputerModule
{
	public struct RecordStruct
	{
		public double TimeSinceMark;

		public int CurrentStage;

		public double AltitudeASL;

		public double DownRange;

		public double SpeedSurface;

		public double SpeedOrbital;

		public double Acceleration;

		public double Q;

		public double AoA;

		public double AoS;

		public double AoD;

		public double AltitudeTrue;

		public double Pitch;

		public double Mass;

		public double GravityLosses;

		public double DragLosses;

		public double SteeringLosses;

		public double DeltaVExpended;

		public double this[RecordType type] => type switch
		{
			RecordType.TIME_SINCE_MARK => TimeSinceMark, 
			RecordType.CURRENT_STAGE => CurrentStage, 
			RecordType.ALTITUDE_ASL => AltitudeASL, 
			RecordType.DOWN_RANGE => DownRange, 
			RecordType.SPEED_SURFACE => SpeedSurface, 
			RecordType.SPEED_ORBITAL => SpeedOrbital, 
			RecordType.MASS => Mass, 
			RecordType.ACCELERATION => Acceleration, 
			RecordType.Q => Q, 
			RecordType.AO_A => AoA, 
			RecordType.AO_S => AoS, 
			RecordType.AO_D => AoD, 
			RecordType.ALTITUDE_TRUE => AltitudeTrue, 
			RecordType.PITCH => Pitch, 
			RecordType.GRAVITY_LOSSES => GravityLosses, 
			RecordType.DRAG_LOSSES => DragLosses, 
			RecordType.STEERING_LOSSES => SteeringLosses, 
			RecordType.DELTA_V_EXPENDED => DeltaVExpended, 
			_ => 0.0, 
		};
	}

	public enum RecordType
	{
		TIME_SINCE_MARK,
		CURRENT_STAGE,
		ALTITUDE_ASL,
		DOWN_RANGE,
		SPEED_SURFACE,
		SPEED_ORBITAL,
		MASS,
		ACCELERATION,
		Q,
		AO_A,
		AO_S,
		AO_D,
		ALTITUDE_TRUE,
		PITCH,
		GRAVITY_LOSSES,
		DRAG_LOSSES,
		STEERING_LOSSES,
		DELTA_V_EXPENDED
	}

	public RecordStruct[] History = new RecordStruct[1];

	public int HistoryIdx = -1;

	[Persistent(pass = 4)]
	public readonly int HistorySize = 3000;

	[Persistent(pass = 4)]
	public readonly double Precision = 0.2;

	[Persistent(pass = 4)]
	public bool Downrange = true;

	[Persistent(pass = 4)]
	public bool RealAtmo;

	[Persistent(pass = 4)]
	public bool Stages;

	[Persistent(pass = 4)]
	public int HSize = 4;

	[Persistent(pass = 4)]
	public int VSize = 2;

	private static readonly int _typeCount = Enum.GetValues(typeof(RecordType)).Length;

	public readonly double[] Maximums;

	public readonly double[] Minimums;

	private readonly bool _paused;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_MarkUT", InfoItem.Category.Recorder, format = "TIME")]
	public double MarkUT;

	[ValueInfoItem("#MechJeb_TimeSinceMark", InfoItem.Category.Recorder, format = "TIME")]
	public double TimeSinceMark;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_DVExpended", InfoItem.Category.Recorder, format = "F1", units = "m/s")]
	public double DeltaVExpended;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_DragLosses", InfoItem.Category.Recorder, format = "F1", units = "m/s")]
	public double DragLosses;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_GravityLosses", InfoItem.Category.Recorder, format = "F1", units = "m/s")]
	public double GravityLosses;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_SteeringLosses", InfoItem.Category.Recorder, format = "F1", units = "m/s")]
	public double SteeringLosses;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_MarkLAN", InfoItem.Category.Recorder, format = "ANGLE_EW")]
	public double MarkLAN;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_MarkLatitude", InfoItem.Category.Recorder, format = "ANGLE_NS")]
	public double MarkLatitude;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_MarkLongitude", InfoItem.Category.Recorder, format = "ANGLE_EW")]
	public double MarkLongitude;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_MarkAltitudeASL", InfoItem.Category.Recorder, format = "SI", units = "m")]
	public double MarkAltitude;

	[Persistent(pass = 1)]
	public int MarkBodyIndex = 1;

	[Persistent(pass = 1)]
	[ValueInfoItem("#MechJeb_MaxDragGees", InfoItem.Category.Recorder, format = "F2")]
	public double MaxDragGees;

	private double _lastRecordTime;

	[ValueInfoItem("#MechJeb_PhaseAngleFromMark", InfoItem.Category.Recorder, format = "F2", units = "º")]
	public double PhaseAngleFromMark()
	{
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		CelestialBody val = FlightGlobals.Bodies[MarkBodyIndex];
		Vector3d surfaceNVector = val.GetSurfaceNVector(MarkLatitude, MarkLongitude - TimeSinceMark * 360.0 / val.rotationPeriod);
		Vector3d val2 = base.VesselState.CoM - ((Component)val).transform.position;
		double num = Vector3d.Angle(surfaceNVector, val2);
		return MuUtils.ClampDegrees360(360.0 * TimeSinceMark / base.Orbit.CircularOrbitPeriod() - num);
	}

	[ValueInfoItem("#MechJeb_MarkBody", InfoItem.Category.Recorder)]
	public string MarkBody()
	{
		return FlightGlobals.Bodies[MarkBodyIndex].bodyName;
	}

	[ValueInfoItem("#MechJeb_DistanceFromMark", InfoItem.Category.Recorder, format = "SI", units = "m")]
	public double DistanceFromMark()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		return Vector3d.Distance(base.VesselState.CoM, FlightGlobals.Bodies[MarkBodyIndex].GetWorldSurfacePosition(MarkLatitude, MarkLongitude, MarkAltitude) - FlightGlobals.Bodies[MarkBodyIndex].position);
	}

	[ValueInfoItem("#MechJeb_DownrangeDistance", InfoItem.Category.Recorder, format = "SI", units = "m")]
	public double GroundDistanceFromMark()
	{
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		CelestialBody val = FlightGlobals.Bodies[MarkBodyIndex];
		Vector3d surfaceNVector = val.GetSurfaceNVector(MarkLatitude, MarkLongitude);
		Vector3d val2 = base.VesselState.CoM - ((Component)val).transform.position;
		return val.Radius * Vector3d.Angle(surfaceNVector, val2) * (Math.PI / 180.0);
	}

	[ActionInfoItem("MARK", InfoItem.Category.Recorder)]
	public void Mark()
	{
		MarkUT = base.VesselState.Time;
		DeltaVExpended = (DragLosses = (GravityLosses = (SteeringLosses = 0.0)));
		MarkLatitude = base.VesselState.Latitude;
		MarkLongitude = base.VesselState.Longitude;
		MarkLAN = base.VesselState.OrbitLAN;
		MarkAltitude = base.VesselState.AltitudeASL;
		MarkBodyIndex = FlightGlobals.Bodies.IndexOf(base.MainBody);
		MaxDragGees = 0.0;
		TimeSinceMark = 0.0;
		for (int i = 0; i < Maximums.Length; i++)
		{
			Minimums[i] = double.MaxValue;
			Maximums[i] = double.MinValue;
		}
		HistoryIdx = 0;
		Record(HistoryIdx);
	}

	public MechJebModuleFlightRecorder(MechJebCore core)
		: base(core)
	{
		Priority = 2000;
		Maximums = new double[_typeCount];
		Minimums = new double[_typeCount];
	}

	public override void OnStart(StartState state)
	{
		if (History.Length != HistorySize)
		{
			History = new RecordStruct[HistorySize];
		}
		Users.Add(this);
	}

	public override void OnFixedUpdate()
	{
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Invalid comparison between Unknown and I4
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		if (MarkUT == 0.0)
		{
			Mark();
		}
		TimeSinceMark = base.VesselState.Time - MarkUT;
		if ((int)base.Vessel.situation == 4)
		{
			Mark();
			return;
		}
		GravityLosses += base.VesselState.DeltaT * Vector3d.Dot(-((Vector3d)(ref base.VesselState.OrbitalVelocity)).normalized, base.VesselState.GravityForce);
		DragLosses += base.VesselState.DeltaT * base.VesselState.DragAcceleration;
		DeltaVExpended += base.VesselState.DeltaT * base.VesselState.CurrentThrustAcceleration;
		SteeringLosses += base.VesselState.DeltaT * base.VesselState.CurrentThrustAcceleration * (1.0 - Vector3d.Dot(((Vector3d)(ref base.VesselState.OrbitalVelocity)).normalized, base.VesselState.Forward));
		MaxDragGees = Math.Max(MaxDragGees, base.VesselState.DragAcceleration / 9.81);
		if (!_paused && base.VesselState.Time >= _lastRecordTime + Precision && HistoryIdx < History.Length - 1)
		{
			_lastRecordTime = base.VesselState.Time;
			HistoryIdx++;
			Record(HistoryIdx);
		}
	}

	private void Record(int idx)
	{
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		History[idx].TimeSinceMark = TimeSinceMark;
		History[idx].AltitudeASL = base.VesselState.AltitudeASL;
		History[idx].DownRange = GroundDistanceFromMark();
		History[idx].SpeedSurface = base.VesselState.SpeedSurface;
		History[idx].SpeedOrbital = base.VesselState.SpeedOrbital;
		History[idx].Acceleration = base.Vessel.geeForce;
		History[idx].Q = base.VesselState.DynamicPressure;
		History[idx].AltitudeTrue = base.VesselState.AltitudeTrue;
		History[idx].Pitch = base.VesselState.Pitch;
		History[idx].Mass = base.VesselState.Mass;
		History[idx].GravityLosses = GravityLosses;
		History[idx].DragLosses = DragLosses;
		History[idx].SteeringLosses = SteeringLosses;
		History[idx].DeltaVExpended = DeltaVExpended;
		if ((int)TimeWarp.WarpMode != 0)
		{
			History[idx].CurrentStage = base.Vessel.currentStage;
		}
		else
		{
			History[idx].CurrentStage = ((idx > 0) ? History[idx - 1].CurrentStage : base.Vessel.currentStage);
		}
		History[idx].AoA = base.VesselState.AoA;
		History[idx].AoS = base.VesselState.AoS;
		History[idx].AoD = base.VesselState.AoD;
		for (int i = 0; i < _typeCount; i++)
		{
			double val = History[idx][(RecordType)i];
			Minimums[i] = Math.Min(Minimums[i], val);
			Maximums[i] = Math.Max(Maximums[i], val);
		}
	}

	public void DumpCsv()
	{
		string text = KSPUtil.ApplicationRootPath + "/GameData/MechJeb2/Export/";
		if (!Directory.Exists(text))
		{
			Directory.CreateDirectory(text);
		}
		string text2 = (((Object)(object)base.Vessel != (Object)null) ? string.Join("_", base.Vessel.vesselName.Split(Path.GetInvalidFileNameChars())) : "");
		string text3 = DateTime.Now.ToString("yyyyMMddHHmmss");
		string text4 = text + text2 + "_" + text3 + ".csv";
		using (StreamWriter streamWriter = new StreamWriter(text4))
		{
			streamWriter.WriteLine(string.Join(",", Enum.GetNames(typeof(RecordType))));
			for (int i = 0; i <= HistoryIdx; i++)
			{
				RecordStruct recordStruct = History[i];
				streamWriter.Write(recordStruct[RecordType.TIME_SINCE_MARK]);
				for (int j = 1; j < _typeCount; j++)
				{
					streamWriter.Write(',');
					streamWriter.Write(recordStruct[(RecordType)j]);
				}
				streamWriter.WriteLine();
			}
		}
		ScreenMessages.PostScreenMessage("Exported as\n" + text4, 5f);
	}
}
