using UnityEngine;

namespace MuMech;

public class MechJebWaypoint
{
	private const float DEFAULT_RADIUS = 5f;

	public double Latitude;

	public double Longitude;

	public double Altitude;

	public Vector3d Position;

	public float Radius;

	public string Name;

	public readonly Vessel Target;

	public float MinSpeed;

	public float MaxSpeed;

	public bool Quicksave;

	public CelestialBody Body
	{
		get
		{
			if (!((Object)(object)Target != (Object)null))
			{
				return FlightGlobals.ActiveVessel.mainBody;
			}
			return Target.mainBody;
		}
	}

	public MechJebWaypoint(double latitude, double longitude, float radius = 5f, string name = "", float minSpeed = 0f, float maxSpeed = 0f)
	{
		Latitude = latitude;
		Longitude = longitude;
		Radius = radius;
		Name = name ?? "";
		MinSpeed = minSpeed;
		MaxSpeed = maxSpeed;
		Update();
	}

	public MechJebWaypoint(Vector3d position, float radius = 5f, string name = "", float minSpeed = 0f, float maxSpeed = 0f)
	{
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		Latitude = Body.GetLatitude(position, false);
		Longitude = Body.GetLongitude(position, false);
		Radius = radius;
		Name = name ?? "";
		MinSpeed = minSpeed;
		MaxSpeed = maxSpeed;
		Update();
	}

	public MechJebWaypoint(Vessel target, float radius = 5f, string name = "", float minSpeed = 0f, float maxSpeed = 0f)
	{
		Target = target;
		Radius = radius;
		Name = name ?? "";
		MinSpeed = minSpeed;
		MaxSpeed = maxSpeed;
		Update();
	}

	public MechJebWaypoint(ConfigNode node)
	{
		if (node.HasValue("Latitude"))
		{
			double.TryParse(node.GetValue("Latitude"), out Latitude);
		}
		if (node.HasValue("Longitude"))
		{
			double.TryParse(node.GetValue("Longitude"), out Longitude);
		}
		Target = (node.HasValue("Target") ? FlightGlobals.Vessels.Find((Vessel v) => v.id.ToString() == node.GetValue("Target")) : null);
		if (node.HasValue("Radius"))
		{
			float.TryParse(node.GetValue("Radius"), out Radius);
		}
		else
		{
			Radius = 5f;
		}
		Name = (node.HasValue("Name") ? node.GetValue("Name") : "");
		if (node.HasValue("MinSpeed"))
		{
			float.TryParse(node.GetValue("MinSpeed"), out MinSpeed);
		}
		if (node.HasValue("MaxSpeed"))
		{
			float.TryParse(node.GetValue("MaxSpeed"), out MaxSpeed);
		}
		if (node.HasValue("Quicksave"))
		{
			bool.TryParse(node.GetValue("Quicksave"), out Quicksave);
		}
		Update();
	}

	public ConfigNode ToConfigNode()
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Expected O, but got Unknown
		ConfigNode val = new ConfigNode("Waypoint");
		if (Target != null)
		{
			val.AddValue("Target", (object)Target.id);
		}
		if (Name != "")
		{
			val.AddValue("Name", Name);
		}
		val.AddValue("Latitude", Latitude);
		val.AddValue("Longitude", Longitude);
		val.AddValue("Radius", Radius);
		val.AddValue("MinSpeed", MinSpeed);
		val.AddValue("MaxSpeed", MaxSpeed);
		val.AddValue("Quicksave", Quicksave);
		return val;
	}

	public string GetNameWithCoords()
	{
		return ((Name != "") ? Name : ((Target == null) ? "Waypoint" : Target.vesselName)) + " - " + Coordinates.ToStringDMS(Latitude, Longitude);
	}

	public void Update()
	{
		//IL_014b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_0155: Unknown result type (might be due to invalid IL or missing references)
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_017a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0081: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0109: Unknown result type (might be due to invalid IL or missing references)
		//IL_010e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0133: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013d: Unknown result type (might be due to invalid IL or missing references)
		if (Target == null)
		{
			Position = Body.GetWorldSurfacePosition(Latitude, Longitude, Body.TerrainAltitude(Latitude, Longitude, false));
			if (Vector3d.Distance(Position, Vector3d.op_Implicit(FlightGlobals.ActiveVessel.CoM)) < 200.0)
			{
				Vector3d val = Position - Body.position;
				Vector3d val2 = ((Vector3d)(ref val)).normalized;
				Vector3d val3 = Body.position + val2 * (Body.Radius + 50000.0);
				val = Body.position - val3;
				val2 = ((Vector3d)(ref val)).normalized;
				RaycastHit val4 = default(RaycastHit);
				if (Physics.Raycast(Vector3d.op_Implicit(val3), Vector3d.op_Implicit(val2), ref val4, (float)Body.Radius, 32768, (QueryTriggerInteraction)1))
				{
					val2 = ((RaycastHit)(ref val4)).point - Body.position;
					Position = Body.position + ((Vector3d)(ref val2)).normalized * (((Vector3d)(ref val2)).magnitude + 0.5);
				}
			}
		}
		else
		{
			Position = Vector3d.op_Implicit(Target.CoM);
			Latitude = Body.GetLatitude(Position, false);
			Longitude = Body.GetLongitude(Position, false);
		}
		if (MinSpeed > 0f && MaxSpeed > 0f && MinSpeed > MaxSpeed)
		{
			MinSpeed = MaxSpeed;
		}
		else if (MinSpeed > 0f && MaxSpeed > 0f && MaxSpeed < MinSpeed)
		{
			MaxSpeed = MinSpeed;
		}
	}
}
