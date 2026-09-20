using System.Collections.Generic;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebWaypointRoute : List<MechJebWaypoint>
{
	public readonly string Name;

	private string _stats;

	public CelestialBody Body { get; }

	public string Mode { get; }

	public string Stats
	{
		get
		{
			UpdateStats();
			return _stats;
		}
	}

	public ConfigNode ToConfigNode()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Expected O, but got Unknown
		ConfigNode cn = new ConfigNode("Waypoints");
		cn.AddValue("Name", Name);
		cn.AddValue("Body", Body.bodyName);
		cn.AddValue("Mode", Mode);
		ForEach(delegate(MechJebWaypoint wp)
		{
			cn.AddNode(wp.ToConfigNode());
		});
		return cn;
	}

	private void UpdateStats()
	{
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		float num = 0f;
		if (base.Count > 1)
		{
			for (int i = 1; i < base.Count; i++)
			{
				num += Vector3.Distance(Vector3d.op_Implicit(base[i - 1].Position), Vector3d.op_Implicit(base[i].Position));
			}
		}
		_stats = $"{base.Count} waypoints over {Statics.ToSI(num, 4, int.MaxValue)}m";
	}

	public MechJebWaypointRoute(string name = "", CelestialBody body = null, string mode = "Rover")
	{
		Name = name;
		Body = (((Object)(object)body != (Object)null) ? body : FlightGlobals.currentMainBody);
		Mode = mode;
	}

	public MechJebWaypointRoute(ConfigNode node)
	{
		if (node == null)
		{
			return;
		}
		Name = (node.HasValue("Name") ? node.GetValue("Name") : "");
		Body = (node.HasValue("Body") ? FlightGlobals.Bodies.Find((CelestialBody b) => b.bodyName == node.GetValue("Body")) : null);
		Mode = (node.HasValue("Mode") ? node.GetValue("Mode") : "Rover");
		if (node.HasNode("Waypoint"))
		{
			ConfigNode[] nodes = node.GetNodes("Waypoint");
			foreach (ConfigNode node2 in nodes)
			{
				Add(new MechJebWaypoint(node2));
			}
		}
	}
}
