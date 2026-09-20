using UnityEngine;

namespace MuMech;

public class MechJebRouteRenderer : MonoBehaviour
{
	private static readonly Material _material = new Material(Shader.Find("Legacy Shaders/Particles/Additive"));

	public MechJebModuleRoverController AP;

	private LineRenderer _pastPath;

	private LineRenderer _currPath;

	private LineRenderer _nextPath;

	private LineRenderer _selWp;

	private readonly Color _pastPathColor = new Color(0f, 0f, 1f, 0.5f);

	private readonly Color _currPathColor = new Color(0f, 1f, 0f, 0.5f);

	private readonly Color _nextPathColor = new Color(1f, 1f, 0f, 0.5f);

	private readonly Color _selWpColor = new Color(1f, 0f, 0f, 0.5f);

	private double _addHeight;

	public bool enabled
	{
		get
		{
			return ((Behaviour)this).enabled;
		}
		set
		{
			((Behaviour)this).enabled = value;
			if ((Object)(object)_pastPath != (Object)null)
			{
				((Renderer)_pastPath).enabled = value;
			}
			if ((Object)(object)_currPath != (Object)null)
			{
				((Renderer)_currPath).enabled = value;
			}
			if ((Object)(object)_nextPath != (Object)null)
			{
				((Renderer)_nextPath).enabled = value;
			}
		}
	}

	public static MechJebRouteRenderer AttachToMapView(MechJebCore core)
	{
		MechJebRouteRenderer mechJebRouteRenderer = ((Component)MapView.MapCamera).gameObject.GetComponent<MechJebRouteRenderer>();
		if (!Object.op_Implicit((Object)(object)mechJebRouteRenderer))
		{
			mechJebRouteRenderer = ((Component)MapView.MapCamera).gameObject.AddComponent<MechJebRouteRenderer>();
		}
		mechJebRouteRenderer.AP = core.GetComputerModule<MechJebModuleRoverController>();
		return mechJebRouteRenderer;
	}

	private static bool NewLineRenderer(ref LineRenderer line)
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Expected O, but got Unknown
		if ((Object)(object)line != (Object)null)
		{
			return false;
		}
		GameObject val = new GameObject("LineRenderer");
		line = val.AddComponent<LineRenderer>();
		line.useWorldSpace = true;
		((Renderer)line).material = _material;
		line.startWidth = 10f;
		line.endWidth = 10f;
		line.positionCount = 2;
		return true;
	}

	private static Vector3 RaisePositionOverTerrain(Vector3 position, float heightOffset)
	{
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		CelestialBody mainBody = FlightGlobals.ActiveVessel.mainBody;
		if (!MapView.MapIsEnabled)
		{
			return Vector3d.op_Implicit(mainBody.GetWorldSurfacePosition(mainBody.GetLatitude(Vector3d.op_Implicit(position), false), mainBody.GetLongitude(Vector3d.op_Implicit(position), false), mainBody.GetAltitude(Vector3d.op_Implicit(position)) + (double)heightOffset));
		}
		double latitude = mainBody.GetLatitude(Vector3d.op_Implicit(position), false);
		double longitude = mainBody.GetLongitude(Vector3d.op_Implicit(position), false);
		return Vector3d.op_Implicit(ScaledSpace.LocalToScaledSpace(mainBody.position + (mainBody.Radius + (double)heightOffset + mainBody.TerrainAltitude(latitude, longitude, false)) * mainBody.GetSurfaceNVector(latitude, longitude)));
	}

	public void OnPreRender()
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_042b: Unknown result type (might be due to invalid IL or missing references)
		//IL_043b: Unknown result type (might be due to invalid IL or missing references)
		//IL_056e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0585: Unknown result type (might be due to invalid IL or missing references)
		//IL_058a: Unknown result type (might be due to invalid IL or missing references)
		//IL_05de: Unknown result type (might be due to invalid IL or missing references)
		//IL_05e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_05f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0613: Unknown result type (might be due to invalid IL or missing references)
		//IL_0618: Unknown result type (might be due to invalid IL or missing references)
		//IL_062e: Unknown result type (might be due to invalid IL or missing references)
		//IL_071e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0725: Unknown result type (might be due to invalid IL or missing references)
		//IL_0751: Unknown result type (might be due to invalid IL or missing references)
		//IL_0756: Unknown result type (might be due to invalid IL or missing references)
		//IL_075d: Unknown result type (might be due to invalid IL or missing references)
		//IL_068c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0691: Unknown result type (might be due to invalid IL or missing references)
		//IL_0698: Unknown result type (might be due to invalid IL or missing references)
		//IL_06d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_06da: Unknown result type (might be due to invalid IL or missing references)
		//IL_07fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_07e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_07e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0802: Unknown result type (might be due to invalid IL or missing references)
		//IL_083b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0840: Unknown result type (might be due to invalid IL or missing references)
		//IL_0847: Unknown result type (might be due to invalid IL or missing references)
		if (NewLineRenderer(ref _pastPath))
		{
			_pastPath.startColor = _pastPathColor;
			_pastPath.endColor = _pastPathColor;
		}
		if (NewLineRenderer(ref _currPath))
		{
			_currPath.startColor = _currPathColor;
			_currPath.endColor = _currPathColor;
		}
		if (NewLineRenderer(ref _nextPath))
		{
			_nextPath.startColor = _nextPathColor;
			_nextPath.endColor = _nextPathColor;
		}
		if (NewLineRenderer(ref _selWp))
		{
			_selWp.startColor = _selWpColor;
			_selWp.endColor = _selWpColor;
		}
		MechJebModuleWaypointWindow computerModule = AP.Core.GetComputerModule<MechJebModuleWaypointWindow>();
		_addHeight = AP.Vessel.mainBody.bodyName switch
		{
			"Moho" => computerModule.MohoMapdist, 
			"Eve" => computerModule.EveMapdist, 
			"Gilly" => computerModule.GillyMapdist, 
			"Kerbin" => computerModule.KerbinMapdist, 
			"Mun" => computerModule.MunMapdist, 
			"Minmus" => computerModule.MinmusMapdist, 
			"Duna" => computerModule.DunaMapdist, 
			"Ike" => computerModule.IkeMapdist, 
			"Dres" => computerModule.DresMapdist, 
			"Jool" => computerModule.JoolMapdist, 
			"Laythe" => computerModule.LaytheMapdist, 
			"Vall" => computerModule.VallMapdist, 
			"Tylo" => computerModule.TyloMapdist, 
			"Bop" => computerModule.BopMapdist, 
			"Pol" => computerModule.PolMapdist, 
			"Eeloo" => computerModule.EelooMapdist, 
			_ => computerModule.KerbinMapdist, 
		};
		if (AP != null && AP.Waypoints.Count > 0 && AP.Vessel.isActiveVessel && HighLogic.LoadedSceneIsFlight)
		{
			float num = (MapView.MapIsEnabled ? ((float)_addHeight) : 3f);
			float num2 = Vector3.Distance(((Component)FlightCamera.fetch.mainCamera).transform.position, AP.Vessel.CoM) / 700f;
			float num3 = (MapView.MapIsEnabled ? ((float)(0.005 * (double)PlanetariumCamera.fetch.Distance)) : (num2 + 0.1f));
			_pastPath.startWidth = num3;
			_pastPath.endWidth = num3;
			_currPath.startWidth = num3;
			_currPath.endWidth = num3;
			_nextPath.startWidth = num3;
			_nextPath.endWidth = num3;
			GameObject gameObject = ((Component)_selWp).gameObject;
			GameObject gameObject2 = ((Component)_pastPath).gameObject;
			GameObject gameObject3 = ((Component)_currPath).gameObject;
			int num5 = (((Component)_nextPath).gameObject.layer = (MapView.MapIsEnabled ? 9 : 0));
			int num7 = (gameObject3.layer = num5);
			int layer = (gameObject2.layer = num7);
			gameObject.layer = layer;
			int selIndex = AP.Core.GetComputerModule<MechJebModuleWaypointWindow>().SelIndex;
			((Renderer)_selWp).enabled = selIndex > -1 && !MapView.MapIsEnabled;
			if (((Renderer)_selWp).enabled)
			{
				float num9 = Vector3.Distance(((Component)FlightCamera.fetch.mainCamera).transform.position, Vector3d.op_Implicit(AP.Waypoints[selIndex].Position)) / 600f + 0.1f;
				_selWp.startWidth = 0f;
				_selWp.endWidth = num9 * 10f;
				_selWp.SetPosition(0, RaisePositionOverTerrain(Vector3d.op_Implicit(AP.Waypoints[selIndex].Position), num + 3f));
				_selWp.SetPosition(1, RaisePositionOverTerrain(Vector3d.op_Implicit(AP.Waypoints[selIndex].Position), num + 3f + num9 * 15f));
			}
			if (AP.WaypointIndex > 0)
			{
				((Renderer)_pastPath).enabled = true;
				_pastPath.positionCount = AP.WaypointIndex + 1;
				for (int i = 0; i < AP.WaypointIndex; i++)
				{
					_pastPath.SetPosition(i, RaisePositionOverTerrain(Vector3d.op_Implicit(AP.Waypoints[i].Position), num));
				}
				_pastPath.SetPosition(AP.WaypointIndex, RaisePositionOverTerrain(AP.Vessel.CoM, num));
			}
			else
			{
				((Renderer)_pastPath).enabled = false;
			}
			if (AP.WaypointIndex > -1)
			{
				((Renderer)_currPath).enabled = true;
				_currPath.SetPosition(0, RaisePositionOverTerrain(AP.Vessel.CoM, num));
				_currPath.SetPosition(1, RaisePositionOverTerrain(Vector3d.op_Implicit(AP.Waypoints[AP.WaypointIndex].Position), num));
			}
			else
			{
				((Renderer)_currPath).enabled = false;
			}
			int num10 = AP.Waypoints.Count - AP.WaypointIndex;
			if (num10 > 1)
			{
				((Renderer)_nextPath).enabled = true;
				_nextPath.positionCount = num10;
				_nextPath.SetPosition(0, RaisePositionOverTerrain((AP.WaypointIndex == -1) ? AP.Vessel.CoM : Vector3d.op_Implicit(AP.Waypoints[AP.WaypointIndex].Position), num));
				for (int j = 0; j < num10 - 1; j++)
				{
					_nextPath.SetPosition(j + 1, RaisePositionOverTerrain(Vector3d.op_Implicit(AP.Waypoints[AP.WaypointIndex + 1 + j].Position), num));
				}
			}
			else
			{
				((Renderer)_nextPath).enabled = false;
			}
		}
		else
		{
			LineRenderer selWp = _selWp;
			LineRenderer pastPath = _pastPath;
			LineRenderer currPath = _currPath;
			bool flag2 = (((Renderer)_nextPath).enabled = false);
			bool flag4 = (((Renderer)currPath).enabled = flag2);
			bool flag6 = (((Renderer)pastPath).enabled = flag4);
			((Renderer)selWp).enabled = flag6;
		}
	}
}
