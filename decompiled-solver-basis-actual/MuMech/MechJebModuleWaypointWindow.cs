using System;
using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleWaypointWindow : DisplayModule
{
	public enum WaypointMode
	{
		ROVER,
		PLANE
	}

	private enum Pages
	{
		WAYPOINTS,
		SETTINGS,
		ROUTES
	}

	public WaypointMode Mode;

	private MechJebModuleRoverController _ap;

	private static readonly List<MechJebWaypointRoute> _routes = new List<MechJebWaypointRoute>();

	[EditableInfoItem("#MechJeb_MohoMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble MohoMapdist = 5000.0;

	[EditableInfoItem("#MechJeb_EveMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble EveMapdist = 5000.0;

	[EditableInfoItem("#MechJeb_GillyMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble GillyMapdist = -500.0;

	[EditableInfoItem("#MechJeb_KerbinMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble KerbinMapdist = 500.0;

	[EditableInfoItem("#MechJeb_MunMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble MunMapdist = 4000.0;

	[EditableInfoItem("#MechJeb_MinmusMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble MinmusMapdist = 3500.0;

	[EditableInfoItem("#MechJeb_DunaMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble DunaMapdist = 5000.0;

	[EditableInfoItem("#MechJeb_IkeMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble IkeMapdist = 4000.0;

	[EditableInfoItem("#MechJeb_DresMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble DresMapdist = 1500.0;

	[EditableInfoItem("#MechJeb_EelooMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble EelooMapdist = 2000.0;

	[EditableInfoItem("#MechJeb_JoolMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble JoolMapdist = 30000.0;

	[EditableInfoItem("#MechJeb_TyloMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble TyloMapdist = 5000.0;

	[EditableInfoItem("#MechJeb_LaytheMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble LaytheMapdist = 1000.0;

	[EditableInfoItem("#MechJeb_PolMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble PolMapdist = 500.0;

	[EditableInfoItem("#MechJeb_BopMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble BopMapdist = 1000.0;

	[EditableInfoItem("#MechJeb_VallMapdist", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble VallMapdist = 5000.0;

	internal int SelIndex = -1;

	private int _saveIndex = -1;

	private string _tmpRadius = "";

	private string _tmpMinSpeed = "";

	private string _tmpMaxSpeed = "";

	private string _tmpLat = "";

	private string _tmpLon = "";

	private const string COORD_REG_EX = "^([nsew])?\\s*(-?\\d+(?:\\.\\d+)?)(?:[°:\\s]+(-?\\d+(?:\\.\\d+)?))?(?:[':\\s]+(-?\\d+(?:\\.\\d+)?))?(?:[^nsew]*([nsew])?)?$";

	private Vector2 _scroll;

	private GUIStyle _styleActive;

	private GUIStyle _styleInactive;

	private GUIStyle _styleQuicksave;

	private string _titleAdd = "";

	private string _saveName = "";

	private bool _waitingForPick;

	private Pages _showPage;

	private static MechJebRouteRenderer _renderer;

	private Rect[] _waypointRects = Array.Empty<Rect>();

	private int _lastIndex = -1;

	private int _settingPageIndex;

	private readonly string[] _settingPages = new string[2] { "Rover", "Waypoints" };

	public int SelectedWaypointIndex
	{
		get
		{
			return SelIndex;
		}
		set
		{
			SelIndex = value;
		}
	}

	public MechJebModuleWaypointWindow(MechJebCore core)
		: base(core)
	{
	}

	public override void OnStart(StartState state)
	{
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		Hidden = true;
		_ap = Core.GetComputerModule<MechJebModuleRoverController>();
		if (HighLogic.LoadedSceneIsFlight && base.Vessel.isActiveVessel)
		{
			_renderer = MechJebRouteRenderer.AttachToMapView(Core);
			_renderer.enabled = base.Enabled;
		}
		base.OnStart(state);
	}

	protected override void OnModuleEnabled()
	{
		if ((Object)(object)_renderer != (Object)null)
		{
			_renderer.enabled = true;
		}
		base.OnModuleEnabled();
	}

	protected override void OnModuleDisabled()
	{
		if ((Object)(object)_renderer != (Object)null)
		{
			_renderer.enabled = false;
		}
		base.OnModuleDisabled();
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Expected O, but got Unknown
		base.OnLoad(local, type, global);
		if (!FlightGlobals.ready)
		{
			return;
		}
		ConfigNode val = new ConfigNode("Routes");
		if (MuUtils.FileExistsCreateDirectory(MuUtils.GetCfgPath("mechjeb_routes.cfg")))
		{
			try
			{
				val = ConfigNode.Load(MuUtils.GetCfgPath("mechjeb_routes.cfg"));
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJebModuleWaypointWindow.OnLoad caught an exception trying to load mechjeb_routes.cfg: " + ex));
			}
		}
		if (val.HasNode("Waypoints"))
		{
			_routes.Clear();
			ConfigNode[] nodes = val.GetNodes("Waypoints");
			foreach (ConfigNode node in nodes)
			{
				_routes.Add(new MechJebWaypointRoute(node));
			}
			_routes.Sort(SortRoutes);
		}
	}

	private void SaveRoutes()
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Expected O, but got Unknown
		ConfigNode val = new ConfigNode("Routes");
		if (_routes.Count > 0)
		{
			_routes.Sort(SortRoutes);
			foreach (MechJebWaypointRoute route in _routes)
			{
				val.AddNode(route.ToConfigNode());
			}
		}
		val.Save(MuUtils.GetCfgPath("mechjeb_routes.cfg"));
	}

	public override string GetName()
	{
		return Mode.ToString() + " Waypoints" + ((_titleAdd != "") ? (" - " + _titleAdd) : "");
	}

	private static Coordinates GetMouseFlightCoordinates()
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_009f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		CelestialBody currentMainBody = FlightGlobals.currentMainBody;
		Ray val = FlightCamera.fetch.mainCamera.ScreenPointToRay(Input.mousePosition);
		Vector3d val2 = ((Ray)(ref val)).origin - currentMainBody.position;
		RaycastHit val3 = default(RaycastHit);
		if (Physics.Raycast(val, ref val3, (float)currentMainBody.Radius * 4f, 32768, (QueryTriggerInteraction)1))
		{
			return new Coordinates(currentMainBody.GetLatitude(Vector3d.op_Implicit(((RaycastHit)(ref val3)).point), false), MuUtils.ClampDegrees180(currentMainBody.GetLongitude(Vector3d.op_Implicit(((RaycastHit)(ref val3)).point), false)));
		}
		double num = currentMainBody.pqsController.radiusMax;
		double num2 = 0.0;
		int num3 = 0;
		Vector3d val4 = default(Vector3d);
		while (num3 < 50)
		{
			if (PQS.LineSphereIntersection(val2, Vector3d.op_Implicit(((Ray)(ref val)).direction), num, ref val4))
			{
				Vector3d val5 = currentMainBody.position + val4;
				double surfaceHeight = currentMainBody.pqsController.GetSurfaceHeight(QuaternionD.AngleAxis(currentMainBody.GetLongitude(val5, false), Vector3d.down) * QuaternionD.AngleAxis(currentMainBody.GetLatitude(val5, false), Vector3d.forward) * Vector3d.right);
				if (Math.Abs(num - surfaceHeight) < (currentMainBody.pqsController.radiusMax - currentMainBody.pqsController.radiusMin) / 100.0)
				{
					return new Coordinates(currentMainBody.GetLatitude(val5, false), MuUtils.ClampDegrees180(currentMainBody.GetLongitude(val5, false)));
				}
				num2 = num;
				num = surfaceHeight;
				num3++;
			}
			else
			{
				if (num3 == 0)
				{
					break;
				}
				num = (num2 * 9.0 + num) / 10.0;
				num3++;
			}
		}
		return null;
	}

	private static string LatToString(double lat)
	{
		while (lat > 90.0)
		{
			lat -= 180.0;
		}
		while (lat < -90.0)
		{
			lat += 180.0;
		}
		string text = ((lat >= 0.0) ? "N" : "S");
		lat = Math.Abs(lat);
		int num = (int)lat;
		lat -= (double)num;
		lat *= 60.0;
		int num2 = (int)lat;
		lat -= (double)num2;
		lat *= 60.0;
		float num3 = (float)lat;
		return $"{text} {num}° {num2}' {num3:F3}\"";
	}

	private static string LonToString(double lon)
	{
		while (lon > 180.0)
		{
			lon -= 360.0;
		}
		while (lon < -180.0)
		{
			lon += 360.0;
		}
		string text = ((lon >= 0.0) ? "E" : "W");
		lon = Math.Abs(lon);
		int num = (int)lon;
		lon -= (double)num;
		lon *= 60.0;
		int num2 = (int)lon;
		lon -= (double)num2;
		lon *= 60.0;
		float num3 = (float)lon;
		return $"{text} {num}° {num2}' {num3:F3}\"";
	}

	private static double ParseCoord(string latLon, bool isLongitute = false)
	{
		Match match = new Regex("^([nsew])?\\s*(-?\\d+(?:\\.\\d+)?)(?:[°:\\s]+(-?\\d+(?:\\.\\d+)?))?(?:[':\\s]+(-?\\d+(?:\\.\\d+)?))?(?:[^nsew]*([nsew])?)?$", RegexOptions.IgnoreCase).Match(latLon);
		int num = (isLongitute ? 180 : 90);
		float num2 = 1f;
		if (match.Groups[5] != null)
		{
			if (match.Groups[5].Value.ToUpper() == "N" || match.Groups[5].Value.ToUpper() == "E")
			{
				num2 = 1f;
			}
			else if (match.Groups[5].Value.ToUpper() == "S" || match.Groups[5].Value.ToUpper() == "W")
			{
				num2 = -1f;
			}
			else if (match.Groups[1] != null)
			{
				if (match.Groups[1].Value.ToUpper() == "N" || match.Groups[1].Value.ToUpper() == "E")
				{
					num2 = 1f;
				}
				else if (match.Groups[1].Value.ToUpper() == "S" || match.Groups[1].Value.ToUpper() == "W")
				{
					num2 = -1f;
				}
			}
		}
		float result = 0f;
		if (match.Groups[2] != null)
		{
			float.TryParse(match.Groups[2].Value, out result);
		}
		if (result < 0f)
		{
			num2 *= -1f;
			result *= -1f;
		}
		float result2 = 0f;
		if (match.Groups[3] != null)
		{
			float.TryParse(match.Groups[3].Value, out result2);
		}
		float result3 = 0f;
		if (match.Groups[4] != null)
		{
			float.TryParse(match.Groups[4].Value, out result3);
		}
		for (result = (result + result2 / 60f + result3 / 3600f) * num2; result > (float)num; result -= (float)(num * 2))
		{
		}
		for (; result < (float)(-num); result += (float)(num * 2))
		{
		}
		return result;
	}

	private int SortRoutes(MechJebWaypointRoute a, MechJebWaypointRoute b)
	{
		int num = string.Compare(a.Body.bodyName, b.Body.bodyName, ignoreCase: true);
		if (num != 0)
		{
			return num;
		}
		return string.Compare(a.Name, b.Name, ignoreCase: true);
	}

	public MechJebWaypoint SelectedWaypoint()
	{
		if (SelIndex <= -1)
		{
			return null;
		}
		return _ap.Waypoints[SelIndex];
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(500f),
			GUILayout.Height(400f)
		};
	}

	private void DrawPageWaypoints()
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_031d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0307: Unknown result type (might be due to invalid IL or missing references)
		//IL_0270: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_027c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0281: Unknown result type (might be due to invalid IL or missing references)
		//IL_025e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0263: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0419: Unknown result type (might be due to invalid IL or missing references)
		//IL_041f: Invalid comparison between Unknown and I4
		//IL_0bc7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bcc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0432: Unknown result type (might be due to invalid IL or missing references)
		//IL_0428: Unknown result type (might be due to invalid IL or missing references)
		//IL_042d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bea: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bef: Unknown result type (might be due to invalid IL or missing references)
		bool key = GameSettings.MODIFIER_KEY.GetKey(false);
		_scroll = GUILayout.BeginScrollView(_scroll, Array.Empty<GUILayoutOption>());
		if (_ap.Waypoints.Count > 0)
		{
			_waypointRects = (Rect[])(object)new Rect[_ap.Waypoints.Count];
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			double num = 0.0;
			double num2 = 0.0;
			for (int i = 0; i < _ap.Waypoints.Count; i++)
			{
				MechJebWaypoint wp = _ap.Waypoints[i];
				double num3 = ((wp.MaxSpeed > 0f) ? ((double)wp.MaxSpeed) : _ap.speed.Val);
				float num4 = ((wp.MinSpeed > 0f) ? wp.MinSpeed : 0f);
				if (MapView.MapIsEnabled && i == SelIndex)
				{
					GLUtils.DrawGroundMarker(base.MainBody, wp.Latitude, wp.Longitude, Color.red, map: true, ((float)DateTime.Now.Second + (float)DateTime.Now.Millisecond / 1000f) * 6f, base.MainBody.Radius / 100.0);
				}
				if (i >= _ap.WaypointIndex)
				{
					if (_ap.WaypointIndex > -1)
					{
						num += GuiUtils.FromToETA((i == _ap.WaypointIndex) ? base.Vessel.CoM : Vector3d.op_Implicit(_ap.Waypoints[i - 1].Position), Vector3d.op_Implicit(wp.Position), ((double)_ap.etaSpeed > 0.1 && (double)_ap.etaSpeed < num3) ? ((double)(float)Math.Round(_ap.etaSpeed, 1)) : num3);
					}
					num2 += (double)Vector3.Distance((i == _ap.WaypointIndex || (_ap.WaypointIndex == -1 && i == 0)) ? base.Vessel.CoM : Vector3d.op_Implicit(_ap.Waypoints[i - 1].Position), Vector3d.op_Implicit(wp.Position));
				}
				string text = $"[{i + 1}] - {wp.GetNameWithCoords()} - R: {wp.Radius:F1} m\n       S: {num4:F0} ~ {num3:F0} - D: {Statics.ToSI(num2, -1, int.MaxValue)}m - ETA: {GuiUtils.TimeToDHMS(num)}";
				GUI.backgroundColor = (Color)((i == _ap.WaypointIndex) ? new Color(0.5f, 1f, 0.5f) : Color.white);
				if (GUILayout.Button(text, (i == SelIndex) ? _styleActive : (wp.Quicksave ? _styleQuicksave : _styleInactive), Array.Empty<GUILayoutOption>()))
				{
					if (key)
					{
						_ap.WaypointIndex = ((_ap.WaypointIndex == i) ? (-1) : i);
					}
					else if (SelIndex == i)
					{
						SelIndex = -1;
					}
					else
					{
						SelIndex = i;
						_tmpRadius = wp.Radius.ToString();
						_tmpMinSpeed = wp.MinSpeed.ToString();
						_tmpMaxSpeed = wp.MaxSpeed.ToString();
						_tmpLat = LatToString(wp.Latitude);
						_tmpLon = LonToString(wp.Longitude);
					}
				}
				if ((int)Event.current.type == 7)
				{
					_waypointRects[i] = GUILayoutUtility.GetLastRect();
				}
				GUI.backgroundColor = Color.white;
				if (SelIndex <= -1 || SelIndex != i)
				{
					continue;
				}
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				GUILayout.Label("  Radius: ", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
				_tmpRadius = GUILayout.TextField(_tmpRadius, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(50f) });
				float.TryParse(_tmpRadius, out wp.Radius);
				if (GUILayout.Button("A", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
				{
					_ap.Waypoints.GetRange(i, _ap.Waypoints.Count - i).ForEach(delegate(MechJebWaypoint fewp)
					{
						fewp.Radius = wp.Radius;
					});
				}
				GUILayout.Label("- Speed: ", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
				_tmpMinSpeed = GUILayout.TextField(_tmpMinSpeed, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(40f) });
				float.TryParse(_tmpMinSpeed, out wp.MinSpeed);
				if (GUILayout.Button("A", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
				{
					_ap.Waypoints.GetRange(i, _ap.Waypoints.Count - i).ForEach(delegate(MechJebWaypoint fewp)
					{
						fewp.MinSpeed = wp.MinSpeed;
					});
				}
				GUILayout.Label(" - ", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
				_tmpMaxSpeed = GUILayout.TextField(_tmpMaxSpeed, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(40f) });
				float.TryParse(_tmpMaxSpeed, out wp.MaxSpeed);
				if (GUILayout.Button("A", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
				{
					_ap.Waypoints.GetRange(i, _ap.Waypoints.Count - i).ForEach(delegate(MechJebWaypoint fewp)
					{
						fewp.MaxSpeed = wp.MaxSpeed;
					});
				}
				GUILayout.FlexibleSpace();
				if (GUILayout.Button("QS", wp.Quicksave ? _styleQuicksave : _styleInactive, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
				{
					if (key)
					{
						_ap.Waypoints.GetRange(i, _ap.Waypoints.Count - i).ForEach(delegate(MechJebWaypoint fewp)
						{
							fewp.Quicksave = !fewp.Quicksave;
						});
					}
					else
					{
						wp.Quicksave = !wp.Quicksave;
					}
				}
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				GUILayout.Label("Lat ", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
				_tmpLat = GUILayout.TextField(_tmpLat, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(125f) });
				wp.Latitude = ParseCoord(_tmpLat);
				GUILayout.Label(" -  Lon ", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
				_tmpLon = GUILayout.TextField(_tmpLon, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(125f) });
				wp.Longitude = ParseCoord(_tmpLon, isLongitute: true);
				GUILayout.EndHorizontal();
			}
			_titleAdd = "Distance: " + Statics.ToSI(num2, -1, int.MaxValue) + "m - ETA: " + GuiUtils.TimeToDHMS(num);
			GUILayout.EndVertical();
		}
		else
		{
			_titleAdd = "";
		}
		GUILayout.EndScrollView();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(key ? "Reverse" : ((!_waitingForPick) ? "Add Waypoint" : "Abort Adding"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(110f) }))
		{
			if (key)
			{
				_ap.Waypoints.Reverse();
				if (_ap.WaypointIndex > -1)
				{
					_ap.WaypointIndex = _ap.Waypoints.Count - 1 - _ap.WaypointIndex;
				}
				if (SelIndex > -1)
				{
					SelIndex = _ap.Waypoints.Count - 1 - SelIndex;
				}
			}
			else if (!_waitingForPick)
			{
				_waitingForPick = true;
				if (MapView.MapIsEnabled)
				{
					Core.Target.Unset();
					Core.Target.PickPositionTargetOnMap();
				}
			}
			else
			{
				_waitingForPick = false;
			}
		}
		if (GUILayout.Button(key ? "Clear" : "Remove", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(65f) }) && _ap.Waypoints.Count > 0)
		{
			if (key)
			{
				_ap.WaypointIndex = -1;
				_ap.Waypoints.Clear();
			}
			else if (SelIndex >= 0)
			{
				_ap.Waypoints.RemoveAt(SelIndex);
				if (_ap.WaypointIndex > SelIndex)
				{
					_ap.WaypointIndex--;
				}
				if (_ap.WaypointIndex >= _ap.Waypoints.Count)
				{
					_ap.WaypointIndex = _ap.Waypoints.Count - 1;
				}
			}
			SelIndex = -1;
		}
		if (GUILayout.Button(key ? "Top" : "Up", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(57f) }) && SelIndex > 0 && SelIndex < _ap.Waypoints.Count && _ap.Waypoints.Count >= 2)
		{
			if (key)
			{
				MechJebWaypoint item = _ap.Waypoints[SelIndex];
				_ap.Waypoints.RemoveAt(SelIndex);
				_ap.Waypoints.Insert(0, item);
				SelIndex = 0;
			}
			else
			{
				_ap.Waypoints.Swap(SelIndex, --SelIndex);
			}
		}
		if (GUILayout.Button(key ? "Bottom" : "Down", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(57f) }) && SelIndex >= 0 && SelIndex < _ap.Waypoints.Count - 1 && _ap.Waypoints.Count >= 2)
		{
			if (key)
			{
				MechJebWaypoint item2 = _ap.Waypoints[SelIndex];
				_ap.Waypoints.RemoveAt(SelIndex);
				_ap.Waypoints.Add(item2);
				SelIndex = _ap.Waypoints.Count - 1;
			}
			else
			{
				_ap.Waypoints.Swap(SelIndex, ++SelIndex);
			}
		}
		if (GUILayout.Button("Routes", Array.Empty<GUILayoutOption>()))
		{
			_showPage = Pages.ROUTES;
			_scroll = Vector2.zero;
		}
		if (GUILayout.Button("Settings", Array.Empty<GUILayoutOption>()))
		{
			_showPage = Pages.SETTINGS;
			_scroll = Vector2.zero;
		}
		GUILayout.EndHorizontal();
		if (SelIndex >= _ap.Waypoints.Count)
		{
			SelIndex = -1;
		}
		if (SelIndex == -1 && _ap.WaypointIndex > -1 && _lastIndex != _ap.WaypointIndex && _waypointRects.Length != 0)
		{
			_scroll.y = ((Rect)(ref _waypointRects[_ap.WaypointIndex])).y - 160f;
		}
		_lastIndex = _ap.WaypointIndex;
	}

	private void DrawPageSettings()
	{
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_05fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0600: Unknown result type (might be due to invalid IL or missing references)
		//IL_0625: Unknown result type (might be due to invalid IL or missing references)
		//IL_062a: Unknown result type (might be due to invalid IL or missing references)
		_titleAdd = "Settings";
		MechJebModuleCustomWindowEditor computerModule = Core.GetComputerModule<MechJebModuleCustomWindowEditor>();
		if (!_ap.Enabled)
		{
			_ap.CalculateTraction();
		}
		_scroll = GUILayout.BeginScrollView(_scroll, Array.Empty<GUILayoutOption>());
		_settingPageIndex = GUILayout.SelectionGrid(_settingPageIndex, _settingPages, _settingPages.Length, Array.Empty<GUILayoutOption>());
		switch (_settingPageIndex)
		{
		case 0:
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.hPIDp").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.hPIDi").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.hPIDd").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.terrainLookAhead").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.tractionLimit").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.LimitAcceleration").DrawItem();
			GUILayout.EndVertical();
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.sPIDp").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.sPIDi").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.sPIDd").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.turnSpeed").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Value:RoverController.traction").DrawItem();
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();
			break;
		case 1:
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.MohoMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.EveMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.GillyMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.KerbinMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.MunMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.MinmusMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.DunaMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.IkeMapdist").DrawItem();
			GUILayout.EndVertical();
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.DresMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.JoolMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.LaytheMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.VallMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.TyloMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.BopMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.PolMapdist").DrawItem();
			computerModule.registry.Find((InfoItem i) => i.id == "Editable:WaypointWindow.EelooMapdist").DrawItem();
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();
			break;
		}
		GUILayout.EndScrollView();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button("Waypoints", Array.Empty<GUILayoutOption>()))
		{
			_showPage = Pages.WAYPOINTS;
			_scroll = Vector2.zero;
			_lastIndex = -1;
		}
		if (GUILayout.Button("Routes", Array.Empty<GUILayoutOption>()))
		{
			_showPage = Pages.ROUTES;
			_scroll = Vector2.zero;
		}
		if (GUILayout.Button("Help", Array.Empty<GUILayoutOption>()))
		{
			Core.GetComputerModule<MechJebModuleWaypointHelpWindow>().Enabled = !Core.GetComputerModule<MechJebModuleWaypointHelpWindow>().Enabled;
		}
		GUILayout.EndHorizontal();
	}

	private void DrawPageRoutes()
	{
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_034d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0352: Unknown result type (might be due to invalid IL or missing references)
		//IL_0377: Unknown result type (might be due to invalid IL or missing references)
		//IL_037c: Unknown result type (might be due to invalid IL or missing references)
		bool key = GameSettings.MODIFIER_KEY.GetKey(false);
		_titleAdd = "Routes for " + base.Vessel.mainBody.bodyName;
		_scroll = GUILayout.BeginScrollView(_scroll, Array.Empty<GUILayoutOption>());
		List<MechJebWaypointRoute> bodyWPs = _routes.FindAll((MechJebWaypointRoute r) => (Object)(object)r.Body == (Object)(object)base.Vessel.mainBody && r.Mode == Mode.ToString());
		int i;
		for (i = 0; i < bodyWPs.Count; i++)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			if (GUILayout.Button(bodyWPs[i].Name + " - " + bodyWPs[i].Stats, (i == _saveIndex) ? _styleActive : _styleInactive, Array.Empty<GUILayoutOption>()))
			{
				_saveIndex = ((_saveIndex == i) ? (-1) : i);
			}
			if (i == _saveIndex && GUILayout.Button("Delete", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(70f) }))
			{
				_routes.RemoveAll((MechJebWaypointRoute r) => r.Name == bodyWPs[i].Name && (Object)(object)r.Body == (Object)(object)base.Vessel.mainBody && r.Mode == Mode.ToString());
				SaveRoutes();
				_saveIndex = -1;
			}
			GUILayout.EndHorizontal();
		}
		GUILayout.EndScrollView();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		_saveName = GUILayout.TextField(_saveName, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(150f) });
		if (GUILayout.Button("Save", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(50f) }) && _saveName != "" && _ap.Waypoints.Count > 0)
		{
			MechJebWaypointRoute mechJebWaypointRoute = _routes.Find((MechJebWaypointRoute r) => r.Name == _saveName && (Object)(object)r.Body == (Object)(object)base.Vessel.mainBody && r.Mode == Mode.ToString());
			MechJebWaypointRoute wps = new MechJebWaypointRoute(_saveName, base.Vessel.mainBody);
			_ap.Waypoints.ForEach(delegate(MechJebWaypoint wp)
			{
				wps.Add(wp);
			});
			if (mechJebWaypointRoute == null)
			{
				_routes.Add(wps);
			}
			else
			{
				_routes[_routes.IndexOf(mechJebWaypointRoute)] = wps;
			}
			_routes.Sort(SortRoutes);
			SaveRoutes();
		}
		if (GUILayout.Button(key ? "Add" : "Load", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(50f) }) && _saveIndex > -1)
		{
			if (!key)
			{
				_ap.WaypointIndex = -1;
				_ap.Waypoints.Clear();
			}
			_routes[_saveIndex].ForEach(delegate(MechJebWaypoint wp)
			{
				_ap.Waypoints.Add(wp);
			});
		}
		if (GUILayout.Button("Waypoints", Array.Empty<GUILayoutOption>()))
		{
			_showPage = Pages.WAYPOINTS;
			_scroll = Vector2.zero;
			_lastIndex = -1;
		}
		if (GUILayout.Button("Settings", Array.Empty<GUILayoutOption>()))
		{
			_showPage = Pages.SETTINGS;
			_scroll = Vector2.zero;
		}
		GUILayout.EndHorizontal();
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_010d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Expected O, but got Unknown
		//IL_0143: Unknown result type (might be due to invalid IL or missing references)
		//IL_0148: Unknown result type (might be due to invalid IL or missing references)
		//IL_0149: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		//IL_0153: Unknown result type (might be due to invalid IL or missing references)
		//IL_015a: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_015d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0164: Unknown result type (might be due to invalid IL or missing references)
		//IL_017a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0184: Expected O, but got Unknown
		//IL_01b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Expected O, but got Unknown
		//IL_0235: Unknown result type (might be due to invalid IL or missing references)
		//IL_023b: Invalid comparison between Unknown and I4
		//IL_0364: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
		Rect windowPos = base.WindowPos;
		if (GUI.Button(new Rect(((Rect)(ref windowPos)).width - 48f, 0f, 13f, 20f), "?", GuiUtils.YellowOnHover))
		{
			MechJebModuleWaypointHelpWindow computerModule = Core.GetComputerModule<MechJebModuleWaypointHelpWindow>();
			switch (_showPage)
			{
			case Pages.WAYPOINTS:
				computerModule.SelTopic = ((IList)computerModule.Topics).IndexOf((object)"Waypoints");
				break;
			case Pages.SETTINGS:
				computerModule.SelTopic = ((IList)computerModule.Topics).IndexOf((object)"Settings");
				break;
			case Pages.ROUTES:
				computerModule.SelTopic = ((IList)computerModule.Topics).IndexOf((object)"Routes");
				break;
			}
			computerModule.Enabled = computerModule.SelTopic > -1 || computerModule.Enabled;
		}
		if (_styleInactive == null)
		{
			_styleInactive = new GUIStyle(((Object)(object)GuiUtils.Skin != (Object)null) ? GuiUtils.Skin.button : GuiUtils.DefaultSkin.button)
			{
				alignment = (TextAnchor)0
			};
		}
		if (_styleActive == null)
		{
			_styleActive = new GUIStyle(_styleInactive);
			GUIStyleState active = _styleActive.active;
			GUIStyleState focused = _styleActive.focused;
			GUIStyleState hover = _styleActive.hover;
			Color val = (_styleActive.normal.textColor = Color.green);
			Color val3 = (hover.textColor = val);
			Color textColor = (focused.textColor = val3);
			active.textColor = textColor;
		}
		if (_styleQuicksave == null)
		{
			_styleQuicksave = new GUIStyle(_styleActive);
			GUIStyleState active2 = _styleQuicksave.active;
			GUIStyleState focused2 = _styleQuicksave.focused;
			GUIStyleState hover2 = _styleQuicksave.hover;
			Color val = (_styleQuicksave.normal.textColor = Color.yellow);
			Color val3 = (hover2.textColor = val);
			Color textColor = (focused2.textColor = val3);
			active2.textColor = textColor;
		}
		bool key = GameSettings.MODIFIER_KEY.GetKey(false);
		switch (_showPage)
		{
		case Pages.WAYPOINTS:
			DrawPageWaypoints();
			break;
		case Pages.SETTINGS:
			DrawPageSettings();
			break;
		case Pages.ROUTES:
			DrawPageRoutes();
			break;
		}
		if (_waitingForPick && base.Vessel.isActiveVessel && (int)Event.current.type == 7)
		{
			if (MapView.MapIsEnabled)
			{
				if (!Core.Target.pickingPositionTarget)
				{
					if (Core.Target.PositionTargetExists)
					{
						if (SelIndex > -1 && SelIndex < _ap.Waypoints.Count)
						{
							_ap.Waypoints.Insert(SelIndex, new MechJebWaypoint(Core.Target.GetPositionTargetPosition()));
							_tmpRadius = _ap.Waypoints[SelIndex].Radius.ToString();
							_tmpLat = LatToString(_ap.Waypoints[SelIndex].Latitude);
							_tmpLon = LonToString(_ap.Waypoints[SelIndex].Longitude);
						}
						else
						{
							_ap.Waypoints.Add(new MechJebWaypoint(Core.Target.GetPositionTargetPosition()));
						}
						Core.Target.Unset();
						_waitingForPick = key;
					}
					else
					{
						Core.Target.PickPositionTargetOnMap();
					}
				}
			}
			else if (!GuiUtils.MouseIsOverWindow(Core))
			{
				Coordinates mouseFlightCoordinates = GetMouseFlightCoordinates();
				if (mouseFlightCoordinates != null && Input.GetMouseButtonDown(0))
				{
					if (SelIndex > -1 && SelIndex < _ap.Waypoints.Count)
					{
						_ap.Waypoints.Insert(SelIndex, new MechJebWaypoint(mouseFlightCoordinates.Latitude, mouseFlightCoordinates.Longitude));
						_tmpRadius = _ap.Waypoints[SelIndex].Radius.ToString();
						_tmpLat = LatToString(_ap.Waypoints[SelIndex].Latitude);
						_tmpLon = LonToString(_ap.Waypoints[SelIndex].Longitude);
					}
					else
					{
						_ap.Waypoints.Add(new MechJebWaypoint(mouseFlightCoordinates.Latitude, mouseFlightCoordinates.Longitude));
					}
					_waitingForPick = key;
				}
			}
		}
		base.WindowGUI(windowID);
	}

	public override void OnFixedUpdate()
	{
		if (base.Vessel.isActiveVessel && ((Object)(object)_renderer == (Object)null || _renderer.AP != _ap))
		{
			MechJebRouteRenderer.AttachToMapView(Core);
		}
		_ap.Waypoints.ForEach(delegate(MechJebWaypoint wp)
		{
			wp.Update();
		});
		base.OnFixedUpdate();
	}
}
