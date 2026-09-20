using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleTargetController : ComputerModule
{
	public CelestialBody targetBody;

	[Persistent(pass = 4)]
	public EditableAngle targetLatitude = new EditableAngle(0.0);

	[Persistent(pass = 4)]
	public EditableAngle targetLongitude = new EditableAngle(0.0);

	private Vector3d targetDirection;

	private bool wasActiveVessel;

	public bool pickingPositionTarget;

	public bool NormalTargetExists
	{
		get
		{
			if (Target != null)
			{
				if (!(Target is Vessel) && !(Target is CelestialBody))
				{
					return CanAlign;
				}
				return true;
			}
			return false;
		}
	}

	public bool PositionTargetExists
	{
		get
		{
			if (Target != null && (Target is PositionTarget || Target is Vessel))
			{
				return !(Target is DirectionTarget);
			}
			return false;
		}
	}

	public bool CanAlign => (int)Target.GetTargetingMode() == 3;

	public ITargetable Target { get; private set; }

	public Orbit TargetOrbit
	{
		get
		{
			if (Target == null)
			{
				return null;
			}
			return Target.GetOrbit();
		}
	}

	public Vector3 Position => Transform.position;

	public float Distance => Vector3.Distance(Position, base.Vessel.GetTransform().position);

	public Vector3d RelativeVelocity => base.Vessel.orbit.GetVel() - TargetOrbit.GetVel();

	public Vector3d RelativePosition => Vector3d.op_Implicit(base.Vessel.GetTransform().position - Position);

	public Transform Transform => Target.GetTransform();

	public Vector3 DockingAxis
	{
		get
		{
			//IL_001f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0024: Unknown result type (might be due to invalid IL or missing references)
			//IL_000e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			if (CanAlign)
			{
				return -Transform.forward;
			}
			return -Transform.up;
		}
	}

	public string Name => Target.GetName();

	public MechJebModuleTargetController(MechJebCore core)
		: base(core)
	{
	}

	public void Set(ITargetable t)
	{
		Target = t;
		if ((Object)(object)base.Vessel != (Object)null)
		{
			base.Vessel.targetObject = Target;
		}
	}

	public void SetPositionTarget(CelestialBody body, double latitude, double longitude)
	{
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Expected O, but got Unknown
		targetBody = body;
		targetLatitude = latitude;
		targetLongitude = longitude;
		Set((ITargetable)new PositionTarget(string.Format(GetPositionTargetString(), latitude, longitude)));
	}

	[ValueInfoItem("#MechJeb_Targetcoordinates", InfoItem.Category.Target)]
	public string GetPositionTargetString()
	{
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		if (Target is PositionTarget)
		{
			return Coordinates.ToStringDMS(targetLatitude, targetLongitude, newline: true);
		}
		if (NormalTargetExists)
		{
			return Coordinates.ToStringDMS(TargetOrbit.referenceBody.GetLatitude(Vector3d.op_Implicit(Position), false), TargetOrbit.referenceBody.GetLongitude(Vector3d.op_Implicit(Position), false), newline: true);
		}
		return "N/A";
	}

	public Vector3d GetPositionTargetPosition()
	{
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		return targetBody.GetWorldSurfacePosition((double)targetLatitude, (double)targetLongitude, targetBody.TerrainAltitude((double)targetLatitude, (double)targetLongitude, false)) - targetBody.position;
	}

	public void SetDirectionTarget(string name)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Expected O, but got Unknown
		Set((ITargetable)new DirectionTarget(name));
	}

	[ActionInfoItem("#MechJeb_Pickpositiontarget", InfoItem.Category.Target)]
	public void PickPositionTargetOnMap()
	{
		pickingPositionTarget = true;
		MapView.EnterMapView();
		ScreenMessages.PostScreenMessage(Localizer.Format("#MechJeb_pickingPositionMsg", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(base.MainBody.displayName) }), 3f, (ScreenMessageStyle)0);
	}

	public void StopPickPositionTargetOnMap()
	{
		pickingPositionTarget = false;
		Cursor.visible = true;
	}

	public void Unset()
	{
		Set(null);
	}

	public void UpdateDirectionTarget(Vector3d direction)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		targetDirection = direction;
	}

	public override void OnStart(StartState state)
	{
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Expected O, but got Unknown
		Core.AddToPostDrawQueue(new Callback(DoMapView));
		Users.Add(this);
	}

	public override void OnFixedUpdate()
	{
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0172: Unknown result type (might be due to invalid IL or missing references)
		//IL_0178: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_0140: Expected O, but got Unknown
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0197: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0108: Unknown result type (might be due to invalid IL or missing references)
		//IL_010d: Unknown result type (might be due to invalid IL or missing references)
		if (!wasActiveVessel && base.Vessel.isActiveVessel && Target != null && (Object)(object)Target.GetVessel() != (Object)null)
		{
			base.Vessel.targetObject = Target;
		}
		if (Target != base.Vessel.targetObject)
		{
			Target = base.Vessel.targetObject;
			if (Target is Vessel && ((Vessel)Target).LandedOrSplashed && (Object)(object)((Vessel)Target).mainBody == (Object)(object)base.Vessel.mainBody)
			{
				targetBody = base.Vessel.mainBody;
				targetLatitude = base.Vessel.mainBody.GetLatitude(Vector3d.op_Implicit(Target.GetTransform().position), false);
				targetLongitude = base.Vessel.mainBody.GetLongitude(Vector3d.op_Implicit(Target.GetTransform().position), false);
			}
			if (Target is CelestialBody)
			{
				targetBody = (CelestialBody)Target;
			}
		}
		if ((Object)(object)targetBody == (Object)null)
		{
			targetBody = base.Vessel.mainBody;
		}
		if (Target is DirectionTarget)
		{
			((DirectionTarget)Target).Update(targetDirection);
		}
		else if (Target is PositionTarget)
		{
			((PositionTarget)Target).Update(targetBody, (double)targetLatitude, (double)targetLongitude);
		}
		wasActiveVessel = base.Vessel.isActiveVessel;
	}

	public override void OnUpdate()
	{
		if (MapView.MapIsEnabled && pickingPositionTarget)
		{
			if (!GuiUtils.MouseIsOverWindow(Core) && GuiUtils.GetMouseCoordinates(base.MainBody) != null)
			{
				Cursor.visible = false;
			}
			else
			{
				Cursor.visible = true;
			}
		}
	}

	private void DoMapView()
	{
		DoCoordinatePicking();
		DrawMapViewTarget();
	}

	private void DoCoordinatePicking()
	{
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00db: Unknown result type (might be due to invalid IL or missing references)
		if (pickingPositionTarget && !MapView.MapIsEnabled)
		{
			StopPickPositionTargetOnMap();
		}
		if (!pickingPositionTarget || !MapView.MapIsEnabled || !base.Vessel.isActiveVessel || GuiUtils.MouseIsOverWindow(Core))
		{
			return;
		}
		Coordinates mouseCoordinates = GuiUtils.GetMouseCoordinates(base.MainBody);
		if (mouseCoordinates != null)
		{
			GLUtils.DrawGroundMarker(base.MainBody, mouseCoordinates.Latitude, mouseCoordinates.Longitude, new Color(1f, 0.56f, 0f), map: true, 60.0);
			string experimentBiomeSafe = base.MainBody.GetExperimentBiomeSafe(mouseCoordinates.Latitude, mouseCoordinates.Longitude);
			GUI.Label(new Rect(Input.mousePosition.x + 15f, (float)Screen.height - Input.mousePosition.y, 200f, 50f), mouseCoordinates.ToStringDecimal() + "\n" + experimentBiomeSafe);
			if (Input.GetMouseButtonDown(0))
			{
				SetPositionTarget(base.MainBody, mouseCoordinates.Latitude, mouseCoordinates.Longitude);
				StopPickPositionTargetOnMap();
			}
		}
	}

	private void DrawMapViewTarget()
	{
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		if (!HighLogic.LoadedSceneIsEditor && MapView.MapIsEnabled && base.Vessel.isActiveVessel && !((Object)(object)base.Vessel.GetMasterMechJeb() != (Object)(object)Core) && Target != null && (Target is PositionTarget || Target is Vessel) && (!(Target is Vessel) || (((Vessel)Target).LandedOrSplashed && !((Object)(object)((Vessel)Target).mainBody != (Object)(object)base.Vessel.mainBody))) && !(Target is DirectionTarget))
		{
			GLUtils.DrawGroundMarker(targetBody, targetLatitude, targetLongitude, Color.red, map: true);
		}
	}
}
