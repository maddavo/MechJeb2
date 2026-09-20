using System;
using System.Collections.Generic;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public static class GLUtils
{
	private static Material _materialInternal;

	private static readonly List<Vector3d> _points = new List<Vector3d>();

	private static Material _material
	{
		get
		{
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001e: Expected O, but got Unknown
			object obj = _materialInternal;
			if (obj == null)
			{
				Material val = new Material(Shader.Find("Legacy Shaders/Particles/Additive"));
				_materialInternal = val;
				obj = (object)val;
			}
			return (Material)obj;
		}
	}

	public static void DrawMapViewGroundMarker(CelestialBody body, double latitude, double longitude, Color c, double rotation = 0.0, double radius = 0.0)
	{
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		DrawGroundMarker(body, latitude, longitude, c, map: true, rotation, radius);
	}

	public static void DrawGroundMarker(CelestialBody body, double latitude, double longitude, Color c, bool map, double rotation = 0.0, double radius = 0.0)
	{
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
		//IL_0070: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_0127: Unknown result type (might be due to invalid IL or missing references)
		//IL_012c: Unknown result type (might be due to invalid IL or missing references)
		//IL_012e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0133: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013d: Unknown result type (might be due to invalid IL or missing references)
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_014d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		//IL_0154: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_015e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0163: Unknown result type (might be due to invalid IL or missing references)
		//IL_016b: Unknown result type (might be due to invalid IL or missing references)
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_017b: Unknown result type (might be due to invalid IL or missing references)
		//IL_017c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0181: Unknown result type (might be due to invalid IL or missing references)
		//IL_0183: Unknown result type (might be due to invalid IL or missing references)
		//IL_0188: Unknown result type (might be due to invalid IL or missing references)
		//IL_018d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0192: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0203: Unknown result type (might be due to invalid IL or missing references)
		//IL_0208: Unknown result type (might be due to invalid IL or missing references)
		//IL_020d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		Vector3d surfaceNVector = body.GetSurfaceNVector(latitude, longitude);
		double num = body.pqsController.GetSurfaceHeight(QuaternionD.AngleAxis(longitude, Vector3d.down) * QuaternionD.AngleAxis(latitude, Vector3d.forward) * Vector3d.right);
		if (num < body.Radius)
		{
			num = body.Radius;
		}
		Vector3d val = body.position + num * surfaceNVector;
		Vector3d camPos = (map ? ScaledSpace.ScaledToLocalSpace(Vector3d.op_Implicit(((Component)PlanetariumCamera.Camera).transform.position)) : Vector3d.op_Implicit(((Component)FlightCamera.fetch.mainCamera).transform.position));
		if (!IsOccluded(val, body, camPos))
		{
			Vector3d val2 = Vector3d.Exclude(surfaceNVector, Vector3d.op_Implicit(((Component)body).transform.up));
			Vector3d normalized = ((Vector3d)(ref val2)).normalized;
			if (radius <= 0.0)
			{
				radius = (map ? (body.Radius / 15.0) : 5.0);
			}
			if (map || !(FlightCamera.fetch.mainCamera.WorldToViewportPoint(Vector3d.op_Implicit(val)).z < 0f))
			{
				GLTriangle(val, val + radius * (QuaternionD.AngleAxis(rotation - 10.0, surfaceNVector) * normalized), val + radius * (QuaternionD.AngleAxis(rotation + 10.0, surfaceNVector) * normalized), c, map);
				GLTriangle(val, val + radius * (QuaternionD.AngleAxis(rotation + 110.0, surfaceNVector) * normalized), val + radius * (QuaternionD.AngleAxis(rotation + 130.0, surfaceNVector) * normalized), c, map);
				GLTriangle(val, val + radius * (QuaternionD.AngleAxis(rotation - 110.0, surfaceNVector) * normalized), val + radius * (QuaternionD.AngleAxis(rotation - 130.0, surfaceNVector) * normalized), c, map);
			}
		}
	}

	private static void GLTriangle(Vector3d worldVertices1, Vector3d worldVertices2, Vector3d worldVertices3, Color c, bool map)
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		GL.PushMatrix();
		_material.SetPass(0);
		GL.LoadOrtho();
		GL.Begin(4);
		GL.Color(c);
		GLVertex(worldVertices1, map);
		GLVertex(worldVertices2, map);
		GLVertex(worldVertices3, map);
		GL.End();
		GL.PopMatrix();
	}

	private static void GLVertex(Vector3d worldPosition, bool map = false)
	{
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		Vector3 val = (map ? PlanetariumCamera.Camera.WorldToViewportPoint(Vector3d.op_Implicit(ScaledSpace.LocalToScaledSpace(worldPosition))) : FlightCamera.fetch.mainCamera.WorldToViewportPoint(Vector3d.op_Implicit(worldPosition)));
		GL.Vertex3(val.x, val.y, 0f);
	}

	private static void GLPixelLine(Vector3d worldPosition1, Vector3d worldPosition2, bool map)
	{
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		Vector3 val;
		Vector3 val2;
		if (map)
		{
			val = PlanetariumCamera.Camera.WorldToScreenPoint(Vector3d.op_Implicit(ScaledSpace.LocalToScaledSpace(worldPosition1)));
			val2 = PlanetariumCamera.Camera.WorldToScreenPoint(Vector3d.op_Implicit(ScaledSpace.LocalToScaledSpace(worldPosition2)));
		}
		else
		{
			val = FlightCamera.fetch.mainCamera.WorldToScreenPoint(Vector3d.op_Implicit(worldPosition1));
			val2 = FlightCamera.fetch.mainCamera.WorldToScreenPoint(Vector3d.op_Implicit(worldPosition2));
		}
		if (val.z > 0f && val2.z > 0f)
		{
			GL.Vertex3(val.x, val.y, 0f);
			GL.Vertex3(val2.x, val2.y, 0f);
		}
	}

	private static bool IsOccluded(Vector3d worldPosition, CelestialBody byBody, Vector3d camPos)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = (byBody.position - camPos) / (byBody.Radius - 100.0);
		Vector3d val2 = (worldPosition - camPos) / (byBody.Radius - 100.0);
		double num = Vector3d.Dot(val2, val);
		if (num < ((Vector3d)(ref val)).sqrMagnitude - 1.0)
		{
			return false;
		}
		return num * num / ((Vector3d)(ref val2)).sqrMagnitude > ((Vector3d)(ref val)).sqrMagnitude - 1.0;
	}

	public static void DrawPath(CelestialBody mainBody, List<Vector3d> points, Color c, bool map, bool dashed = false)
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0081: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		GL.PushMatrix();
		_material.SetPass(0);
		GL.LoadPixelMatrix();
		GL.Begin(1);
		GL.Color(c);
		Vector3d camPos = (map ? ScaledSpace.ScaledToLocalSpace(Vector3d.op_Implicit(((Component)PlanetariumCamera.Camera).transform.position)) : Vector3d.op_Implicit(((Component)FlightCamera.fetch.mainCamera).transform.position));
		int num = ((!dashed) ? 1 : 2);
		for (int i = 0; i < points.Count - 1; i += num)
		{
			if (!IsOccluded(points[i], mainBody, camPos) && !IsOccluded(points[i + 1], mainBody, camPos))
			{
				GLPixelLine(points[i], points[i + 1], map);
			}
		}
		GL.End();
		GL.PopMatrix();
	}

	public static void DrawBoundingBox(CelestialBody mainBody, Vessel vessel, MechJebModuleDockingAutopilot.Box3d box, Color c)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0109: Unknown result type (might be due to invalid IL or missing references)
		//IL_010e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0110: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0113: Unknown result type (might be due to invalid IL or missing references)
		//IL_011f: Unknown result type (might be due to invalid IL or missing references)
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0141: Unknown result type (might be due to invalid IL or missing references)
		//IL_0146: Unknown result type (might be due to invalid IL or missing references)
		//IL_014b: Unknown result type (might be due to invalid IL or missing references)
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_014e: Unknown result type (might be due to invalid IL or missing references)
		//IL_015a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0167: Unknown result type (might be due to invalid IL or missing references)
		//IL_0173: Unknown result type (might be due to invalid IL or missing references)
		//IL_0178: Unknown result type (might be due to invalid IL or missing references)
		//IL_017d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0182: Unknown result type (might be due to invalid IL or missing references)
		//IL_0187: Unknown result type (might be due to invalid IL or missing references)
		//IL_0189: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0199: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0204: Unknown result type (might be due to invalid IL or missing references)
		//IL_0222: Unknown result type (might be due to invalid IL or missing references)
		//IL_0228: Unknown result type (might be due to invalid IL or missing references)
		//IL_022f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0236: Unknown result type (might be due to invalid IL or missing references)
		//IL_023d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0245: Unknown result type (might be due to invalid IL or missing references)
		//IL_024d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0255: Unknown result type (might be due to invalid IL or missing references)
		//IL_025d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0264: Unknown result type (might be due to invalid IL or missing references)
		//IL_026b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0273: Unknown result type (might be due to invalid IL or missing references)
		//IL_027b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0283: Unknown result type (might be due to invalid IL or missing references)
		//IL_028b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0293: Unknown result type (might be due to invalid IL or missing references)
		//IL_029b: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02af: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02be: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d6: Unknown result type (might be due to invalid IL or missing references)
		Transform transform = ((Component)vessel).transform;
		Vector3d val = Vector3d.op_Implicit(transform.TransformPoint(box.center));
		Quaternion rotation = transform.rotation;
		Vector3d worldPosition = val + rotation * Vector3d.op_Implicit(new Vector3d((double)box.size.x, (double)box.size.y, (double)box.size.z));
		Vector3d worldPosition2 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)box.size.x, (double)(0f - box.size.y), (double)box.size.z));
		Vector3d worldPosition3 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)(0f - box.size.x), (double)(0f - box.size.y), (double)box.size.z));
		Vector3d worldPosition4 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)(0f - box.size.x), (double)box.size.y, (double)box.size.z));
		Vector3d worldPosition5 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)box.size.x, (double)box.size.y, (double)(0f - box.size.z)));
		Vector3d worldPosition6 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)box.size.x, (double)(0f - box.size.y), (double)(0f - box.size.z)));
		Vector3d worldPosition7 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)(0f - box.size.x), (double)(0f - box.size.y), (double)(0f - box.size.z)));
		Vector3d worldPosition8 = val + rotation * Vector3d.op_Implicit(new Vector3d((double)(0f - box.size.x), (double)box.size.y, (double)(0f - box.size.z)));
		GL.PushMatrix();
		_material.SetPass(0);
		GL.LoadOrtho();
		GL.Begin(1);
		GL.Color(c);
		GLVertex(worldPosition);
		GLVertex(worldPosition2);
		GLVertex(worldPosition2);
		GLVertex(worldPosition3);
		GLVertex(worldPosition3);
		GLVertex(worldPosition4);
		GLVertex(worldPosition4);
		GLVertex(worldPosition);
		GLVertex(worldPosition5);
		GLVertex(worldPosition6);
		GLVertex(worldPosition6);
		GLVertex(worldPosition7);
		GLVertex(worldPosition7);
		GLVertex(worldPosition8);
		GLVertex(worldPosition8);
		GLVertex(worldPosition5);
		GLVertex(worldPosition);
		GLVertex(worldPosition5);
		GLVertex(worldPosition2);
		GLVertex(worldPosition6);
		GLVertex(worldPosition3);
		GLVertex(worldPosition7);
		GLVertex(worldPosition4);
		GLVertex(worldPosition8);
		GL.End();
		GL.PopMatrix();
	}

	public static void DrawOrbit(Orbit o, Color c)
	{
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		_points.Clear();
		if (o.eccentricity < 1.0)
		{
			for (int i = 0; i < 360; i++)
			{
				_points.Add(o.WorldPositionAtUT(o.TimeOfTrueAnomaly(Statics.Deg2Rad((double)i), 0.0)));
			}
			_points.Add(_points[0]);
		}
		else
		{
			for (int j = -1000; j <= 1000; j += 5)
			{
				_points.Add(o.WorldPositionAtUT(o.UTAtMeanAnomaly((double)j * (Math.PI / 180.0), 0.0)));
			}
		}
		DrawPath(o.referenceBody, _points, c, map: true);
	}
}
