using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace MuMech;

internal class DebugIcoSphere
{
	private struct TriangleIndices
	{
		public readonly int v1;

		public readonly int v2;

		public readonly int v3;

		public TriangleIndices(int v1, int v2, int v3)
		{
			this.v1 = v1;
			this.v2 = v2;
			this.v3 = v3;
		}
	}

	private readonly GameObject gameObject;

	private readonly MeshRenderer _meshRenderer;

	private float radius;

	private bool seeThrough;

	public DebugIcoSphere(Color color, bool seeThrough = false)
	{
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		gameObject = CreateIcoSphere(1f);
		gameObject.layer = 15;
		_meshRenderer = gameObject.AddComponent<MeshRenderer>();
		((Renderer)_meshRenderer).material.color = color;
		((Renderer)_meshRenderer).shadowCastingMode = (ShadowCastingMode)0;
		((Renderer)_meshRenderer).receiveShadows = false;
		SetRadius(0.09f);
		SeeThrough(seeThrough);
	}

	public void Destroy()
	{
		Object.Destroy((Object)(object)gameObject);
	}

	public void Set(Vector3d position)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		gameObject.transform.position = Vector3d.op_Implicit(position);
	}

	public void State(bool state)
	{
		if (state != gameObject.activeSelf)
		{
			gameObject.SetActive(state);
		}
	}

	public void SetRadius(float radius)
	{
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		if (this.radius != radius && !(radius <= 0f))
		{
			this.radius = radius;
			gameObject.transform.localScale = new Vector3(radius, radius, radius);
		}
	}

	public void SeeThrough(bool state)
	{
		if (seeThrough != state)
		{
			seeThrough = state;
			((Renderer)_meshRenderer).material.shader = (state ? MechJebBundlesManager.diffuseAmbientIgnoreZ : MechJebBundlesManager.diffuseAmbient);
		}
	}

	private static GameObject CreateIcoSphere(float radius, int recursionLevel = 3)
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Expected O, but got Unknown
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0081: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_011f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0125: Unknown result type (might be due to invalid IL or missing references)
		//IL_013d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0142: Unknown result type (might be due to invalid IL or missing references)
		//IL_0146: Unknown result type (might be due to invalid IL or missing references)
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0164: Unknown result type (might be due to invalid IL or missing references)
		//IL_0169: Unknown result type (might be due to invalid IL or missing references)
		//IL_016d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0173: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0193: Unknown result type (might be due to invalid IL or missing references)
		//IL_0199: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0203: Unknown result type (might be due to invalid IL or missing references)
		//IL_0207: Unknown result type (might be due to invalid IL or missing references)
		//IL_020d: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f3: Unknown result type (might be due to invalid IL or missing references)
		GameObject val = new GameObject("DebugIcoSphere");
		val.layer = 15;
		Mesh mesh = val.AddComponent<MeshFilter>().mesh;
		mesh.Clear();
		List<Vector3> vertices = new List<Vector3>();
		Dictionary<long, int> cache = new Dictionary<long, int>();
		float num = (1f + Mathf.Sqrt(5f)) / 2f;
		List<Vector3> list = vertices;
		Vector3 val2 = new Vector3(-1f, num, 0f);
		list.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list2 = vertices;
		val2 = new Vector3(1f, num, 0f);
		list2.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list3 = vertices;
		val2 = new Vector3(-1f, 0f - num, 0f);
		list3.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list4 = vertices;
		val2 = new Vector3(1f, 0f - num, 0f);
		list4.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list5 = vertices;
		val2 = new Vector3(0f, -1f, num);
		list5.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list6 = vertices;
		val2 = new Vector3(0f, 1f, num);
		list6.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list7 = vertices;
		val2 = new Vector3(0f, -1f, 0f - num);
		list7.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list8 = vertices;
		val2 = new Vector3(0f, 1f, 0f - num);
		list8.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list9 = vertices;
		val2 = new Vector3(num, 0f, -1f);
		list9.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list10 = vertices;
		val2 = new Vector3(num, 0f, 1f);
		list10.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list11 = vertices;
		val2 = new Vector3(0f - num, 0f, -1f);
		list11.Add(((Vector3)(ref val2)).normalized * radius);
		List<Vector3> list12 = vertices;
		val2 = new Vector3(0f - num, 0f, 1f);
		list12.Add(((Vector3)(ref val2)).normalized * radius);
		List<TriangleIndices> list13 = new List<TriangleIndices>();
		list13.Add(new TriangleIndices(0, 11, 5));
		list13.Add(new TriangleIndices(0, 5, 1));
		list13.Add(new TriangleIndices(0, 1, 7));
		list13.Add(new TriangleIndices(0, 7, 10));
		list13.Add(new TriangleIndices(0, 10, 11));
		list13.Add(new TriangleIndices(1, 5, 9));
		list13.Add(new TriangleIndices(5, 11, 4));
		list13.Add(new TriangleIndices(11, 10, 2));
		list13.Add(new TriangleIndices(10, 7, 6));
		list13.Add(new TriangleIndices(7, 1, 8));
		list13.Add(new TriangleIndices(3, 9, 4));
		list13.Add(new TriangleIndices(3, 4, 2));
		list13.Add(new TriangleIndices(3, 2, 6));
		list13.Add(new TriangleIndices(3, 6, 8));
		list13.Add(new TriangleIndices(3, 8, 9));
		list13.Add(new TriangleIndices(4, 9, 5));
		list13.Add(new TriangleIndices(2, 4, 11));
		list13.Add(new TriangleIndices(6, 2, 10));
		list13.Add(new TriangleIndices(8, 6, 7));
		list13.Add(new TriangleIndices(9, 8, 1));
		for (int i = 0; i < recursionLevel; i++)
		{
			List<TriangleIndices> list14 = new List<TriangleIndices>();
			for (int j = 0; j < list13.Count; j++)
			{
				TriangleIndices triangleIndices = list13[j];
				int middlePoint = getMiddlePoint(triangleIndices.v1, triangleIndices.v2, ref vertices, ref cache, radius);
				int middlePoint2 = getMiddlePoint(triangleIndices.v2, triangleIndices.v3, ref vertices, ref cache, radius);
				int middlePoint3 = getMiddlePoint(triangleIndices.v3, triangleIndices.v1, ref vertices, ref cache, radius);
				list14.Add(new TriangleIndices(triangleIndices.v1, middlePoint, middlePoint3));
				list14.Add(new TriangleIndices(triangleIndices.v2, middlePoint2, middlePoint));
				list14.Add(new TriangleIndices(triangleIndices.v3, middlePoint3, middlePoint2));
				list14.Add(new TriangleIndices(middlePoint, middlePoint2, middlePoint3));
			}
			list13 = list14;
		}
		mesh.vertices = vertices.ToArray();
		List<int> list15 = new List<int>();
		for (int k = 0; k < list13.Count; k++)
		{
			list15.Add(list13[k].v1);
			list15.Add(list13[k].v2);
			list15.Add(list13[k].v3);
		}
		mesh.triangles = list15.ToArray();
		mesh.uv = (Vector2[])(object)new Vector2[vertices.Count];
		Vector3[] array = (Vector3[])(object)new Vector3[vertices.Count];
		for (int l = 0; l < array.Length; l++)
		{
			int num2 = l;
			val2 = vertices[l];
			array[num2] = ((Vector3)(ref val2)).normalized;
		}
		mesh.normals = array;
		mesh.RecalculateBounds();
		return val;
	}

	private static int getMiddlePoint(int p1, int p2, ref List<Vector3> vertices, ref Dictionary<long, int> cache, float radius)
	{
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		bool num = p1 < p2;
		long num2 = (num ? p1 : p2);
		long num3 = (num ? p2 : p1);
		long key = (num2 << 32) + num3;
		if (cache.TryGetValue(key, out var value))
		{
			return value;
		}
		Vector3 val = vertices[p1];
		Vector3 val2 = vertices[p2];
		Vector3 val3 = default(Vector3);
		((Vector3)(ref val3))._002Ector((val.x + val2.x) / 2f, (val.y + val2.y) / 2f, (val.z + val2.z) / 2f);
		int count = vertices.Count;
		vertices.Add(((Vector3)(ref val3)).normalized * radius);
		cache.Add(key, count);
		return count;
	}
}
