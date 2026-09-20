using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace MuMech;

internal class DebugArrow
{
	private readonly GameObject gameObject;

	private readonly GameObject haft;

	private GameObject cone;

	private const float coneLength = 0.5f;

	private float length;

	private bool seeThrough;

	private readonly MeshRenderer _haftMeshRenderer;

	private readonly MeshRenderer _coneMeshRenderer;

	public DebugArrow(Color color, bool seeThrough = false)
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Expected O, but got Unknown
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_0160: Unknown result type (might be due to invalid IL or missing references)
		gameObject = new GameObject("DebugArrow");
		gameObject.layer = 15;
		haft = CreateCone(1f, 0.05f, 0.05f, 0f, 20);
		haft.transform.parent = gameObject.transform;
		haft.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
		haft.layer = 15;
		cone = CreateCone(0.5f, 0.15f, 0f, 0f, 20);
		cone.transform.parent = gameObject.transform;
		cone.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
		cone.layer = 15;
		SetLength(4f);
		_haftMeshRenderer = haft.AddComponent<MeshRenderer>();
		_coneMeshRenderer = cone.AddComponent<MeshRenderer>();
		((Renderer)_haftMeshRenderer).material.color = color;
		((Renderer)_haftMeshRenderer).shadowCastingMode = (ShadowCastingMode)0;
		((Renderer)_haftMeshRenderer).receiveShadows = false;
		((Renderer)_coneMeshRenderer).material.color = color;
		((Renderer)_coneMeshRenderer).shadowCastingMode = (ShadowCastingMode)0;
		((Renderer)_coneMeshRenderer).receiveShadows = false;
		SeeThrough(seeThrough);
	}

	public void Destroy()
	{
		if ((Object)(object)gameObject != (Object)null)
		{
			Object.Destroy((Object)(object)gameObject);
		}
	}

	public void SeeThrough(bool state)
	{
		if (seeThrough != state)
		{
			seeThrough = state;
			((Renderer)_coneMeshRenderer).material.shader = (state ? MechJebBundlesManager.diffuseAmbientIgnoreZ : MechJebBundlesManager.diffuseAmbient);
			((Renderer)_haftMeshRenderer).material.shader = (state ? MechJebBundlesManager.diffuseAmbientIgnoreZ : MechJebBundlesManager.diffuseAmbient);
		}
	}

	public void SetLength(float length)
	{
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		if (this.length != length)
		{
			float num = length - 0.5f;
			if (num > 0f)
			{
				this.length = length;
				haft.transform.localScale = new Vector3(1f, num, 1f);
				cone.transform.localPosition = new Vector3(0f, 0f, num);
				cone.transform.localScale = new Vector3(1f, 1f, 1f);
			}
			else
			{
				this.length = length;
				haft.transform.localScale = new Vector3(1f, 0f, 1f);
				cone.transform.localPosition = new Vector3(0f, 0f, 0f);
				cone.transform.localScale = new Vector3(length / 0.5f, length / 0.5f, length / 0.5f);
			}
		}
	}

	public void Set(Vector3d position, Vector3d direction)
	{
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		if (((Vector3d)(ref direction)).sqrMagnitude < 0.001)
		{
			State(state: false);
		}
		else
		{
			Set(position, Quaternion.LookRotation(Vector3d.op_Implicit(direction)));
		}
	}

	public void Set(Vector3d position, Quaternion direction)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		gameObject.transform.position = Vector3d.op_Implicit(position);
		gameObject.transform.rotation = direction;
	}

	public void State(bool state)
	{
		if (state != gameObject.activeSelf)
		{
			gameObject.SetActive(state);
		}
	}

	private GameObject CreateCone(float height, float bottomRadius, float topRadius, float offset, int nbSides)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Expected O, but got Unknown
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00df: Unknown result type (might be due to invalid IL or missing references)
		//IL_011a: Unknown result type (might be due to invalid IL or missing references)
		//IL_011f: Unknown result type (might be due to invalid IL or missing references)
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0141: Unknown result type (might be due to invalid IL or missing references)
		//IL_0161: Unknown result type (might be due to invalid IL or missing references)
		//IL_0166: Unknown result type (might be due to invalid IL or missing references)
		//IL_0176: Unknown result type (might be due to invalid IL or missing references)
		//IL_017b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0195: Unknown result type (might be due to invalid IL or missing references)
		//IL_019a: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0201: Unknown result type (might be due to invalid IL or missing references)
		//IL_0223: Unknown result type (might be due to invalid IL or missing references)
		//IL_0228: Unknown result type (might be due to invalid IL or missing references)
		//IL_023a: Unknown result type (might be due to invalid IL or missing references)
		//IL_023f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0264: Unknown result type (might be due to invalid IL or missing references)
		//IL_0269: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0311: Unknown result type (might be due to invalid IL or missing references)
		//IL_0316: Unknown result type (might be due to invalid IL or missing references)
		//IL_0344: Unknown result type (might be due to invalid IL or missing references)
		//IL_0349: Unknown result type (might be due to invalid IL or missing references)
		//IL_034e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0360: Unknown result type (might be due to invalid IL or missing references)
		//IL_0365: Unknown result type (might be due to invalid IL or missing references)
		//IL_036a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0393: Unknown result type (might be due to invalid IL or missing references)
		//IL_0398: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_03b2: Unknown result type (might be due to invalid IL or missing references)
		cone = new GameObject();
		Mesh mesh = cone.AddComponent<MeshFilter>().mesh;
		mesh.Clear();
		int num = nbSides + 1;
		Vector3[] array = (Vector3[])(object)new Vector3[num + num + nbSides * 2 + 2];
		int i = 0;
		float num2 = (float)Math.PI * 2f;
		array[i++] = new Vector3(0f, offset, 0f);
		for (; i <= nbSides; i++)
		{
			float num3 = (float)i / (float)nbSides * num2;
			array[i] = new Vector3(Mathf.Cos(num3) * bottomRadius, offset, Mathf.Sin(num3) * bottomRadius);
		}
		array[i++] = new Vector3(0f, offset + height, 0f);
		for (; i <= nbSides * 2 + 1; i++)
		{
			float num4 = (float)(i - nbSides - 1) / (float)nbSides * num2;
			array[i] = new Vector3(Mathf.Cos(num4) * topRadius, offset + height, Mathf.Sin(num4) * topRadius);
		}
		int num5 = 0;
		while (i <= array.Length - 4)
		{
			float num6 = (float)num5 / (float)nbSides * num2;
			array[i] = new Vector3(Mathf.Cos(num6) * topRadius, offset + height, Mathf.Sin(num6) * topRadius);
			array[i + 1] = new Vector3(Mathf.Cos(num6) * bottomRadius, offset, Mathf.Sin(num6) * bottomRadius);
			i += 2;
			num5++;
		}
		array[i] = array[nbSides * 2 + 2];
		array[i + 1] = array[nbSides * 2 + 3];
		Vector3[] array2 = (Vector3[])(object)new Vector3[array.Length];
		i = 0;
		while (i <= nbSides)
		{
			array2[i++] = Vector3.down;
		}
		while (i <= nbSides * 2 + 1)
		{
			array2[i++] = Vector3.up;
		}
		num5 = 0;
		while (i <= array.Length - 4)
		{
			float num7 = (float)num5 / (float)nbSides * num2;
			float num8 = Mathf.Cos(num7);
			float num9 = Mathf.Sin(num7);
			array2[i] = new Vector3(num8, 0f, num9);
			array2[i + 1] = array2[i];
			i += 2;
			num5++;
		}
		array2[i] = array2[nbSides * 2 + 2];
		array2[i + 1] = array2[nbSides * 2 + 3];
		Vector2[] array3 = (Vector2[])(object)new Vector2[array.Length];
		int j = 0;
		array3[j++] = new Vector2(0.5f, 0.5f);
		for (; j <= nbSides; j++)
		{
			float num10 = (float)j / (float)nbSides * num2;
			array3[j] = new Vector2(Mathf.Cos(num10) * 0.5f + 0.5f, Mathf.Sin(num10) * 0.5f + 0.5f);
		}
		array3[j++] = new Vector2(0.5f, 0.5f);
		for (; j <= nbSides * 2 + 1; j++)
		{
			float num11 = (float)j / (float)nbSides * num2;
			array3[j] = new Vector2(Mathf.Cos(num11) * 0.5f + 0.5f, Mathf.Sin(num11) * 0.5f + 0.5f);
		}
		int num12 = 0;
		while (j <= array3.Length - 4)
		{
			float num13 = (float)num12 / (float)nbSides;
			array3[j] = Vector2.op_Implicit(new Vector3(num13, 1f));
			array3[j + 1] = Vector2.op_Implicit(new Vector3(num13, 0f));
			j += 2;
			num12++;
		}
		array3[j] = new Vector2(1f, 1f);
		array3[j + 1] = new Vector2(1f, 0f);
		int num14 = nbSides + nbSides + nbSides * 2;
		int[] array4 = new int[num14 * 3 + 3];
		int num15 = 0;
		int num16 = 0;
		while (num15 < nbSides - 1)
		{
			array4[num16] = 0;
			array4[num16 + 1] = num15 + 1;
			array4[num16 + 2] = num15 + 2;
			num15++;
			num16 += 3;
		}
		array4[num16] = 0;
		array4[num16 + 1] = num15 + 1;
		array4[num16 + 2] = 1;
		num15++;
		num16 += 3;
		while (num15 < nbSides * 2)
		{
			array4[num16] = num15 + 2;
			array4[num16 + 1] = num15 + 1;
			array4[num16 + 2] = num;
			num15++;
			num16 += 3;
		}
		array4[num16] = num + 1;
		array4[num16 + 1] = num15 + 1;
		array4[num16 + 2] = num;
		num15++;
		num16 += 3;
		num15++;
		while (num15 <= num14)
		{
			array4[num16] = num15 + 2;
			array4[num16 + 1] = num15 + 1;
			array4[num16 + 2] = num15;
			num15++;
			num16 += 3;
			array4[num16] = num15 + 1;
			array4[num16 + 1] = num15 + 2;
			array4[num16 + 2] = num15;
			num15++;
			num16 += 3;
		}
		mesh.vertices = array;
		mesh.normals = array2;
		mesh.uv = array3;
		mesh.triangles = array4;
		mesh.RecalculateBounds();
		return cone;
	}
}
