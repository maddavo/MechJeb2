using System;
using UnityEngine;

namespace MuMech;

public class Porkchop
{
	public static void RefreshTexture(double[,] nodes, Texture2D texture)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Expected O, but got Unknown
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_0118: Unknown result type (might be due to invalid IL or missing references)
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0289: Unknown result type (might be due to invalid IL or missing references)
		Gradient val = new Gradient();
		GradientColorKey[] array = (GradientColorKey[])(object)new GradientColorKey[6];
		array[0].color = new Color(0.25f, 0.25f, 1f);
		array[0].time = 0f;
		array[1].color = new Color(0.5f, 0.5f, 1f);
		array[1].time = 0.01f;
		array[2].color = new Color(0.5f, 1f, 1f);
		array[2].time = 0.25f;
		array[3].color = new Color(0.5f, 1f, 0.5f);
		array[3].time = 0.5f;
		array[4].color = new Color(1f, 1f, 0.5f);
		array[4].time = 0.75f;
		array[5].color = new Color(1f, 0.5f, 0.5f);
		array[5].time = 1f;
		GradientAlphaKey[] array2 = (GradientAlphaKey[])(object)new GradientAlphaKey[2];
		array2[0].alpha = 1f;
		array2[0].time = 0f;
		array2[1].alpha = 1f;
		array2[1].time = 1f;
		val.SetKeys(array, array2);
		int length = nodes.GetLength(0);
		int length2 = nodes.GetLength(1);
		double num = double.MaxValue;
		double num2 = double.MinValue;
		for (int i = 0; i < length; i++)
		{
			for (int j = 0; j < length2; j++)
			{
				if (nodes[i, j].IsFinite())
				{
					double val2 = nodes[i, j] * nodes[i, j];
					num = Math.Min(num, val2);
					num2 = Math.Max(num2, val2);
				}
			}
		}
		Debug.Log((object)("[MechJeb] porkchop scanning found DVminsqr = " + num + " DVmaxsqr = " + num2));
		double num3 = Math.Log(num);
		double num4 = Math.Min(Math.Log(num2), num3 + 4.0);
		for (int k = 0; k < length; k++)
		{
			for (int l = 0; l < length2; l++)
			{
				double num5 = (Math.Log(nodes[k, l] * nodes[k, l]) - num3) / (num4 - num3);
				texture.SetPixel(k, l, val.Evaluate((float)num5));
			}
		}
		texture.Apply();
	}
}
