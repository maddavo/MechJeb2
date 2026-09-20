using UnityEngine;

namespace MuMech;

public static class ColorPickerRGB
{
	private const int TEXTURE_WIDTH = 240;

	private const int TEXTURE_HEIGHT = 10;

	private static Texture2D _rTexture;

	private static Texture2D _gTexture;

	private static Texture2D _bTexture;

	private static Texture2D _aTexture;

	private static void Init()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Expected O, but got Unknown
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Expected O, but got Unknown
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Expected O, but got Unknown
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Expected O, but got Unknown
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		_rTexture = new Texture2D(240, 1);
		_gTexture = new Texture2D(240, 1);
		_bTexture = new Texture2D(240, 1);
		_aTexture = new Texture2D(240, 1);
		for (int i = 0; i < 240; i++)
		{
			float num = (float)i / 239f;
			_rTexture.SetPixel(i, 0, new Color(num, 0f, 0f));
			_gTexture.SetPixel(i, 0, new Color(0f, num, 0f));
			_bTexture.SetPixel(i, 0, new Color(0f, 0f, num));
			_aTexture.SetPixel(i, 0, new Color(num, num, num));
		}
		_rTexture.Apply();
		_gTexture.Apply();
		_bTexture.Apply();
		_aTexture.Apply();
		((Texture)_rTexture).wrapMode = (TextureWrapMode)0;
		((Texture)_gTexture).wrapMode = (TextureWrapMode)0;
		((Texture)_bTexture).wrapMode = (TextureWrapMode)0;
		((Texture)_aTexture).wrapMode = (TextureWrapMode)0;
	}

	public static Color DrawGUI(int positionLeft, int positionTop, Color c)
	{
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_0087: Unknown result type (might be due to invalid IL or missing references)
		//IL_008c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0132: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_0175: Unknown result type (might be due to invalid IL or missing references)
		//IL_017a: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_020b: Unknown result type (might be due to invalid IL or missing references)
		if (!Object.op_Implicit((Object)(object)_rTexture))
		{
			Init();
		}
		GUI.Box(new Rect((float)(positionLeft - 3), (float)(positionTop - 3), 243f, 135f), "");
		float num = positionTop + 5;
		GUI.DrawTextureWithTexCoords(new Rect((float)positionLeft, num, 240f, 10f), (Texture)(object)_rTexture, new Rect(0f, 0f, 1f, 10f));
		c.r = GUI.HorizontalSlider(new Rect((float)positionLeft, num + 10f + 5f, 240f, 10f), c.r, 0f, 1f);
		num += 30f;
		GUI.DrawTextureWithTexCoords(new Rect((float)positionLeft, num, 240f, 10f), (Texture)(object)_gTexture, new Rect(0f, 0f, 1f, 10f));
		c.g = GUI.HorizontalSlider(new Rect((float)positionLeft, num + 10f + 5f, 240f, 10f), c.g, 0f, 1f);
		num += 30f;
		GUI.DrawTextureWithTexCoords(new Rect((float)positionLeft, num, 240f, 10f), (Texture)(object)_bTexture, new Rect(0f, 0f, 1f, 10f));
		c.b = GUI.HorizontalSlider(new Rect((float)positionLeft, num + 10f + 5f, 240f, 10f), c.b, 0f, 1f);
		num += 30f;
		GUI.DrawTextureWithTexCoords(new Rect((float)positionLeft, num, 240f, 10f), (Texture)(object)_aTexture, new Rect(0f, 0f, 1f, 10f));
		c.a = GUI.HorizontalSlider(new Rect((float)positionLeft, num + 10f + 5f, 240f, 10f), c.a, 0f, 1f);
		return c;
	}
}
