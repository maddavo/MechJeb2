using UnityEngine;

namespace MuMech;

public static class ColorPickerHSV
{
	private static Texture2D _displayPicker;

	public static Color SetColor;

	private static Color _lastSetColor;

	private const int TEXTURE_WIDTH = 240;

	private const int TEXTURE_HEIGHT = 240;

	private static float _saturationSlider;

	private static float _alphaSlider;

	private static Texture2D _saturationTexture;

	private static void Init()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Expected O, but got Unknown
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Expected O, but got Unknown
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		_displayPicker = new Texture2D(240, 240, (TextureFormat)5, false);
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 240; j++)
			{
				_displayPicker.SetPixel(i, j, MuUtils.HSVtoRGB(1.5f * (float)i, 1f / (float)j * 240f, 1f, 1f));
			}
		}
		_displayPicker.Apply();
		float num = 0f;
		float num2 = 0.004166667f;
		_saturationTexture = new Texture2D(20, 240);
		for (int k = 0; k < ((Texture)_saturationTexture).width; k++)
		{
			for (int l = 0; l < ((Texture)_saturationTexture).height; l++)
			{
				_saturationTexture.SetPixel(k, l, new Color(num, num, num));
				num += num2;
			}
			num = 0f;
		}
		_saturationTexture.Apply();
	}

	public static void DrawGUI(int positionLeft, int positionTop)
	{
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0178: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		if (!Object.op_Implicit((Object)(object)_displayPicker))
		{
			Init();
		}
		GUI.Box(new Rect((float)(positionLeft - 3), (float)(positionTop - 3), 330f, 270f), "");
		if (GUI.RepeatButton(new Rect((float)positionLeft, (float)positionTop, 240f, 240f), (Texture)(object)_displayPicker))
		{
			int num = (int)Input.mousePosition.x;
			int num2 = Screen.height - (int)Input.mousePosition.y;
			SetColor = _displayPicker.GetPixel(num - positionLeft, -(num2 - positionTop));
			_lastSetColor = SetColor;
		}
		_saturationSlider = GUI.VerticalSlider(new Rect((float)(positionLeft + 240 + 3), (float)positionTop, 10f, 240f), _saturationSlider, 1f, 0f);
		SetColor = _lastSetColor + new Color(_saturationSlider, _saturationSlider, _saturationSlider);
		GUI.Box(new Rect((float)(positionLeft + 240 + 20), (float)positionTop, 20f, 240f), (Texture)(object)_saturationTexture);
		_alphaSlider = GUI.VerticalSlider(new Rect((float)(positionLeft + 240 + 3 + 10 + 20 + 10), (float)positionTop, 10f, 240f), _alphaSlider, 1f, 0f);
		SetColor.a = _alphaSlider;
		GUI.Box(new Rect((float)(positionLeft + 240 + 20 + 10 + 20 + 10), (float)positionTop, 20f, 240f), (Texture)(object)_saturationTexture);
	}
}
