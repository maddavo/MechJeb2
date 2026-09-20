using System.Collections;
using UnityEngine;

namespace MuMech;

[KSPAddon(/*Could not decode attribute arguments.*/)]
public class MechJebBundlesManager : MonoBehaviour
{
	private const string shaderBundle = "shaders.bundle";

	private string shaderPath;

	private const string diffuseAmbientName = "Assets/Shaders/MJ_DiffuseAmbiant.shader";

	private const string diffuseAmbientIgnoreZName = "Assets/Shaders/MJ_DiffuseAmbiantIgnoreZ.shader";

	public static Shader diffuseAmbient;

	public static Shader diffuseAmbientIgnoreZ;

	public static Texture2D comboBoxBackground;

	private void Awake()
	{
		string text = KSPUtil.ApplicationRootPath + "/GameData/MechJeb2/Bundles/";
		shaderPath = text + "shaders.bundle";
	}

	private IEnumerator Start()
	{
		if (Object.op_Implicit((Object)(object)diffuseAmbient))
		{
			MechJebCore.Print("Shaders already loaded");
		}
		MechJebCore.Print("Loading Shaders Bundles");
		AssetBundleCreateRequest bundleLoadRequest = AssetBundle.LoadFromFileAsync(shaderPath);
		yield return bundleLoadRequest;
		AssetBundle assetBundle = bundleLoadRequest.assetBundle;
		if ((Object)(object)assetBundle == (Object)null)
		{
			MechJebCore.Print("Failed to load AssetBundle " + shaderPath);
			yield break;
		}
		AssetBundleRequest assetLoadRequest2 = assetBundle.LoadAssetAsync<Shader>("Assets/Shaders/MJ_DiffuseAmbiant.shader");
		yield return assetLoadRequest2;
		Object asset = assetLoadRequest2.asset;
		diffuseAmbient = (Shader)(object)((asset is Shader) ? asset : null);
		assetLoadRequest2 = assetBundle.LoadAssetAsync<Shader>("Assets/Shaders/MJ_DiffuseAmbiantIgnoreZ.shader");
		yield return assetLoadRequest2;
		Object asset2 = assetLoadRequest2.asset;
		diffuseAmbientIgnoreZ = (Shader)(object)((asset2 is Shader) ? asset2 : null);
		assetBundle.Unload(false);
		MechJebCore.Print("Loaded Shaders Bundles");
		comboBoxBackground = new Texture2D(16, 16, (TextureFormat)4, false);
		((Texture)comboBoxBackground).wrapMode = (TextureWrapMode)1;
		for (int i = 0; i < ((Texture)comboBoxBackground).width; i++)
		{
			for (int j = 0; j < ((Texture)comboBoxBackground).height; j++)
			{
				if (i == 0 || i == ((Texture)comboBoxBackground).width - 1 || j == 0 || j == ((Texture)comboBoxBackground).height - 1)
				{
					comboBoxBackground.SetPixel(i, j, new Color(0f, 0f, 0f, 1f));
				}
				else
				{
					comboBoxBackground.SetPixel(i, j, new Color(0.05f, 0.05f, 0.05f, 0.95f));
				}
			}
		}
		comboBoxBackground.Apply();
	}
}
