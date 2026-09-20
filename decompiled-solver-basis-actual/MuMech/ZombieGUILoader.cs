using UnityEngine;

namespace MuMech;

public class ZombieGUILoader : MonoBehaviour
{
	private void OnGUI()
	{
		GuiUtils.CopyDefaultSkin();
		if ((Object)(object)GuiUtils.Skin == (Object)null)
		{
			GuiUtils.Skin = GuiUtils.DefaultSkin;
		}
		Object.Destroy((Object)(object)((Component)this).gameObject);
	}
}
