using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using KSP.Localization;
using UnityEngine;
using UnityEngine.UI;

namespace MuMech;

[KSPAddon(/*Could not decode attribute arguments.*/)]
internal class InstallChecker : MonoBehaviour
{
	[Serializable]
	[CompilerGenerated]
	private sealed class _003C_003Ec
	{
		public static readonly _003C_003Ec _003C_003E9 = new _003C_003Ec();

		public static Func<LoadedAssembly, bool> _003C_003E9__0_0;

		public static Func<LoadedAssembly, bool> _003C_003E9__0_1;

		public static Func<LoadedAssembly, string> _003C_003E9__0_3;

		public static Func<string, string> _003C_003E9__0_4;

		public static Callback _003C_003E9__0_5;

		public static Func<LoadedAssembly, bool> _003C_003E9__0_2;

		public static Func<LoadedAssembly, string> _003C_003E9__0_6;

		public static Func<string, string> _003C_003E9__0_7;

		internal bool _003CStart_003Eb__0_0(LoadedAssembly a)
		{
			return a.assembly.GetName().Name == Assembly.GetExecutingAssembly().GetName().Name;
		}

		internal bool _003CStart_003Eb__0_1(LoadedAssembly a)
		{
			return a.url != "MechJeb2/Plugins";
		}

		internal string _003CStart_003Eb__0_3(LoadedAssembly a)
		{
			return a.path;
		}

		internal string _003CStart_003Eb__0_4(string p)
		{
			return Uri.UnescapeDataString(new Uri(Path.GetFullPath(KSPUtil.ApplicationRootPath)).MakeRelativeUri(new Uri(p)).ToString().Replace('/', Path.DirectorySeparatorChar));
		}

		internal void _003CStart_003Eb__0_5()
		{
		}

		internal bool _003CStart_003Eb__0_2(LoadedAssembly a)
		{
			return a.assembly.GetName().Name == "MechJebMenuToolbar";
		}

		internal string _003CStart_003Eb__0_6(LoadedAssembly a)
		{
			return a.path;
		}

		internal string _003CStart_003Eb__0_7(string p)
		{
			return Uri.UnescapeDataString(new Uri(Path.GetFullPath(KSPUtil.ApplicationRootPath)).MakeRelativeUri(new Uri(p)).ToString().Replace('/', Path.DirectorySeparatorChar));
		}
	}

	protected void Start()
	{
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Expected O, but got Unknown
		//IL_010a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0110: Expected O, but got Unknown
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_013d: Expected O, but got Unknown
		//IL_013d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0153: Expected O, but got Unknown
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0130: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Expected O, but got Unknown
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f1: Unknown result type (might be due to invalid IL or missing references)
		List<LoadedAssembly> source = (from a in (IEnumerable<LoadedAssembly>)AssemblyLoader.loadedAssemblies
			where a.assembly.GetName().Name == Assembly.GetExecutingAssembly().GetName().Name
			where a.url != "MechJeb2/Plugins"
			select a).ToList();
		if (source.Any())
		{
			IEnumerable<string> source2 = from a in source
				select a.path into p
				select Uri.UnescapeDataString(new Uri(Path.GetFullPath(KSPUtil.ApplicationRootPath)).MakeRelativeUri(new Uri(p)).ToString().Replace('/', Path.DirectorySeparatorChar));
			string text = Localizer.Format("#MechJeb_InstallCheckA_title");
			UISkinDef uISkin = HighLogic.UISkin;
			Rect val = new Rect(0.5f, 0.5f, 100f, 100f);
			DialogGUIBase[] obj = new DialogGUIBase[3]
			{
				(DialogGUIBase)new DialogGUIContentSizer((FitMode)2, (FitMode)1, false),
				(DialogGUIBase)new DialogGUILabel(Localizer.Format("#MechJeb_InstallCheckA_msg") + string.Join("\n", source2.ToArray()), false, false),
				default(DialogGUIBase)
			};
			object obj2 = _003C_003Ec._003C_003E9__0_5;
			if (obj2 == null)
			{
				Callback val2 = delegate
				{
				};
				_003C_003Ec._003C_003E9__0_5 = val2;
				obj2 = (object)val2;
			}
			obj[2] = (DialogGUIBase)new DialogGUIButton("OK", (Callback)obj2, true);
			PopupDialog.SpawnPopupDialog(new MultiOptionDialog("InstallCheckerA", (string)null, text, uISkin, val, (DialogGUIBase[])(object)obj), false, HighLogic.UISkin, true, "");
		}
		source = ((IEnumerable<LoadedAssembly>)AssemblyLoader.loadedAssemblies).Where((LoadedAssembly a) => a.assembly.GetName().Name == "MechJebMenuToolbar").ToList();
		if (source.Any())
		{
			IEnumerable<string> source3 = from a in source
				select a.path into p
				select Uri.UnescapeDataString(new Uri(Path.GetFullPath(KSPUtil.ApplicationRootPath)).MakeRelativeUri(new Uri(p)).ToString().Replace('/', Path.DirectorySeparatorChar));
			PopupDialog.SpawnPopupDialog(new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), "InstallCheckerB", Localizer.Format("#MechJeb_InstallCheckB_title"), Localizer.Format("#MechJeb_InstallCheckB_msg") + string.Join("\n", source3.ToArray()), "OK", false, HighLogic.UISkin, true, "");
		}
	}
}
