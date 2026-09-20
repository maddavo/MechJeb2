using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;

namespace MuMech;

[KSPAddon(/*Could not decode attribute arguments.*/)]
internal class CompatibilityChecker : MonoBehaviour
{
	private static int _version = 5;

	public static bool IsCompatible()
	{
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_0076: Unknown result type (might be due to invalid IL or missing references)
		if (Versioning.version_major == 1 && Versioning.version_minor >= 8)
		{
			foreach (LoadedAssembly loadedAssembly in AssemblyLoader.loadedAssemblies)
			{
				AssemblyName name = loadedAssembly.assembly.GetName();
				if (name.Name == "Firespitter" && name.Version <= Version.Parse("7.3.7175.38653"))
				{
					PopupDialog.SpawnPopupDialog(new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), "MJBobNeedsToFixStuff", "Outdated Firespitter version detected", "You are using a version of Firespitter that does not run properly on KSP 1.8+\nThis version may prevent the game from loading properly and may create problems for other mods", "OK", true, HighLogic.UISkin, true, "");
				}
			}
		}
		if (Versioning.version_major == 1)
		{
			return Versioning.version_minor == 12;
		}
		return false;
	}

	public static bool IsUnityCompatible()
	{
		return true;
	}

	public void Start()
	{
		//IL_038d: Unknown result type (might be due to invalid IL or missing references)
		//IL_039c: Unknown result type (might be due to invalid IL or missing references)
		FieldInfo[] source = (from t in getAllTypes()
			where t.Name == "CompatibilityChecker"
			select t.GetField("_version", BindingFlags.Static | BindingFlags.NonPublic) into f
			where f != null
			where f.FieldType == typeof(int)
			select f).ToArray();
		if (_version != source.Max((FieldInfo f) => (int)f.GetValue(null)))
		{
			return;
		}
		Debug.Log((object)$"[CompatibilityChecker] Running checker version {_version} from '{Assembly.GetExecutingAssembly().GetName().Name}'");
		_version = int.MaxValue;
		string[] array = (from m in (from f in source
				select f.DeclaringType.GetMethod("IsCompatible", Type.EmptyTypes) into m
				where m.IsStatic
				where m.ReturnType == typeof(bool)
				select m).Where(delegate(MethodInfo m)
			{
				try
				{
					return !(bool)m.Invoke(null, new object[0]);
				}
				catch (Exception arg2)
				{
					Debug.LogWarning((object)$"[CompatibilityChecker] Exception while invoking IsCompatible() from '{m.DeclaringType.Assembly.GetName().Name}':\n\n{arg2}");
					return true;
				}
			})
			select m.DeclaringType.Assembly.GetName().Name).ToArray();
		string[] array2 = (from m in (from f in source
				select f.DeclaringType.GetMethod("IsUnityCompatible", Type.EmptyTypes) into m
				where m != null
				where m.IsStatic
				where m.ReturnType == typeof(bool)
				select m).Where(delegate(MethodInfo m)
			{
				try
				{
					return !(bool)m.Invoke(null, new object[0]);
				}
				catch (Exception arg)
				{
					Debug.LogWarning((object)$"[CompatibilityChecker] Exception while invoking IsUnityCompatible() from '{m.DeclaringType.Assembly.GetName().Name}':\n\n{arg}");
					return true;
				}
			})
			select m.DeclaringType.Assembly.GetName().Name).ToArray();
		Array.Sort(array);
		Array.Sort(array2);
		string text = string.Empty;
		if (array.Length != 0 || array2.Length != 0)
		{
			text = text + ((text == string.Empty) ? "Some" : "\n\nAdditionally, some") + " installed mods may be incompatible with this version of Kerbal Space Program. Features may be broken or disabled. Please check for updates to the listed mods.";
			if (array.Length != 0)
			{
				Debug.LogWarning((object)("[CompatibilityChecker] Incompatible mods detected: " + string.Join(", ", array)));
				text += $"\n\nThese mods are incompatible with KSP {Versioning.version_major}.{Versioning.version_minor}.{Versioning.Revision}:\n\n";
				text += string.Join("\n", array);
			}
			if (array2.Length != 0)
			{
				Debug.LogWarning((object)("[CompatibilityChecker] Incompatible mods (Unity) detected: " + string.Join(", ", array2)));
				text = text + "\n\nThese mods are incompatible with Unity " + Application.unityVersion + ":\n\n";
				text += string.Join("\n", array2);
			}
		}
		if (array.Length != 0 || array2.Length != 0)
		{
			PopupDialog.SpawnPopupDialog(new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), "CompatibilityChecker", "Incompatible Mods Detected", text, "OK", true, HighLogic.UISkin, true, "");
		}
	}

	public static bool IsWin64()
	{
		if (IntPtr.Size == 8)
		{
			return Environment.OSVersion.Platform == PlatformID.Win32NT;
		}
		return false;
	}

	public static bool IsAllCompatible()
	{
		if (IsCompatible())
		{
			return IsUnityCompatible();
		}
		return false;
	}

	private static IEnumerable<Type> getAllTypes()
	{
		Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();
		foreach (Assembly assembly in assemblies)
		{
			Type[] array;
			try
			{
				array = assembly.GetTypes();
			}
			catch (Exception)
			{
				array = Type.EmptyTypes;
			}
			Type[] array2 = array;
			for (int j = 0; j < array2.Length; j++)
			{
				yield return array2[j];
			}
		}
	}
}
