using System;
using System.Reflection;
using UnityEngine;

namespace MuMech;

public class ActionInfoItem : InfoItem
{
	private readonly Action action;

	private readonly GUIContent _labelContent;

	public ActionInfoItem(object obj, MethodInfo method, ActionInfoItemAttribute attribute)
		: base(attribute)
	{
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Expected O, but got Unknown
		id = GetType().Name.Replace("InfoItem", "") + ":" + obj.GetType().Name.Replace("MechJebModule", "") + "." + method.Name;
		action = (Action)Delegate.CreateDelegate(typeof(Action), obj, method);
		_labelContent = (string.IsNullOrEmpty(localizedTooltip) ? new GUIContent(localizedName) : new GUIContent(localizedName, localizedTooltip));
	}

	public override void DrawItem()
	{
		if (GUILayout.Button(_labelContent, Array.Empty<GUILayoutOption>()))
		{
			action();
		}
	}
}
