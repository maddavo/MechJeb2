using System;
using System.Reflection;
using UnityEngine;

namespace MuMech;

public class ToggleInfoItem : InfoItem
{
	private readonly object obj;

	private readonly MemberInfo member;

	private readonly GUIContent _labelContent;

	public ToggleInfoItem(object obj, MemberInfo member, ToggleInfoItemAttribute attribute)
		: base(attribute)
	{
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_00aa: Expected O, but got Unknown
		id = GetType().Name.Replace("InfoItem", "") + ":" + obj.GetType().Name.Replace("MechJebModule", "") + "." + member.Name;
		this.obj = obj;
		this.member = member;
		_labelContent = (string.IsNullOrEmpty(localizedTooltip) ? new GUIContent(localizedName) : new GUIContent(localizedName, localizedTooltip));
	}

	public override void DrawItem()
	{
		bool flag = false;
		if (member is FieldInfo)
		{
			flag = (bool)((FieldInfo)member).GetValue(obj);
		}
		else if (member is PropertyInfo)
		{
			flag = (bool)((PropertyInfo)member).GetValue(obj, new object[0]);
		}
		bool flag2 = GUILayout.Toggle(flag, _labelContent, Array.Empty<GUILayoutOption>());
		if (flag2 != flag)
		{
			if (member is FieldInfo)
			{
				((FieldInfo)member).SetValue(obj, flag2);
			}
			else if (member is PropertyInfo)
			{
				((PropertyInfo)member).SetValue(obj, flag2, new object[0]);
			}
		}
	}
}
