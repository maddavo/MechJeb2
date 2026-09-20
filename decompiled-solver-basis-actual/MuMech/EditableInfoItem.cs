using System.Reflection;
using UnityEngine;

namespace MuMech;

public class EditableInfoItem : InfoItem
{
	public readonly string rightLabel;

	public readonly float width;

	public readonly bool expandWidth;

	private readonly IEditable val;

	private readonly GUIContent _labelContent;

	public EditableInfoItem(object obj, MemberInfo member, EditableInfoItemAttribute attribute)
		: base(attribute)
	{
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Expected O, but got Unknown
		id = GetType().Name.Replace("InfoItem", "") + ":" + obj.GetType().Name.Replace("MechJebModule", "") + "." + member.Name;
		rightLabel = attribute.rightLabel;
		width = attribute.width;
		expandWidth = attribute.expandWidth;
		_labelContent = (string.IsNullOrEmpty(localizedTooltip) ? new GUIContent(localizedName) : new GUIContent(localizedName, localizedTooltip));
		if (member is FieldInfo)
		{
			val = (IEditable)((FieldInfo)member).GetValue(obj);
		}
		else if (member is PropertyInfo)
		{
			val = (IEditable)((PropertyInfo)member).GetValue(obj, new object[0]);
		}
	}

	public override void DrawItem()
	{
		if (val != null)
		{
			GuiUtils.SimpleTextBox(_labelContent, val, rightLabel, width, null, horizontalFraming: true, expandWidth);
		}
	}
}
