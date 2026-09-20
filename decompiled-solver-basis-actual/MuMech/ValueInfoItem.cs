using System;
using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Emit;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class ValueInfoItem : InfoItem
{
	private readonly string units;

	private readonly string format;

	private readonly float width;

	public const string SI = "SI";

	public const string TIME = "TIME";

	public const string ANGLE = "ANGLE";

	public const string ANGLE_NS = "ANGLE_NS";

	public const string ANGLE_EW = "ANGLE_EW";

	private readonly int siSigFigs;

	private readonly int siMaxPrecision;

	private readonly int timeDecimalPlaces;

	private readonly Func<object, object> getValue;

	private static readonly Dictionary<MemberInfo, Func<object, object>> _getterCache = new Dictionary<MemberInfo, Func<object, object>>();

	private readonly object _obj;

	private string stringValue;

	private int cacheValidity = -1;

	public bool externalRefresh;

	private readonly GUILayoutOption[] _widthOption;

	private readonly GUIContent _labelContent;

	public ValueInfoItem(object obj, MemberInfo member, ValueInfoItemAttribute attribute)
		: base(attribute)
	{
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Expected O, but got Unknown
		id = GetType().Name.Replace("InfoItem", "") + ":" + obj.GetType().Name.Replace("MechJebModule", "") + "." + member.Name;
		units = attribute.units;
		format = attribute.format;
		siSigFigs = attribute.siSigFigs;
		siMaxPrecision = attribute.siMaxPrecision;
		timeDecimalPlaces = attribute.timeDecimalPlaces;
		width = attribute.width;
		_labelContent = (string.IsNullOrEmpty(localizedTooltip) ? new GUIContent(localizedName) : new GUIContent(localizedName, localizedTooltip));
		_obj = obj;
		if (!_getterCache.ContainsKey(member))
		{
			_getterCache.Add(member, CompileAccessor(obj, member));
		}
		getValue = _getterCache[member];
		_widthOption = (GUILayoutOption[])(object)new GUILayoutOption[1] { (width > 0f) ? GuiUtils.LayoutWidth(width) : GuiUtils.LayoutNoExpandWidth };
	}

	private Func<object, object> CompileAccessor(object obj, MemberInfo member)
	{
		Type type = obj.GetType();
		DynamicMethod dynamicMethod = new DynamicMethod("GetMemberValue", typeof(object), new Type[1] { typeof(object) }, type, skipVisibility: true);
		ILGenerator iLGenerator = dynamicMethod.GetILGenerator();
		iLGenerator.Emit(OpCodes.Ldarg_0);
		iLGenerator.Emit(OpCodes.Castclass, type);
		if (!(member is MethodInfo meth))
		{
			if (!(member is PropertyInfo propertyInfo))
			{
				if (!(member is FieldInfo field))
				{
					throw new ArgumentException("MemberInfo must be of type MethodInfo, PropertyInfo, or FieldInfo", "member");
				}
				iLGenerator.Emit(OpCodes.Ldfld, field);
			}
			else
			{
				iLGenerator.Emit(OpCodes.Callvirt, propertyInfo.GetGetMethod());
			}
		}
		else
		{
			iLGenerator.Emit(OpCodes.Callvirt, meth);
		}
		if (member is PropertyInfo { PropertyType: { IsValueType: not false } } || member is FieldInfo { FieldType: { IsValueType: not false } } || member is MethodInfo { ReturnType: { IsValueType: not false } })
		{
			iLGenerator.Emit(cls: (member is PropertyInfo propertyInfo3) ? propertyInfo3.PropertyType : ((!(member is FieldInfo fieldInfo2)) ? ((MethodInfo)member).ReturnType : fieldInfo2.FieldType), opcode: OpCodes.Box);
		}
		iLGenerator.Emit(OpCodes.Ret);
		return (Func<object, object>)dynamicMethod.CreateDelegate(typeof(Func<object, object>));
	}

	private string GetStringValue(object value)
	{
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		if (value == null)
		{
			return "null";
		}
		if (value is string)
		{
			return (string)value + " " + units;
		}
		if (value is int)
		{
			return $"{(int)value} {units}";
		}
		double num = -999.0;
		if (value is double)
		{
			num = (double)value;
		}
		else if (value is float)
		{
			num = (float)value;
		}
		else if (value is MovingAverage)
		{
			num = (MovingAverage)value;
		}
		else if (value is Vector3d val)
		{
			num = ((Vector3d)(ref val)).magnitude;
		}
		else if (value is Vector3 val2)
		{
			num = ((Vector3)(ref val2)).magnitude;
		}
		else if (value is EditableDouble)
		{
			num = (EditableDouble)value;
		}
		if (format == "TIME")
		{
			return GuiUtils.TimeToDHMS(num, timeDecimalPlaces);
		}
		if (format == "ANGLE")
		{
			return Coordinates.AngleToDMS(num);
		}
		if (format == "ANGLE_NS")
		{
			return Coordinates.AngleToDMS(num) + ((num > 0.0) ? " N" : " S");
		}
		if (format == "ANGLE_EW")
		{
			return Coordinates.AngleToDMS(num) + ((num > 0.0) ? " E" : " W");
		}
		if (format == "SI")
		{
			return Statics.ToSI(num, siSigFigs, siMaxPrecision) + units;
		}
		return num.ToString(format) + " " + units;
	}

	private void UpdateItemCache()
	{
		int frameCount = Time.frameCount;
		if (frameCount != cacheValidity)
		{
			object value = getValue(_obj);
			stringValue = Localizer.Format(GetStringValue(value));
			cacheValidity = frameCount;
		}
	}

	public override void UpdateItem()
	{
		externalRefresh = true;
		cacheValidity = 0;
		UpdateItemCache();
	}

	public override void DrawItem()
	{
		if (!externalRefresh)
		{
			UpdateItemCache();
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(_labelContent, Array.Empty<GUILayoutOption>());
		GUILayout.Label(stringValue, _widthOption);
		GUILayout.EndHorizontal();
	}
}
