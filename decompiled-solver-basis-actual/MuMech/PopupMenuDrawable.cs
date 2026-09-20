using System;
using System.Reflection;
using UnityEngine;

namespace MuMech;

public class PopupMenuDrawable : IDrawable
{
	private readonly object realPopupMenuDrawable;

	private readonly MethodInfo updateMethod;

	private readonly MethodInfo drawMethod;

	private readonly MethodInfo addOptionMethod;

	private readonly MethodInfo addSeparatorMethod;

	private readonly MethodInfo destroyMethod;

	private readonly EventInfo onAnyOptionClickedEvent;

	public event Action OnAnyOptionClicked
	{
		add
		{
			onAnyOptionClickedEvent.AddEventHandler(realPopupMenuDrawable, value);
		}
		remove
		{
			onAnyOptionClickedEvent.RemoveEventHandler(realPopupMenuDrawable, value);
		}
	}

	public PopupMenuDrawable()
	{
		Type type = ToolbarTypes.getType("Toolbar.PopupMenuDrawable");
		realPopupMenuDrawable = Activator.CreateInstance(type, null);
		updateMethod = ToolbarTypes.getMethod(type, "Update");
		drawMethod = ToolbarTypes.getMethod(type, "Draw");
		addOptionMethod = ToolbarTypes.getMethod(type, "AddOption");
		addSeparatorMethod = ToolbarTypes.getMethod(type, "AddSeparator");
		destroyMethod = ToolbarTypes.getMethod(type, "Destroy");
		onAnyOptionClickedEvent = ToolbarTypes.getEvent(type, "OnAnyOptionClicked");
	}

	public void Update()
	{
		updateMethod.Invoke(realPopupMenuDrawable, null);
	}

	public Vector2 Draw(Vector2 position)
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		return (Vector2)drawMethod.Invoke(realPopupMenuDrawable, new object[1] { position });
	}

	public IButton AddOption(string text)
	{
		return new Button(addOptionMethod.Invoke(realPopupMenuDrawable, new object[1] { text }), new ToolbarTypes());
	}

	public void AddSeparator()
	{
		addSeparatorMethod.Invoke(realPopupMenuDrawable, null);
	}

	public void Destroy()
	{
		destroyMethod.Invoke(realPopupMenuDrawable, null);
	}
}
