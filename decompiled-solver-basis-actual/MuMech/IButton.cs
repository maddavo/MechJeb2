using UnityEngine;

namespace MuMech;

public interface IButton
{
	string Text { get; set; }

	Color TextColor { get; set; }

	string TexturePath { get; set; }

	string BigTexturePath { get; set; }

	string ToolTip { get; set; }

	bool Visible { get; set; }

	IVisibility Visibility { get; set; }

	bool EffectivelyVisible { get; }

	bool IsHovering { get; }

	bool Enabled { get; set; }

	bool Important { get; set; }

	IDrawable Drawable { get; set; }

	event ClickHandler OnClick;

	event MouseEnterHandler OnMouseEnter;

	event MouseLeaveHandler OnMouseLeave;

	void Destroy();
}
