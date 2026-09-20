using System;
using System.Reflection;

namespace MuMech;

public class GameScenesVisibility : IVisibility
{
	private readonly object realGameScenesVisibility;

	private readonly PropertyInfo visibleProperty;

	public bool Visible => (bool)visibleProperty.GetValue(realGameScenesVisibility, null);

	public GameScenesVisibility(params GameScenes[] gameScenes)
	{
		Type type = ToolbarTypes.getType("Toolbar.GameScenesVisibility");
		realGameScenesVisibility = Activator.CreateInstance(type, gameScenes);
		visibleProperty = ToolbarTypes.getProperty(type, "Visible");
	}
}
