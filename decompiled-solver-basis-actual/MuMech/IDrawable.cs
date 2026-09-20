using UnityEngine;

namespace MuMech;

public interface IDrawable
{
	void Update();

	Vector2 Draw(Vector2 position);
}
