namespace MuMech;

public interface IDescentSpeedPolicy
{
	double MaxAllowedSpeed(Vector3d pos, Vector3d surfaceVel);
}
