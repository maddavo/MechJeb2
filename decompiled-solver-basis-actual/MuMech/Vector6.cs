using System;

namespace MuMech;

public class Vector6 : IConfigNode
{
	public enum Direction
	{
		FORWARD,
		BACK,
		UP,
		DOWN,
		RIGHT,
		LEFT
	}

	public Vector3d Positive = Vector3d.zero;

	public Vector3d Negative = Vector3d.zero;

	public static readonly Vector3d[] Directions = (Vector3d[])(object)new Vector3d[6]
	{
		Vector3d.forward,
		Vector3d.back,
		Vector3d.up,
		Vector3d.down,
		Vector3d.right,
		Vector3d.left
	};

	public static readonly Direction[] Values = (Direction[])Enum.GetValues(typeof(Direction));

	public double Forward
	{
		get
		{
			return Positive.z;
		}
		set
		{
			Positive.z = value;
		}
	}

	public double Back
	{
		get
		{
			return Negative.z;
		}
		set
		{
			Negative.z = value;
		}
	}

	public double Up
	{
		get
		{
			return Positive.y;
		}
		set
		{
			Positive.y = value;
		}
	}

	public double Down
	{
		get
		{
			return Negative.y;
		}
		set
		{
			Negative.y = value;
		}
	}

	public double Right
	{
		get
		{
			return Positive.x;
		}
		set
		{
			Positive.x = value;
		}
	}

	public double Left
	{
		get
		{
			return Negative.x;
		}
		set
		{
			Negative.x = value;
		}
	}

	public double this[Direction index]
	{
		get
		{
			return index switch
			{
				Direction.FORWARD => Forward, 
				Direction.BACK => Back, 
				Direction.UP => Up, 
				Direction.DOWN => Down, 
				Direction.RIGHT => Right, 
				Direction.LEFT => Left, 
				_ => 0.0, 
			};
		}
		set
		{
			switch (index)
			{
			case Direction.FORWARD:
				Forward = value;
				break;
			case Direction.BACK:
				Back = value;
				break;
			case Direction.UP:
				Up = value;
				break;
			case Direction.DOWN:
				Down = value;
				break;
			case Direction.RIGHT:
				Right = value;
				break;
			case Direction.LEFT:
				Left = value;
				break;
			}
		}
	}

	public Vector6()
	{
	}//IL_0001: Unknown result type (might be due to invalid IL or missing references)
	//IL_0006: Unknown result type (might be due to invalid IL or missing references)
	//IL_000c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0011: Unknown result type (might be due to invalid IL or missing references)


	public Vector6(Vector3d positive, Vector3d negative)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		Positive = positive;
		Negative = negative;
	}

	public void Reset()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		Positive = Vector3d.zero;
		Negative = Vector3d.zero;
	}

	public void Add(Vector3d vector)
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		for (int i = 0; i < Values.Length; i++)
		{
			Direction direction = Values[i];
			double num = Vector3d.Dot(vector, Directions[(int)direction]);
			if (num > 0.0)
			{
				this[direction] += num;
			}
		}
	}

	public double GetMagnitude(Vector3d direction)
	{
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		double num = 0.0;
		for (int i = 0; i < Values.Length; i++)
		{
			Direction direction2 = Values[i];
			double num2 = Vector3d.Dot(((Vector3d)(ref direction)).normalized, Directions[(int)direction2]);
			if (num2 > 0.0)
			{
				num += Math.Pow(num2 * this[direction2], 2.0);
			}
		}
		return Math.Sqrt(num);
	}

	public double MaxMagnitude()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		return Math.Max(Positive.MaxMagnitude(), Negative.MaxMagnitude());
	}

	public void Load(ConfigNode node)
	{
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		if (node.HasValue("positive"))
		{
			Positive = KSPUtil.ParseVector3d(node.GetValue("positive"));
		}
		if (node.HasValue("negative"))
		{
			Negative = KSPUtil.ParseVector3d(node.GetValue("negative"));
		}
	}

	public void Save(ConfigNode node)
	{
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		node.SetValue("positive", KSPUtil.WriteVector(Positive), false);
		node.SetValue("negative", KSPUtil.WriteVector(Negative), false);
	}
}
