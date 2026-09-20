using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech;

public class RCSSolver
{
	public class Thruster
	{
		public readonly Part Part;

		public readonly ModuleRCS PartModule;

		public readonly float OriginalForce;

		private readonly Vector3 _pos;

		private readonly Vector3[] _thrustDirections;

		public Thruster(Vector3 pos, Vector3[] thrustDirections, Part p, ModuleRCS pm)
		{
			//IL_0007: Unknown result type (might be due to invalid IL or missing references)
			//IL_0008: Unknown result type (might be due to invalid IL or missing references)
			_pos = pos;
			_thrustDirections = thrustDirections;
			OriginalForce = pm.thrusterPower;
			Part = p;
			PartModule = pm;
		}

		public void RestoreOriginalForce()
		{
			PartModule.thrusterPower = OriginalForce;
		}

		public Vector3 GetThrust(Vector3 direction, Vector3 rotation)
		{
			//IL_0000: Unknown result type (might be due to invalid IL or missing references)
			//IL_0005: Unknown result type (might be due to invalid IL or missing references)
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001a: Unknown result type (might be due to invalid IL or missing references)
			//IL_001f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0020: Unknown result type (might be due to invalid IL or missing references)
			//IL_0025: Unknown result type (might be due to invalid IL or missing references)
			//IL_002a: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_002d: Unknown result type (might be due to invalid IL or missing references)
			//IL_0033: Unknown result type (might be due to invalid IL or missing references)
			//IL_0034: Unknown result type (might be due to invalid IL or missing references)
			//IL_0047: Unknown result type (might be due to invalid IL or missing references)
			//IL_0048: Unknown result type (might be due to invalid IL or missing references)
			//IL_004b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0050: Unknown result type (might be due to invalid IL or missing references)
			//IL_0055: Unknown result type (might be due to invalid IL or missing references)
			//IL_0060: Unknown result type (might be due to invalid IL or missing references)
			Vector3 val = Vector3.zero;
			Vector3[] thrustDirections = _thrustDirections;
			foreach (Vector3 val2 in thrustDirections)
			{
				Vector3 val3 = -Vector3.Cross(_pos, val2);
				float num = Vector3.Dot(direction, val2);
				float num2 = Vector3.Dot(rotation, val3);
				float num3 = Mathf.Clamp01(num + num2);
				val += val2 * num3;
			}
			return val;
		}

		public Vector3 GetTorque(Vector3 thrust)
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_0006: Unknown result type (might be due to invalid IL or missing references)
			//IL_0007: Unknown result type (might be due to invalid IL or missing references)
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			return -Vector3.Cross(_pos, thrust);
		}
	}

	private enum Params
	{
		TORQUE_X,
		TORQUE_Y,
		TORQUE_Z,
		TRANS_X,
		TRANS_Y,
		TRANS_Z,
		WASTE,
		FUDGE
	}

	private double[,] _a;

	private double[] _b;

	private double _factorTorque = 1.0;

	private double _factorTranslate = 0.005;

	private double _factorWaste = 1.0;

	private double _wasteThreshold = 0.25;

	private readonly int _paramLength = Enum.GetValues(typeof(Params)).Length;

	public void UpdateTuningParameters(RCSSolverTuningParams tuningParams)
	{
		_factorTorque = tuningParams.FactorTorque;
		_factorTranslate = tuningParams.FactorTranslate;
		_factorWaste = tuningParams.FactorWaste;
		_wasteThreshold = tuningParams.WasteThreshold;
	}

	private void cost_func(double[] x, ref double func, object obj)
	{
		func = 0.0;
		for (int i = 0; i < _b.Length; i++)
		{
			double num = 0.0;
			for (int j = 0; j < x.Length; j++)
			{
				num += x[j] * _a[i, j];
			}
			num -= _b[i];
			func += num * num;
		}
	}

	public double[] Run(List<Thruster> fullThrusters, Vector3 direction, Vector3 rotation)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0109: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d7: Expected O, but got Unknown
		//IL_0129: Unknown result type (might be due to invalid IL or missing references)
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0132: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_0139: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0141: Unknown result type (might be due to invalid IL or missing references)
		//IL_0143: Unknown result type (might be due to invalid IL or missing references)
		//IL_0144: Unknown result type (might be due to invalid IL or missing references)
		//IL_0149: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		//IL_0176: Unknown result type (might be due to invalid IL or missing references)
		//IL_0193: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_0207: Unknown result type (might be due to invalid IL or missing references)
		direction = ((Vector3)(ref direction)).normalized;
		int count = fullThrusters.Count;
		List<Thruster> list = new List<Thruster>();
		Vector3[] array = (Vector3[])(object)new Vector3[count];
		for (int i = 0; i < count; i++)
		{
			Thruster thruster = fullThrusters[i];
			array[i] = thruster.GetThrust(direction, rotation);
			if (((Vector3)(ref array[i])).magnitude > 0f)
			{
				list.Add(thruster);
			}
		}
		int count2 = list.Count;
		if (count2 == 0)
		{
			return new double[count];
		}
		_a = new double[_paramLength, count2];
		_b = new double[_paramLength];
		for (int j = 0; j < _b.Length; j++)
		{
			_b[j] = 0.0;
		}
		_b[_b.Length - 1] = 0.001 * (double)count2;
		double[] array2 = new double[count2];
		double[] array3 = new double[count2];
		double[] array4 = new double[count2];
		int num = -1;
		for (int k = 0; k < count; k++)
		{
			Vector3 thrust = array[k];
			if (((Vector3)(ref thrust)).magnitude != 0f)
			{
				Vector3 torque = list[++num].GetTorque(thrust);
				Vector3 normalized = ((Vector3)(ref thrust)).normalized;
				Vector3 val = torque - rotation;
				Vector3 val2 = normalized - direction;
				float num2 = 1f - Vector3.Dot(normalized, direction);
				if ((double)num2 < _wasteThreshold)
				{
					num2 = 0f;
				}
				_a[0, num] = (double)val.x * _factorTorque;
				_a[1, num] = (double)val.y * _factorTorque;
				_a[2, num] = (double)val.z * _factorTorque;
				_a[3, num] = (double)val2.x * _factorTranslate;
				_a[4, num] = (double)val2.y * _factorTranslate;
				_a[5, num] = (double)val2.z * _factorTranslate;
				_a[6, num] = (double)num2 * _factorWaste;
				_a[7, num] = 0.001;
				array2[num] = 1.0;
				array3[num] = 0.0;
				array4[num] = 1.0;
			}
		}
		minbleicstate val3 = default(minbleicstate);
		alglib.minbleiccreatef(array2, 1E-06, ref val3);
		alglib.minbleicsetbc(val3, array3, array4);
		alglib.minbleicsetcond(val3, 0.01, 0.0, 0.0, 0);
		alglib.minbleicoptimize(val3, new ndimensional_func(cost_func), (ndimensional_rep)null, (object)null);
		double[] array5 = default(double[]);
		minbleicreport val4 = default(minbleicreport);
		alglib.minbleicresults(val3, ref array5, ref val4);
		double num3 = array5.Max();
		if (num3 > 0.0)
		{
			for (int l = 0; l < count2; l++)
			{
				array5[l] /= num3;
			}
		}
		double[] array6 = new double[count];
		int num4 = 0;
		for (int m = 0; m < count; m++)
		{
			array6[m] = ((((Vector3)(ref array[m])).magnitude == 0f) ? 0.0 : array5[num4++]);
		}
		return array6;
	}
}
