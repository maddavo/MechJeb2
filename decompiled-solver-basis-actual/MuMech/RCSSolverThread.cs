using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace MuMech;

public class RCSSolverThread
{
	private class SolverTask
	{
		public readonly RCSSolverKey Key;

		public readonly Vector3 Direction;

		public readonly Vector3 Rotation;

		public SolverTask(RCSSolverKey key, Vector3 direction, Vector3 rotation)
		{
			//IL_000e: Unknown result type (might be due to invalid IL or missing references)
			//IL_000f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0015: Unknown result type (might be due to invalid IL or missing references)
			//IL_0016: Unknown result type (might be due to invalid IL or missing references)
			Key = key;
			Direction = direction;
			Rotation = rotation;
		}
	}

	private class SolverResult
	{
		public readonly RCSSolverKey Key;

		public readonly double[] Throttles;

		public SolverResult(RCSSolverKey key, double[] throttles)
		{
			Key = key;
			Throttles = throttles;
		}
	}

	private readonly RCSSolver _solver = new RCSSolver();

	private readonly MovingAverage _calculationTime = new MovingAverage();

	private readonly MovingAverage _comError = new MovingAverage();

	private readonly Queue _tasks = Queue.Synchronized(new Queue());

	private readonly AutoResetEvent _workEvent = new AutoResetEvent(initialState: false);

	private bool _stopRunning;

	private Thread _t;

	private bool _isWorking;

	private int _lastPartCount;

	private readonly List<ModuleRCS> _lastDisabled = new List<ModuleRCS>();

	private Vector3 _lastCoM = Vector3.zero;

	private readonly Queue _resultsQueue = Queue.Synchronized(new Queue());

	private readonly Dictionary<RCSSolverKey, double[]> _results = new Dictionary<RCSSolverKey, double[]>();

	private readonly HashSet<RCSSolverKey> _pending = new HashSet<RCSSolverKey>();

	private List<RCSSolver.Thruster> _thrusters = new List<RCSSolver.Thruster>();

	private double[] _originalThrottles;

	private double[] _zeroThrottles;

	private readonly double[] _double0 = Array.Empty<double>();

	private readonly List<RCSSolver.Thruster> _callerThrusters = new List<RCSSolver.Thruster>();

	public double CalculationTime { get; private set; }

	public double ComError => _comError.Value;

	public double ComErrorThreshold { get; private set; }

	public double MaxComError { get; private set; }

	public string StatusString { get; private set; }

	public string ErrorString { get; private set; }

	public int TaskCount => _tasks.Count + _resultsQueue.Count + (_isWorking ? 1 : 0);

	public int CacheHits { get; private set; }

	public int CacheMisses { get; private set; }

	public int CacheSize => _results.Count;

	public void UpdateTuningParameters(RCSSolverTuningParams tuningParams)
	{
		_solver.UpdateTuningParameters(tuningParams);
		ClearResults();
	}

	public void Start()
	{
		lock (_solver)
		{
			if (_t == null)
			{
				ClearResults();
				int cacheHits = (CacheMisses = 0);
				CacheHits = cacheHits;
				_isWorking = false;
				_lastPartCount = 0;
				MaxComError = 0.0;
				_stopRunning = false;
				_t = new Thread(Run);
				_t.Start();
			}
		}
	}

	public void Stop()
	{
		lock (_solver)
		{
			if (_t != null)
			{
				_stopRunning = true;
				_workEvent.Set();
				_t.Abort();
				_t = null;
			}
		}
	}

	private void ClearResults()
	{
		_tasks.Clear();
		_results.Clear();
		_resultsQueue.Clear();
		_pending.Clear();
	}

	public void ResetThrusterForces()
	{
		foreach (RCSSolver.Thruster thruster in _thrusters)
		{
			thruster.RestoreOriginalForce();
		}
	}

	private static Vector3 WorldToVessel(Vessel vessel, Vector3 pos)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		return Quaternion.Inverse(vessel.GetTransform().rotation) * pos;
	}

	private static Vector3 VesselRelativePos(Vector3 com, Vessel vessel, Part p)
	{
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		if ((Object)(object)p.Rigidbody == (Object)null)
		{
			return Vector3.zero;
		}
		return WorldToVessel(vessel, p.Rigidbody.worldCenterOfMass - com);
	}

	private void CheckVessel(Vessel vessel, VesselState state)
	{
		//IL_00e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0100: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0105: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0114: Unknown result type (might be due to invalid IL or missing references)
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0120: Unknown result type (might be due to invalid IL or missing references)
		//IL_0160: Unknown result type (might be due to invalid IL or missing references)
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0207: Unknown result type (might be due to invalid IL or missing references)
		//IL_020c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0227: Unknown result type (might be due to invalid IL or missing references)
		//IL_022c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0231: Unknown result type (might be due to invalid IL or missing references)
		//IL_023c: Unknown result type (might be due to invalid IL or missing references)
		//IL_024c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0251: Unknown result type (might be due to invalid IL or missing references)
		//IL_0256: Unknown result type (might be due to invalid IL or missing references)
		//IL_025b: Unknown result type (might be due to invalid IL or missing references)
		//IL_025f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0264: Unknown result type (might be due to invalid IL or missing references)
		//IL_0280: Unknown result type (might be due to invalid IL or missing references)
		bool flag = false;
		if (vessel.parts.Count != _lastPartCount)
		{
			_lastPartCount = vessel.parts.Count;
			flag = true;
		}
		foreach (RCSSolver.Thruster thruster in _thrusters)
		{
			if (!((PartModule)thruster.PartModule).isEnabled)
			{
				flag = true;
				break;
			}
		}
		foreach (ModuleRCS item in _lastDisabled)
		{
			if (((PartModule)item).isEnabled)
			{
				flag = true;
				break;
			}
		}
		Vector3 val4;
		if ((Object)(object)vessel.rootPart.rb != (Object)null)
		{
			ComErrorThreshold = (Math.Max(((Vector3d)(ref state.MoI)).magnitude, 2.34) - 1.0) / 542.0;
			Vector3 val = Vector3d.op_Implicit(state.CoM);
			Vector3 val2 = Vector3d.op_Implicit(state.RootPartPosition);
			Vector3 val3 = WorldToVessel(vessel, val - val2);
			val4 = _lastCoM - val3;
			double num = ((Vector3)(ref val4)).magnitude;
			MaxComError = Math.Max(MaxComError, num);
			_comError.Value = num;
			if ((double)_comError > ComErrorThreshold)
			{
				_lastCoM = val3;
				flag = true;
			}
		}
		if (!flag)
		{
			return;
		}
		_lastDisabled.Clear();
		ResetThrusterForces();
		List<RCSSolver.Thruster> list = new List<RCSSolver.Thruster>();
		foreach (Part part in vessel.parts)
		{
			foreach (ModuleRCS item2 in ((IEnumerable)part.Modules).OfType<ModuleRCS>())
			{
				if (!((PartModule)item2).isEnabled)
				{
					_lastDisabled.Add(item2);
				}
				else if ((Object)(object)part.Rigidbody != (Object)null && !item2.isJustForShow)
				{
					Vector3 pos = VesselRelativePos(Vector3d.op_Implicit(state.CoM), vessel, part);
					Vector3[] array = (Vector3[])(object)new Vector3[item2.thrusterTransforms.Count];
					Quaternion val5 = Quaternion.Inverse(vessel.GetTransform().rotation);
					for (int i = 0; i < item2.thrusterTransforms.Count; i++)
					{
						int num2 = i;
						val4 = val5 * -item2.thrusterTransforms[i].up;
						array[num2] = ((Vector3)(ref val4)).normalized;
					}
					list.Add(new RCSSolver.Thruster(pos, array, part, item2));
				}
			}
		}
		_callerThrusters.Clear();
		_originalThrottles = new double[list.Count];
		_zeroThrottles = new double[list.Count];
		for (int j = 0; j < list.Count; j++)
		{
			_originalThrottles[j] = list[j].OriginalForce;
			_zeroThrottles[j] = 0.0;
			_callerThrusters.Add(list[j]);
		}
		_thrusters = list;
		ClearResults();
	}

	public void GetThrottles(Vessel vessel, VesselState state, Vector3 direction, out double[] throttles, out List<RCSSolver.Thruster> thrustersOut)
	{
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0122: Unknown result type (might be due to invalid IL or missing references)
		//IL_0123: Unknown result type (might be due to invalid IL or missing references)
		thrustersOut = _callerThrusters;
		Vector3 zero = Vector3.zero;
		CheckVessel(vessel, state);
		Vector3 d = ((Vector3)(ref direction)).normalized;
		RCSSolverKey rCSSolverKey = new RCSSolverKey(ref d, zero);
		if (_thrusters.Count == 0)
		{
			throttles = _double0;
		}
		else if (direction == Vector3.zero)
		{
			throttles = _originalThrottles;
		}
		else if (_results.TryGetValue(rCSSolverKey, out throttles))
		{
			CacheHits++;
		}
		else
		{
			CacheMisses++;
			throttles = _double0;
			if (_pending.Contains(rCSSolverKey))
			{
				while (_resultsQueue.Count > 0)
				{
					SolverResult solverResult = (SolverResult)_resultsQueue.Dequeue();
					_results[solverResult.Key] = solverResult.Throttles;
					_pending.Remove(solverResult.Key);
					if (solverResult.Key == rCSSolverKey)
					{
						throttles = solverResult.Throttles;
					}
				}
			}
			else
			{
				_pending.Add(rCSSolverKey);
				_tasks.Enqueue(new SolverTask(rCSSolverKey, d, zero));
				_workEvent.Set();
			}
		}
		throttles = (double[])throttles.Clone();
	}

	private void Run()
	{
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		while (_workEvent.WaitOne() && !_stopRunning)
		{
			try
			{
				StatusString = "working";
				while (_tasks.Count > 0 && !_stopRunning)
				{
					SolverTask solverTask = (SolverTask)_tasks.Dequeue();
					DateTime now = DateTime.Now;
					_isWorking = true;
					double[] throttles = _solver.Run(_thrusters, solverTask.Direction, solverTask.Rotation);
					_isWorking = false;
					_resultsQueue.Enqueue(new SolverResult(solverTask.Key, throttles));
					_calculationTime.Value = (DateTime.Now - now).TotalSeconds;
					CalculationTime = _calculationTime;
				}
				StatusString = "idle";
			}
			catch (InvalidOperationException)
			{
			}
			catch (Exception ex2)
			{
				ErrorString = ex2.Message + " ..[" + ex2.Source + "].. " + ex2.StackTrace;
			}
		}
	}
}
