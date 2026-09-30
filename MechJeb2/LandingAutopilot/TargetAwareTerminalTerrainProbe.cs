using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace MuMech.Landing
{
    // Offline terrain-oracle check for an already computed terminal path. The
    // synchronous implementation must not be called with KSP/PQS on the flight
    // thread. A production resolver needs bounded incremental queries. This
    // also cannot certify the ideal-attitude terminal control model.
    internal readonly struct TargetAwareTerminalTerrainProbeResult
    {
        internal readonly bool Resolved, HasContact;
        internal readonly AirlessTargetAwareState Contact;
        internal readonly double ContactTerrainASL, MinimumClearance, EndClearance;
        internal readonly int QueryCount;

        internal TargetAwareTerminalTerrainProbeResult(bool resolved, bool hasContact,
            AirlessTargetAwareState contact, double contactTerrainASL,
            double minimumClearance, double endClearance, int queryCount)
        {
            Resolved = resolved;
            HasContact = hasContact;
            Contact = contact;
            ContactTerrainASL = contactTerrainASL;
            MinimumClearance = minimumClearance;
            EndClearance = endClearance;
            QueryCount = queryCount;
        }
    }

    internal static class TargetAwareTerminalTerrainProbe
    {
        // Supply a deterministic terrain oracle in offline replay. Unknown or
        // budget-exhausted paths stay unresolved.
        internal static TargetAwareTerminalTerrainProbeResult Evaluate(
            AirlessTargetAwareSnapshot snapshot,
            IReadOnlyList<AirlessTargetAwareState> path,
            Func<double, double, double> terrainAltitude,
            int maximumQueries)
        {
            if (path == null || path.Count == 0 || terrainAltitude == null ||
                maximumQueries <= 0)
                throw new ArgumentException("Incomplete terminal terrain probe");
            int queries = 0;
            double minimumClearance = double.PositiveInfinity;
            double previousClearance = double.NaN;
            double previousTerrain = double.NaN;
            AirlessTargetAwareState previous = default(AirlessTargetAwareState);
            AirlessTargetAwareState end = path[path.Count - 1];
            for (int segment = 0; segment < path.Count; segment++)
            {
                AirlessTargetAwareState start = segment == 0 ? path[0] : path[segment - 1];
                AirlessTargetAwareState finish = path[segment];
                double distance = (finish.Position - start.Position).magnitude;
                double elapsed = finish.UT - start.UT;
                if (double.IsNaN(distance) || double.IsInfinity(distance) ||
                    double.IsNaN(elapsed) || double.IsInfinity(elapsed) || elapsed < 0)
                    return new TargetAwareTerminalTerrainProbeResult(false, false,
                        default(AirlessTargetAwareState), double.NaN,
                        minimumClearance, double.NaN, queries);
                if (distance / 25 > maximumQueries || elapsed > maximumQueries)
                    return new TargetAwareTerminalTerrainProbeResult(false, false,
                        default(AirlessTargetAwareState), double.NaN,
                        minimumClearance, double.NaN, queries);
                int divisions = segment == 0 ? 1 : Math.Max(1,
                    (int)Math.Ceiling(Math.Max(
                        distance / 25, elapsed)));
                for (int part = 1; part <= divisions; part++)
                {
                    if (queries == maximumQueries)
                        return new TargetAwareTerminalTerrainProbeResult(false, false,
                            default(AirlessTargetAwareState), double.NaN,
                            minimumClearance, double.NaN, queries);
                    double fraction = segment == 0 ? 0 : (double)part / divisions;
                    var state = new AirlessTargetAwareState(
                        start.Position + fraction * (finish.Position - start.Position),
                        start.Velocity + fraction * (finish.Velocity - start.Velocity),
                        start.UT + fraction * (finish.UT - start.UT));
                    AbsoluteVector location = AirlessTargetAwareSimulation.ToAbsolute(
                        state.Position, state.UT, snapshot);
                    double terrain = terrainAltitude(location.Latitude, location.Longitude);
                    queries++;
                    if (double.IsNaN(terrain) || double.IsInfinity(terrain))
                        return new TargetAwareTerminalTerrainProbeResult(false, false,
                            default(AirlessTargetAwareState), double.NaN,
                            minimumClearance, double.NaN, queries);
                    double clearance = state.Position.magnitude - snapshot.BodyRadius -
                        snapshot.BottomOffset - terrain;
                    minimumClearance = Math.Min(minimumClearance, clearance);
                    if (clearance <= 0)
                    {
                        double contactFraction = double.IsNaN(previousClearance) ? 1 :
                            Math.Max(0, Math.Min(1, previousClearance /
                                (previousClearance - clearance)));
                        var contact = double.IsNaN(previousClearance) ? state :
                            new AirlessTargetAwareState(
                                previous.Position + contactFraction *
                                    (state.Position - previous.Position),
                                previous.Velocity + contactFraction *
                                    (state.Velocity - previous.Velocity),
                                previous.UT + contactFraction * (state.UT - previous.UT));
                        return new TargetAwareTerminalTerrainProbeResult(true, true,
                            contact, double.IsNaN(previousTerrain) ? terrain :
                                previousTerrain + contactFraction * (terrain - previousTerrain),
                            minimumClearance, 0, queries);
                    }
                    previous = state;
                    previousClearance = clearance;
                    previousTerrain = terrain;
                }
            }
            return new TargetAwareTerminalTerrainProbeResult(true, false,
                default(AirlessTargetAwareState), double.NaN,
                minimumClearance, end.Position.magnitude - snapshot.BodyRadius -
                    snapshot.BottomOffset - previousTerrain, queries);
        }
    }

    // KSP terrain is only available on the flight thread. This is the
    // incremental counterpart of the offline probe: each call has both a
    // query and wall-time limit, while the immutable worker path is retained
    // until the entire path or its first contact has been resolved.
    internal sealed class TargetAwareTerminalTerrainResolver
    {
        private readonly AirlessTargetAwareSnapshot _snapshot;
        private readonly IReadOnlyList<AirlessTargetAwareState> _path;
        private readonly Func<double, double, double> _terrainAltitude;
        private readonly int _maximumQueries;
        private int _segment, _part, _divisions;
        private double _previousClearance = double.NaN, _previousTerrain = double.NaN;
        private double _minimumClearance = double.PositiveInfinity;
        private AirlessTargetAwareState _previous;

        internal int QueryCount { get; private set; }
        internal bool Done { get; private set; }
        internal TargetAwareTerminalTerrainProbeResult Result { get; private set; }

        internal TargetAwareTerminalTerrainResolver(AirlessTargetAwareSnapshot snapshot,
            IReadOnlyList<AirlessTargetAwareState> path,
            Func<double, double, double> terrainAltitude, int maximumQueries)
        {
            if (path == null || path.Count == 0 || terrainAltitude == null || maximumQueries <= 0)
                throw new ArgumentException("Incomplete terminal terrain resolver");
            _snapshot = snapshot;
            _path = path;
            _terrainAltitude = terrainAltitude;
            _maximumQueries = maximumQueries;
        }

        internal void Advance(int queriesPerTick, double millisecondsPerTick)
        {
            if (Done) return;
            if (queriesPerTick <= 0 || millisecondsPerTick <= 0 ||
                double.IsNaN(millisecondsPerTick) ||
                double.IsInfinity(millisecondsPerTick))
                throw new ArgumentException("Invalid terrain tick budget");
            long tickStart = Stopwatch.GetTimestamp();
            int tickQueries = 0;
            while (tickQueries < queriesPerTick &&
                   (tickQueries == 0 ||
                    1000d * (Stopwatch.GetTimestamp() - tickStart) / Stopwatch.Frequency <
                    millisecondsPerTick))
            {
                if (_segment == _path.Count)
                {
                    Finish(true, false, default(AirlessTargetAwareState), double.NaN,
                        _path[_path.Count - 1].Position.magnitude - _snapshot.BodyRadius -
                        _snapshot.BottomOffset - _previousTerrain);
                    return;
                }
                AirlessTargetAwareState start = _segment == 0 ?
                    _path[0] : _path[_segment - 1];
                AirlessTargetAwareState end = _path[_segment];
                if (_part == 0)
                {
                    double distance = (end.Position - start.Position).magnitude;
                    double elapsed = end.UT - start.UT;
                    if (double.IsNaN(distance) || double.IsInfinity(distance) ||
                        double.IsNaN(elapsed) || double.IsInfinity(elapsed) || elapsed < 0 ||
                        distance / 25 > _maximumQueries || elapsed > _maximumQueries)
                    {
                        Finish(false, false, default(AirlessTargetAwareState),
                            double.NaN, double.NaN);
                        return;
                    }
                    _divisions = _segment == 0 ? 1 : Math.Max(1,
                        (int)Math.Ceiling(Math.Max(distance / 25, elapsed)));
                }
                if (QueryCount == _maximumQueries)
                {
                    Finish(false, false, default(AirlessTargetAwareState),
                        double.NaN, double.NaN);
                    return;
                }
                double fraction = _segment == 0 ? 0 : (double)(_part + 1) / _divisions;
                var state = new AirlessTargetAwareState(
                    start.Position + fraction * (end.Position - start.Position),
                    start.Velocity + fraction * (end.Velocity - start.Velocity),
                    start.UT + fraction * (end.UT - start.UT));
                AbsoluteVector location = AirlessTargetAwareSimulation.ToAbsolute(
                    state.Position, state.UT, _snapshot);
                QueryCount++;
                tickQueries++;
                double terrain;
                try
                {
                    terrain = _terrainAltitude(location.Latitude, location.Longitude);
                }
                catch
                {
                    Finish(false, false, default(AirlessTargetAwareState),
                        double.NaN, double.NaN);
                    return;
                }
                if (double.IsNaN(terrain) || double.IsInfinity(terrain))
                {
                    Finish(false, false, default(AirlessTargetAwareState),
                        double.NaN, double.NaN);
                    return;
                }
                double clearance = state.Position.magnitude - _snapshot.BodyRadius -
                    _snapshot.BottomOffset - terrain;
                _minimumClearance = Math.Min(_minimumClearance, clearance);
                if (clearance <= 0)
                {
                    double contactFraction = double.IsNaN(_previousClearance) ? 1 :
                        Math.Max(0, Math.Min(1, _previousClearance /
                            (_previousClearance - clearance)));
                    var contact = double.IsNaN(_previousClearance) ? state :
                        new AirlessTargetAwareState(
                            _previous.Position + contactFraction *
                                (state.Position - _previous.Position),
                            _previous.Velocity + contactFraction *
                                (state.Velocity - _previous.Velocity),
                            _previous.UT + contactFraction * (state.UT - _previous.UT));
                    Finish(true, true, contact,
                        double.IsNaN(_previousTerrain) ? terrain :
                            _previousTerrain + contactFraction *
                                (terrain - _previousTerrain), 0);
                    return;
                }
                _previous = state;
                _previousClearance = clearance;
                _previousTerrain = terrain;
                _part++;
                if (_part == _divisions)
                {
                    _segment++;
                    _part = 0;
                }
            }
        }

        private void Finish(bool resolved, bool hasContact,
            AirlessTargetAwareState contact, double terrain, double endClearance)
        {
            Done = true;
            Result = new TargetAwareTerminalTerrainProbeResult(resolved, hasContact,
                contact, terrain, _minimumClearance, endClearance, QueryCount);
        }
    }
}
