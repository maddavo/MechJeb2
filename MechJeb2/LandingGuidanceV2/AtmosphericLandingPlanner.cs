using System;
using MechJebLib.HoverslamSimulation;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Converts the independent re-entry simulation into a conservative V2
    /// entry-corridor decision.  It intentionally reports a corridor and an
    /// uncertainty rather than claiming an atmospheric endpoint is exact.
    /// </summary>
    public static class AtmosphericLandingPlanner
    {
        private const int StrategicSamples = 72;

        public static AtmosphericLandingPlan Plan(LandingGuidanceV2Snapshot snapshot, ReentrySimulation.Result estimate)
        {
            if (snapshot == null || snapshot.Body == null || !snapshot.Body.atmosphere)
                return new AtmosphericLandingPlan(snapshot?.Version ?? -1, AtmosphericLandingPlanState.NotApplicable,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    "An atmospheric body snapshot is required.");

            AtmosphericEntryCandidate strategic = FindStrategicEntry(snapshot);
            if (estimate == null || estimate.Body != snapshot.Body)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.WaitingForEstimate,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    strategic.Valid
                        ? "A V2 strategic entry candidate is awaiting independent atmospheric trajectory validation."
                        : "Waiting for a feasible V2 strategic entry candidate and atmospheric trajectory estimate.",
                    strategic.DeltaV, strategic.BurnUT, strategic.EntryUT, strategic.TargetError);

            if (estimate.Outcome == ReentrySimulation.Outcome.NO_REENTRY ||
                estimate.Outcome == ReentrySimulation.Outcome.AEROBRAKED)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.WaitingForEstimate,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    strategic.Valid
                        ? "A V2 strategic entry candidate is awaiting independent atmospheric trajectory validation."
                        : "No feasible V2 strategic entry candidate is currently available.",
                    strategic.DeltaV, strategic.BurnUT, strategic.EntryUT, strategic.TargetError);

            if (estimate.Outcome != ReentrySimulation.Outcome.LANDED)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    "The atmospheric estimator did not produce a landing outcome: " + estimate.Outcome + ".");

            Vector3d predicted = snapshot.Body.GetWorldSurfacePosition(estimate.EndPosition.Latitude, estimate.EndPosition.Longitude, estimate.EndASL);
            Vector3d target = snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0);
            double targetError = Vector3d.Distance(predicted, target);
            double endpointSpeed = estimate.ReferenceFrame == null
                ? double.NaN
                : estimate.ReferenceFrame.WorldVelocityAtCurrentTime(estimate.EndVelocity).magnitude;

            // The re-entry simulation is model based but not exact.  Use the
            // simulated drag exposure and current endpoint scale to calculate a
            // conservative corridor, then preserve a powered terminal reserve.
            double uncertainty = Math.Max(500.0, Math.Min(15000.0,
                250.0 + 25.0 * Math.Max(0, estimate.MaxDragGees) + 0.10 * Math.Max(0, endpointSpeed)));
            double corridor = Math.Max(3000.0, 3.0 * uncertainty);
            double radius = snapshot.Body.Radius + Math.Max(0, estimate.EndASL);
            double gravity = snapshot.Body.gravParameter / (radius * radius);
            AirlessLandingBudget terminalBudget = AirlessLandingBudget.For(strategic.DeltaV.magnitude, endpointSpeed,
                snapshot.MaximumAcceleration, gravity, uncertainty, corridor);
            double margin = snapshot.AvailableDeltaV - terminalBudget.Total;
            if (targetError > corridor)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    targetError, corridor, uncertainty, terminalBudget.Reserve, margin,
                    "The predicted atmospheric endpoint is outside the robust entry corridor; V2 will not present it as a touchdown solution.",
                    strategic.DeltaV, strategic.BurnUT, strategic.EntryUT, strategic.TargetError);
            if (!terminalBudget.Fits(snapshot.AvailableDeltaV))
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    targetError, corridor, uncertainty, terminalBudget.Reserve, margin,
                    "Usable delta-V does not retain the powered terminal reserve required by the atmospheric plan.",
                    strategic.DeltaV, strategic.BurnUT, strategic.EntryUT, strategic.TargetError);
            return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Candidate,
                targetError, corridor, uncertainty, terminalBudget.Reserve, margin,
                "Atmospheric entry corridor is feasible; endpoint remains an uncertainty-bounded estimate until local terminal guidance.",
                strategic.DeltaV, strategic.BurnUT, strategic.EntryUT, strategic.TargetError);
        }

        private static AtmosphericEntryCandidate FindStrategicEntry(LandingGuidanceV2Snapshot snapshot)
        {
            if (snapshot == null || snapshot.Body == null || !snapshot.Body.atmosphere || snapshot.IsLandedOrSplashed)
                return default(AtmosphericEntryCandidate);
            try
            {
                var coast = new Orbit();
                coast.UpdateFromStateVectors(snapshot.Position, snapshot.Velocity, snapshot.Body, snapshot.UT);
                if (!Finite(coast.period) || coast.eccentricity >= 1.0)
                    return default(AtmosphericEntryCandidate);

                double interfaceAltitude = Math.Max(1000.0, snapshot.Body.RealMaxAtmosphereAltitude() * 0.90);
                double entryRadius = snapshot.Body.Radius + interfaceAltitude;
                double desiredPe = snapshot.Body.Radius + Math.Max(1000.0, snapshot.Body.RealMaxAtmosphereAltitude() * 0.15);
                double entryCorridor = Math.Max(5000.0, Math.Min(25000.0, snapshot.Body.RealMaxAtmosphereAltitude() * 0.25));
                AtmosphericEntryCandidate best = default(AtmosphericEntryCandidate);
                double bestScore = double.PositiveInfinity;
                for (int i = 0; i <= StrategicSamples; ++i)
                {
                    double burnUT = snapshot.UT + 10.0 + coast.period * i / StrategicSamples;
                    Vector3d position = coast.WorldBCIPositionAtUT(burnUT);
                    Vector3d velocity = coast.WorldOrbitalVelocityAtUT(burnUT);
                    var burnOrbit = new Orbit();
                    burnOrbit.UpdateFromStateVectors(position, velocity, snapshot.Body, burnUT);
                    Vector3d deltaV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(burnOrbit, burnUT, desiredPe);
                    if (deltaV.magnitude > snapshot.AvailableDeltaV) continue;
                    Orbit entryOrbit = burnOrbit.PerturbedOrbit(burnUT, deltaV);
                    double entryUT = entryOrbit.NextTimeOfRadius(burnUT, entryRadius);
                    if (!Finite(entryUT) || entryUT < burnUT) continue;
                    Vector3d entryPosition = entryOrbit.WorldBCIPositionAtUT(entryUT);
                    Vector3d target = TargetAt(snapshot, entryUT);
                    double error = Vector3d.Distance(entryPosition.normalized * snapshot.Body.Radius, target.normalized * snapshot.Body.Radius);
                    double score = error + 5.0 * deltaV.magnitude;
                    if (score < bestScore)
                    {
                        best = new AtmosphericEntryCandidate(deltaV, burnUT, entryUT, error);
                        bestScore = score;
                    }
                }
                return best.Valid && best.TargetError <= entryCorridor ? best : default(AtmosphericEntryCandidate);
            }
            catch (Exception)
            {
                return default(AtmosphericEntryCandidate);
            }
        }

        private static Vector3d TargetAt(LandingGuidanceV2Snapshot snapshot, double ut)
        {
            Vector3d target = snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0) - snapshot.Body.position;
            return Quaternion.AngleAxis((float)(360d * (ut - snapshot.UT) / snapshot.Body.rotationPeriod), snapshot.Body.angularVelocity) * target;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private struct AtmosphericEntryCandidate
        {
            public readonly Vector3d DeltaV;
            public readonly double BurnUT;
            public readonly double EntryUT;
            public readonly double TargetError;
            public bool Valid => BurnUT > 0 && !double.IsNaN(BurnUT) && !double.IsInfinity(BurnUT);

            public AtmosphericEntryCandidate(Vector3d deltaV, double burnUT, double entryUT, double targetError)
            {
                DeltaV = deltaV;
                BurnUT = burnUT;
                EntryUT = entryUT;
                TargetError = targetError;
            }
        }
    }
}
