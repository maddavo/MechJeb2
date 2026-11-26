using System;
using System.Globalization;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace MuMech.Landing
{
    // Main-thread passive readout. Module outputs and live transforms can have
    // different update epochs. Do not sum them as independently measured force.
    internal static class DunaM1RuntimeObservation
    {
        private const int MaxParts = 512, MaxModules = 2048, MaxNozzles = 32, MaxCurveKeys = 256;
        private static string N(double n) => DunaM1Capture.N(n);
        private static string Q(string s) => DunaM1Capture.Q(s);
        private static string B(bool b) => b ? "true" : "false";
        private static string V(Vector3d v) => "[" + N(v.x) + "," + N(v.y) + "," + N(v.z) + "]";
        private static string R(Quaternion q) => "[" + N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w) + "]";
        private static string I(int n) => n.ToString(CultureInfo.InvariantCulture);
        private static void Field(StringBuilder b, string key, string json) =>
            b.Append(',').Append(Q(key)).Append(':').Append(json);

        internal static string BuildVersion() =>
            typeof(DunaM1RuntimeObservation).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion;

        private static string Curve(FloatCurve curve)
        {
            if (curve == null || curve.Curve == null) return "null";
            Keyframe[] keys = curve.Curve.keys;
            if (keys.Length > MaxCurveKeys) return "null";
            var b = new StringBuilder("[");
            for (int i = 0; i < keys.Length; i++)
            {
                if (i != 0) b.Append(',');
                Keyframe k = keys[i];
                b.Append('[').Append(N(k.time)).Append(',').Append(N(k.value)).Append(',')
                    .Append(N(k.inTangent)).Append(',').Append(N(k.outTangent)).Append(']');
            }
            return b.Append(']').ToString();
        }

        private static string Atmosphere(CelestialBody body)
        {
            var b = new StringBuilder("{\"curveKeyLayout\":\"time,value,inTangent,outTangent\"");
            Field(b,"pressureCurveEnabled",B(body.atmosphereUsePressureCurve));
            Field(b,"pressureCurveNormalized",B(body.atmospherePressureCurveIsNormalized));
            Field(b,"temperatureCurveEnabled",B(body.atmosphereUseTemperatureCurve));
            Field(b,"temperatureCurveNormalized",B(body.atmosphereTemperatureCurveIsNormalized));
            Field(b,"pressure",Curve(body.atmospherePressureCurve));
            Field(b,"temperature",Curve(body.atmosphereTemperatureCurve));
            Field(b,"temperatureSunMult",Curve(body.atmosphereTemperatureSunMultCurve));
            Field(b,"latitudeTemperatureBias",Curve(body.latitudeTemperatureBiasCurve));
            Field(b,"latitudeTemperatureSunMult",Curve(body.latitudeTemperatureSunMultCurve));
            Field(b,"axialTemperatureSunMult",Curve(body.axialTemperatureSunMultCurve));
            return b.Append('}').ToString();
        }

        private static string Nozzles(ModuleEngines e, Vector3d com)
        {
            if (e.thrustTransforms == null || e.thrustTransformMultipliers == null ||
                e.thrustTransforms.Count != e.thrustTransformMultipliers.Count ||
                e.thrustTransforms.Count > MaxNozzles) return "null";
            var b = new StringBuilder("[");
            for (int i = 0; i < e.thrustTransforms.Count; i++)
            {
                if (i != 0) b.Append(',');
                Transform t = e.thrustTransforms[i];
                if (t == null) { b.Append("null"); continue; }
                b.Append("{\"index\":").Append(I(i));
                Field(b,"name",Q(t.name));
                Field(b,"worldPosition",V(t.position));
                Field(b,"worldForward",V(t.forward));
                Field(b,"worldRotationXYZW",R(t.rotation));
                Field(b,"relativeToSampledCoM",V((Vector3d)t.position-com));
                Field(b,"multiplier",N(e.thrustTransformMultipliers[i]));
                b.Append('}');
            }
            return b.Append(']').ToString();
        }

        private static string Engines(Vessel v, Vector3d com, bool includeModel)
        {
            if (v.parts == null || v.parts.Count > MaxParts) return "null";
            var b = new StringBuilder("[");
            bool first = true;
            int pi = 0, totalModules = 0;
            foreach (Part p in v.parts)
            {
                if (p == null || p.Modules == null) return "null";
                int mi = 0;
                foreach (PartModule module in p.Modules)
                {
                    if (++totalModules > MaxModules) return "null";
                    if (module is ModuleEngines e)
                    {
                        if (!first) b.Append(',');
                        first = false;
                        b.Append("{\"partIndex\":").Append(I(pi));
                        Field(b,"moduleIndex",I(mi));
                        Field(b,"partName",Q(p.name));
                        Field(b,"engineID",Q(e.engineID));
                        Field(b,"ignited",B(e.EngineIgnited));
                        Field(b,"enabled",B(e.isEnabled));
                        Field(b,"operational",B(e.isOperational));
                        Field(b,"throttleLocked",B(e.throttleLocked));
                        Field(b,"finalThrustRaw",N(e.finalThrust));
                        Field(b,"resultingThrustRaw",N(e.resultingThrust));
                        Field(b,"realIspRaw",N(e.realIsp));
                        Field(b,"fuelFlowGuiRaw",N(e.fuelFlowGui));
                        Field(b,"requestedMassFlowRaw",N(e.requestedMassFlow));
                        Field(b,"requestedThrottleRaw",N(e.requestedThrottle));
                        Field(b,"currentThrottleRaw",N(e.currentThrottle));
                        Field(b,"flowMultiplierRaw",N(e.flowMultiplier));
                        Field(b,"multFlowRaw",N(e.multFlow));
                        Field(b,"multIspRaw",N(e.multIsp));
                        Field(b,"thrustPercentageRaw",N(e.thrustPercentage));
                        Field(b,"minFuelFlowRaw",N(e.minFuelFlow));
                        Field(b,"maxFuelFlowRaw",N(e.maxFuelFlow));
                        Field(b,"gRaw",N(e.g));
                        Field(b,"useEngineResponseTime",B(e.useEngineResponseTime));
                        Field(b,"engineAccelerationSpeedRaw",N(e.engineAccelerationSpeed));
                        Field(b,"engineDecelerationSpeedRaw",N(e.engineDecelerationSpeed));
                        Field(b,"nozzles",Nozzles(e,com));
                        if (includeModel)
                        {
                            Field(b,"atmosphereCurve",Curve(e.atmosphereCurve));
                            Field(b,"atmCurve",Curve(e.atmCurve));
                            Field(b,"atmCurveIsp",Curve(e.atmCurveIsp));
                            Field(b,"velCurveIsp",Curve(e.velCurveIsp));
                        }
                        b.Append('}');
                    }
                    mi++;
                }
                pi++;
            }
            return b.Append(']').ToString();
        }

        internal static string Read(Vessel v, VesselState s, bool includeModel)
        {
            try
            {
                CelestialBody body = v.mainBody;
                var b = new StringBuilder("{\"status\":\"raw_unvalidated\",\"samplingPoint\":\"landing_drive_callback\",\"physicsEpochVerified\":false");
                Field(b,"stateUT",N(s.Time));
                Field(b,"readUT",N(Planetarium.GetUniversalTime()));
                Field(b,"sampledCoMWorld",V(s.CoM));
                Field(b,"sampledPositionBCIWorldAxes",V(s.OrbitalPosition));
                Field(b,"sampledOrbitalVelocityWorld",V(s.OrbitalVelocity));
                Field(b,"sampledSurfaceVelocityWorld",V(s.SurfaceVelocity));
                Field(b,"sampledNorthWorld",V(s.North));
                Field(b,"sampledEastWorld",V(s.East));
                Field(b,"sampledUpWorld",V(s.Up));
                Field(b,"sampledAngularVelocityWorld",V(s.AngularVelocity));
                Field(b,"referenceRotationWorldXYZW",v.ReferenceTransform == null ? "null" : R(v.ReferenceTransform.rotation));
                Field(b,"bodyAngularVelocityWorld",V(body.angularVelocity));
                Field(b,"bodyRotationPeriod",N(body.rotationPeriod));
                Field(b,"bodyRotationAngleRaw",N(body.rotationAngle));
                Field(b,"surfaceNormalLat0Lon0",V(body.GetSurfaceNVector(0,0)));
                Field(b,"surfaceNormalLat0Lon90",V(body.GetSurfaceNVector(0,90)));
                Field(b,"surfaceNormalLat90Lon0",V(body.GetSurfaceNVector(90,0)));
                Field(b,"partCount",I(v.parts.Count));
                Field(b,"engineIndexMeaning",Q("part/module indices are snapshot-local, not persistent identity"));
                Field(b,"engines",Engines(v,s.CoM,includeModel));
                if (includeModel) Field(b,"atmosphereModel",Atmosphere(body));
                return b.Append('}').ToString();
            }
            catch (Exception ex)
            {
                return "{\"status\":\"unavailable\",\"errorType\":" + Q(ex.GetType().Name) + "}";
            }
        }
    }
}
