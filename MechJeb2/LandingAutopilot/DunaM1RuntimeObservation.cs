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
        private const int MaxCubesPerPart = 32, MaxResources = 4096;
        private static readonly FieldInfo PropellantReqMet = typeof(ModuleEngines).GetField("propellantReqMet", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        private static string N(double n) => DunaM1Capture.N(n);
        private static string Q(string s) => DunaM1Capture.Q(s);
        private static string B(bool b) => b ? "true" : "false";
        private static string V(Vector3d v) => "[" + N(v.x) + "," + N(v.y) + "," + N(v.z) + "]";
        private static string R(Quaternion q) => "[" + N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w) + "]";
        private static string R(QuaternionD q) => "[" + N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w) + "]";
        private static string U(uint n) => n.ToString(CultureInfo.InvariantCulture);
        private static string AcceptedRequest(ModuleEngines e)
        {
            // IL identifies this field as accepted-request percent. Absence or
            // a different type is unknown, not an assumed 100 percent.
            object value = PropellantReqMet?.GetValue(e);
            if (value is float f) return N(f);
            if (value is double d) return N(d);
            return "null";
        }

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
                        Field(b,"flightID",U(p.flightID));
                        Field(b,"engineID",Q(e.engineID));
                        Field(b,"ignited",B(e.EngineIgnited));
                        Field(b,"enabled",B(e.isEnabled));
                        Field(b,"operational",B(e.isOperational));
                        Field(b,"throttleLocked",B(e.throttleLocked));
                        Field(b,"finalThrustRaw",N(e.finalThrust));
                        Field(b,"resultingThrustRaw",N(e.resultingThrust));
                        Field(b,"realIspRaw",N(e.realIsp));
                        Field(b,"fuelFlowGuiRaw",N(e.fuelFlowGui));
                        Field(b,"propellantRequestMetPercentRaw",AcceptedRequest(e));
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

        private static string Six(float[] values)
        {
            if (values == null || values.Length != 6) return "null";
            var b = new StringBuilder("[");
            for (int i=0;i<6;i++) { if(i!=0)b.Append(','); b.Append(N(values[i])); }
            return b.Append(']').ToString();
        }

        private static string Cubes(DragCubeList cubes, bool model)
        {
            if (cubes == null || cubes.Cubes == null || cubes.Cubes.Count > MaxCubesPerPart) return "null";
            var b = new StringBuilder("{\"areaDragRaw\":").Append(N(cubes.AreaDrag));
            Field(b,"dragCoeffRaw",N(cubes.DragCoeff));
            Field(b,"liftForceLocalRaw",V(cubes.LiftForce));
            Field(b,"weightedArea",Six(cubes.WeightedArea));
            Field(b,"weightedDrag",Six(cubes.WeightedDrag));
            Field(b,"weightedDepth",Six(cubes.WeightedDepth));
            Field(b,"areaOccluded",Six(cubes.AreaOccluded));
            b.Append(",\"cubes\":[");
            for(int i=0;i<cubes.Cubes.Count;i++)
            {
                if(i!=0)b.Append(',');
                DragCube c = cubes.Cubes[i];
                b.Append("{\"name\":").Append(Q(c.Name));
                Field(b,"weightRaw",N(c.Weight));
                if(model)
                {
                    Field(b,"center",V(c.Center)); Field(b,"size",V(c.Size));
                    Field(b,"area",Six(c.Area)); Field(b,"drag",Six(c.Drag));
                    Field(b,"depth",Six(c.Depth)); Field(b,"dragModifiers",Six(c.DragModifiers));
                }
                b.Append('}');
            }
            return b.Append("]}").ToString();
        }

        private static string Parts(Vessel v, bool model)
        {
            // Raw asynchronous part and resource observations. Do not sum
            // lift-module fields with queued forces or label them integrated.
            if (v.parts == null || v.parts.Count > MaxParts) return "null";
            var b = new StringBuilder("[");
            int pi=0, totalModules=0, resources=0;
            foreach(Part p in v.parts)
            {
                if(p==null || p.Modules==null || p.Resources==null) return "null";
                if(pi!=0)b.Append(',');
                b.Append("{\"partIndex\":").Append(I(pi++));
                Field(b,"flightID",U(p.flightID)); Field(b,"name",Q(p.name));
                Field(b,"worldRotationXYZW",R(p.transform.rotation));
                Field(b,"worldPosition",V(p.transform.position));
                Field(b,"worldCoMRaw",V(p.WCoM));
                Field(b,"rigidbodyMassRaw",p.rb==null?"null":N(p.rb.mass));
                Field(b,"shielded",B(p.ShieldedFromAirstream));
                Field(b,"hasLiftModule",B(p.hasLiftModule));
                Field(b,"dragDirectionWorld",V(p.dragVectorDir));
                Field(b,"dragDirectionLocal",V(p.dragVectorDirLocal));
                Field(b,"dragScalarRaw",N(p.dragScalar));
                Field(b,"bodyLiftMultiplierRaw",N(p.bodyLiftMultiplier));
                Field(b,"bodyLiftScalarRaw",N(p.bodyLiftScalar));
                Field(b,"groundContact",B(p.GroundContact));
                Field(b,"permanentGroundContact",B(p.PermanentGroundContact));
                Field(b,"waterContact",B(p.WaterContact));
                Field(b,"dragCubes",Cubes(p.DragCubes,model));
                b.Append(",\"resources\":[");
                bool first=true;
                foreach(PartResource r in p.Resources)
                {
                    if(++resources>MaxResources)return "null";
                    if(!first)b.Append(','); first=false;
                    b.Append("{\"resourceID\":").Append(I(r.info.id));
                    Field(b,"amountRaw",N(r.amount)); Field(b,"maxAmountRaw",N(r.maxAmount));
                    Field(b,"densityRaw",N(r.info.density)); b.Append('}');
                }
                b.Append("],\"liftingModules\":["); first=true;
                int mi=0;
                foreach(PartModule m in p.Modules)
                {
                    if(++totalModules>MaxModules)return "null";
                    if(m is ModuleLiftingSurface l)
                    {
                        if(!first)b.Append(',');first=false;
                        b.Append("{\"moduleIndex\":").Append(I(mi));
                        Field(b,"enabled",B(l.isEnabled));
                        Field(b,"liftForceRaw",V(l.liftForce)); Field(b,"dragForceRaw",V(l.dragForce));
                        if(model)
                        {
                            Field(b,"liftCurve",Curve(l.liftCurve)); Field(b,"liftMachCurve",Curve(l.liftMachCurve));
                            Field(b,"dragCurve",Curve(l.dragCurve)); Field(b,"dragMachCurve",Curve(l.dragMachCurve));
                        }
                        b.Append('}');
                    }
                    mi++;
                }
                b.Append("]}");
            }
            return b.Append(']').ToString();
        }

        private static string Controller(MechJebModuleThrustController t, MechJebModuleAttitudeController a)
        {
            var b = new StringBuilder("{\"epochVerified\":false");
            Field(b,"differentialThrottle",B(t.DifferentialThrottle));
            Field(b,"differentialStatus",Q(t.DifferentialThrottleSuccess.ToString()));
            Field(b,"differentialDemandedTorqueRaw",V(t.DifferentialThrottleDemandedTorque));
            Field(b,"previousTransThrottle",N(t.PreviousTransThrottle));
            // These are effective predictor inputs: a pending mode reset returns zero.
            Field(b,"verticalPidPreviousErrorEffective",N(t.VerticalPidPreviousError));
            Field(b,"verticalPidIntegralEffective",N(t.VerticalPidIntegral));
            Field(b,"lastThrottle",N(t.LastThrottle));
            Field(b,"targetThrottle",N(t.TargetThrottle));
            Field(b,"throttleLimit",N(t.ThrottleLimit));
            Field(b,"limiter",Q(t.Limiter.ToString()));
            Field(b,"smoothThrottle",B(t.SmoothThrottle));
            Field(b,"throttleSmoothingTime",N(t.ThrottleSmoothingTime));
            Field(b,"attitudeReference",Q(a.attitudeReference.ToString()));
            Field(b,"attitudeTargetReferenceXYZW",R(a.attitudeTarget));
            Field(b,"lastRequestedAttitudeXYZW",R(a.RequestedAttitude));
            Field(b,"omegaTargetRaw",V(a.OmegaTarget));
            Field(b,"torqueRaw",V(a.torque));
            return b.Append('}').ToString();
        }

        internal static string Read(Vessel v, VesselState s, bool includeModel, bool includeParts,
            MechJebModuleThrustController thrust, MechJebModuleAttitudeController attitude)
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
                Field(b,"sampledLatitude",N(s.Latitude)); Field(b,"sampledLongitude",N(s.Longitude));
                Field(b,"sampledGravityWorld",V(s.GravityForce));
                Field(b,"sampledTorqueAvailableRaw",V(s.TorqueAvailable));
                Field(b,"sampledThrottleFixedLimit",N(s.ThrottleFixedLimit));
                Field(b,"sampledThrottleLimit",N(s.ThrottleLimit));
                Field(b,"sampledSurfaceAltitudeASL",N(s.SurfaceAltitudeASL));
                Field(b,"sampledAngularVelocityWorld",V(s.AngularVelocity));
                Field(b,"referenceRotationWorldXYZW",v.ReferenceTransform == null ? "null" : R(v.ReferenceTransform.rotation));
                Field(b,"bodyAngularVelocityWorld",V(body.angularVelocity));
                Field(b,"bodyRotationPeriod",N(body.rotationPeriod));
                Field(b,"bodyRotationAngleRaw",N(body.rotationAngle));
                Field(b,"surfaceNormalLat0Lon0",V(body.GetSurfaceNVector(0,0)));
                Field(b,"surfaceNormalLat0Lon90",V(body.GetSurfaceNVector(0,90)));
                Field(b,"surfaceNormalLat90Lon0",V(body.GetSurfaceNVector(90,0)));
                Field(b,"partCount",I(v.parts.Count));
                Field(b,"engineIndexMeaning",Q("flightID identifies loaded part within this session; part/module indices are snapshot-local"));
                Field(b,"controller",Controller(thrust,attitude));
                Field(b,"computedFullTemperatureAtSampledCoMRaw",N(body.GetFullTemperature(s.CoM)));
                Field(b,"engines",Engines(v,s.CoM,includeModel));
                if (includeModel) Field(b,"atmosphereModel",Atmosphere(body));
                if(includeParts)
                {
                    try { Field(b,"parts",Parts(v,includeModel)); }
                    catch(Exception ex) { Field(b,"parts","null"); Field(b,"partReadError",Q(ex.GetType().Name)); }
                }
                return b.Append('}').ToString();
            }
            catch (Exception ex)
            {
                return "{\"status\":\"unavailable\",\"errorType\":" + Q(ex.GetType().Name) + "}";
            }
        }
    }
}
