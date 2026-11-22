"""The outputs this repository owns, from every spec under vehicles/ (roadmap 051):

    generated/<name>/carla_sensors.yaml   the sensors acb_bridge spawns on CARLA
    generated/<name>/carla_physics.yaml   CARLA's VehiclePhysicsControl for the car, and its lags
    generated/<name>/carla_physics_1.0.yaml  the same for CARLA 1.0's Chaos vehicle
    generated/<name>/articulation[_<body>].yaml  the car as links and joints (articulation.py)
    catalogs/vehicles.xosc               the SSv2 vehicle catalog (one entry per spec)
    catalogs/<body>/vehicles.xosc        the same for each spec's other bodies

A spec with bodies (roadmap 083) is generated as each of them (spec.effective): its first body
is the default, whose catalog is vehicles.xosc; another body's is <body>/vehicles.xosc, in a
directory of its own, since SSv2 reads every file of a catalog directory and keeps the first
catalog of a name it finds (both are `vehicles`). A CARLA physics file comes from the body that
calibrates it (carla.physics: carla_physics.yaml, carla.physics_1_0: carla_physics_1.0.yaml).

    python3 -m sim_vehicle_spec.gen [--root .]           # write
    python3 -m sim_vehicle_spec.gen [--root .] --check   # exit 1 when one is stale

For a car kept outside this repository (a user's own, docs/vehicle-from-another-repository
in autoware_isaac_bridge), the same outputs for one spec, anywhere:

    python3 -m sim_vehicle_spec.gen --spec <car.yaml> --catalog <vehicles.xosc> \
        [--spec-hash <spec_hash.yaml>] [--carla <dir>] [--check]

--catalog is that car's SSv2 vehicle catalog (the bridge's vehicle_catalogs, a scenario's
CatalogLocations); --spec-hash is the YAML the vehicle agent registers (its spec_hash_file);
--carla writes carla_sensors.yaml and carla_physics.yaml there.

Every file starts with a header naming the spec and its spec_hash. Other repositories generate
their own outputs with this library (autoware_sim_vehicle: Autoware's packages;
autoware_isaac_bridge: the Isaac rig).
"""
import argparse
import math
import sys
import textwrap
from pathlib import Path

from . import articulation, emit
from . import spec as specmod

TOOL = "sim_vehicle_spec.gen (`just generate`)"


# --- CARLA: the sensors ---------------------------------------------------------------------

def _attr(v):
    """A CARLA blueprint attribute: always a string, numbers as Python writes them."""
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def carla_attributes(sensor):
    """A sensor's CARLA blueprint attributes: what the shared fields give, then its own."""
    t = sensor["type"]
    out = {}
    if t == "lidar":
        out["channels"] = sensor["channels"]
        out["range"] = sensor["range"]
        out["rotation_frequency"] = sensor["rate_hz"]
        out["upper_fov"] = sensor["vertical_fov"][1]
        out["lower_fov"] = sensor["vertical_fov"][0]
        if "horizontal_fov" in sensor:
            out["horizontal_fov"] = sensor["horizontal_fov"]
        if sensor.get("message") == "laser_scan":
            # CARLA spreads a turn's points over horizontal_fov: one per beam
            out["points_per_second"] = sensor["beams"] * sensor["rate_hz"]
    elif t == "camera":
        out["image_size_x"] = sensor["width"]
        out["image_size_y"] = sensor["height"]
        out["fov"] = sensor["horizontal_fov"]
        if "rate_hz" in sensor:
            out["sensor_tick"] = 1.0 / sensor["rate_hz"]
    elif "rate_hz" in sensor:
        out["sensor_tick"] = 1.0 / sensor["rate_hz"]
    out.update((sensor.get("carla") or {}).get("attributes") or {})
    return {k: _attr(v) for k, v in out.items()}


def carla_sensors(spec, header):
    sensors = {}
    for name, s in (spec.doc.get("sensors") or {}).items():
        carla = s.get("carla")
        if not carla:
            continue
        entry = {"blueprint": carla["blueprint"], "parameters": carla_attributes(s)}
        if "intensity" in carla:
            # By semantic tag id: acb_bridge indexes it with each point's tag.
            entry["intensity"] = {"reflectance": specmod.lidar_reflectance(carla)}
        sensors[carla.get("key", s["link"])] = entry
    doc = {"spec_hash": specmod.spec_hash(spec),
           "vehicle": {"blueprint": spec.section("body").get("catalogue_model", "")},
           "sensors": sensors}
    return emit.comment(header) + emit.comment(
        "The sensors acb_bridge spawns on CARLA, keyed by the URDF frame each is mounted at\n"
        "(the mounts come from Autoware's robot_description), with their blueprint attributes.\n"
        "A semantic lidar's intensity is reflectance[its tag] x cos(incidence), clamped to 0-255;\n"
        "reflectance is by CARLA semantic tag id:\n"
        + textwrap.fill(", ".join(f"{i}={n}" for i, n in enumerate(specmod.CARLA_SEMANTIC_TAGS)),
                        88)) \
        + emit.yaml_text(doc)


# --- CARLA: the physics ------------------------------------------------------------------

def motor_curve(p, points=8):
    """The ev motor's torque (N m, at the motor) against its speed (rpm): flat to base speed,
    constant power above it, tapering to 0 over the last 5 % of max_rpm."""
    peak, power, top = p["motor_peak_torque"], p["motor_peak_power"], p["motor_max_rpm"]
    base = power / peak * 60 / (2 * math.pi)
    taper = 0.95 * top
    curve = [[0.0, peak], [round(base, 1), peak]]
    for i in range(1, points + 1):
        rpm = base + (taper - base) * i / points
        curve.append([round(rpm, 1), round(power / (rpm * 2 * math.pi / 60), 2)])
    curve.append([round(top, 1), 0.0])
    return curve


def contact_not_applied(spec):
    """What CARLA cannot apply of the spec's contact properties: CARLA has no physics material
    per actor, so a body.contact is generated without, and named in the file (W24)."""
    if (spec.doc.get("body") or {}).get("contact"):
        return ["body.contact: CARLA has no per-actor physics material; the body collides with "
                "its blueprint's (the contact material is Isaac's)"]
    return []


def carla_physics_doc(spec):
    """CARLA's VehiclePhysicsControl for the spec's car, SI units, CARLA's wheel order (front
    left, front right, rear left, rear right); None when the spec gives CARLA no physics."""
    cp = spec.section("carla").get("physics")
    if not cp:
        return None
    g, i, w, t = (spec.doc[k] for k in ("geometry", "inertia", "wheels", "tires"))
    b, p, r = spec.doc["brakes"], spec.doc["powertrain"], spec.section("resistance")
    if p["model"] != "ev":
        raise ValueError(f"{spec.path}: carla_physics needs an ev powertrain, not {p['model']!r}")
    # The CoM from the rear axle (base_link), in the actor frame, less the CoM UE computes for
    # the body: CARLA's center_of_mass is that nudge (CARLA's y is ROS's -y).
    com = (cp["rear_axle_x"] + i["com_front_fraction"] * g["wheel_base"],
           i.get("com_lateral", 0.0), i["com_height"])
    nudge = [round(c - o, 4) for c, o in zip(com, cp["body_com"])]
    nudge[1] = -nudge[1] or 0.0
    drag = (r.get("drag_coefficient", 0.0) * r.get("frontal_area", 0.0) / cp["drag_area"])
    hold = b.get("hold_brake_torque", 0.0)

    def wheel(front):
        t = specmod.axle_tires(spec.doc, front)
        friction = t["friction"] * cp["tire_friction_scale"]
        return {"tire_friction": round(friction, 4),
                "damping_rate": w.get("damping_rate", 0.25),
                "max_steer_angle_deg": cp["max_steer_angle_deg"] if front else 0.0,
                "radius_m": g["wheel_radius"],
                "max_brake_torque": b["max_brake_torque"],
                "max_handbrake_torque": 0.0 if front else hold,
                "lat_stiff_max_load": t.get("lat_stiff_max_load", 3.0),
                "lat_stiff_value": t.get("lat_stiff_value", 20.0),
                "long_stiff_value": t.get("long_stiff_value", 3000.0)}

    return {
        "vehicle": {
            "mass": i["mass"],
            "center_of_mass": nudge,
            "drag_coefficient": round(drag, 4),
            "max_rpm": p["motor_max_rpm"],
            "torque_curve": motor_curve(p),
            "final_ratio": p["gear_ratio"],
            "forward_gears": [{"ratio": 1.0, "down_ratio": 0.0, "up_ratio": 0.0}],
            "use_gear_autobox": True,
            "moi": cp["engine_moi"],
            "damping_rate_full_throttle": cp["engine_damping"],
            "damping_rate_zero_throttle_clutch_engaged": cp["engine_damping"],
            "damping_rate_zero_throttle_clutch_disengaged": cp["engine_damping"],
        },
        "wheels": [wheel(True), wheel(True), wheel(False), wheel(False)],
        # What CARLA's model lacks, applied by the vehicle bridge to the commands it sends.
        "command_lags": {"brake_dead_time": b.get("dead_time", 0.0),
                         "brake_time_constant": b.get("time_constant", 0.0),
                         "motor_time_constant": p.get("motor_time_constant", 0.0)},
        **({"not_applied": contact_not_applied(spec)} if contact_not_applied(spec) else {}),
    }


# CARLA 1.0's Chaos (roadmap 083 step 5): what the server takes from a client and what it keeps
# from the blueprint's wheel asset (CarlaWheeledVehicle.cpp, ApplyWheelPhysicsControl).
CHAOS_NOT_APPLIED = (
    "tires.lat_stiff_max_load, tires.lat_stiff_value, tires.friction_vs_slip: Chaos's "
    "cornering_stiffness, lateral_slip_graph, slip and skid thresholds are the asset's",
    "suspension (frequency, damping_ratio, travel): Chaos's spring, damper and travel are "
    "the asset's",
    "wheels.mass, geometry.wheel_width: the asset's wheel mass and width",
    "resistance.rolling_coefficient: Chaos has a speed-proportional loss of its own "
    "(measured 0.0099 x v m/s^2)",
    "brakes.abs, powertrain.traction_control, powertrain regen: off, as on 0.9.16 (the "
    "car's own ECU models them)",
)


def carla_physics_1_0_doc(spec):
    """CARLA 1.0's VehiclePhysicsControl (Chaos) for the spec's car, SI units, CARLA's wheel
    order; None when the spec gives CARLA 1.0 no calibration (carla.physics_1_0). The wheel
    radius and the reduction are the body's (spec.effective)."""
    cp = spec.section("carla").get("physics_1_0")
    if not cp:
        return None
    g, i, t = (spec.doc[k] for k in ("geometry", "inertia", "tires"))
    b, p, r = spec.doc["brakes"], spec.doc["powertrain"], spec.section("resistance")
    if p["model"] != "ev":
        raise ValueError(f"{spec.path}: carla_physics_1.0 needs an ev powertrain, not {p['model']!r}")
    # 1.0 takes the centre of mass as an absolute position in the actor frame (measured: z 0
    # puts it on the ground). The spec's load share is kept on the body's own wheel base.
    base = cp["front_axle_x"] - cp["rear_axle_x"]
    com = [round(cp["rear_axle_x"] + i["com_front_fraction"] * base, 4),
           round(-i.get("com_lateral", 0.0), 4) or 0.0, i["com_height"]]
    drag = r.get("drag_coefficient", 0.0) * r.get("frontal_area", 0.0) / cp["chassis_area"]
    curve = motor_curve(p)
    peak = max(tq for _, tq in curve)
    steer = round(math.degrees(g["max_steer_angle"]), 2)
    hold = b.get("hold_brake_torque", 0.0)
    drive = p.get("drive_axle", "rear")

    def wheel(front):
        friction = specmod.axle_tires(spec.doc, front)["friction"]
        return {"radius_m": g["wheel_radius"],
                "friction_force_multiplier": round(friction * cp["friction_multiplier_scale"], 4),
                "max_steer_angle_deg": steer if front else 0.0,
                "max_brake_torque": b["max_brake_torque"],
                "max_handbrake_torque": 0.0 if front else hold}

    return {
        "vehicle": {
            "mass": i["mass"],
            "center_of_mass": com,
            "drag_coefficient": round(drag, 4),
            "downforce_coefficient": 0.0,
            "max_torque": peak,
            "torque_curve": [[rpm, round(tq / peak, 4)] for rpm, tq in curve],
            "max_rpm": p["motor_max_rpm"],
            "idle_rpm": 0.0,
            "brake_effect": 0.0,
            "differential": drive,
            "final_ratio": p["gear_ratio"],
            "forward_gear_ratios": [1.0],
            "reverse_gear_ratios": [1.0],
            "change_up_rpm": p["motor_max_rpm"] * 2,
            "change_down_rpm": 0.0,
            "transmission_efficiency": 1.0,
            "steering_curve": [[0.0, 1.0], [300.0, 1.0]],
        },
        "wheels": [wheel(True), wheel(True), wheel(False), wheel(False)],
        "command_lags": {"brake_dead_time": b.get("dead_time", 0.0),
                         "brake_time_constant": b.get("time_constant", 0.0),
                         "motor_time_constant": p.get("motor_time_constant", 0.0)},
        "not_applied": list(CHAOS_NOT_APPLIED) + contact_not_applied(spec),
    }


def carla_physics_1_0(spec, header):
    doc = carla_physics_1_0_doc(spec)
    body = {"spec_hash": specmod.spec_hash(spec), "physics_hash": specmod.physics_hash(spec),
            "carla": "1.0", "blueprint": spec.section("body").get("catalogue_model", ""), **doc}
    return emit.comment(header) + emit.comment(
        "CARLA 1.0's physics for the car (VehiclePhysicsControl on Chaos): applied to the ego\n"
        "by the CARLA world server after the spawn (with SSv2) or by acb_bridge on attach\n"
        "(standalone), and read back by acb_bridge. SI units (the appliers convert: radius to\n"
        "centimetres); wheels front left, front right, rear left, rear right; center_of_mass\n"
        "is absolute in the actor frame (CARLA's axes, y right); torque_curve is the motor's,\n"
        "normalised to max_torque (N m). ABS and traction control off, a flat steering_curve.\n"
        "not_applied: what Chaos keeps from the blueprint's asset however it is asked.") \
        + emit.yaml_text(body)


def carla_physics(spec, header):
    doc = carla_physics_doc(spec)
    body = {"spec_hash": specmod.spec_hash(spec), "physics_hash": specmod.physics_hash(spec),
            "blueprint": spec.section("body").get("catalogue_model", ""), **doc}
    return emit.comment(header) + emit.comment(
        "CARLA's physics for the car (VehiclePhysicsControl): applied to the ego by the CARLA\n"
        "world server after the spawn (with SSv2) or by acb_bridge on attach (standalone), and\n"
        "read back by acb_bridge. SI units; wheels front left, front right, rear left, rear\n"
        "right; center_of_mass is CARLA's nudge of the body's own (CARLA's axes, y right).\n"
        "Not modelled by CARLA, named here: the rear-wheel drive (CARLA drives its 4W\n"
        "differential), ABS, regen and traction control, rolling resistance.") \
        + "".join(emit.comment(f"Not applied on CARLA: {n}") for n in contact_not_applied(spec)) \
        + emit.yaml_text(body)


# --- SSv2: the vehicle catalog --------------------------------------------------------------

# A scenario assigns these, every one, in its CatalogReference. The entry declares no defaults:
# SSv2 evaluates an entry's ParameterDeclarations in the entry's own scope, where they would
# shadow the reference's assignments (one scope out), so a declared default always won.
ASSIGNED = ("maxSpeed", "maxAcceleration", "maxDeceleration", "maxSteering")


def _x(v):
    """A number as the catalog writes it: millimetres, trailing zeros off."""
    text = f"{v:.3f}".rstrip("0")
    text = text + "0" if text.endswith(".") else text
    return "0.0" if text in ("-0.0", "0.0") else text


def catalog_entry(spec):
    (cx, cy, cz), (length, width, height) = specmod.box(spec)
    a = specmod.axles(spec)
    body = spec.section("body").get("catalogue_model", "")
    model3d = f' model3d="{body}"' if body else ""
    axle = (f'wheelDiameter="{_x(2 * a["wheel_radius"])}" trackWidth="{_x(a["track"])}" '
            f'positionZ="{_x(a["wheel_radius"])}"')
    return f"""    <Vehicle name="{spec.name}" vehicleCategory="car"{model3d}>
      <BoundingBox>
        <Center x="{_x(cx)}" y="{_x(cy)}" z="{_x(cz)}" />
        <Dimensions width="{_x(width)}" height="{_x(height)}" length="{_x(length)}" />
      </BoundingBox>
      <Performance maxSpeed="$maxSpeed" maxAcceleration="$maxAcceleration" maxDeceleration="$maxDeceleration" />
      <Axles>
        <FrontAxle maxSteering="$maxSteering" positionX="{_x(a["front_x"])}" {axle} />
        <RearAxle maxSteering="0.0" positionX="0.0" {axle} />
      </Axles>
      <Properties>
        <Property name="scensim.spec_hash" value="{specmod.spec_hash(spec)}" />
      </Properties>
    </Vehicle>
"""


def catalog(specs, rels):
    lines = "".join(f"<!-- {emit.header(s, TOOL, r)} -->\n" for s, r in zip(specs, rels))
    entries = "".join(catalog_entry(s) for s in specs)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
{lines}<!-- The simulated cars as SSv2 vehicles: one entry per spec, its body box and axles in the
     spec's base_link (the rear axle centre on the ground), model3d its body. A scenario refers to
     an entry with CatalogReference and assigns maxSpeed, maxAcceleration, maxDeceleration and
     maxSteering, all four (the entry declares no defaults: in SSv2 a declared default would
     shadow the assignment); the box and the axles are the car's and are not parameters. -->
<OpenSCENARIO>
  <FileHeader author="sim_vehicle_spec" date="2026-10-23T00:00:00" description="Simulated cars, generated from sim_vehicle_spec's specs" revMajor="1" revMinor="1" />
  <Catalog name="vehicles">
{entries}  </Catalog>
</OpenSCENARIO>
"""


# --- driver ---------------------------------------------------------------------------------

def as_bodies(spec):
    """[(body, the spec as it)] in the spec's order, the default first; [(None, spec)] for a
    spec without bodies."""
    names = specmod.bodies(spec)
    return [(b, specmod.effective(spec, b)) for b in names] or [(None, spec)]


def carla_files(spec, header):
    """{file name: content} of the spec's CARLA files, each from the body that calibrates it;
    none for a car with no CARLA body (body.catalogue_model empty: its catalog entry has no
    model3d, and nothing spawns it on CARLA; scensim 094 P2's MuSHR)."""
    out = {}
    if not spec.section("body").get("catalogue_model"):
        return out
    for i, (body, e) in enumerate(as_bodies(spec)):
        if i == 0:
            out["carla_sensors.yaml"] = carla_sensors(e, header)
        for name, doc_of, text_of in (("carla_physics.yaml", carla_physics_doc, carla_physics),
                                      ("carla_physics_1.0.yaml", carla_physics_1_0_doc,
                                       carla_physics_1_0)):
            if doc_of(e) is None:
                continue
            if name in out:
                raise ValueError(f"{spec.path}: two bodies calibrate {name}")
            out[name] = text_of(e, header)
    return out


def catalog_name(body, default):
    return "vehicles.xosc" if body == default else f"{body}/vehicles.xosc"


def catalogs(specs, rels):
    """{catalog path under catalogs/: content}: each spec's default body in vehicles.xosc, its
    other bodies in <body>/vehicles.xosc."""
    groups = {}
    for s, rel in zip(specs, rels):
        builds = as_bodies(s)
        for body, e in builds:
            groups.setdefault(catalog_name(body, builds[0][0]), []).append((e, rel))
    return {name: catalog([e for e, _ in g], [r for _, r in g]) for name, g in groups.items()}


def outputs(root):
    """{path relative to root: content} for every spec under root/vehicles."""
    root = Path(root).resolve()
    paths = sorted((root / "vehicles").glob("*.yaml"))
    specs = [specmod.load(p) for p in paths]
    rels = [p.relative_to(root).as_posix() for p in paths]
    out = {}
    for s, rel in zip(specs, rels):
        for name, text in carla_files(s, emit.header(s, TOOL, rel)).items():
            out[f"generated/{s.name}/{name}"] = text
        for name, text in articulation.files(s, emit.header(s, TOOL, rel)).items():
            out[f"generated/{s.name}/{name}"] = text
    for name, text in catalogs(specs, rels).items():
        out[f"catalogs/{name}"] = text
    return out


def single(spec_path, catalog_path=None, spec_hash_path=None, carla_dir=None):
    """{absolute path: content} of one spec's outputs, each header naming the spec relative to
    the output."""
    import os

    spec_path = Path(spec_path).resolve()
    s = specmod.load(spec_path)
    rel = lambda out: os.path.relpath(spec_path, Path(out).resolve().parent)  # noqa: E731
    out = {}
    if catalog_path:
        # The default body's at catalog_path, each other body's in a directory beside it
        # (<dir>/<body>/<file>: SSv2 keeps one catalog of a name per directory).
        c = Path(catalog_path).resolve()
        for name, text in catalogs([s], [rel(c)]).items():
            dest = c if name == "vehicles.xosc" else c.parent / name.split("/")[0] / c.name
            out[str(dest)] = (text if dest == c else
                              catalog([specmod.effective(s, name.split("/")[0])],
                                      [os.path.relpath(spec_path, dest.parent)]))
    if spec_hash_path:
        out[str(Path(spec_hash_path).resolve())] = (
            emit.comment(emit.header(s, TOOL, rel(spec_hash_path)))
            + f"spec_hash: {specmod.spec_hash(s)}\n")
    if carla_dir:
        d = Path(carla_dir).resolve()
        for name, text in carla_files(s, emit.header(s, TOOL, rel(d / "x"))).items():
            out[str(d / name)] = text
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--spec", type=Path, help="one spec, anywhere (with --catalog, --spec-hash or --carla)")
    ap.add_argument("--catalog", type=Path, help="the spec's SSv2 vehicle catalog to write")
    ap.add_argument("--spec-hash", type=Path, help="a YAML with the spec's spec_hash to write")
    ap.add_argument("--carla", type=Path, help="a directory for the spec's CARLA files")
    args = ap.parse_args(argv)
    if args.spec is not None:
        if not (args.catalog or args.spec_hash or args.carla):
            ap.error("--spec needs --catalog, --spec-hash or --carla")
        out = single(args.spec, args.catalog, args.spec_hash, args.carla)
        if args.check:
            stale = emit.stale(out, "/")
            for p in stale:
                print(f"stale: {p}", file=sys.stderr)
            return 1 if stale else 0
        for p in emit.write(out, "/"):
            print(f"wrote {p}")
        return 0
    out = outputs(args.root)
    if args.check:
        stale = emit.stale(out, args.root)
        for p in stale:
            print(f"stale: {p} (run `just generate`)", file=sys.stderr)
        return 1 if stale else 0
    for p in emit.write(out, args.root):
        print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
