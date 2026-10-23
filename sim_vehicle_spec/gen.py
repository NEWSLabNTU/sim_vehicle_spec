"""The outputs this repository owns, from every spec under vehicles/ (roadmap 051):

    generated/<name>/carla_sensors.yaml   the sensors acb_bridge spawns on CARLA
    catalogs/vehicles.xosc               the SSv2 vehicle catalog (one entry per spec)

    python3 -m sim_vehicle_spec.gen [--root .]           # write
    python3 -m sim_vehicle_spec.gen [--root .] --check   # exit 1 when one is stale

Every file starts with a header naming the spec and its spec_hash. Other repositories generate
their own outputs with this library (autoware_sim_vehicle: Autoware's packages;
autoware_isaac_bridge: the Isaac rig).
"""
import argparse
import math
import sys
from pathlib import Path

from . import emit
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
        sensors[carla.get("key", s["link"])] = {"blueprint": carla["blueprint"],
                                                "parameters": carla_attributes(s)}
    doc = {"spec_hash": specmod.spec_hash(spec),
           "vehicle": {"blueprint": spec.section("body").get("catalogue_model", "")},
           "sensors": sensors}
    return emit.comment(header) + emit.comment(
        "The sensors acb_bridge spawns on CARLA, keyed by the URDF frame each is mounted at\n"
        "(the mounts come from Autoware's robot_description), with their blueprint attributes.") \
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
    friction = t["friction"] * cp["tire_friction_scale"]
    hold = b.get("hold_brake_torque", 0.0)

    def wheel(front):
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
    }


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
        <Property name="swb.spec_hash" value="{specmod.spec_hash(spec)}" />
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

def outputs(root):
    """{path relative to root: content} for every spec under root/vehicles."""
    root = Path(root).resolve()
    paths = sorted((root / "vehicles").glob("*.yaml"))
    specs = [specmod.load(p) for p in paths]
    rels = [p.relative_to(root).as_posix() for p in paths]
    out = {}
    for s, rel in zip(specs, rels):
        out[f"generated/{s.name}/carla_sensors.yaml"] = carla_sensors(s, emit.header(s, TOOL, rel))
        if carla_physics_doc(s) is not None:
            out[f"generated/{s.name}/carla_physics.yaml"] = carla_physics(
                s, emit.header(s, TOOL, rel))
    out["catalogs/vehicles.xosc"] = catalog(specs, rels)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
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
