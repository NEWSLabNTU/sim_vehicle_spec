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


# --- SSv2: the vehicle catalog --------------------------------------------------------------

# A scenario assigns these; the catalog's defaults are the spec's envelope.
PERFORMANCE = ("maxSpeed", "maxAcceleration", "maxDeceleration")


def _x(v):
    """A number as the catalog writes it: millimetres, trailing zeros off."""
    text = f"{v:.3f}".rstrip("0")
    text = text + "0" if text.endswith(".") else text
    return "0.0" if text in ("-0.0", "0.0") else text


def performance(spec):
    """The envelope a scenario's SSv2 may drive the car to, when it assigns none: top speed from
    the motor's top speed, acceleration and deceleration from what the tires can carry."""
    p, g = spec.doc["powertrain"], spec.doc["geometry"]
    top = p.get("motor_max_rpm", 0.0) / p.get("gear_ratio", 1.0) * 2 * 3.141592653589793 / 60 \
        * g["wheel_radius"] if p.get("model") == "ev" else 50.0
    grip = spec.doc["tires"]["friction"] * 9.81
    return {"maxSpeed": round(top, 1), "maxAcceleration": round(grip, 1),
            "maxDeceleration": round(grip, 1)}


def catalog_entry(spec):
    (cx, cy, cz), (length, width, height) = specmod.box(spec)
    a = specmod.axles(spec)
    perf = performance(spec)
    steering = spec.doc["geometry"]["max_steer_angle"]
    body = spec.section("body").get("catalogue_model", "")
    model3d = f' model3d="{body}"' if body else ""
    decl = "".join(
        f'\n        <ParameterDeclaration name="{k}" parameterType="double" value="{perf[k]}" />'
        for k in PERFORMANCE) + (
        f'\n        <ParameterDeclaration name="maxSteering" parameterType="double" '
        f'value="{_x(steering)}" />')
    axle = (f'wheelDiameter="{_x(2 * a["wheel_radius"])}" trackWidth="{_x(a["track"])}" '
            f'positionZ="{_x(a["wheel_radius"])}"')
    return f"""    <Vehicle name="{spec.name}" vehicleCategory="car"{model3d}>
      <ParameterDeclarations>{decl}
      </ParameterDeclarations>
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
     an entry with CatalogReference and may assign maxSpeed, maxAcceleration, maxDeceleration and
     maxSteering; the box and the axles are the car's and are not parameters. -->
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
