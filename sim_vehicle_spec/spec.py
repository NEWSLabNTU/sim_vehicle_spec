"""A vehicle spec: the one YAML a simulated car is described in (docs/spec-format.md). Pure.

`load(path)` reads and validates it. From a spec: `box` the body box and `axles` the wheel
layout in base_link, `sensors` each sensor with its base_link transform, `camera_intrinsics`
a pinhole camera's matrix, `spec_hash` its fingerprint and `physics_hash` the fingerprint of
what drives the car. Every generator works from these: this repository's (CARLA, the SSv2
catalog), autoware_sim_vehicle's (Autoware), autoware_isaac_bridge's (the Isaac rig).
"""
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

from .frames import compose, mount_transform

SPEC_VERSION = 2
# The powertrain models and drive axles a spec may name (autoware_isaac_bridge's aib.powertrain
# realizes them; its tests check the two lists agree).
MODELS = ("ideal", "physx", "ev", "fmu")
DRIVE_AXLES = ("rear", "front", "all")
SENSOR_TYPES = ("lidar", "camera", "imu", "gnss")
MOUNT_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")

# section -> (required keys, optional keys)
SECTIONS = {
    "body": ((), ("catalogue_model",)),
    "geometry": (("wheel_base", "wheel_tread", "wheel_radius", "wheel_width", "front_overhang",
                  "rear_overhang", "left_overhang", "right_overhang", "vehicle_height",
                  "max_steer_angle"), ()),
    "mirror": (("min_longitudinal_offset", "max_longitudinal_offset", "min_lateral_offset",
                "max_lateral_offset", "min_height_offset", "max_height_offset"), ()),
    "inertia": (("mass", "com_front_fraction", "com_height"), ("com_lateral", "inertia_box")),
    "wheels": (("mass",), ("damping_rate",)),
    "tires": (("friction",), ("lat_stiff_max_load", "lat_stiff_value", "long_stiff_value",
                              "friction_vs_slip")),
    "brakes": (("max_brake_torque",), ("hold_brake_torque", "distribution", "dead_time",
                                        "time_constant", "abs")),
    "powertrain": (("model", "max_drive_torque"),
                   ("drive_axle", "front_drive_share", "gear_ratio", "motor_peak_torque",
                    "motor_peak_power", "motor_max_rpm", "motor_time_constant",
                    "regen_max_torque", "regen_max_power", "regen_fade_speeds",
                    "traction_control")),
    "resistance": ((), ("rolling_coefficient", "drag_coefficient", "frontal_area",
                        "air_density")),
    "steering": ((), ("ackermann", "time_constant", "rate_limit")),
    "suspension": (("frequency", "damping_ratio", "travel"), ()),
    "sensor_kit": (("mount",), ()),
    # Per simulator: what only that simulator's model needs.
    "isaac": ((), ("physx",)),
    "carla": ((), ("physics",)),
}
# isaac.physx: the PhysX vehicle's solver settings
PHYSX_KEYS = ("substeps", "substep_threshold_speed", "min_active_slip_denominator",
              "min_passive_slip_denominator", "min_lateral_slip_denominator",
              "sticky_tire_threshold_speed", "sticky_tire_threshold_time",
              "longitudinal_sticky_tire_damping", "lateral_sticky_tire_damping")
# carla.physics: what CARLA's PhysX vehicle needs that the physics sections do not say (the
# body's own facts and CARLA's scales); the rest of carla_physics.yaml derives from the spec
CARLA_PHYSICS_KEYS = ("tire_friction_scale", "drag_area", "body_com", "rear_axle_x",
                      "max_steer_angle_deg", "engine_moi", "engine_damping")
# A sensor's carla: the key acb_bridge spawns it under (a TF frame of the URDF), the CARLA
# blueprint, and the attributes not derived from the shared fields.
CARLA_SENSOR_KEYS = ("key", "blueprint", "attributes")
TOP = {"spec_version", "name", "description", "sensors", "provenance", *SECTIONS}
SENSOR_COMMON = ("type", "calibration_key", "link", "frame", "topic", "mount", "isaac", "carla")
SENSOR_EXTRA = {
    "lidar": ("channels", "rate_hz", "range", "vertical_fov"),
    "camera": ("width", "height", "horizontal_fov", "rate_hz", "info_topic"),
    "imu": ("rate_hz",),
    "gnss": ("rate_hz",),
}
BRAKE_DISTRIBUTIONS = ("dynamic_axle_load", "static")
# Nested mappings: (section, key) -> their keys (all optional)
SUBSECTIONS = {
    ("brakes", "abs"): ("enabled", "slip_release", "slip_reapply", "release_rate",
                        "apply_rate", "min_speed"),
    ("powertrain", "traction_control"): ("enabled", "slip_limit", "slip_reapply",
                                         "release_rate", "apply_rate"),
    ("isaac", "physx"): PHYSX_KEYS,
    ("carla", "physics"): CARLA_PHYSICS_KEYS,
}
# What an `ev` powertrain cannot do without
EV_REQUIRED = ("gear_ratio", "motor_peak_torque", "motor_peak_power", "motor_max_rpm")
# Keys that moved, with where to
MOVED = {("powertrain", "rolling_resistance"):
         "resistance.rolling_coefficient (a road load applied through the physics)"}
# Top-level sections of version 1 and where they went
MOVED_TOP = {"autoware": "nothing: the Autoware package names follow `name` (<name>_description, "
                         "<name>_sensor_kit_description, ...)",
             "physx": "isaac.physx"}


class SpecError(ValueError):
    """A spec that is not valid, with the file and the key in the message."""


@dataclass
class Spec:
    path: Path
    doc: dict

    @property
    def name(self):
        return self.doc["name"]

    def section(self, name):
        return self.doc.get(name) or {}


def load(path):
    import yaml

    path = Path(path)
    doc = yaml.safe_load(path.read_text()) or {}
    validate(doc, str(path))
    return Spec(path=path, doc=doc)


def validate(doc, where="spec"):
    def fail(msg):
        raise SpecError(f"{where}: {msg}")

    if doc.get("spec_version") != SPEC_VERSION:
        fail(f"spec_version must be {SPEC_VERSION}, is {doc.get('spec_version')!r} "
             "(docs/spec-format.md, \"Version 2\")")
    if not doc.get("name"):
        fail("no name")
    for key, where_to in MOVED_TOP.items():
        if key in doc:
            fail(f"{key} is gone in version {SPEC_VERSION}: {where_to}")
    unknown = set(doc) - TOP
    if unknown:
        fail(f"unknown key(s) {sorted(unknown)}")
    for section, (required, optional) in SECTIONS.items():
        body = doc.get(section)
        if body is None:
            if required:
                fail(f"no {section}")
            continue
        if not isinstance(body, dict):
            fail(f"{section} is not a mapping")
        missing = [k for k in required if k not in body]
        if missing:
            fail(f"{section}: missing {missing}")
        for k in body:
            if (section, k) in MOVED:
                fail(f"{section}.{k} is gone: use {MOVED[(section, k)]}")
        extra = set(body) - set(required) - set(optional)
        if extra:
            fail(f"{section}: unknown key(s) {sorted(extra)}")
        for k, v in body.items():
            keys = SUBSECTIONS.get((section, k))
            if keys is None:
                continue
            if not isinstance(v, dict):
                fail(f"{section}.{k} is not a mapping")
            extra = set(v) - set(keys)
            if extra:
                fail(f"{section}.{k}: unknown key(s) {sorted(extra)}")
    _check_mount(doc["sensor_kit"]["mount"], "sensor_kit.mount", fail)
    pt = doc["powertrain"]
    model = pt["model"]
    if model not in MODELS:
        fail(f"powertrain.model {model!r}: one of {', '.join(MODELS)}")
    if model == "ev":
        missing = [k for k in EV_REQUIRED if k not in pt]
        if missing:
            fail(f"powertrain: an ev needs {missing}")
        for k in EV_REQUIRED:
            if not float(pt[k]) > 0:
                fail(f"powertrain.{k} must be positive")
    axle = pt.get("drive_axle", "rear")
    if axle not in DRIVE_AXLES:
        fail(f"powertrain.drive_axle {axle!r}: one of {', '.join(DRIVE_AXLES)}")
    fade = pt.get("regen_fade_speeds")
    if fade is not None and not (isinstance(fade, list) and len(fade) == 2
                                 and 0 <= float(fade[0]) < float(fade[1])):
        fail("powertrain.regen_fade_speeds must be [low, high] m/s, 0 <= low < high")
    dist = doc["brakes"].get("distribution", "dynamic_axle_load")
    if dist not in BRAKE_DISTRIBUTIONS:
        fail(f"brakes.distribution {dist!r}: one of {', '.join(BRAKE_DISTRIBUTIONS)}")
    g = doc["geometry"]
    for k in ("wheel_base", "wheel_tread", "wheel_radius", "wheel_width", "max_steer_angle"):
        if not float(g[k]) > 0:
            fail(f"geometry.{k} must be positive")
    if not 0.0 < float(doc["inertia"]["com_front_fraction"]) < 1.0:
        fail("inertia.com_front_fraction must be in (0, 1)")
    sensors = doc.get("sensors") or {}
    if not isinstance(sensors, dict):
        fail("sensors is not a mapping")
    keys, links = set(), set()
    for name, s in sensors.items():
        where_s = f"sensors.{name}"
        t = s.get("type")
        if t not in SENSOR_TYPES:
            fail(f"{where_s}.type {t!r}: one of {', '.join(SENSOR_TYPES)}")
        extra = set(s) - set(SENSOR_COMMON) - set(SENSOR_EXTRA[t])
        if extra:
            fail(f"{where_s}: unknown key(s) {sorted(extra)} for a {t}")
        for k in ("link", "frame", "topic", "mount"):
            if k not in s:
                fail(f"{where_s}: missing {k}")
        _check_mount(s["mount"], f"{where_s}.mount", fail)
        key = s.get("calibration_key", s["link"])
        if key in keys:
            fail(f"{where_s}: calibration_key {key!r} used twice")
        if s["link"] in links:
            fail(f"{where_s}: link {s['link']!r} used twice")
        keys.add(key)
        links.add(s["link"])
        if t == "camera":
            for k in ("width", "height", "horizontal_fov"):
                if k not in s:
                    fail(f"{where_s}: a camera needs {k}")
        carla = s.get("carla")
        if carla is not None:
            if not isinstance(carla, dict):
                fail(f"{where_s}.carla is not a mapping")
            extra = set(carla) - set(CARLA_SENSOR_KEYS)
            if extra:
                fail(f"{where_s}.carla: unknown key(s) {sorted(extra)}")
            if "blueprint" not in carla:
                fail(f"{where_s}.carla: missing blueprint")
    for key, entry in (doc.get("provenance") or {}).items():
        head = key.split(".")[0]
        if head not in TOP:
            fail(f"provenance.{key}: no such section")
        if not isinstance(entry, dict) or "basis" not in entry:
            fail(f"provenance.{key}: needs a basis")


def _check_mount(m, where, fail):
    if not isinstance(m, dict) or set(m) != set(MOUNT_KEYS):
        fail(f"{where} must have exactly {', '.join(MOUNT_KEYS)}")


def assumptions(spec):
    """The provenance keys marked `assumed: true`."""
    return sorted(k for k, v in (spec.doc.get("provenance") or {}).items() if v.get("assumed"))


def box(spec):
    """The chassis box from the geometry: (centre, dims) in base_link."""
    g = spec.doc["geometry"]
    length = g["wheel_base"] + g["front_overhang"] + g["rear_overhang"]
    width = g["wheel_tread"] + g["left_overhang"] + g["right_overhang"]
    cx = (g["wheel_base"] + g["front_overhang"] - g["rear_overhang"]) / 2
    cy = (g["left_overhang"] - g["right_overhang"]) / 2
    h = g["vehicle_height"]
    return (cx, cy, h / 2), (length, width, h)


def sensors(spec):
    """name -> dict of the sensor with `key` (calibration key), `base_link` (translation,
    quaternion wxyz) and `enabled` (on Isaac)."""
    kit = mount_transform(spec.doc["sensor_kit"]["mount"])
    out = {}
    for name, s in (spec.doc.get("sensors") or {}).items():
        t, q = compose(kit, mount_transform(s["mount"]))
        isaac = dict(s.get("isaac") or {})
        out[name] = dict(s, key=s.get("calibration_key", s["link"]), base_link=(t, q),
                         enabled=bool(isaac.pop("enabled", True)), isaac=isaac)
    return out


def camera_intrinsics(width, height, horizontal_fov_deg):
    """Pinhole K (row-major 3x3) for a camera with square pixels, principal point centred."""
    fx = (width / 2) / math.tan(math.radians(horizontal_fov_deg) / 2)
    return [fx, 0.0, width / 2, 0.0, fx, height / 2, 0.0, 0.0, 1.0]


def axles(spec):
    """The wheel layout in base_link (the rear axle centre): front axle x, track, wheel radius."""
    g = spec.doc["geometry"]
    return {"front_x": g["wheel_base"], "rear_x": 0.0, "track": g["wheel_tread"],
            "wheel_radius": g["wheel_radius"]}


# What a fingerprint leaves out: words, not the car.
UNHASHED = ("description", "provenance")
# What drives the car: the pedal maps are measured against these (docs/spec-format.md).
PHYSICS_SECTIONS = ("geometry", "inertia", "wheels", "tires", "brakes", "powertrain",
                    "resistance", "steering", "suspension")


def _canonical(value):
    """JSON of a parsed spec value: keys sorted, numbers as floats or ints as written,
    so a comment, the key order or the layout never changes the fingerprint."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def spec_hash(spec):
    """The spec's fingerprint (SHA-256 of the canonical spec, words left out): every generated
    file carries it, and every part of a run reports the one it loaded (roadmap 051)."""
    doc = {k: v for k, v in spec.doc.items() if k not in UNHASHED}
    return hashlib.sha256(_canonical(doc).encode()).hexdigest()


def physics_hash(spec):
    """The fingerprint of what drives the car (PHYSICS_SECTIONS and carla.physics): the pedal
    maps measured on CARLA carry it, so moving a sensor does not invalidate them."""
    doc = {k: spec.doc.get(k) for k in PHYSICS_SECTIONS}
    doc["carla.physics"] = spec.section("carla").get("physics")
    return hashlib.sha256(_canonical(doc).encode()).hexdigest()
