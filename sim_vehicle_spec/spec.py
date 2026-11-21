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
MODELS = ("ideal", "physx", "ev", "fmu", "component")
DRIVE_AXLES = ("rear", "front", "all")
SENSOR_TYPES = ("lidar", "camera", "imu", "gnss")
MOUNT_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")

# section -> (required keys, optional keys)
SECTIONS = {
    "body": ((), ("catalogue_model", "ground_clearance", "contact")),
    "geometry": (("wheel_base", "wheel_tread", "wheel_radius", "wheel_width", "front_overhang",
                  "rear_overhang", "left_overhang", "right_overhang", "vehicle_height",
                  "max_steer_angle"), ()),
    "mirror": (("min_longitudinal_offset", "max_longitudinal_offset", "min_lateral_offset",
                "max_lateral_offset", "min_height_offset", "max_height_offset"), ()),
    "inertia": (("mass", "com_front_fraction", "com_height"), ("com_lateral", "inertia_box")),
    "wheels": (("mass",), ("damping_rate",)),
    "tires": (("friction",), ("lat_stiff_max_load", "lat_stiff_value", "long_stiff_value",
                              "friction_vs_slip", "front", "rear")),
    "brakes": (("max_brake_torque",), ("hold_brake_torque", "distribution", "dead_time",
                                        "time_constant", "abs")),
    "powertrain": (("model",),
                   ("max_drive_torque", "component", "params", "drive_axle", "front_drive_share", "gear_ratio", "motor_peak_torque",
                    "motor_peak_power", "motor_max_rpm", "motor_time_constant",
                    "regen_max_torque", "regen_max_power", "regen_fade_speeds",
                    "traction_control", "differential")),
    "resistance": ((), ("rolling_coefficient", "drag_coefficient", "frontal_area",
                        "air_density")),
    "steering": ((), ("ackermann", "time_constant", "rate_limit")),
    "suspension": (("frequency", "damping_ratio", "travel"), ()),
    "sensor_kit": (("mount",), ()),
    # Per simulator: what only that simulator's model needs.
    "isaac": ((), ("physx",)),
    "carla": ((), ("physics",)),
    # The car as an articulation (scensim 092 K2.2b): drives and contact that the generator
    # would otherwise choose (sim_vehicle_spec.articulation names its defaults)
    "articulation": ((), ("spin_drive", "steer_drive", "wheel_contact")),
}
# articulation.spin_drive / steer_drive: a joint drive
ARTICULATION_DRIVE_KEYS = ("type", "stiffness", "damping", "effort_limit", "velocity_limit")
ARTICULATION_DRIVE_TYPES = ("effort", "velocity", "position")
# isaac.physx: the PhysX vehicle's solver settings
PHYSX_KEYS = ("substeps", "substep_threshold_speed", "min_active_slip_denominator",
              "min_passive_slip_denominator", "min_lateral_slip_denominator",
              "sticky_tire_threshold_speed", "sticky_tire_threshold_time",
              "longitudinal_sticky_tire_damping", "lateral_sticky_tire_damping")
# carla.physics: what CARLA's PhysX vehicle needs that the physics sections do not say (the
# body's own facts and CARLA's scales); the rest of carla_physics.yaml derives from the spec
CARLA_PHYSICS_KEYS = ("tire_friction_scale", "drag_area", "body_com", "rear_axle_x",
                      "max_steer_angle_deg", "engine_moi", "engine_damping")
# carla.physics_1_0: what CARLA 1.0's Chaos vehicle needs that the physics sections do not say
# (roadmap 083 step 5): its scales and the body's own facts, measured on the packaged 1.0. The
# rest of carla_physics_1.0.yaml derives from the spec. Not in physics_hash: a calibration of
# how 1.0 reproduces the spec's car does not change the car the pedal maps were measured on.
CARLA_PHYSICS_1_0_KEYS = ("friction_multiplier_scale", "chassis_area", "front_axle_x",
                          "rear_axle_x")
# A sensor's carla: the key acb_bridge spawns it under (a TF frame of the URDF), the CARLA
# blueprint, and the attributes not derived from the shared fields.
CARLA_SENSOR_KEYS = ("key", "blueprint", "attributes", "intensity")
# A lidar's carla.intensity (with sensor.lidar.ray_cast_semantic): each point's intensity is
# reflectance[its semantic tag] x cos(incidence), clamped to 0-255 -- a Lambertian surface
# seen by a sensor that reports reflectivity, as a real lidar's intensity is. CARLA's own
# ray-cast intensity is only the atmosphere's attenuation by range (1.0 with none), the same
# on the road as on a car.
SEMANTIC_LIDAR = "sensor.lidar.ray_cast_semantic"
LIDAR_INTENSITY_KEYS = ("default", "reflectance")
# CARLA 0.9.16's semantic tags (carla.CityObjectLabel), by id: the names reflectance takes.
CARLA_SEMANTIC_TAGS = ("NONE", "Roads", "Sidewalks", "Buildings", "Walls", "Fences", "Poles",
                       "TrafficLight", "TrafficSigns", "Vegetation", "Terrain", "Sky",
                       "Pedestrians", "Rider", "Car", "Truck", "Bus", "Train", "Motorcycle",
                       "Bicycle", "Static", "Dynamic", "Other", "Water", "RoadLines", "Ground",
                       "Bridge", "RailTrack", "GuardRail")
TOP = {"spec_version", "name", "description", "sensors", "provenance", "bodies", "simulators",
       *SECTIONS}
# bodies.<name>: a simulator's body for the car (roadmap 083). The spec's own geometry is the
# real car's; a simulator drives the body its asset has, and a body says how it differs: its
# measured geometry (all of it), the powertrain values scaled to its wheel, its catalogue model
# and its simulator's calibration. `effective(spec, body)` is the spec as that body.
BODY_KEYS = ("catalogue_model", "geometry", "powertrain", "carla")
# The powertrain values that depend on the body's wheel (the force at the road kept the car's)
BODY_POWERTRAIN_KEYS = ("gear_ratio", "regen_max_torque")
# simulators.<simulator>: the body a simulator runs when its version does not name one
SIMULATORS = ("isaac",)
SENSOR_COMMON = ("type", "calibration_key", "link", "frame", "topic", "mount", "isaac", "carla")
SENSOR_EXTRA = {
    "lidar": ("channels", "rate_hz", "range", "vertical_fov", "horizontal_fov", "beams",
              "min_range", "message"),
    "camera": ("width", "height", "horizontal_fov", "rate_hz", "info_topic"),
    "imu": ("rate_hz",),
    "gnss": ("rate_hz",),
}
# a lidar's `message`: a spinning lidar's cloud, or a planar scanner's LaserScan (scensim 091 W19)
LIDAR_MESSAGES = ("point_cloud", "laser_scan")
BRAKE_DISTRIBUTIONS = ("dynamic_axle_load", "static")
# powertrain.differential.type: each driven axle's differential
DIFFERENTIALS = ("open", "locked", "lsd")
# Nested mappings: (section, key) -> their keys (all optional)
# tires.front, tires.rear: one axle's tyres where they differ from `tires` (scensim roadmap 091
# W24): the keys each simulator takes per wheel. friction_vs_slip stays the car's.
AXLE_TIRE_KEYS = ("friction", "lat_stiff_max_load", "lat_stiff_value", "long_stiff_value")
# body.contact: the body's physics material where it touches walls and objects (W24); absent,
# the simulator's default material. Not a physics section's: collisions do not drive the car.
CONTACT_KEYS = ("restitution", "static_friction", "dynamic_friction", "combine")
COMBINE_MODES = ("average", "min", "max", "multiply")
SUBSECTIONS = {
    ("tires", "front"): AXLE_TIRE_KEYS,
    ("tires", "rear"): AXLE_TIRE_KEYS,
    ("body", "contact"): CONTACT_KEYS,
    ("brakes", "abs"): ("enabled", "slip_release", "slip_reapply", "release_rate",
                        "apply_rate", "min_speed"),
    ("powertrain", "traction_control"): ("enabled", "slip_limit", "slip_reapply",
                                         "release_rate", "apply_rate"),
    ("powertrain", "differential"): ("type", "preload", "power_lock", "coast_lock"),
    ("isaac", "physx"): PHYSX_KEYS,
    ("carla", "physics"): CARLA_PHYSICS_KEYS,
    ("carla", "physics_1_0"): CARLA_PHYSICS_1_0_KEYS,
    ("articulation", "spin_drive"): ARTICULATION_DRIVE_KEYS,
    ("articulation", "steer_drive"): ARTICULATION_DRIVE_KEYS,
    ("articulation", "wheel_contact"): CONTACT_KEYS,
}
# A body's carla: the calibration of its simulator's version
BODY_CARLA_KEYS = ("physics", "physics_1_0")
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
    # The body this view of the spec is (`effective`), None for the spec as written
    body: str = None
    # The spec as written, when this is a body's view: its fingerprint is the file's
    source: dict = None

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


def bodies(spec):
    """The names of the spec's bodies (empty: the spec is its own body)."""
    return list((spec.source or spec.doc).get("bodies") or {})


def simulator_body(spec, simulator):
    """The body `simulator` runs (`simulators`), None when the spec names none."""
    return ((spec.source or spec.doc).get("simulators") or {}).get(simulator)


def effective(spec, body):
    """The spec as `body` (roadmap 083): its geometry, its powertrain scaling, its catalogue
    model and its simulator's calibration in place of the spec's. Every consumer that
    builds a simulator's car takes this. Its spec_hash is the file's; its physics_hash is the
    body's. A spec without bodies is its own body: `body` must then be None."""
    import copy

    source = spec.source or spec.doc
    all_bodies = source.get("bodies") or {}
    if body is None:
        if all_bodies:
            raise SpecError(f"{spec.path}: name a body, one of {', '.join(all_bodies)}")
        return spec
    if body not in all_bodies:
        raise SpecError(f"{spec.path}: no body {body!r} (bodies: {', '.join(all_bodies) or 'none'})")
    b = all_bodies[body]
    doc = copy.deepcopy({k: v for k, v in source.items() if k not in ("bodies", "simulators")})
    doc["geometry"] = copy.deepcopy(b["geometry"])
    doc["powertrain"].update(copy.deepcopy(b.get("powertrain") or {}))
    doc.setdefault("body", {})["catalogue_model"] = b["catalogue_model"]
    carla = dict(doc.get("carla") or {})
    carla.update(copy.deepcopy(b.get("carla") or {}))
    if carla:
        doc["carla"] = carla
    return Spec(path=spec.path, doc=doc, body=body, source=source)


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
    for k in ("spin_drive", "steer_drive"):
        d = (doc.get("articulation") or {}).get(k)
        if d is not None and d.get("type") not in ARTICULATION_DRIVE_TYPES:
            fail(f"articulation.{k}.type {d.get('type')!r}: one of "
                 f"{', '.join(ARTICULATION_DRIVE_TYPES)}")
    _check_mount(doc["sensor_kit"]["mount"], "sensor_kit.mount", fail)
    pt = doc["powertrain"]
    model = pt["model"]
    if model not in MODELS:
        fail(f"powertrain.model {model!r}: one of {', '.join(MODELS)}")
    if model == "component":
        # the car's control component is the user's (scensim roadmap 091): named by its entry
        # point in sim_vehicle_ecu's group, with its own parameters
        name = pt.get("component")
        if not (isinstance(name, str) and name.strip()):
            fail("powertrain: a component needs component: <entry point name> (scensim's "
                 "group scensim.vehicle_components)")
        if "params" in pt and not isinstance(pt["params"], dict):
            fail("powertrain.params is not a mapping")
    else:
        for k in ("component", "params"):
            if k in pt:
                fail(f"powertrain.{k}: only a component powertrain has it (model: component)")
        if "max_drive_torque" not in pt:
            fail("powertrain: missing ['max_drive_torque']")
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
    diff = pt.get("differential") or {}
    kind = diff.get("type", "open")
    if kind not in DIFFERENTIALS:
        fail(f"powertrain.differential.type {kind!r}: one of {', '.join(DIFFERENTIALS)}")
    for k in ("preload", "power_lock", "coast_lock"):
        if k in diff and kind != "lsd":
            fail(f"powertrain.differential.{k}: only an lsd has it")
        if k in diff and not float(diff[k]) >= 0:
            fail(f"powertrain.differential.{k} must be >= 0")
    for k in ("power_lock", "coast_lock"):
        if k in diff and not float(diff[k]) <= 1:
            fail(f"powertrain.differential.{k} must be a share of the axle torque, 0..1")
    dist = doc["brakes"].get("distribution", "dynamic_axle_load")
    if dist not in BRAKE_DISTRIBUTIONS:
        fail(f"brakes.distribution {dist!r}: one of {', '.join(BRAKE_DISTRIBUTIONS)}")
    g = doc["geometry"]
    for k in ("wheel_base", "wheel_tread", "wheel_radius", "wheel_width", "max_steer_angle"):
        if not float(g[k]) > 0:
            fail(f"geometry.{k} must be positive")
    gc = (doc.get("body") or {}).get("ground_clearance")
    if gc is not None and not 0.0 < float(gc) < float(g["wheel_radius"]):
        fail("body.ground_clearance must be positive and below the wheel radius")
    for axle in ("front", "rear"):
        over = doc["tires"].get(axle)
        if over is None:
            continue
        if not over:
            fail(f"tires.{axle} is empty: name what differs from tires, or leave it out")
        for k, v in over.items():
            if not float(v) > 0:
                fail(f"tires.{axle}.{k} must be positive")
    contact = (doc.get("body") or {}).get("contact")
    if contact is not None:
        if not contact:
            fail("body.contact is empty: name its restitution, frictions or combine mode")
        if "restitution" in contact and not 0.0 <= float(contact["restitution"]) <= 1.0:
            fail("body.contact.restitution must be in [0, 1]")
        for k in ("static_friction", "dynamic_friction"):
            if k in contact and not float(contact[k]) >= 0.0:
                fail(f"body.contact.{k} must be >= 0")
        mode = contact.get("combine", "average")
        if mode not in COMBINE_MODES:
            fail(f"body.contact.combine {mode!r}: one of {', '.join(COMBINE_MODES)}")
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
        if t == "lidar":
            _check_planar(s, where_s, fail)
        carla = s.get("carla")
        if carla is not None:
            if not isinstance(carla, dict):
                fail(f"{where_s}.carla is not a mapping")
            extra = set(carla) - set(CARLA_SENSOR_KEYS)
            if extra:
                fail(f"{where_s}.carla: unknown key(s) {sorted(extra)}")
            if "blueprint" not in carla:
                fail(f"{where_s}.carla: missing blueprint")
            _check_intensity(t, carla, f"{where_s}.carla", fail)
    _check_bodies(doc, fail)
    for key, entry in (doc.get("provenance") or {}).items():
        head = key.split(".")[0]
        if head not in TOP:
            fail(f"provenance.{key}: no such section")
        if not isinstance(entry, dict) or "basis" not in entry:
            fail(f"provenance.{key}: needs a basis")


def _check_bodies(doc, fail):
    bodies = doc.get("bodies")
    if bodies is None:
        if doc.get("simulators"):
            fail("simulators: no bodies to name")
        return
    if not isinstance(bodies, dict) or not bodies:
        fail("bodies is not a mapping of body names")
    geo_keys = SECTIONS["geometry"][0]
    for name, b in bodies.items():
        where = f"bodies.{name}"
        if not isinstance(b, dict):
            fail(f"{where} is not a mapping")
        extra = set(b) - set(BODY_KEYS)
        if extra:
            fail(f"{where}: unknown key(s) {sorted(extra)}")
        for k in ("catalogue_model", "geometry"):
            if k not in b:
                fail(f"{where}: missing {k}")
        g = b["geometry"]
        missing = [k for k in geo_keys if k not in g]
        if missing:
            fail(f"{where}.geometry: missing {missing} (a body states all of its geometry)")
        extra = set(g) - set(geo_keys)
        if extra:
            fail(f"{where}.geometry: unknown key(s) {sorted(extra)}")
        extra = set(b.get("powertrain") or {}) - set(BODY_POWERTRAIN_KEYS)
        if extra:
            fail(f"{where}.powertrain: unknown key(s) {sorted(extra)} "
                 f"(a body scales only {', '.join(BODY_POWERTRAIN_KEYS)})")
        carla = b.get("carla") or {}
        extra = set(carla) - set(BODY_CARLA_KEYS)
        if extra:
            fail(f"{where}.carla: unknown key(s) {sorted(extra)}")
        for k, v in carla.items():
            extra = set(v) - set(SUBSECTIONS[("carla", k)])
            if extra:
                fail(f"{where}.carla.{k}: unknown key(s) {sorted(extra)}")
    for sim, name in (doc.get("simulators") or {}).items():
        if sim not in SIMULATORS:
            fail(f"simulators.{sim}: one of {', '.join(SIMULATORS)}")
        if name not in bodies:
            fail(f"simulators.{sim}: no body {name!r}")
    if (doc.get("carla") or {}).get("physics") is not None:
        fail("carla.physics with bodies: it is a body's (bodies.<name>.carla.physics)")


def _check_intensity(sensor_type, carla, where, fail):
    """carla.intensity: on a semantic lidar, and only there; values 0-255, tags CARLA's."""
    semantic = carla["blueprint"] == SEMANTIC_LIDAR
    if "intensity" not in carla:
        if semantic:
            fail(f"{where}: a {SEMANTIC_LIDAR} needs intensity (its points carry no intensity)")
        return
    if sensor_type != "lidar" or not semantic:
        fail(f"{where}.intensity: only a lidar of blueprint {SEMANTIC_LIDAR} takes it")
    m = carla["intensity"]
    if not isinstance(m, dict) or set(m) - set(LIDAR_INTENSITY_KEYS) or "reflectance" not in m:
        fail(f"{where}.intensity: a mapping of reflectance (and default)")
    r = m["reflectance"]
    if not isinstance(r, dict) or not r:
        fail(f"{where}.intensity.reflectance: a mapping of semantic tag to 0-255")
    unknown = sorted(set(r) - set(CARLA_SEMANTIC_TAGS))
    if unknown:
        fail(f"{where}.intensity.reflectance: unknown tag(s) {unknown}; CARLA's are "
             f"{', '.join(CARLA_SEMANTIC_TAGS)}")
    for k, v in [*r.items(), ("default", m.get("default", 0))]:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 <= v <= 255:
            fail(f"{where}.intensity: {k} {v!r} is not in 0-255")


def _check_planar(s, where, fail):
    """A lidar's `message` and its planar fields: `laser_scan` is a one-ring scanner (channels
    1, vertical_fov [a, a]) whose `beams` span `horizontal_fov` inclusively, as urg_node counts
    a UST-10LX's 1081 over 270 deg; its step must divide 360 deg into whole columns (Isaac
    fires a full turn and keeps the scan's window)."""
    msg = s.get("message", "point_cloud")
    if msg not in LIDAR_MESSAGES:
        fail(f"{where}.message {msg!r}: one of {', '.join(LIDAR_MESSAGES)}")
    hfov = s.get("horizontal_fov")
    if hfov is not None and not (isinstance(hfov, (int, float)) and 0 < hfov <= 360):
        fail(f"{where}.horizontal_fov {hfov!r}: degrees, in (0, 360]")
    if s.get("min_range") is not None and not (isinstance(s["min_range"], (int, float))
                                               and 0 <= s["min_range"] < s.get("range", math.inf)):
        fail(f"{where}.min_range {s['min_range']!r}: m, 0 or more and below range")
    if msg != "laser_scan":
        return
    for k in ("channels", "rate_hz", "range", "vertical_fov", "horizontal_fov", "beams"):
        if k not in s:
            fail(f"{where}: a laser_scan lidar needs {k}")
    v = s["vertical_fov"]
    if s["channels"] != 1 or not (isinstance(v, (list, tuple)) and len(v) == 2 and v[0] == v[1]):
        fail(f"{where}: a laser_scan lidar has one ring: channels 1, vertical_fov [a, a]")
    beams = s["beams"]
    if not (isinstance(beams, int) and beams >= 2):
        fail(f"{where}.beams {beams!r}: a whole number, 2 or more")
    columns = (beams - 1) * 360.0 / hfov
    if abs(columns - round(columns)) > 1e-6:
        fail(f"{where}: {beams} beams over {hfov} deg is a step of {hfov / (beams - 1):g} deg, "
             "which does not divide 360 deg into whole columns")


def planar_columns(sensor):
    """A laser_scan lidar's columns over a full turn: (beams - 1) x 360 / horizontal_fov."""
    return int(round((sensor["beams"] - 1) * 360.0 / sensor["horizontal_fov"]))


def lidar_reflectance(carla):
    """A semantic lidar's reflectance by tag id (CARLA_SEMANTIC_TAGS), the default for the
    tags the spec does not name."""
    m = carla["intensity"]
    d = float(m.get("default", 0))
    return [float(m["reflectance"].get(name, d)) for name in CARLA_SEMANTIC_TAGS]


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


def axle_tires(doc, front):
    """One axle's tyres: `tires` with that axle's override (tires.front or tires.rear) on top."""
    t = doc["tires"]
    out = {k: v for k, v in t.items() if k not in ("front", "rear")}
    out.update(t.get("front" if front else "rear") or {})
    return out


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
    doc = {k: v for k, v in (spec.source or spec.doc).items() if k not in UNHASHED}
    return hashlib.sha256(_canonical(doc).encode()).hexdigest()


def physics_hash(spec):
    """The fingerprint of what drives the car (PHYSICS_SECTIONS and carla.physics): the pedal
    maps measured on CARLA carry it, so moving a sensor does not invalidate them. A body's
    (spec.effective): a spec with bodies has no physics_hash of its own."""
    if spec.source is None and spec.doc.get("bodies"):
        raise SpecError(f"{spec.path}: physics_hash is a body's: spec.effective(spec, body), "
                        f"body one of {', '.join(spec.doc['bodies'])}")
    doc = {k: spec.doc.get(k) for k in PHYSICS_SECTIONS}
    doc["carla.physics"] = spec.section("carla").get("physics")
    return hashlib.sha256(_canonical(doc).encode()).hexdigest()
