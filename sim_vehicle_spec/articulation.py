"""The car as an articulation (scensim roadmap 092 K2.2b): links, joints, drives, masses and
contact materials, in SI units and base_link's frame (x forward, y left, z up; the rear axle's
centre on the ground), for any simulator that builds a car from rigid bodies and joints rather
than from a vehicle model. Neutral: no simulator is imported here; scensim_batch turns the file
into an Isaac Lab ArticulationCfg.

    generated/<name>/articulation.yaml          a spec without bodies
    generated/<name>/articulation_<body>.yaml   each body of a spec with bodies

The structure is a chassis with four corners. Each corner hangs from the chassis on a prismatic
suspension joint (z), and its wheel spins on a revolute joint (y). A front corner has a revolute
steer joint (z) between the two:

    chassis -[<c>_suspension]-> <c>_upright -[<c>_steer]-> <c>_knuckle -[<c>_spin]-> <c>_wheel
    chassis -[<c>_suspension]-> <c>_upright -[<c>_spin]-> <c>_wheel          (rear)

What comes from the spec, and what is this generator's own choice, is said per value below and
in `choices:` of every file, so a difference from another model is named, not hidden:

- the chassis is a box over the body's footprint (wheel base, overhangs, tread, height above the
  ground clearance) carrying the sprung mass at the spec's centre of mass and, given
  `inertia.inertia_box`, that box's inertia;
- a corner's unsprung mass is `wheels.mass`, split 0.8 wheel and 0.1 each to the upright and
  knuckle (rear: 0.9 wheel); the wheel's spin inertia is the spec's, 0.5 x wheels.mass x r^2;
- the suspension spring is the spec's ride frequency on the corner's sprung mass, its damper the
  spec's damping ratio; q > 0 compresses it, and its drive targets minus the static sag, so the
  car rests at its design height (q = 0);
- the wheel spins under an effort drive (the powertrain's torque, applied by the consumer) unless
  the spec's `articulation.spin_drive` names another drive (a velocity drive, as an RC car's);
- the steer is a position drive, 10 Hz and critically damped on the knuckle and wheel's inertia
  about z, unless `articulation.steer_drive` gives one;
- the tyre is a rigid cylinder in Coulomb contact: static and dynamic friction `tires.friction`
  per axle, combined by multiplication, unless `articulation.wheel_contact` says otherwise. It
  has no slip curve and no load sensitivity: that is the named difference from a vehicle model.
"""
import math

from . import emit
from . import spec as specmod

G = 9.81
CORNERS = ("front_left", "front_right", "rear_left", "rear_right")
UPRIGHT_SHARE = 0.1          # of wheels.mass, each of the upright and the knuckle
STEER_BANDWIDTH_HZ = 10.0    # the steer's position drive when the spec gives none
STEER_DAMPING_RATIO = 1.0
DRIVE_TYPES = ("effort", "velocity", "position")
DRIVE_KEYS = ("type", "stiffness", "damping", "effort_limit", "velocity_limit")


def _box_inertia(m, lx, ly, lz):
    return [m * (ly * ly + lz * lz) / 12, m * (lx * lx + lz * lz) / 12,
            m * (lx * lx + ly * ly) / 12]


def _cylinder_inertia(m, r, w):
    """About x, y (the axis), z for a cylinder whose axis is y."""
    side = m * (3 * r * r + w * w) / 12
    return [side, m * r * r / 2, side]


def _axle_friction(spec, axle):
    tires = spec.section("tires")
    return (tires.get(axle) or {}).get("friction", tires["friction"])


def articulation_doc(spec):
    """The articulation of `spec` (a spec without bodies, or spec.effective(spec, body))."""
    geo, inertia = spec.section("geometry"), spec.section("inertia")
    wheels, susp = spec.section("wheels"), spec.section("suspension")
    power, brakes = spec.section("powertrain"), spec.section("brakes")
    body = spec.section("body")
    art = spec.section("articulation")

    L, T, r, w = geo["wheel_base"], geo["wheel_tread"], geo["wheel_radius"], geo["wheel_width"]
    m_total, m_corner = inertia["mass"], wheels["mass"]
    m_sprung = m_total - 4 * m_corner
    if m_sprung <= 0:
        raise specmod.SpecError(f"{spec.path}: inertia.mass {m_total} is not more than four "
                                f"wheels.mass {m_corner}")
    front = inertia["com_front_fraction"]
    clearance = body.get("ground_clearance", 0.0)

    length = L + geo["front_overhang"] + geo["rear_overhang"]
    width = T + geo["left_overhang"] + geo["right_overhang"]
    height = geo["vehicle_height"] - clearance
    box_center = [(L + geo["front_overhang"] - geo["rear_overhang"]) / 2,
                  (geo["left_overhang"] - geo["right_overhang"]) / 2, clearance + height / 2]
    com = [front * L, inertia.get("com_lateral", 0.0), inertia["com_height"]]
    ibox = inertia.get("inertia_box") or [length, width, height]
    links = {"chassis": {
        "shape": {"box": [length, width, height], "center": box_center},
        "mass": m_sprung, "com": com, "inertia": _box_inertia(m_sprung, *ibox)}}
    joints = {}

    k_steer = None
    for c in CORNERS:
        is_front = c.startswith("front")
        x = L if is_front else 0.0
        y = T / 2 if c.endswith("left") else -T / 2
        share = front if is_front else 1 - front
        m_sp = m_sprung * share / 2                         # this corner's sprung mass
        wn = 2 * math.pi * susp["frequency"]
        k = m_sp * wn * wn
        c_damp = 2 * susp["damping_ratio"] * math.sqrt(k * m_sp)
        sag = m_sp * G / k
        hub = [x, y, r]
        m_wheel = m_corner * (1 - (2 if is_front else 1) * UPRIGHT_SHARE)
        spin_I = 0.5 * m_corner * r * r
        wheel_I = _cylinder_inertia(m_wheel, r, w)
        wheel_I[1] = spin_I
        small = m_corner * UPRIGHT_SHARE
        small_I = _box_inertia(small, r / 2, w / 2, r / 2)
        links[f"{c}_upright"] = {"mass": small, "com": hub, "inertia": small_I, "position": hub}
        joints[f"{c}_suspension"] = {
            "type": "prismatic", "parent": "chassis", "child": f"{c}_upright", "origin": hub,
            "axis": [0, 0, 1], "limits": [-susp["travel"] / 2, susp["travel"] / 2],
            "drive": {"type": "position", "stiffness": k, "damping": c_damp, "target": -sag}}
        parent = f"{c}_upright"
        if is_front:
            links[f"{c}_knuckle"] = {"mass": small, "com": hub, "inertia": small_I,
                                     "position": hub}
            if k_steer is None:
                I_z = small_I[2] + wheel_I[2]
                if art.get("steer_drive"):
                    steer = dict(art["steer_drive"])
                else:
                    wn_s = 2 * math.pi * STEER_BANDWIDTH_HZ
                    steer = {"type": "position", "stiffness": I_z * wn_s * wn_s,
                             "damping": 2 * STEER_DAMPING_RATIO * I_z * wn_s}
                k_steer = steer
            joints[f"{c}_steer"] = {
                "type": "revolute", "parent": parent, "child": f"{c}_knuckle", "origin": hub,
                "axis": [0, 0, 1], "limits": [-geo["max_steer_angle"], geo["max_steer_angle"]],
                "drive": dict(k_steer)}
            parent = f"{c}_knuckle"
        mu = _axle_friction(spec, "front" if is_front else "rear")
        contact = {"static_friction": mu, "dynamic_friction": mu, "restitution": 0.0,
                   "combine": "multiply"}
        contact.update(art.get("wheel_contact") or {})
        links[f"{c}_wheel"] = {"shape": {"cylinder": {"radius": r, "width": w, "axis": "y"}},
                               "mass": m_wheel, "com": hub, "inertia": wheel_I,
                               "position": hub, "contact": contact}
        driven = _driven(power, is_front)
        if art.get("spin_drive"):
            spin = dict(art["spin_drive"])
        else:
            spin = {"type": "effort", "stiffness": 0.0, "damping": 0.0,
                    "effort_limit": max(_wheel_torque(power) if driven else 0.0,
                                        brakes["max_brake_torque"])}
            vmax = _wheel_speed_limit(power)
            if vmax:
                spin["velocity_limit"] = vmax
        joints[f"{c}_spin"] = {"type": "revolute", "parent": parent, "child": f"{c}_wheel",
                               "origin": hub, "axis": [0, 1, 0], "driven": driven,
                               "drive": spin}
    if body.get("contact"):
        links["chassis"]["contact"] = dict(body["contact"])
    return {
        "name": spec.name, "body": spec.body,
        "spec_hash": specmod.spec_hash(spec), "physics_hash": specmod.physics_hash(spec),
        "frame": "base_link: x forward, y left, z up; the rear axle's centre on the ground",
        "links": links, "joints": joints,
        "actuation": {
            "steer": "bicycle angle to the front steer joints' targets"
                     + (", as Ackermann angles" if spec.section("steering").get("ackermann",
                                                                             True) else ""),
            "wheel_base": L, "wheel_tread": T, "wheel_radius": r,
            "max_steer_angle": geo["max_steer_angle"],
            "max_brake_torque": brakes["max_brake_torque"],
            "drive_axle": power.get("drive_axle", "rear"),
        },
        "choices": [
            f"unsprung mass split: {1 - 2 * UPRIGHT_SHARE:g} wheel and {UPRIGHT_SHARE:g} each "
            f"upright and knuckle (rear {1 - UPRIGHT_SHARE:g} wheel)",
            "wheel spin inertia 0.5 x wheels.mass x r^2 (the spec's), whatever the link's mass",
            "suspension drive targets minus the static sag (q > 0 compresses), so the car "
            "rests at its design height (q = 0)",
            "the chassis box encloses the wheels: the consumer disables self-collisions",
            ("steer drive: the spec's articulation.steer_drive" if art.get("steer_drive") else
             f"steer drive: {STEER_BANDWIDTH_HZ:g} Hz, damping ratio {STEER_DAMPING_RATIO:g}, "
             "on the knuckle and wheel's inertia about z"),
            ("spin drive: the spec's articulation.spin_drive" if art.get("spin_drive") else
             "spin drive: effort (the consumer applies the powertrain's and brakes' torque)"),
            "tyre: a rigid cylinder in Coulomb contact, no slip curve, no load sensitivity",
        ],
    }


def _driven(power, is_front):
    axle = power.get("drive_axle", "rear")
    return axle in ("all", "both") or axle == ("front" if is_front else "rear")


def _wheel_torque(power):
    if power.get("max_drive_torque") is not None:
        return power["max_drive_torque"]
    if power.get("motor_peak_torque") is not None and power.get("gear_ratio"):
        return power["motor_peak_torque"] * power["gear_ratio"]
    return 0.0


def _wheel_speed_limit(power):
    if power.get("motor_max_rpm") and power.get("gear_ratio"):
        return power["motor_max_rpm"] * 2 * math.pi / 60 / power["gear_ratio"]
    return None


def articulation(spec, header):
    return emit.comment(header) + emit.yaml_text(articulation_doc(spec))


def files(spec, header):
    """{file name: content}: one articulation per body (articulation.yaml without bodies)."""
    names = specmod.bodies(spec)
    if not names:
        return {"articulation.yaml": articulation(spec, header)}
    return {f"articulation_{b}.yaml": articulation(specmod.effective(spec, b), header)
            for b in names}
