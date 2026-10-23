"""carla_physics.yaml on a CARLA actor, from Python (roadmap 051 K3): what the probes and the
measurement scripts use. The vehicle bridge (acb_bridge) and the CARLA world server do the
same in Rust; the file is the contract (docs/spec-format.md, "CARLA's physics").

    from sim_vehicle_spec import carla_physics
    doc = carla_physics.load(path)
    carla_physics.apply(vehicle, doc)          # VehiclePhysicsControl, then the wheels
    carla_physics.differences(vehicle, doc)    # [] when the actor runs it

`carla` is imported by the functions that need it, so the module loads without CARLA.
"""
import yaml

# What a read-back compares, and how close is the same (CARLA stores f32).
TOLERANCE = 1e-3


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def apply(vehicle, doc):
    """Put the file's physics on `vehicle` (a carla.Vehicle)."""
    import carla

    pc = vehicle.get_physics_control()
    v = doc["vehicle"]
    pc.mass = v["mass"]
    x, y, z = v["center_of_mass"]
    pc.center_of_mass = carla.Vector3D(x=x, y=y, z=z)
    pc.drag_coefficient = v["drag_coefficient"]
    pc.max_rpm = v["max_rpm"]
    pc.torque_curve = [carla.Vector2D(x=r, y=t) for r, t in v["torque_curve"]]
    pc.final_ratio = v["final_ratio"]
    pc.forward_gears = [carla.GearPhysicsControl(ratio=g["ratio"], down_ratio=g["down_ratio"],
                                                 up_ratio=g["up_ratio"])
                        for g in v["forward_gears"]]
    pc.use_gear_autobox = v["use_gear_autobox"]
    pc.moi = v["moi"]
    pc.damping_rate_full_throttle = v["damping_rate_full_throttle"]
    pc.damping_rate_zero_throttle_clutch_engaged = v["damping_rate_zero_throttle_clutch_engaged"]
    pc.damping_rate_zero_throttle_clutch_disengaged = \
        v["damping_rate_zero_throttle_clutch_disengaged"]
    wheels = list(pc.wheels)
    for w, s in zip(wheels, doc["wheels"]):
        w.tire_friction = s["tire_friction"]
        w.damping_rate = s["damping_rate"]
        w.max_steer_angle = s["max_steer_angle_deg"]
        w.radius = s["radius_m"] * 100.0  # CARLA: cm
        w.max_brake_torque = s["max_brake_torque"]
        w.max_handbrake_torque = s["max_handbrake_torque"]
        w.lat_stiff_max_load = s["lat_stiff_max_load"]
        w.lat_stiff_value = s["lat_stiff_value"]
        w.long_stiff_value = s["long_stiff_value"]
    pc.wheels = wheels
    vehicle.apply_physics_control(pc)


def differences(vehicle, doc):
    """What the actor runs that the file does not say: ["field: file X, actor Y", ...].
    Read after a tick: CARLA applies physics at the next one, and a read-back before it
    returns the old."""
    pc = vehicle.get_physics_control()
    v = doc["vehicle"]
    out = []

    def cmp(name, want, got):
        if abs(float(want) - float(got)) > TOLERANCE * max(1.0, abs(float(want))):
            out.append(f"{name}: file {want}, actor {round(float(got), 4)}")

    cmp("mass", v["mass"], pc.mass)
    for axis, want in zip("xyz", v["center_of_mass"]):
        cmp(f"center_of_mass.{axis}", want, getattr(pc.center_of_mass, axis))
    cmp("drag_coefficient", v["drag_coefficient"], pc.drag_coefficient)
    cmp("max_rpm", v["max_rpm"], pc.max_rpm)
    cmp("final_ratio", v["final_ratio"], pc.final_ratio)
    cmp("moi", v["moi"], pc.moi)
    cmp("damping_rate_zero_throttle_clutch_engaged",
        v["damping_rate_zero_throttle_clutch_engaged"], pc.damping_rate_zero_throttle_clutch_engaged)
    peak = max(t for _, t in v["torque_curve"])
    cmp("torque_curve peak", peak, max(p.y for p in pc.torque_curve))
    for i, (w, s) in enumerate(zip(pc.wheels, doc["wheels"])):
        cmp(f"wheels[{i}].tire_friction", s["tire_friction"], w.tire_friction)
        cmp(f"wheels[{i}].radius_m", s["radius_m"], w.radius / 100.0)
        cmp(f"wheels[{i}].max_brake_torque", s["max_brake_torque"], w.max_brake_torque)
        cmp(f"wheels[{i}].max_steer_angle_deg", s["max_steer_angle_deg"], w.max_steer_angle)
        cmp(f"wheels[{i}].long_stiff_value", s["long_stiff_value"], w.long_stiff_value)
    return out
