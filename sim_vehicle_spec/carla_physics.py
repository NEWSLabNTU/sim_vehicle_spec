"""carla_physics.yaml on a CARLA actor, from Python (roadmap 051 K3): what the probes and the
measurement scripts use. The vehicle bridge (acb_bridge) and the CARLA world server do the
same in Rust; the file is the contract (docs/spec-format.md, "CARLA's physics").

    from sim_vehicle_spec import carla_physics
    doc = carla_physics.load(path)
    carla_physics.apply(vehicle, doc)          # VehiclePhysicsControl, then the wheels
    carla_physics.differences(vehicle, doc)    # [] when the actor runs it

A `carla_physics_1.0.yaml` (`carla: "1.0"`, CARLA 1.0's Chaos vehicle) goes on and is read
back through the same two functions, with a CARLA 1.0 client (`apply_1_0`,
`differences_1_0`); what Chaos keeps from the asset (its `not_applied`) is left alone.

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
    if str(doc.get("carla")) == "1.0":
        return apply_1_0(vehicle, doc)
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
    if str(doc.get("carla")) == "1.0":
        return differences_1_0(vehicle, doc)
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


DIFFERENTIAL_1_0 = {"all": 1, "front": 2, "rear": 3}  # Unreal's EVehicleDifferential


def apply_1_0(vehicle, doc):
    """A carla_physics_1.0.yaml on a CARLA 1.0 vehicle (Chaos). An apply stops a moving car
    on 1.0: its motion is set back."""
    import carla

    pc = vehicle.get_physics_control()
    v = doc["vehicle"]
    pc.mass = v["mass"]
    pc.center_of_mass = carla.Location(*v["center_of_mass"])
    pc.drag_coefficient = v["drag_coefficient"]
    pc.downforce_coefficient = v["downforce_coefficient"]
    pc.max_torque = v["max_torque"]
    pc.torque_curve = [carla.Vector2D(x=r, y=t) for r, t in v["torque_curve"]]
    pc.max_rpm, pc.idle_rpm, pc.brake_effect = v["max_rpm"], v["idle_rpm"], v["brake_effect"]
    pc.differential_type = DIFFERENTIAL_1_0[v["differential"]]
    pc.final_ratio = v["final_ratio"]
    pc.forward_gear_ratios = list(v["forward_gear_ratios"])
    pc.reverse_gear_ratios = list(v["reverse_gear_ratios"])
    pc.change_up_rpm, pc.change_down_rpm = v["change_up_rpm"], v["change_down_rpm"]
    pc.transmission_efficiency = v["transmission_efficiency"]
    pc.steering_curve = [carla.Vector2D(x=a, y=b) for a, b in v["steering_curve"]]
    wheels = list(pc.wheels)
    for w, s in zip(wheels, doc["wheels"]):
        w.wheel_radius = s["radius_m"] * 100.0  # CARLA 1.0: cm
        w.friction_force_multiplier = s["friction_force_multiplier"]
        w.max_steer_angle = s["max_steer_angle_deg"]
        w.max_brake_torque = s["max_brake_torque"]
        w.max_hand_brake_torque = s["max_handbrake_torque"]
        w.abs_enabled = w.traction_control_enabled = False
    pc.wheels = wheels
    vel, ang = vehicle.get_velocity(), vehicle.get_angular_velocity()
    vehicle.apply_physics_control(pc)
    if vel.length() > 1e-3 or ang.length() > 1e-3:
        vehicle.set_target_velocity(vel)
        vehicle.set_target_angular_velocity(ang)


def differences_1_0(vehicle, doc):
    """differences() for a carla_physics_1.0.yaml: the fields 1.0 applies from a client."""
    pc = vehicle.get_physics_control()
    v = doc["vehicle"]
    out = []

    def cmp(name, want, got):
        if abs(float(want) - float(got)) > TOLERANCE * max(1.0, abs(float(want))):
            out.append(f"{name}: file {want}, actor {round(float(got), 4)}")

    if DIFFERENTIAL_1_0[v["differential"]] != int(pc.differential_type):
        out.append(f"differential_type: file {v['differential']}, actor {pc.differential_type}")
    cmp("mass", v["mass"], pc.mass)
    for axis, want in zip("xyz", v["center_of_mass"]):
        cmp(f"center_of_mass.{axis}", want, getattr(pc.center_of_mass, axis))
    for k in ("drag_coefficient", "max_torque", "max_rpm", "final_ratio",
              "transmission_efficiency"):
        cmp(k, v[k], getattr(pc, k))
    for i, (w, s) in enumerate(zip(pc.wheels, doc["wheels"])):
        cmp(f"wheels[{i}].radius_m", s["radius_m"], w.wheel_radius / 100.0)
        cmp(f"wheels[{i}].friction_force_multiplier", s["friction_force_multiplier"],
            w.friction_force_multiplier)
        cmp(f"wheels[{i}].max_steer_angle_deg", s["max_steer_angle_deg"], w.max_steer_angle)
        cmp(f"wheels[{i}].max_brake_torque", s["max_brake_torque"], w.max_brake_torque)
        cmp(f"wheels[{i}].max_handbrake_torque", s["max_handbrake_torque"],
            w.max_hand_brake_torque)
    return out
