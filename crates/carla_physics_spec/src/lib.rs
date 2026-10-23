//! sim_vehicle_spec's `carla_physics.yaml` (roadmap 051 K3): CARLA's physics for a car,
//! generated from its vehicle spec. SI units; wheels in CARLA's order (front left, front
//! right, rear left, rear right). A consumer maps it onto carla-rust's
//! `VehiclePhysicsControl` (radius in centimetres there) and compares a read-back with
//! [`CarlaPhysics::differences`].

use serde::Deserialize;
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct CarlaPhysics {
    /// The vehicle spec the file came from.
    pub spec_hash: String,
    /// The fingerprint of what drives the car (the pedal maps carry it).
    pub physics_hash: String,
    /// The CARLA blueprint the physics is for: another body is not this car.
    pub blueprint: String,
    pub vehicle: Vehicle,
    pub wheels: Vec<Wheel>,
    pub command_lags: CommandLags,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Vehicle {
    pub mass: f64,
    /// CARLA's nudge of the body's own centre of mass, metres, CARLA's axes (y right).
    pub center_of_mass: [f64; 3],
    pub drag_coefficient: f64,
    pub max_rpm: f64,
    /// (rpm, N m) at the engine.
    pub torque_curve: Vec<[f64; 2]>,
    pub final_ratio: f64,
    pub forward_gears: Vec<Gear>,
    pub use_gear_autobox: bool,
    /// The engine's moment of inertia, kg m^2.
    pub moi: f64,
    pub damping_rate_full_throttle: f64,
    pub damping_rate_zero_throttle_clutch_engaged: f64,
    pub damping_rate_zero_throttle_clutch_disengaged: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Gear {
    pub ratio: f64,
    pub down_ratio: f64,
    pub up_ratio: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Wheel {
    pub tire_friction: f64,
    pub damping_rate: f64,
    pub max_steer_angle_deg: f64,
    pub radius_m: f64,
    pub max_brake_torque: f64,
    pub max_handbrake_torque: f64,
    pub lat_stiff_max_load: f64,
    pub lat_stiff_value: f64,
    pub long_stiff_value: f64,
}

/// What CARLA's model lacks, for the vehicle bridge's command path, seconds.
#[derive(Debug, Clone, Copy, PartialEq, Default, Deserialize)]
pub struct CommandLags {
    pub brake_dead_time: f64,
    pub brake_time_constant: f64,
    pub motor_time_constant: f64,
}

/// What a read-back of an actor's physics holds, in the file's units: compared with
/// [`CarlaPhysics::differences`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ReadBack {
    pub mass: f64,
    pub center_of_mass: [f64; 3],
    pub drag_coefficient: f64,
    pub max_rpm: f64,
    pub final_ratio: f64,
    pub peak_torque: f64,
    pub moi: f64,
    pub damping_rate_zero_throttle_clutch_engaged: f64,
    /// (tire_friction, radius_m, max_brake_torque, max_steer_angle_deg, long_stiff_value)
    pub wheels: Vec<[f64; 5]>,
}

impl CarlaPhysics {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("reading {}: {e}", path.display()))?;
        let doc: Self =
            serde_yaml::from_str(&text).map_err(|e| format!("parsing {}: {e}", path.display()))?;
        if doc.wheels.len() != 4 {
            return Err(format!(
                "{}: {} wheels, not 4",
                path.display(),
                doc.wheels.len()
            ));
        }
        Ok(doc)
    }

    pub fn peak_torque(&self) -> f64 {
        self.vehicle
            .torque_curve
            .iter()
            .map(|p| p[1])
            .fold(0.0, f64::max)
    }

    /// What `actor` runs that the file does not say, each "field: file X, actor Y"; empty
    /// when it is this physics. `friction_scale` is the road's (1 on a dry road): a world
    /// server that scales the tires for a wet road is still running this car.
    pub fn differences(&self, actor: &ReadBack, friction_scale: f64) -> Vec<String> {
        let mut out = Vec::new();
        let mut cmp = |name: String, want: f64, got: f64| {
            // CARLA stores f32.
            if (want - got).abs() > 1e-3 * want.abs().max(1.0) {
                out.push(format!("{name}: file {want}, actor {got:.4}"));
            }
        };
        let v = &self.vehicle;
        cmp("mass".into(), v.mass, actor.mass);
        for (i, axis) in ["x", "y", "z"].iter().enumerate() {
            cmp(
                format!("center_of_mass.{axis}"),
                v.center_of_mass[i],
                actor.center_of_mass[i],
            );
        }
        cmp(
            "drag_coefficient".into(),
            v.drag_coefficient,
            actor.drag_coefficient,
        );
        cmp("max_rpm".into(), v.max_rpm, actor.max_rpm);
        cmp("final_ratio".into(), v.final_ratio, actor.final_ratio);
        cmp(
            "torque_curve peak".into(),
            self.peak_torque(),
            actor.peak_torque,
        );
        cmp("moi".into(), v.moi, actor.moi);
        cmp(
            "damping_rate_zero_throttle_clutch_engaged".into(),
            v.damping_rate_zero_throttle_clutch_engaged,
            actor.damping_rate_zero_throttle_clutch_engaged,
        );
        if actor.wheels.len() != self.wheels.len() {
            out.push(format!(
                "wheels: file {}, actor {}",
                self.wheels.len(),
                actor.wheels.len()
            ));
            return out;
        }
        for (i, (w, a)) in self.wheels.iter().zip(&actor.wheels).enumerate() {
            cmp(
                format!("wheels[{i}].tire_friction"),
                w.tire_friction * friction_scale,
                a[0],
            );
            cmp(format!("wheels[{i}].radius_m"), w.radius_m, a[1]);
            cmp(
                format!("wheels[{i}].max_brake_torque"),
                w.max_brake_torque,
                a[2],
            );
            cmp(
                format!("wheels[{i}].max_steer_angle_deg"),
                w.max_steer_angle_deg,
                a[3],
            );
            cmp(
                format!("wheels[{i}].long_stiff_value"),
                w.long_stiff_value,
                a[4],
            );
        }
        out
    }

    /// The file's own values as a read-back, for tests and for a consumer's no-op check.
    pub fn as_read_back(&self) -> ReadBack {
        ReadBack {
            mass: self.vehicle.mass,
            center_of_mass: self.vehicle.center_of_mass,
            drag_coefficient: self.vehicle.drag_coefficient,
            max_rpm: self.vehicle.max_rpm,
            final_ratio: self.vehicle.final_ratio,
            peak_torque: self.peak_torque(),
            moi: self.vehicle.moi,
            damping_rate_zero_throttle_clutch_engaged: self
                .vehicle
                .damping_rate_zero_throttle_clutch_engaged,
            wheels: self
                .wheels
                .iter()
                .map(|w| {
                    [
                        w.tire_friction,
                        w.radius_m,
                        w.max_brake_torque,
                        w.max_steer_angle_deg,
                        w.long_stiff_value,
                    ]
                })
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../generated/tesla_model3/carla_physics.yaml"
    );

    #[test]
    fn the_shipped_file_loads_and_matches_itself() {
        let p = CarlaPhysics::load(SHIPPED).unwrap();
        assert_eq!(p.blueprint, "vehicle.tesla.model3");
        assert_eq!(p.spec_hash.len(), 64);
        assert_eq!(p.vehicle.mass, 1760.0);
        assert_eq!(p.peak_torque(), 340.0);
        assert_eq!(p.wheels[2].max_handbrake_torque, 1500.0);
        assert!(p.differences(&p.as_read_back(), 1.0).is_empty());
    }

    #[test]
    fn a_different_actor_is_named_field_by_field() {
        let p = CarlaPhysics::load(SHIPPED).unwrap();
        let mut a = p.as_read_back();
        a.mass = 1845.0;
        a.wheels[0][2] = 700.0;
        let off = p.differences(&a, 1.0);
        assert_eq!(off.len(), 2, "{off:?}");
        assert!(off[0].starts_with("mass: file 1760, actor 1845"), "{off:?}");
        assert!(
            off[1].starts_with("wheels[0].max_brake_torque: file 3000"),
            "{off:?}"
        );
    }

    #[test]
    fn a_wet_road_scales_the_tires_only() {
        let p = CarlaPhysics::load(SHIPPED).unwrap();
        let mut a = p.as_read_back();
        for w in &mut a.wheels {
            w[0] *= 0.7;
        }
        assert!(p.differences(&a, 0.7).is_empty());
        assert_eq!(p.differences(&a, 1.0).len(), 4);
    }
}
