//! sim_vehicle_spec's `carla_physics_1.0.yaml` (roadmap 083 step 5): CARLA 1.0's physics for a
//! car, on its Chaos vehicle. SI units; wheels in CARLA's order (front left, front right, rear
//! left, rear right). A consumer maps it onto carla-rust's 1.0 `VehiclePhysicsControl`:
//! `wheel_radius` in centimetres, `torque_curve` normalised to `max_torque`, `center_of_mass`
//! absolute in the actor frame, `differential` as CARLA's `differential_type`, ABS and
//! traction control off. What 1.0 keeps from the blueprint's asset is `not_applied` and is
//! neither applied nor compared.

use serde::Deserialize;
use std::path::Path;

pub use crate::CommandLags;

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ChaosPhysics {
    /// The vehicle spec the file came from.
    pub spec_hash: String,
    /// The fingerprint of what drives the car (the pedal maps carry it).
    pub physics_hash: String,
    /// The CARLA release the file is for: "1.0".
    pub carla: String,
    /// The CARLA blueprint the physics is for: another body is not this car.
    pub blueprint: String,
    pub vehicle: Vehicle,
    pub wheels: Vec<Wheel>,
    pub command_lags: CommandLags,
    /// What the spec says that Chaos keeps from the blueprint's asset.
    #[serde(default)]
    pub not_applied: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Vehicle {
    pub mass: f64,
    /// Absolute, metres, the actor frame, CARLA's axes (y right).
    pub center_of_mass: [f64; 3],
    pub drag_coefficient: f64,
    pub downforce_coefficient: f64,
    /// N m: what the normalised curve is scaled by.
    pub max_torque: f64,
    /// (rpm, fraction of `max_torque`).
    pub torque_curve: Vec<[f64; 2]>,
    pub max_rpm: f64,
    pub idle_rpm: f64,
    /// Engine braking as a fraction of `max_torque` (0: none).
    pub brake_effect: f64,
    /// The driven axle: `rear`, `front` or `all`.
    pub differential: String,
    pub final_ratio: f64,
    pub forward_gear_ratios: Vec<f64>,
    pub reverse_gear_ratios: Vec<f64>,
    pub change_up_rpm: f64,
    pub change_down_rpm: f64,
    pub transmission_efficiency: f64,
    /// (speed, steering scale): flat, the spec has no speed-dependent steering.
    pub steering_curve: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Wheel {
    pub radius_m: f64,
    pub friction_force_multiplier: f64,
    pub max_steer_angle_deg: f64,
    pub max_brake_torque: f64,
    pub max_handbrake_torque: f64,
}

impl Vehicle {
    /// CARLA's `differential_type` (Unreal's EVehicleDifferential: 1 all, 2 front, 3 rear).
    pub fn differential_type(&self) -> Result<u8, String> {
        match self.differential.as_str() {
            "all" => Ok(1),
            "front" => Ok(2),
            "rear" => Ok(3),
            other => Err(format!("differential {other:?}: all, front or rear")),
        }
    }
}

/// What a read-back of a 1.0 actor's physics holds, in the file's units: compared with
/// [`ChaosPhysics::differences`]. Only what 1.0 applies from a client.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ReadBack {
    pub mass: f64,
    pub center_of_mass: [f64; 3],
    pub drag_coefficient: f64,
    pub max_torque: f64,
    pub max_rpm: f64,
    pub final_ratio: f64,
    pub transmission_efficiency: f64,
    pub differential_type: u8,
    /// (radius_m, friction_force_multiplier, max_steer_angle_deg, max_brake_torque,
    /// max_handbrake_torque)
    pub wheels: Vec<[f64; 5]>,
}

impl ChaosPhysics {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("reading {}: {e}", path.display()))?;
        let doc: Self =
            serde_yaml::from_str(&text).map_err(|e| format!("parsing {}: {e}", path.display()))?;
        if doc.carla != "1.0" {
            return Err(format!(
                "{}: physics for CARLA {}, not 1.0",
                path.display(),
                doc.carla
            ));
        }
        if doc.wheels.len() != 4 {
            return Err(format!(
                "{}: {} wheels, not 4",
                path.display(),
                doc.wheels.len()
            ));
        }
        doc.vehicle
            .differential_type()
            .map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(doc)
    }

    /// What `actor` runs that the file does not say, each "field: file X, actor Y"; empty
    /// when it is this physics. `friction_scale` is the road's (1 on a dry road): a world
    /// server that scales the tires for a wet road is still running this car.
    pub fn differences(&self, actor: &ReadBack, friction_scale: f64) -> Vec<String> {
        let mut out = Vec::new();
        let v = &self.vehicle;
        if let Ok(want) = v.differential_type() {
            if want != actor.differential_type {
                out.push(format!(
                    "differential_type: file {want} ({}), actor {}",
                    v.differential, actor.differential_type
                ));
            }
        }
        let mut cmp = |name: String, want: f64, got: f64| {
            // CARLA stores f32.
            if (want - got).abs() > 1e-3 * want.abs().max(1.0) {
                out.push(format!("{name}: file {want}, actor {got:.4}"));
            }
        };
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
        cmp("max_torque".into(), v.max_torque, actor.max_torque);
        cmp("max_rpm".into(), v.max_rpm, actor.max_rpm);
        cmp("final_ratio".into(), v.final_ratio, actor.final_ratio);
        cmp(
            "transmission_efficiency".into(),
            v.transmission_efficiency,
            actor.transmission_efficiency,
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
            cmp(format!("wheels[{i}].radius_m"), w.radius_m, a[0]);
            cmp(
                format!("wheels[{i}].friction_force_multiplier"),
                w.friction_force_multiplier * friction_scale,
                a[1],
            );
            cmp(
                format!("wheels[{i}].max_steer_angle_deg"),
                w.max_steer_angle_deg,
                a[2],
            );
            cmp(
                format!("wheels[{i}].max_brake_torque"),
                w.max_brake_torque,
                a[3],
            );
            cmp(
                format!("wheels[{i}].max_handbrake_torque"),
                w.max_handbrake_torque,
                a[4],
            );
        }
        out
    }

    /// The file's own values as a read-back, for tests and for a consumer's no-op check.
    pub fn as_read_back(&self) -> ReadBack {
        let v = &self.vehicle;
        ReadBack {
            mass: v.mass,
            center_of_mass: v.center_of_mass,
            drag_coefficient: v.drag_coefficient,
            max_torque: v.max_torque,
            max_rpm: v.max_rpm,
            final_ratio: v.final_ratio,
            transmission_efficiency: v.transmission_efficiency,
            differential_type: v.differential_type().unwrap_or(0),
            wheels: self
                .wheels
                .iter()
                .map(|w| {
                    [
                        w.radius_m,
                        w.friction_force_multiplier,
                        w.max_steer_angle_deg,
                        w.max_brake_torque,
                        w.max_handbrake_torque,
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
        "/../../generated/tesla_model3/carla_physics_1.0.yaml"
    );

    #[test]
    fn the_shipped_file_loads_and_matches_itself() {
        let p = ChaosPhysics::load(SHIPPED).unwrap();
        assert_eq!(p.carla, "1.0");
        assert_eq!(p.blueprint, "vehicle.tesla.model3");
        assert_eq!(p.vehicle.max_torque, 340.0);
        assert_eq!(p.vehicle.torque_curve[0], [0.0, 1.0]);
        assert_eq!(p.vehicle.differential_type(), Ok(3));
        assert_eq!(p.wheels[0].radius_m, 0.336);
        assert_eq!(p.vehicle.final_ratio, 9.084);
        assert_eq!(p.wheels[2].max_steer_angle_deg, 0.0);
        assert!(!p.not_applied.is_empty());
        assert!(p.differences(&p.as_read_back(), 1.0).is_empty());
    }

    #[test]
    fn a_different_actor_is_named_field_by_field() {
        let p = ChaosPhysics::load(SHIPPED).unwrap();
        let mut a = p.as_read_back();
        a.mass = 1850.0; // the blueprint's own
        a.differential_type = 1;
        a.wheels[1][1] = 2.0;
        let off = p.differences(&a, 1.0);
        assert_eq!(off.len(), 3, "{off:?}");
        assert!(
            off[0].starts_with("differential_type: file 3 (rear)"),
            "{off:?}"
        );
        assert!(off[1].starts_with("mass: file 1760, actor 1850"), "{off:?}");
        assert!(
            off[2].starts_with("wheels[1].friction_force_multiplier: file 1.94"),
            "{off:?}"
        );
    }

    #[test]
    fn a_wet_road_scales_the_friction_only() {
        let p = ChaosPhysics::load(SHIPPED).unwrap();
        let mut a = p.as_read_back();
        for w in &mut a.wheels {
            w[1] *= 0.7;
        }
        assert!(p.differences(&a, 0.7).is_empty());
        assert_eq!(p.differences(&a, 1.0).len(), 4);
    }

    #[test]
    fn the_0_9_16_file_is_refused() {
        let old = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../generated/tesla_model3/carla_physics.yaml"
        );
        assert!(ChaosPhysics::load(old).is_err());
    }
}
