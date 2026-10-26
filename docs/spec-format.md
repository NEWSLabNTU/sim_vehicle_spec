# Vehicle spec format

One YAML file per vehicle under `vehicles/`, validated strictly by `sim_vehicle_spec.spec`
(an unknown key, a missing required key or a malformed mount is an error naming the file and
the key). Every consumer of the car generates its files from it with this library, in its own
repository:

| output | repository | generator |
|---|---|---|
| `generated/<name>/carla_sensors.yaml`, `generated/<name>/carla_physics.yaml` (when the spec has `carla.physics`), `catalogs/vehicles.xosc` | sim_vehicle_spec | `python3 -m sim_vehicle_spec.gen` (`just generate`) |
| `<name>_description`, `<name>_launch`, `<name>_sensor_kit_description`, `<name>_sensor_kit_launch`, `individual_params` | autoware_sim_vehicle | `python3 -m autoware_gen` (its `just generate`) |
| the Isaac rig, `src/aib/config/<name>.rig.yaml` | autoware_isaac_bridge | `python3 -m aib.gen` (its `just generate`) |

Each repository's `just check-generated` (the generator with `--check`) fails when one of its
outputs is stale: it compares every file byte for byte with what the specs give now, prints
`stale: <path>` for each that differs or is missing, and exits 1.

Units: metres, kilograms, seconds, radians, newton-metres, unless a key says otherwise.
Frame: base_link, the rear axle centre on the ground, x forward, y left, z up.

## Top level

| key | required | |
|---|---|---|
| `spec_version` | yes | `2` |
| `name` | yes | names everything generated: Autoware's packages (`<name>_description`, `<name>_sensor_kit_description`, ...), the catalog entry, `generated/<name>/` |
| `description` | no | one line |
| `body` | no | `catalogue_model`: the body, a CARLA blueprint id; also the Isaac vehicle catalogue's key and the catalog entry's `model3d` |
| `geometry` | yes | below |
| `mirror` | yes | Autoware's `mirror.param.yaml` keys, as is |
| `inertia` | yes | below |
| `wheels`, `tires`, `brakes`, `powertrain`, `suspension` | yes | below |
| `resistance`, `steering` | no | below |
| `isaac` | no | `physx`: the PhysX vehicle's solver settings (below) |
| `carla` | no | `physics`: what CARLA's model needs beyond the physics sections ("CARLA's physics") |
| `sensor_kit` | yes | `mount`: base_link -> sensor_kit_base_link |
| `sensors` | no | name -> sensor, below |
| `provenance` | no | where numbers come from, below |

## Geometry -> vehicle_info

All required, written to `vehicle_info.param.yaml` as is: `wheel_base`, `wheel_tread`,
`wheel_radius`, `wheel_width`, `front_overhang`, `rear_overhang`, `left_overhang`,
`right_overhang`, `vehicle_height`, `max_steer_angle` (the bicycle-model tire angle limit
Autoware plans with; the PhysX vehicle clamps its request to it). The chassis box is the
footprint and height these give.

## Physics -> the rig's `vehicle`

| key | rig field | |
|---|---|---|
| `inertia.mass` | `mass` | kg |
| `inertia.com_front_fraction` | `com[0] = fraction x wheel_base` | share of the static load on the front axle, (0, 1) |
| `inertia.com_lateral` | `com[1]` | default 0 |
| `inertia.com_height` | `com[2]` | above the ground |
| `inertia.inertia_box` | `inertia_box` | the body's inertia as a uniform box of these l w h |
| `wheels.mass`, `wheels.damping_rate` | `wheel_mass`, `wheel_damping_rate` | moment of inertia 0.5 m r^2 |
| `tires.friction` | `tire_friction` | each wheel's friction table default (a host's surface layer may write another per wheel) |
| `tires.lat_stiff_max_load`, `lat_stiff_value`, `long_stiff_value`, `friction_vs_slip` | same | PhysX's tire model: lateral stiffness graph (rest loads, N/rad per N of rest load), longitudinal stiffness per g, friction vs slip points |
| `brakes.max_brake_torque`, `hold_brake_torque` | same | per wheel |
| `brakes.distribution` | `brake_distribution` | `dynamic_axle_load` (ideal / EBD) or `static` |
| `brakes.dead_time`, `time_constant` | `brake_dead_time`, `brake_time_constant` | `ev`: the hydraulics, a pure delay then a first-order pressure lag, s |
| `brakes.abs.{enabled, slip_release, slip_reapply, release_rate, apply_rate, min_speed}` | `abs_*` | `ev`: per wheel, the pressure factor falls at `release_rate` (1/s) while the wheel's slip is past `slip_release`, climbs at `apply_rate` once within `slip_reapply`; off below `min_speed` m/s |
| `powertrain.model` | `powertrain` | `ideal` or `ev`; `physx`, `fmu` reserved (design.md) |
| `powertrain.max_drive_torque` | same | `ideal`'s cap, per wheel, N m |
| `powertrain.drive_axle`, `front_drive_share` | same | `ev`: `rear`, `front` or `all` (then the front share of motor torque) |
| `powertrain.differential.{type, preload, power_lock, coast_lock}` | `diff_type`, `diff_preload`, `diff_power_lock`, `diff_coast_lock` | `ev`: each driven axle's differential. `open` (default: each wheel half the axle torque), `locked` (a spool: both wheels at one speed), `lsd` (clutch type: torque moves from the faster wheel to the slower one up to `preload` N m plus `power_lock` of the axle torque under drive, `coast_lock` of it on the overrun, e.g. 0.5 / 0.25 for a 1.5-way) |
| `powertrain.gear_ratio` | same | `ev`, required: motor to wheel, single speed |
| `powertrain.motor_peak_torque`, `motor_peak_power`, `motor_max_rpm` | same | `ev`, required: the envelope, N m at the motor to base speed, then W; torque tapers to 0 over the last 5 % below the max rpm |
| `powertrain.motor_time_constant` | same | `ev`: first-order torque lag, s |
| `powertrain.regen_max_torque`, `regen_max_power` | same | `ev`: regeneration's limits, N m at the motor and W |
| `powertrain.regen_fade_speeds` | `regen_fade_low`, `regen_fade_high` | `ev`: `[low, high]` m/s, no regen at or below `low`, full at or above `high` |
| `powertrain.traction_control.{enabled, slip_limit, slip_reapply, release_rate, apply_rate}` | `tc_*` | `ev`: the motor torque factor on the driven wheels' largest slip, as ABS |
| `resistance.rolling_coefficient`, `drag_coefficient`, `frontal_area`, `air_density` | same | the road load, applied to the body by the physics (all models): Crr m g, rho Cd A v^2 / 2. No section: no road load |
| `steering.ackermann` | `ackermann` | split the bicycle angle into Ackermann wheel angles (default true) |
| `steering.time_constant`, `rate_limit` | `steer_time_constant`, `steer_rate_limit` | actuator lag, rate limit; 0: none |
| `suspension.frequency`, `damping_ratio`, `travel` | `suspension_*` | per axle on its sprung mass |
| `isaac.physx.substeps` | `physx_substeps` | vehicle substeps per physics step, at every speed |
| `isaac.physx.*` (others) | same name | slip denominators, sticky tire thresholds and damping |

An `ev` spec must give `gear_ratio`, `motor_peak_torque`, `motor_peak_power` and
`motor_max_rpm`; the other `ev` keys default to the shipped spec's. `ideal` ignores them, so
one spec carries both and `powertrain.model` (or an ego config's `vehicle: {powertrain: ev}`)
picks. `powertrain.rolling_resistance` (a force folded into `ideal`'s command, with no
physical counterpart) is gone: a spec naming it is refused with a pointer to `resistance`.

Every rig field is a `VehicleParams` field (autoware_isaac_bridge's `aib/params.py`), so an
ego config's `vehicle:` override uses the same names.

## Sensors

```yaml
sensors:
  <name>:
    type: lidar | camera | imu | gnss
    calibration_key: <key>   # its key in sensor_kit_calibration.yaml (default: link)
    link: <urdf link>        # the link the mount places, child of sensor_kit_base_link
    frame: <frame_id>        # the messages' frame; a child of link (identity, or the
                             # optical rotation for a camera) when it differs
    topic: <topic>
    mount: {x, y, z, roll, pitch, yaw}   # in sensor_kit_base_link, URDF rpy
    isaac: {enabled: true, ...}          # Isaac options; enabled: false keeps it out of the rig's run
    carla:                               # CARLA: acb_bridge spawns it (carla_sensors.yaml)
      key: <frame>                       #   the URDF frame it is mounted at
      blueprint: <sensor.*>
      attributes: {<name>: <value>}      #   CARLA's own; the shared fields give the rest
    # lidar:  channels, rate_hz, range, vertical_fov
    # camera: width, height, horizontal_fov (deg), rate_hz, info_topic
    # imu, gnss: rate_hz
```

The generator writes the mounts to `sensor_kit_calibration.yaml` (under the calibration
key), a link and frame per sensor to `sensor_kit.xacro`, and the rig's sensors with
base_link transforms composed from the kit's mount and the sensor's. A camera's rig entry
also carries `k`, the pinhole matrix from its width, height and horizontal FOV.

A sensor's CARLA attributes come from the shared fields where they say the same thing (lidar:
`channels`, `range`, `rotation_frequency` from `rate_hz`, `upper_fov` and `lower_fov` from
`vertical_fov`; camera: `image_size_x`, `image_size_y`, `fov`, `sensor_tick` from `rate_hz`;
imu, gnss: `sensor_tick` from `rate_hz`), then from `carla.attributes`.

## CARLA's physics

`generated/<name>/carla_physics.yaml`, when the spec has `carla.physics`: CARLA's
`VehiclePhysicsControl` for the car, in SI units, wheels front left, front right, rear left,
rear right. The CARLA world server applies it to the ego and agent vehicles of its blueprint
at the spawn (with SSv2), and acb_bridge applies it to the hero on attach (standalone,
`carla_physics_mode:=apply`) or reads it back (`verify`). CARLA applies physics at its next
tick: a read-back before one returns the old. Python: `sim_vehicle_spec.carla_physics`
(`apply`, `differences`); Rust: the `carla_physics_spec` crate (`crates/`), parsing and the
comparison, which each consumer maps onto carla-rust.

From the spec: `mass`; `center_of_mass` (CARLA's nudge: the CoM at `com_front_fraction` of
the wheel base from the rear axle and `com_height` up, less the body's own CoM, both in the
actor frame); the motor's torque curve (flat `motor_peak_torque` to base speed, then
`motor_peak_power`, tapering to 0 at `motor_max_rpm`) with `final_ratio` the `gear_ratio`
and one forward gear; `drag_coefficient` (Cd A over CARLA's drag area); per wheel the tire
friction (`tires.friction` x `tire_friction_scale`), the PhysX tire stiffnesses,
`max_brake_torque`, `hold_brake_torque` as the rear wheels' handbrake, the radius;
`command_lags` (`brakes.dead_time`, `brakes.time_constant`, `powertrain.motor_time_constant`)
for the vehicle bridge's command path, since CARLA's model has none.

`carla.physics` holds what the spec's sections cannot say:

| key | what |
|---|---|
| `tire_friction_scale` | CARLA's tire friction for a tire friction of 1.0 (measured: 1.2 stops from 100 km/h in 42.8 m, as the spec's car does in Isaac) |
| `drag_area` | m^2, what CARLA multiplies its drag coefficient by |
| `body_com` | m, actor frame: the CoM UE computes for the body (CARLA does not expose it) |
| `rear_axle_x` | m, actor frame: the rear axle (swb_carla_world's measured table) |
| `max_steer_angle_deg` | the front wheels' limit (acb_bridge's steering map assumes it) |
| `engine_moi`, `engine_damping` | CARLA's engine is a combustion engine's: an EV motor's light rotor and its small drag (measured: 0-100 km/h in 6.3 s, coasting 0.20 m/s^2 at 25 m/s) |

Not modelled by CARLA, and so not in the file: the rear-wheel drive (CARLA drives its own
differential), ABS, regen and traction control, rolling resistance. `physics_hash` covers
the physics sections and `carla.physics`: the pedal maps measured on CARLA carry it.
`carla_physics.yaml` needs an `ev` powertrain; the generator refuses another.

The file, key by key (YAML, after the generated header):

| key | |
|---|---|
| `spec_hash`, `physics_hash` | the spec's fingerprints ("Fingerprints") |
| `blueprint` | `body.catalogue_model`: the CARLA blueprint the physics is for. A consumer refuses to put it on another body |
| `vehicle.mass` | kg |
| `vehicle.center_of_mass` | `[x, y, z]`, m: CARLA's nudge of the body's own centre of mass, CARLA's axes (y right) |
| `vehicle.drag_coefficient` | Cd x frontal area / `drag_area` |
| `vehicle.max_rpm`, `vehicle.torque_curve` | the motor: `[[rpm, N m], ...]` at the engine, 11 points |
| `vehicle.final_ratio`, `vehicle.forward_gears`, `vehicle.use_gear_autobox` | `gear_ratio`; one gear of ratio 1; true |
| `vehicle.moi`, `vehicle.damping_rate_full_throttle`, `vehicle.damping_rate_zero_throttle_clutch_engaged`, `vehicle.damping_rate_zero_throttle_clutch_disengaged` | `engine_moi`, `engine_damping` (all three) |
| `wheels` | four, front left, front right, rear left, rear right: `tire_friction`, `damping_rate`, `max_steer_angle_deg` (0 on the rear), `radius_m` (CARLA takes centimetres), `max_brake_torque`, `max_handbrake_torque` (`hold_brake_torque` on the rear, 0 on the front), `lat_stiff_max_load`, `lat_stiff_value`, `long_stiff_value` |
| `command_lags` | `brake_dead_time`, `brake_time_constant`, `motor_time_constant`, s: for the vehicle bridge's command path |

Applying it and reading it back:

- **Python**, `sim_vehicle_spec.carla_physics` (imports `carla` only inside the functions):
  `load(path)`; `apply(vehicle, doc)` puts the file on a `carla.Vehicle`'s
  `VehiclePhysicsControl`, wheels included; `differences(vehicle, doc)` returns what the
  actor runs that the file does not say, as `"<field>: file X, actor Y"` strings, empty when
  it runs the file.
- **Rust**, the crate `crates/carla_physics_spec` (its own workspace, a path dependency of
  swb_carla_world and acb_bridge): `CarlaPhysics::load(path)` (refuses a file without four
  wheels), `peak_torque()`, `as_read_back()`, and `differences(&ReadBack, friction_scale)`.
  `ReadBack` holds what a consumer reads off the actor, in the file's units; `friction_scale`
  is the road's (1 on a dry road), so a world server that scales the tires for a wet road
  still matches. The consumer maps the file onto carla-rust itself.
- Both compare the same fields with a tolerance of 1e-3 x max(1, abs(value)), since CARLA
  stores f32: `mass`, `center_of_mass` x, y, z, `drag_coefficient`, `max_rpm`,
  `final_ratio`, the torque curve's peak, `moi`, `damping_rate_zero_throttle_clutch_engaged`,
  and per wheel `tire_friction`, `radius_m`, `max_brake_torque`, `max_steer_angle_deg`,
  `long_stiff_value`.
- **Read back after a tick.** CARLA applies physics at its next tick: a read straight after
  `apply_physics_control` returns the old values. The CARLA world server applies the file,
  ticks once, then compares and refuses an actor that differs.

## The catalog

`catalogs/vehicles.xosc` (catalog `vehicles`) has an entry per spec, named `name`
(`vehicleCategory="car"`), with:

- `model3d`: `body.catalogue_model`;
- `BoundingBox`: the box the geometry gives, in base_link (centre `x = (wheel_base +
  front_overhang - rear_overhang) / 2`, `y = (left_overhang - right_overhang) / 2`,
  `z = vehicle_height / 2`; length, width, height), in metres to the millimetre;
- `Axles`: `FrontAxle` at `positionX = wheel_base`, `RearAxle` at 0, both with
  `wheelDiameter = 2 x wheel_radius`, `trackWidth = wheel_tread`, `positionZ =
  wheel_radius`; the rear's `maxSteering` is 0;
- `Performance maxSpeed="$maxSpeed" maxAcceleration="$maxAcceleration"
  maxDeceleration="$maxDeceleration"` and `FrontAxle maxSteering="$maxSteering"`: parameters
  every reference assigns, because what the scenario lets the car do is the scenario's;
- `Properties`: `<Property name="swb.spec_hash" value="<spec_hash>"/>`. The scenario bridge
  reads the installed catalog (`share/sim_vehicle_spec/catalogs/vehicles.xosc` on
  `AMENT_PREFIX_PATH`) and keys entries by `model3d`, for its start check: SSv2 does not pass
  a catalog property on in the spawn.

The entry has **no `ParameterDeclarations`**. SSv2 evaluates an entry's declarations in the
entry's own scope, where a declared default shadows the reference's assignment, so a default
always won (found when SSv2 refused an acceleration of 13 against a default of 9.8). Each
reference must therefore assign all four. A scenario refers to it:

```xml
<CatalogLocations>
  <VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog>
</CatalogLocations>
...
<CatalogReference catalogName="vehicles" entryName="tesla_model3">
  <ParameterAssignments>
    <ParameterAssignment parameterRef="maxSpeed" value="20" />
    <ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
    <ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
    <ParameterAssignment parameterRef="maxSteering" value="0.5" />
  </ParameterAssignments>
</CatalogReference>
```

## Fingerprints

Two SHA-256 fingerprints of a spec, each written as 64 lowercase hex digits
(`sim_vehicle_spec.spec.spec_hash`, `physics_hash`). Messages often show the first 12.

**`spec_hash`** identifies the car. Its canonical form:

1. Parse the file with PyYAML's `safe_load` (YAML 1.1). Comments, layout and key order are
   gone at this point.
2. Drop the top-level keys `description` and `provenance` (words, not the car).
3. Serialise what remains as JSON with keys sorted at every level, separators `,` and `:`
   (no spaces), ASCII only (`json.dumps(doc, sort_keys=True, separators=(",", ":"),
   ensure_ascii=True)`).
4. SHA-256 of that text's UTF-8 bytes, in hex.

Numbers keep the type the YAML parser gave them: an integer is written as an integer and a
float as Python's shortest round-trip repr. So `0.37` and `0.370` hash the same, but `1760`
and `1760.0` do not. Note too that YAML 1.1 reads `1e3` and `1.0e3` as strings (it needs a
sign in the exponent and a dot: `1.0e+3`). Keep numbers as the shipped spec writes them. Another
implementation reproduces the hash only by following these rules, Python's float repr
included.

Every generated file names the `spec_hash` of the spec it came from (a header line, and a
`spec_hash` key where the format has room). Every part of a run reports the one it loaded:
the world server in `SpawnReply.body.spec_hash`, the vehicle agent at registration (from
`<name>_description/config/spec_hash.yaml`), and the catalog entry's `swb.spec_hash`
property. The scenario bridge refuses an ego whose parts disagree, naming who loaded which
(roadmap 051). Standalone, acb_bridge compares its sensor file's hash with the Autoware
packages' and warns.

**`physics_hash`** identifies what drives the car: the same steps over a mapping of the
sections `geometry`, `inertia`, `wheels`, `tires`, `brakes`, `powertrain`, `resistance`,
`steering`, `suspension` (a missing one as null) and one more key, `"carla.physics"`, holding
`carla.physics` (null without it). The pedal maps measured on CARLA carry it
(autoware_sim_vehicle's `pedal_maps.yaml`), so moving a sensor does not invalidate them, and
`carla_physics.yaml` names it.

## Version 2

From version 1 (autoware_isaac_bridge's): `autoware` is gone (the package names follow
`name`), `physx` is `isaac.physx`, a sensor may have `carla`, `carla` is new, and the
geometry's overhangs are the measured body's (roadmap 050). A version 1 file is refused with
where each key went.

## Provenance

```yaml
provenance:
  <section or section.key>: {basis: "<where it comes from>", assumed: true}
```

`basis` is required; `assumed: true` marks an estimate to replace with a measurement. The
rig lists the assumed keys under `assumptions`, so a host can log what is not measured.
