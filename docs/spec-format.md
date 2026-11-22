# Vehicle spec format

One YAML file per vehicle under `vehicles/`, validated strictly by `sim_vehicle_spec.spec`
(an unknown key, a missing required key or a malformed mount is an error naming the file and
the key). Every consumer of the car generates its files from it with this library, in its own
repository:

| output | repository | generator |
|---|---|---|
| `generated/<name>/carla_sensors.yaml`, `generated/<name>/carla_physics.yaml` (when a body has `carla.physics`), `generated/<name>/carla_physics_1.0.yaml` (when a body has `carla.physics_1_0`), `catalogs/vehicles.xosc`, `catalogs/<body>/vehicles.xosc` | sim_vehicle_spec | `python3 -m sim_vehicle_spec.gen` (`just generate`) |
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
| `body` | no | `catalogue_model`: the body, a CARLA blueprint id (with `bodies`, each body's own); also the Isaac vehicle catalogue's key and the catalog entry's `model3d`. `ground_clearance`: m, unladen, from the road to the underbody's lowest point between the axles; the Isaac rig puts a skid plate there (without it, nothing under the body collides). `contact`: the body's physics material where it touches walls and objects (below, "The body's contact"). Not a physics section: moving it leaves the pedal maps alone |
| `geometry` | yes | below; with `bodies`, the real car's |
| `mirror` | yes | Autoware's `mirror.param.yaml` keys, as is |
| `inertia` | yes | below |
| `wheels`, `tires`, `brakes`, `powertrain`, `suspension` | yes | below |
| `resistance`, `steering` | no | below |
| `isaac` | no | `physx`: the PhysX vehicle's solver settings (below) |
| `carla` | no | without `bodies`: `physics`, what CARLA's model needs beyond the physics sections ("CARLA's physics"); with `bodies`, a body's |
| `bodies` | no | name -> the body a simulator drives ("Bodies") |
| `simulators` | no | simulator -> the name of the body it drives, for a simulator whose body is not named for it (`isaac: carla-0.9.16`) |
| `sensor_kit` | yes | `mount`: base_link -> sensor_kit_base_link |
| `sensors` | no | name -> sensor, below |
| `provenance` | no | where numbers come from, below |

## Bodies

A simulator drives a body: an asset with its own dimensions, which may not be the car's. A
spec with `bodies` describes the real car in `geometry` and `powertrain`, and each body as a
measured section:

| key | required | |
|---|---|---|
| `catalogue_model` | yes | the body's CARLA blueprint id (the catalog entry's `model3d`) |
| `geometry` | yes | every geometry key, the body's (measured from the asset) |
| `powertrain` | no | `gear_ratio`, `regen_max_torque`: the reduction scaled to the body's wheel, the real ratio times the body's radius over the real radius, so the force at the road and the motor speed per road speed are the real car's |
| `carla` | no | the body's simulator calibration: `physics` (CARLA 0.9.16's PhysX vehicle) or `physics_1_0` (CARLA 1.0's Chaos vehicle) |

`sim_vehicle_spec.spec.effective(spec, body)` is the spec as that body: the spec's document
with the body's geometry, its powertrain keys, its catalogue model and its `carla` in place of
the spec's, `bodies` and `simulators` dropped. Every consumer that builds a simulator's car
takes it; `bodies(spec)` lists the names, the default first, and `simulator_body(spec,
simulator)` reads `simulators`. A spec without `bodies` is its own body (`effective(spec,
None)` returns it). The shipped Tesla Model 3 has two: `carla-0.9.16` (CARLA 0.9.16's
`vehicle.tesla.model3`, 3.005 m wheel base on a 0.370 m wheel; also Isaac's, whose asset is
that body exported) and `carla-1.0` (CARLA 1.0's, the real car's 2.876 m on a 0.336 m wheel).

The generator writes each body's outputs: the default body's catalog entry in
`catalogs/vehicles.xosc` and another body's in `catalogs/<body>/vehicles.xosc`, a directory
of its own: SSv2 reads every file of a catalog directory and keeps the first catalog of a name
it finds, so two `vehicles` catalogs side by side would be picked by the file system's order;
`carla_physics.yaml` from the body with `carla.physics`, `carla_physics_1.0.yaml` from the
body with `carla.physics_1_0`. A top-level `carla.physics` beside `bodies` is refused.

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
| `tires.front`, `tires.rear` | the axle's wheels' tyre fields | optional: one axle's `friction`, `lat_stiff_max_load`, `lat_stiff_value`, `long_stiff_value` where they differ from `tires` (below, "Tyres per axle") |
| `body.contact.{restitution, static_friction, dynamic_friction, combine}` | `contact` | optional: a physics material on the chassis's colliders (below, "The body's contact") |
| `brakes.max_brake_torque`, `hold_brake_torque` | same | per wheel |
| `brakes.distribution` | `brake_distribution` | `dynamic_axle_load` (ideal / EBD) or `static` |
| `brakes.dead_time`, `time_constant` | `brake_dead_time`, `brake_time_constant` | `ev`: the hydraulics, a pure delay then a first-order pressure lag, s |
| `brakes.abs.{enabled, slip_release, slip_reapply, release_rate, apply_rate, min_speed}` | `abs_*` | `ev`: per wheel, the pressure factor falls at `release_rate` (1/s) while the wheel's slip is past `slip_release`, climbs at `apply_rate` once within `slip_reapply`; off below `min_speed` m/s |
| `powertrain.model` | `powertrain` | `ideal` or `ev`; `component`: the user's own control component (scensim roadmap 091); `physx`, `fmu` reserved (design.md) |
| `powertrain.component` | none | `component`, required: the name of its entry point in group `scensim.vehicle_components` (sim_vehicle_ecu's `component` module) |
| `powertrain.params` | none | `component`: a mapping handed to it as is |
| `powertrain.max_drive_torque` | same | `ideal`'s cap, per wheel, N m; required except for `component` |
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

### Tyres per axle

`tires.front` and `tires.rear` (scensim roadmap 091 W24) each name the keys in which that
axle's tyres differ from `tires`: `friction`, `lat_stiff_max_load`, `lat_stiff_value`,
`long_stiff_value`, each positive. Any other key, `friction_vs_slip` included, is refused by
name, and so is an empty mapping. `tires`' own keys stay required and stay the default for
both axles. `sim_vehicle_spec.spec.axle_tires(doc, front)` is one axle's tyres.

A car's front and rear tyres often differ. ForzaETH's identified F1TENTH tyres are about twice
as stiff at the rear as at the front, and one tyre for all four wheels cannot reproduce their
steady circles (scensim roadmap 091 W8). The Isaac rig gives each wheel its axle's tyre, and
CARLA's `carla_physics.yaml` does the same per wheel (below). The onboard computers' single
`tire_friction` (sim_vehicle_ecu's `Calibration`) is the lower of the two axles' frictions:
ESC's target and the brake module's friction estimate read that one value, a prior per car,
not per axle.

### The body's contact

`body.contact` (W24) is the body's physics material where it touches walls, cones and other
objects: `restitution` in [0, 1], `static_friction` and `dynamic_friction` >= 0, and `combine`,
one of PhysX's modes `average` (PhysX's default), `min`, `max` or `multiply`, applied to both
the friction and the restitution. Any of them may be given. An empty mapping is refused.
Without it the body collides with the simulator's default material, as before.

PhysX combines the two materials of a pair. An object authored at restitution 1.0 returns a
car without a contact material at 0.41 of its impact speed, and at 0.86 with restitution 1.0
on the body too (scensim roadmap 091 W9). So a wall or cone push measured on the real car
needs the body's side authored here, as the stage carries the object's.

The Isaac rig binds it on the chassis's colliders. CARLA has no physics material per actor, so
CARLA's files are generated without it and name it: an entry in their `not_applied`, and a
header line `Not applied on CARLA: body.contact ...` in `carla_physics.yaml`.

Every rig field is a `VehicleParams` field (autoware_isaac_bridge's `aib/params.py`), so an
ego config's `vehicle:` override uses the same names.

### The car as an articulation

`generated/<name>/articulation[_<body>].yaml` is the car as rigid bodies and joints (scensim
092 K2.2b): a chassis, and per corner a prismatic suspension, a front steer and a wheel spin
joint. What it chooses is said in its `choices:`. The optional `articulation:` section names
what a car built that way would otherwise get from the generator:

| key | what |
|---|---|
| `spin_drive` | the wheels' spin drive: the driven ones' when `undriven_spin_drive` is given, else all four's (default: an effort drive the consumer applies torque through) |
| `undriven_spin_drive` | the undriven wheels' (`powertrain.drive_axle`), e.g. a passive joint |
| `steer_drive` | the front steer joints' drive (default: as stiff as a rack) |
| `suspension_drive` | the suspension's drive; its `target` defaults to minus the static sag (default: the ride frequency's spring and the damping ratio's damper) |
| `wheel_contact` | the tyre's material (default: `tires.friction`, combined by multiply) |
| `steer_law` | the front steer targets for a bicycle angle d: `ackermann` (each wheel's angle), `parallel` (d on both) or `tan` (tan(d) on both); default `ackermann`, or `parallel` when `steering.ackermann` is false. Under `tan` the steer joints' travel is tan(`max_steer_angle`) |

A drive has `type` (`effort`, `velocity`, `position`, or `dc_motor`), `stiffness`, `damping`,
`effort_limit`, `velocity_limit` and `friction`. `dc_motor` is Isaac Lab's DC motor (a velocity
drive whose torque is clipped to the motor's torque-speed line, `saturation_effort` at stall and
zero at `velocity_limit`, and to `effort_limit`). It needs `saturation_effort`, which only it
takes, and drives a spin joint only. WheeledLab's MuSHR (scensim 094 P2) is the case: a DC motor
on its rear wheels, passive fronts, a rigid suspension and `tan` steering.

A spec that has an `articulation:` section carries it in its `physics_hash`; one without has the
same fingerprint as before the section existed.

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
      intensity:                         #   a lidar of blueprint sensor.lidar.ray_cast_semantic only (and required there)
        reflectance: {<tag>: <0-255>}    #     by CARLA semantic tag name (Roads, Car, ...)
        default: <0-255>                 #     for the tags not named (default 0)
    # lidar:  channels, rate_hz, range, vertical_fov; horizontal_fov, beams, min_range, message
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

A lidar's `message` is `point_cloud` (default), or `laser_scan` for a planar scanner, such as a
RoboRacer's Hokuyo UST-10LX:

```yaml
  laser:
    type: lidar
    message: laser_scan
    channels: 1
    vertical_fov: [0.0, 0.0]
    horizontal_fov: 270.0   # deg, centred on the sensor's x
    beams: 1081             # across horizontal_fov, both ends included, as urg_node counts them
    rate_hz: 40.0
    range: 10.0             # m
    min_range: 0.06         # m
```

A `laser_scan` lidar has one ring, and its step (`horizontal_fov / (beams - 1)`) must divide
360 deg into whole columns: Isaac fires a full turn of them and keeps the scan's window. On
Isaac it is published as `sensor_msgs/LaserScan` on the sensor's `topic`. `horizontal_fov` also
reaches CARLA's lidar, and a `laser_scan` sets CARLA's `points_per_second` to one point per beam
(`carla.attributes` win, as everywhere).

A lidar's `carla.intensity` makes its points' intensity: acb_bridge publishes the semantic
lidar's points with intensity `reflectance[tag] x |cos(incidence)|`, rounded and clamped to
0-255 (a Lambertian surface seen by a lidar that reports reflectivity). The ray-cast lidar's
own intensity is only the atmosphere's attenuation by range, the same on every surface, and a
detector that reads intensity (TransFusion) finds nothing in it. The tags are CARLA 0.9.16's
(`carla.CityObjectLabel`, `spec.CARLA_SEMANTIC_TAGS`); `carla_sensors.yaml` carries the table
by tag id (`intensity.reflectance`, a list indexed by the tag), the default filled in. The
semantic lidar has no attenuation, dropoff or noise attributes.

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
for the vehicle bridge's command path, since CARLA's model has none. With `tires.front` or
`tires.rear`, each wheel takes its axle's friction and stiffnesses. With `body.contact`, the
file has a `not_applied` list naming it (CARLA has no per-actor material); without, it has
none.

`carla.physics` holds what the spec's sections cannot say:

| key | what |
|---|---|
| `tire_friction_scale` | CARLA's tire friction for a tire friction of 1.0 (measured: 1.2 stops from 100 km/h in 42.8 m, as the spec's car does in Isaac) |
| `drag_area` | m^2, what CARLA multiplies its drag coefficient by |
| `body_com` | m, actor frame: the CoM UE computes for the body (CARLA does not expose it) |
| `rear_axle_x` | m, actor frame: the rear axle (scensim_carla_world's measured table) |
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
  scensim_carla_world and acb_bridge): `CarlaPhysics::load(path)` (refuses a file without four
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

## CARLA 1.0's physics

`generated/<name>/carla_physics_1.0.yaml`, when a body has `carla.physics_1_0`: CARLA 1.0's
`VehiclePhysicsControl` on its Chaos vehicle, SI units, wheels in CARLA's order, with `carla:
"1.0"` and `blueprint`. Rust: `carla_physics_spec::chaos::ChaosPhysics` (`load`, which
refuses a file for another CARLA, `differences`, `as_read_back`). Chaos applies from a client
only part of what 0.9.16's PhysX vehicle did; the rest stays the blueprint asset's and is
listed in the file's `not_applied`, neither applied nor compared.

`carla.physics_1_0`:

| key | what |
|---|---|
| `friction_multiplier_scale` | Chaos's `friction_force_multiplier` for a tire friction of 1.0 (measured on CARLA 1.0: a locked wheel's mu is 0.50 x the multiplier + 0.035) |
| `chassis_area` | m^2: what CARLA 1.0 multiplies its drag coefficient by (the blueprint's chassis width x height; its `drag_area` is unused) |
| `front_axle_x`, `rear_axle_x` | m, actor frame: the body's axles (scensim_carla_world's measured table) |

From the spec as the body: `mass`; `center_of_mass` absolute in the actor frame (at
`com_front_fraction` of the body's wheel base ahead of its rear axle, `com_height` up);
`max_torque` and `torque_curve` normalised to it; `final_ratio` the body's `gear_ratio`;
`differential` from `drive_axle`; `drag_coefficient` (Cd A over `chassis_area`); per wheel the
body's radius, `friction_force_multiplier` (its axle's friction with `tires.front` or
`tires.rear`), the steer limit, the brake torques; the same `command_lags` as
`carla_physics.yaml`. A `body.contact` is added to `not_applied`.

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
- `Properties`: `<Property name="scensim.spec_hash" value="<spec_hash>"/>`. The scenario bridge
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
`<name>_description/config/spec_hash.yaml`), and the catalog entry's `scensim.spec_hash`
property. The scenario bridge refuses an ego whose parts disagree, naming who loaded which
(roadmap 051). Standalone, acb_bridge compares its sensor file's hash with the Autoware
packages' and warns.

**`physics_hash`** identifies what drives the car: the same steps over a mapping of the
sections `geometry`, `inertia`, `wheels`, `tires`, `brakes`, `powertrain`, `resistance`,
`steering`, `suspension` (a missing one as null) and one more key, `"carla.physics"`, holding
`carla.physics` (null without it). The pedal maps measured on CARLA carry it
(autoware_sim_vehicle's `pedal_maps.yaml`), so moving a sensor does not invalidate them, and
`carla_physics.yaml` names it.

The optional keys of W24 move the hashes only when written, so a spec without them keeps both.
`tires.front` and `tires.rear` are in `tires`, so they move `physics_hash` and with it the
pedal maps' key. `body.contact` moves `spec_hash` (it covers the whole spec) but not
`physics_hash`: collisions do not drive the car, and the pedal maps stay valid.

With `bodies`, `spec_hash` is the file's (every body's outputs name the same), and
`physics_hash` is a body's: computed over `effective(spec, body)`, so each body has its own,
and the spec alone has none (`physics_hash` refuses it). The `carla-0.9.16` body of the Tesla
Model 3 is, value for value, the spec of before bodies, so its physics_hash is the one its
pedal maps were measured under.

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
