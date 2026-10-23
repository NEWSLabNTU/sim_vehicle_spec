# sim_vehicle_spec

One YAML per simulated car (`vehicles/<name>.yaml`) is the single source of truth for that car
on every simulator and for every consumer: Autoware, CARLA, Isaac Sim and
scenario_simulator_v2 ([scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge)
roadmap 051). The format, version 2: [docs/spec-format.md](docs/spec-format.md).

This repository holds the specs, the generator library every consumer uses, and the outputs no
single autopilot owns. It depends on nothing but Python 3 and PyYAML (and Rust for the crate).

| Path | What |
|---|---|
| `vehicles/` | The specs: `tesla_model3.yaml`, a Model 3 RWD on CARLA's `vehicle.tesla.model3` body |
| `sim_vehicle_spec/` | The library: `spec` (load, validate, and what a spec gives: `box`, `axles`, `sensors`, `camera_intrinsics`, `spec_hash`, `physics_hash`), `emit` (how every repository writes generated files), `frames` (transforms), `gen` (this repository's outputs), `carla_physics` (`carla_physics.yaml` on a CARLA actor) |
| `generated/<name>/carla_sensors.yaml` | The sensors acb_bridge spawns on CARLA, keyed by the URDF frame each is mounted at, with their blueprint attributes |
| `generated/<name>/carla_physics.yaml` | CARLA's physics for the car (`VehiclePhysicsControl` and the command lags); only for a spec with `carla.physics` |
| `catalogs/vehicles.xosc` | The SSv2 vehicle catalog: one entry per spec |
| `crates/carla_physics_spec/` | `carla_physics.yaml` as Rust types, and the read-back comparison |

## Who generates what

Each consumer generates its own files from a spec with this library, in its own repository,
so every output lives in exactly one place:

| output | repository |
|---|---|
| CARLA's sensor and physics files, the SSv2 catalog | here (`just generate`) |
| Autoware's per-car packages (`<name>_description`, `<name>_launch`, `<name>_sensor_kit_description`, `<name>_sensor_kit_launch`, `individual_params`) | [autoware_sim_vehicle](https://github.com/NEWSLabNTU/autoware_sim_vehicle) |
| the Isaac rig (`src/aib/config/<name>.rig.yaml`) | [autoware_isaac_bridge](https://github.com/NEWSLabNTU/autoware_isaac_bridge) |

Every generated file starts with a header naming its spec and `spec_hash`, and each
repository's `just check-generated` fails when one of its outputs no longer matches the specs.
Consumers read the installed files (`share/sim_vehicle_spec/{vehicles,generated,catalogs}`),
never copies.

## Fingerprints

- `spec_hash`: SHA-256 of the spec in canonical form (parsed, `description` and `provenance`
  dropped, JSON with sorted keys). Comments, layout and key order do not change it. Every
  output carries it, and the parts of a run compare the hashes they loaded: the world server
  (`SpawnReply.body.spec_hash`), the vehicle agent, and the catalog entry's `swb.spec_hash`.
- `physics_hash`: the same over what drives the car (the physics sections and
  `carla.physics`). The pedal maps measured on CARLA carry it.

The exact canonical form: [docs/spec-format.md](docs/spec-format.md), "Fingerprints".

## CARLA's physics

`carla_physics.yaml` is the spec's car as CARLA's `VehiclePhysicsControl`, plus the actuator
lags CARLA's model lacks. With SSv2 the CARLA world server applies it to the ego after the
spawn; standalone, acb_bridge applies it to the hero on attach (`carla_physics_mode:=apply`)
or reads it back (`verify`). CARLA applies physics at its next tick, so read it back after a
tick.

```python
from sim_vehicle_spec import carla_physics
doc = carla_physics.load(path)
carla_physics.apply(vehicle, doc)          # VehiclePhysicsControl, then the wheels
# ... world.tick()
carla_physics.differences(vehicle, doc)    # [] when the actor runs it
```

In Rust, `carla_physics_spec::CarlaPhysics::load(path)` and `differences(&read_back,
friction_scale)`. The keys, the compared fields and the tolerance:
[docs/spec-format.md](docs/spec-format.md), "CARLA's physics".

## The SSv2 catalog

`catalogs/vehicles.xosc` has one `Vehicle` per spec: its box and axles from the geometry,
`model3d` the body, the `swb.spec_hash` property, and `maxSpeed`, `maxAcceleration`,
`maxDeceleration`, `maxSteering` as parameters. The entries declare **no**
`ParameterDeclarations` (in SSv2 a declared default shadows the reference's assignment), so
every `CatalogReference` assigns all four. A scenario finds it at
`$(find-pkg-share sim_vehicle_spec)/catalogs`.

## Generating from a spec in another repository

Add this repository as a submodule (`deps/sim_vehicle_spec`), put it on `sys.path`, and use
the library:

```python
from sim_vehicle_spec import emit, spec as specmod

spec = specmod.load("deps/sim_vehicle_spec/vehicles/tesla_model3.yaml")   # validates
header = emit.header(spec, "my_gen (`just generate`)", "sim_vehicle_spec/vehicles/tesla_model3.yaml")
outputs = {"config/vehicle.yaml": emit.comment(header) + emit.yaml_text({...})}
emit.stale(outputs, root)    # the paths that differ: what --check reports
emit.write(outputs, root)    # writes only what changed
```

`specmod.load` raises `SpecError` naming the file and the key for anything not in the format.

## Commands

```bash
just generate          # python3 -m sim_vehicle_spec.gen: write the outputs
just check-generated   # the same with --check: exit 1, naming each stale output
just test              # check-generated, the spec tests (pytest, no ROS, no simulator),
                       # the crate's tests (cargo)
just build             # the ROS package: share/sim_vehicle_spec/{vehicles,generated,catalogs},
                       # and the library importable
```
