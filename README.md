# sim_vehicle_spec

One YAML per simulated car (`vehicles/<name>.yaml`) is the single source of truth for that car
on every simulator and for every consumer: Autoware, CARLA, Isaac Sim and
scenario_simulator_v2 ([scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge)
roadmap 051). Format: [docs/spec-format.md](docs/spec-format.md).

| Path | What |
|---|---|
| `vehicles/` | The specs (`tesla_model3.yaml`) |
| `sim_vehicle_spec/` | The library: `spec` (load, validate, the geometry, sensors and fingerprints a spec gives), `emit` (how generated files are written), `gen` (this repository's outputs) |
| `generated/<name>/carla_sensors.yaml` | The sensors acb_bridge spawns on CARLA |
| `catalogs/vehicles.xosc` | The SSv2 vehicle catalog, one entry per spec |

Each consumer generates its own files from a spec with this library, in its own repository:
[autoware_sim_vehicle](https://github.com/NEWSLabNTU/autoware_sim_vehicle) Autoware's vehicle and
sensor kit packages, [autoware_isaac_bridge](https://github.com/NEWSLabNTU/autoware_isaac_bridge)
the Isaac rig. Every generated file carries the spec's `spec_hash`, and the parts of a run
compare the hashes they loaded.

```bash
just generate          # write the outputs
just test              # outputs current, and the spec tests (no ROS, no simulator)
just build             # the ROS package: share/sim_vehicle_spec/{vehicles,generated,catalogs}
```
