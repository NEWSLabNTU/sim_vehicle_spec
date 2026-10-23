"""sim_vehicle_spec: one YAML per simulated car, and what every consumer derives from it.

`spec` loads and validates a spec (docs/spec-format.md) and derives its geometry, sensors and
fingerprints; `emit` writes generated files the same way in every repository; `gen` writes the
outputs this repository owns (CARLA's sensor and physics files, SSv2's vehicle catalog). Other
repositories generate their own outputs with this library: autoware_sim_vehicle the Autoware
packages, autoware_isaac_bridge the Isaac rig. Pure: no simulator, no ROS.
"""
