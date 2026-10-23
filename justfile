# sim_vehicle_spec: one spec per simulated car (vehicles/), the generator library, and the
# outputs no single consumer owns (CARLA's sensor and physics files, the SSv2 catalog).

default:
    @just --list

# Write every output from the specs
generate:
    python3 -m sim_vehicle_spec.gen

# Fail when an output is stale (a spec changed, or an output was edited)
check-generated:
    python3 -m sim_vehicle_spec.gen --check

test: check-generated
    python3 -m pytest -q test

build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --base-paths .
