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
    cd crates/carla_physics_spec && cargo test -q

# The tests, then lint: pyflakes, and rustfmt and clippy on the Rust reader
check: test
    python3 -m pyflakes sim_vehicle_spec test
    cd crates/carla_physics_spec && cargo fmt --check && cargo clippy -q -- -D warnings

build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --base-paths .
