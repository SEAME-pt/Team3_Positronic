# 0001. One Dockerfile with runtime, ci and dev targets

2026-10-28, accepted, Felipe Suassuna (issue #82), approved by the PR reviewers.

## Context

One image (`positronic-dev`) served three very different users:

- **Laptops** need everything: build and lint tools, MoveIt, Gazebo, RViz.
- **CI** needs to build, lint and test our code, but no simulator or GUI.
- **The Jetson** (4 GB RAM, small disk, builds its image on-device) only needs to build and run the
  driver: ros2_control, the ROS lint set and pymycobot.

Measured before the change: CI took 26 min, a cold `make image` 11 min, and any Dockerfile change
reinstalled every package, because everything was one `apt-get install` layer followed by the uid/gid
step. The Jetson carried MoveIt and Gazebo it never uses.

## Decision

One `docker/Dockerfile` with package stages `core` → `tools` → `sim` and three targets:

| Target | Built from | Used by | Has |
| --- | --- | --- | --- |
| `runtime` | `core` | The Jetson (`positronic-runtime:nano`) | ros2_control, ROS lint set, pymycobot |
| `ci` | `tools` | GitHub Actions | `runtime` + clang tools + MoveIt |
| `dev` | `sim` | Laptops (`positronic-dev:jazzy`) | `ci` + Gazebo, RViz, rqt |

- One apt layer per theme inside each stage, so adding a package re-downloads one layer.
- The uid/gid setup (`docker/setup-user.sh`) is the last step of each target, so a different uid never
  invalidates the package layers.
- Each target sits right after its stage, and `runtime` right after `core`: the Jetson's Docker may use
  the classic builder, which builds every stage above `--target` whether it's needed or not.
- CI builds `ci` (format check, build, test) and `runtime`, and builds the Jetson's packages in `runtime`.

## Alternatives

- **Three separate Dockerfiles:** the package lists drift apart, and nothing shares layers.
- **`make image` + `make image-extra`:** splits laptops into two environments, and doesn't help CI or
  the Jetson.
- **Better caching only:** helps CI, not the Jetson, which builds on-device without a shared cache.
- **A prebuilt image pulled from a registry:** fastest everywhere, but needs registry credentials, a
  publishing rule and someone owning it.

## Consequences

- Measured after: cold `make image` 4 min 19 s on a test VM (the 11 min was on another machine, so
  the two aren't directly comparable); adding a package to `sim` 17 s; changing the uid 13.9 s; dev
  image 5.43 GB → 5.44 GB (only `mesa-utils` added). CI timings are in the PR for #82.
- A package in the wrong stage fails in CI instead of on a laptop or the Jetson. CI runs on amd64, so
  an arm64-only problem still shows up only on the Jetson.
- Contributors must pick the stage for each new package (rules at the top of the Dockerfile).
- `ci` is tagged `positronic-dev:jazzy` inside CI so `docker compose run` uses it; on laptops that name
  is the `dev` target.

## Revisit when

- CI stays slow even with a warm cache: consider a prebuilt image in a registry.
- The Jetson needs something only `tools` or `sim` has (MoveIt, perception): it goes into `core`, or the
  Jetson gets its own stage.
- M2 needs GPU inference images (CUDA/L4T), which may need a separate base image.
