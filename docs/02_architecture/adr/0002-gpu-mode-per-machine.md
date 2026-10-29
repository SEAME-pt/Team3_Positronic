# 0002. GPU mode chosen per machine with GPU= in .env

2026-10-28, accepted, Felipe Suassuna (issue #82), approved by the PR reviewers.

## Context

Gazebo Harmonic needs OpenGL 3.3 core. How the dev container reaches a GPU depends on the machine, and
the same setting doesn't work everywhere. Tests on the six team machines (outputs attached to #82) showed:

- Intel and AMD laptops work through Mesa once `/dev/dri` and the host's `video` and `render` groups
  are passed in. Their group numbers differ per machine.
- Laptops with Intel + NVIDIA, given only the NVIDIA devices, fell back to zink or llvmpipe: Gazebo
  segfaulted (940MX) or rendered silently in software.
- A Mac VM with virgl gives OpenGL 2.1, and Gazebo crashes. Software rendering (llvmpipe) works there.

## Decision

Each machine sets `GPU=dri|nvidia|sw` in its git-ignored `.env`. The Makefile adds the matching
`docker/gpu/<mode>.yaml` on top of `compose.yaml`:

| Mode | What it adds |
| --- | --- |
| `dri` | `/dev/dri`, plus the host's video and render group numbers, read from `/dev/dri` by the Makefile |
| `nvidia` | `gpus: all` and the variables that make GLX use the NVIDIA driver on two-GPU laptops |
| `sw` | `LIBGL_ALWAYS_SOFTWARE=1` |

Empty means no GPU in the container (software rendering, as before). The groups are added at run time
with `group_add`, not in the image, because the numbers differ per machine. `make doctor` reports the
OpenGL renderer: `[fail]` below 3.3 core, `[warn]` on software rendering without `GPU=sw`.

## Alternatives

- **Auto-detect the GPU in the Makefile:** a Mac VM has `/dev/dri` but only gives OpenGL 2.1, and
  two-GPU laptops need a choice between Intel and NVIDIA. Detection would guess wrong on exactly the
  machines that had problems.
- **Always pass `/dev/dri` in `compose.yaml`:** fails on machines without it, and doesn't cover NVIDIA.
- **Compose profiles:** profiles select whole services, not options of one service, so each mode would
  repeat the full `dev` service.
- **Bake the `render` group into the image:** its number differs per machine, and CI and the Jetson
  don't need it.
- **Per-person notes on which flags to use:** they drift, and nothing checks them.

## Consequences

- One line in `.env` per machine. A new mode is one new file in `docker/gpu/`, and the Makefile finds it
  by name.
- A wrong value stops `make` with the list of valid modes. Trailing spaces in `.env` are stripped.
- The editor's dev container runs the image directly, without the GPU file: Gazebo and RViz run from
  `make shell`.
- `GPU=nvidia` needs the NVIDIA Container Toolkit, and a Docker Compose recent enough for `gpus:`.

## Revisit when

- A new machine type fails `make doctor` in all three modes.
- Gazebo moves to a rendering backend with different needs (for example a newer Gazebo or Isaac Sim in M2).
