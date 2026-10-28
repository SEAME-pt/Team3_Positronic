# Positronic
Repository for our work at @seame-pt: ROS 2 control of a myCobot 280 (Jetson Nano).

## Docs

- [M1 plan](docs/M1_plan.md): epics, user stories and split by pairs
- [Sprints](docs/sprints/): one file per sprint, goals, review, notes
- [Standups](docs/standups/): one file per day, copied from the template,
  committed to this sprint's standups branch, then merged to dev weekly.
- [Build and run](docs/build.md): `make` targets, laptop and Jetson workflow
- [Hardware specs](docs/hardware_specs/): Jetson Nano, pinouts
- [SEA:ME 2.0](docs/seame-2.0/): the programme's modules and hardware classes
- [Board](https://github.com/users/feazeved/projects/6): Backlog, Sprint backlog,
  In progress, In review, Done

## Setup (Ubuntu)

```bash
sudo apt install -y git make curl x11-xserver-utils
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER      # then log out and back in
```

## Daily use

```bash
make image           # build the dev image (first time: a few minutes up to 20)
make doctor          # check the environment
make shell           # shell in the container, repo mounted at /ws
make build           # colcon build
make test            # colcon test (unit tests + linters)
make tidy            # clang-tidy on src/ with our .clang-tidy (run make build first)
make format-check    # ROS lint on src/ (uncrustify, cpplint, cppcheck, copyright...), changes nothing
make format          # format src/ (clang-format, then ROS uncrustify), changes your files
make help            # all targets
```

Per-machine settings (the GPU mode, the Jetson's IP) live in `.env`: `cp .env.example .env`, then edit it
(see [docs/build.md](docs/build.md), "GPU for Gazebo and RViz"). With `ARM_IP` set, `make doctor` also checks
that the Jetson is reachable.

## Errors and warnings in your editor (dev container)

Open the repo **inside our dev image**. The editor's clangd then shows compiler errors and
clang-tidy warnings as you type, with the same ROS headers, compiler and flags as `make build`.
Nothing extra is installed on your machine: `.devcontainer/devcontainer.json` reuses the image
from `make image`.

1. `make image` (once, and again when the Dockerfile changes).
2. Open the repo in the container:
   - **VS Code:** install the **Dev Containers** extension, then run **Dev Containers: Reopen in Container**.
   - **Zed:** accept **Open in Container** when Zed asks, or run **project: open remote** → **Connect Dev Container**.
3. In the editor's terminal (now inside the container, at `/ws`): `make build`.
4. Open a `.cpp` in a package: `#include "rclcpp/rclcpp.hpp"` has no error and Ctrl+click opens the header.

In the editor's terminal, `make build`, `make test`, `make format`, `make format-check`,
`make tidy` and `make doctor` run directly in that container. Run `make image`, the `nano-*`
targets, `git` and GUI apps (RViz through `make shell`) from a normal terminal on your machine.

After a Dockerfile change: `make image`, then stop the old container (`docker ps`,
`docker kill <id>`) and reopen. Zed doesn't rebuild it on its own; VS Code has **Rebuild Container**.

| Symptom | Cause | Fix |
|---|---|---|
| Every ROS include is "file not found" | The file isn't in a package, or `make build` hasn't run since the package was created. clangd only knows files listed in `build/compile_commands.json` | Put the file in a package under `src/`, run `make build` |
| Still red after `make build` | clangd started before `compile_commands.json` existed | Restart the language server once (VS Code: **clangd: Restart language server**; Zed: **editor: restart language server**) |
| `make: docker: No such file or directory` | You ran `make image` or a `nano-*` target inside the container | Run it from a terminal on your machine |
| A `compile_commands.json` in the repo root | Leftover from an old setup. clangd uses it instead of `build/` | Delete it |

## Workflow

Issue → "Create a branch" from the issue (base `dev`) → PR to `dev` with 2 approvals (pair partner + one from another pair), squash merge → `dev` to `main` at the end of the sprint. Full rules: [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
