# Positronic
Repository for our work at @seame-pt: ROS 2 control of a myCobot 280 (Jetson Nano).

## Docs

- [M1 plan](docs/M1_plan.md): epics, user stories and split by pairs
- [Sprints](docs/sprints/): one file per sprint, goals, review, notes
- [Standups](docs/standups/): one file per day, copied from the template,
  committed directly to `dev`
- [Build and run](docs/build.md): `make` targets, VM and Jetson workflow
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
make image     # build the dev image (first time: 10-20 min)
make doctor    # check the environment
make shell     # shell in the container, repo at the same path as on your machine
make build     # colcon build
make test      # colcon test (unit tests + linters)
make format    # format src/ (clang-format, then ROS uncrustify)
make help      # all targets
```

`ARM_IP=<jetson ip> make doctor` also checks that the Jetson is reachable.

## Errors and warnings in your editor

your editor can show compiler errors and clang-tidy warnings as you type, with the same flags
as `make build`. They use **clangd**, which reads `build/compile_commands.json` (written by
`make build`) and our `.clang-tidy`. clangd runs on your machine, so it needs the ROS 2 Jazzy
headers there too. That only works on **Ubuntu 24.04**.

**Once per machine**, with your editor closed:

```bash
make setup-editor
```

It asks for your sudo password, adds the ROS 2 apt repository, installs clangd and the ROS
headers we use, adds `source /opt/ros/jazzy/setup.bash` to `~/.bashrc` (for the `ros2` command),
installs the VS Code clangd extension if you have VS Code, then builds. It's safe to run again.

Then open the repo in your editor. **VS Code:** when the clangd extension asks to disable
Microsoft's C/C++ IntelliSense, accept: the two conflict.

**Check:** open a `.cpp` in a package. `#include "rclcpp/rclcpp.hpp"` has no error and
Ctrl+click opens the header.

**Things that look broken but aren't:**

| Symptom | Cause | Fix |
|---|---|---|
| Every ROS include is "file not found" | The file isn't in a package, or `make build` hasn't run since the package was created. clangd only knows files listed in `build/compile_commands.json` | Put the file in a package under `src/`, run `make build` |
| Still red after `make build` | The editor was open before `compile_commands.json` existed | Restart the language server once (VS Code: **clangd: Restart language server**; Zed: **editor: restart language server**) |
| `which ros2` prints nothing | That terminal was open before setup | Open a new terminal. clangd doesn't need this |
| A `compile_commands.json` in the repo root | Leftover from an old setup. clangd uses it instead of `build/` | Delete it |

## Workflow

Issue → "Create a branch" from the issue (base `dev`) → PR to `dev` with 2 approvals (pair partner + one from another pair), squash merge → `dev` to `main` at the end of the sprint. Full rules: [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
