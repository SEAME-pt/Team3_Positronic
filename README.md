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

## Errors and warnings in your editor (VS Code)

Your editor can show compiler errors and clang-tidy warnings as you type, with the same flags and
headers as `make build`. It uses **clangd**, which reads `build/compile_commands.json` (written by
`make build`) and our `.clang-tidy`.

Needs **Ubuntu 24.04**, because clangd runs on your machine and needs the ROS 2 Jazzy headers there.
The build itself still happens only in Docker.

1. Install clangd, a compiler and the ROS headers we use. This needs the
   [ROS 2 apt repository](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
   set up first:
   ```bash
   sudo apt install -y clangd build-essential \
     ros-jazzy-ros-base ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit
   ```
2. In VS Code, install the **clangd** extension (`llvm-vs-code-extensions.vscode-clangd`). When it
   asks to disable Microsoft's C/C++ IntelliSense, accept: the two conflict.
3. Build once, so the commands file exists:
   ```bash
   make image && make build
   ```
4. Open any `.cpp` in a package. `#include "rclcpp/rclcpp.hpp"` should show no error, and Ctrl+click
   opens the header. If not, run **clangd: Restart language server** from the command palette.

Run `make build` again after adding a package or a source file, so clangd knows about it.
Zed works the same way: do steps 1 and 3, since Zed already includes clangd.

## Workflow

Issue → "Create a branch" from the issue (base `dev`) → PR to `dev` with 2 approvals (pair partner + one from another pair), squash merge → `dev` to `main` at the end of the sprint. Full rules: [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
