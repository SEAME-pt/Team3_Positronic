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
make shell     # shell in the container, repo mounted at /ws
make build     # colcon build
make test      # colcon test (unit tests + linters)
make format    # clang-format on src/
make help      # all targets
```

`ARM_IP=<jetson ip> make doctor` also checks that the Jetson is reachable.

## Workflow

Issue → "Create a branch" from the issue (base `dev`) → PR to `dev` with 1 review → `dev` to `main` at the end of the sprint.
