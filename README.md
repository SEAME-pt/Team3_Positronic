# Mimesis
Repository for our work at @seame-pt: ROS 2 control of a myCobot 280 (Jetson Nano).

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
