# Building and running

Everything runs inside Docker, through `make`. Run every command from the repository root on your VM.
`make help` lists the commands; this page explains when to use each one and what to watch out for.

## Where things run

- **Your VM**: you edit code in `src/`, and build, test and format it in the `mimesis-dev:jazzy` container.
- **The Jetson** (`ssh nano`): only code that needs the arm runs there, in the `mimesis-dev:nano` container.
  Each person has their own copy of the repo in `~/mimesis/<your VM username>`.
- Both containers use ROS 2 Jazzy and `ROS_DOMAIN_ID=17`, so nodes on the VM and the Jetson see each other.

Always edit on the VM. Every `nano-*` command first mirrors your VM's repo onto the Jetson, so anything
you change directly on the Jetson is overwritten.

## First time

```bash
make image                    # build the dev image (10-20 min)
ARM_IP=<jetson ip> make doctor  # every line should say [ok]
```

Get the Jetson's IP with `ssh nano "hostname -I"` (the first address). The Jetson image already
exists, so you don't need `make nano-image`.

## Daily workflow

```bash
# On the VM: write code, then
make build          # compile
make test           # unit tests
make format         # fix formatting before committing

# On the Jetson, when the code needs the arm
make nano-build     # copy your code to the Jetson and compile it there
make nano-shell     # open a shell there, with the arm's serial port
ros2 run <package> <node>
```

In a second terminal, `make shell` on the VM gives you `ros2 topic echo`, `ros2 service call`
and RViz, all talking to the nodes on the Jetson.

## VM commands

**`make image`**: builds `mimesis-dev:jazzy` from `docker/Dockerfile`. Run it once, and again whenever
someone changes the Dockerfile. It doesn't compile our code.

**`make shell`**: opens a terminal inside a fresh container. Your repo is at `/ws`, and ROS is already
loaded. Use it for anything interactive: `ros2 topic list`, `ros2 topic echo /joint_states`, `rviz2`.
For RViz, run it from a terminal on the VM's desktop, not over SSH, otherwise windows can't open.
`exit` leaves; the container is deleted, but your files stay because `/ws` is your real repo folder.

**`make build`**: runs `colcon build` on every package in `src/`. The output goes to `build/`,
`install/` and `log/` (ignored by Git). It also writes `compile_commands.json`, which `make tidy`
and Zed use to understand the code.
To build only one package, use `make shell` and run `colcon build --packages-select <package>`.

**`make test`**: runs every package's tests and prints a summary. Read the last lines: they say how
many tests failed. Run `make build` first, since the tests run against the last build.

**`make format`**: rewrites C++ files in `src/` to match `.clang-format`. It changes files, so check
`git diff` afterwards. **`make format-check`** only reports problems, without changing anything; use
it to check before pushing.

**`make tidy`**: runs clang-tidy (static analysis, rules in `.clang-tidy`) on our code. Needs
`make build` first. Warnings don't fail the command; read them and fix what's real.

**`make doctor`**: checks the environment: ROS, tools, write access to the repo, ROS 2 messaging.
With `ARM_IP=<jetson ip>` it also checks the Jetson is reachable. Run it first when something is
broken.

**`make clean`**: deletes `build/`, `install/` and `log/`. Use it when a build behaves strangely,
then `make build` again.

## Jetson commands

**`make nano-sync`**: copies your repo to `~/mimesis/<you>` on the Jetson. It mirrors: files you
deleted on the VM are deleted there too. `.git`, `build/`, `install/` and `log/` are not copied, since
the Jetson builds its own. You rarely call it directly; the two commands below run it first.

**`make nano-build`**: syncs, then compiles on the Jetson in a throwaway container. It compiles
2 packages at a time at most, because the Jetson has 4 GB of RAM. Slower than the VM; that's normal.

**`make nano-shell`**: syncs, then opens a shell in a Jetson container that can use the arm
(`/dev/ttyTHS1`). Only one person at a time: if someone already has it open, you get
`The container name "/mimesis-arm" is already in use`. Your built packages are loaded automatically,
so `ros2 run` works straight away. If you rebuild inside the shell, run `source install/setup.bash`
afterwards.

**`make nano-image`**: rebuilds the Jetson's image on the Jetson. Only needed when the Dockerfile
changes, and only by one person: tell the group when you run it. Takes 15-25 minutes.

## Changing the defaults

Add a variable before the command to override it for that run:

| Variable | Default | Example |
| --- | --- | --- |
| `NANO` | `nano` (from `~/.ssh/config`) | `NANO=10.20.118.0 make nano-shell` if `nano.local` doesn't resolve |
| `NANO_DIR` | `mimesis/<your VM username>` | `NANO_DIR=mimesis/test make nano-build` for a separate copy |
| `ROS_DOMAIN_ID` | `17` | Must be the same on both sides, and different from other teams |
| `ARM_IP` | empty | `ARM_IP=10.20.118.0 make doctor` |

## When something goes wrong

| Problem | Fix |
| --- | --- |
| Anything strange | `make doctor` first |
| `ros2 topic echo` on the VM shows nothing from the Jetson | Same `ROS_DOMAIN_ID` on both sides? VM network in Bridged mode (`ip addr` must not show `192.168.64.x`)? |
| `mimesis-arm` is already in use | Someone has the arm. Ask before running `ssh nano docker rm -f mimesis-arm` |
| `Could not resolve hostname nano.local` | Use the IP: `NANO=<ip> make ...`, or put the IP in `~/.ssh/config` |
| `Permission denied` on the Jetson's `/ws` | The Jetson image is out of date: `make nano-image` |
| Build fails after pulling someone's Dockerfile change | `make image` (and `make nano-image` for the Jetson) |
| `make tidy` finds nothing | Run `make build` first |
