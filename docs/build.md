# Building and running

Everything runs inside Docker, through `make`. Run every command from the repository root, either in a
terminal on your machine or in your editor's dev container terminal (see the README).
`make help` lists the commands; this page explains when to use each one and what to watch out for.

## Where things run

- **Your Machine**: you edit code in `src/`, and build, test and format it in the `positronic-dev:jazzy` container.
- **The Jetson** (`ssh nano`): only code that needs the arm runs there, in the `positronic-runtime:nano` container.
  Each person has their own copy of the repo in `~/positronic/<your machine username>`.
- Both containers use ROS 2 Jazzy and `ROS_DOMAIN_ID=17`, so nodes on your machine and the Jetson see each other.

Always edit on your machine. Every `nano-*` command first mirrors your machine's repo onto the Jetson, so anything
you change directly on the Jetson is overwritten.

## Docker images

One `docker/Dockerfile`, three targets that share their lower layers (why: issue #82):

| Target | Image | Used by | Has |
| --- | --- | --- | --- |
| `runtime` | `positronic-runtime:nano` | The Jetson (`make nano-image`) | ros2_control, pymycobot, ROS lint set |
| `ci` | `positronic-dev:jazzy` in CI | GitHub Actions | `runtime` + clang tools + MoveIt |
| `dev` | `positronic-dev:jazzy` | Your machine (`make image`) | `ci` + Gazebo, RViz, rqt |

**Adding a package:** put it in the lowest stage that needs it (rules at the top of the Dockerfile),
then `make image`. Only that stage's layer is downloaded again. CI builds `ci` and `runtime`, so a
package missing from the wrong stage fails in CI, not on your laptop. Build one target by hand with
`docker build --target <name> docker`.

## First time

```bash
cp .env.example .env          # your per-machine settings (git-ignored): set GPU=, see "GPU for Gazebo and RViz"
make image                    # build the dev image (a few minutes up to 20, depending on your connection)
make doctor                   # every line should say [ok]
```

Get the Jetson's IP with `ssh nano "hostname -I"` (the first address) and put it in `.env` as `ARM_IP=<ip>`,
so `make doctor` also checks that the Jetson is reachable. The Jetson image is built by
one person, not by everyone (see *Jetson commands*).

For errors and warnings in your editor, open the repo in the dev container: README,
"Errors and warnings in your editor". Nothing is installed on your machine.

## Daily workflow

```bash
# On your machine: write code, then
make build          # compile
make test           # unit tests
make format         # fix formatting before committing

# On the Jetson, when the code needs the arm
make nano-build     # copy your code to the Jetson and compile it there
make nano-shell     # open a shell there, with the arm's serial port
ros2 run <package> <node>
```

In a second terminal, `make shell` on your machine gives you `ros2 topic echo`, `ros2 service call`
and RViz, all talking to the nodes on the Jetson.

## Dev commands

**`make image`**: builds `positronic-dev:jazzy`, the `dev` target of `docker/Dockerfile`. Run it once, and again whenever
someone changes the Dockerfile. It doesn't compile our code.

**`make shell`**: opens a terminal inside a fresh container. Your repo is at `/ws`, and ROS is already
loaded. Use it for anything interactive: `ros2 topic list`, `ros2 topic echo /joint_states`, `rviz2`.
For RViz, don't run it over SSH, it needs to be able to open windows.
`exit` leaves; the container is deleted, but your files stay because the container uses your real repo folder.

**`make build`**: runs `colcon build` on every package in `src/`. The output goes to `build/`,
`install/` and `log/` (ignored by Git). It also writes `build/compile_commands.json`, which `make tidy`
and the editor's clangd use to understand the code (see the README, "Errors and warnings in your editor").
To build only one package, use `make shell` and run `colcon build --packages-select <package>`.

**`make test`**: runs every package's tests plus linters and prints a summary. Read the last lines: they say how
many tests failed. Run `make build` first, since the tests run against the last build.

**`make format`**: rewrites C++ files in `src/` with clang-format (`.clang-format`), then ROS's uncrustify. It changes files, so check
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

**`make nano-sync`**: copies your repo to `~/positronic/<you>` on the Jetson. It mirrors: files you
deleted on your machine are deleted there too. `.git`, `build/`, `install/` and `log/` are not copied, since
the Jetson builds its own. You rarely call it directly; the two commands below run it first.

**`make nano-build`**: syncs, then compiles on the Jetson in a throwaway container. It compiles
2 packages at a time at most, because the Jetson has 4 GB of RAM. Slower than our machines; that's normal.

**`make nano-shell`**: syncs, then opens a shell in a Jetson container that can use the arm
(`/dev/ttyTHS1`). Only one person at a time: if someone already has it open, you get
`The container name "/positronic-arm" is already in use`. Your built packages are loaded automatically,
so `ros2 run` works straight away. If you rebuild inside the shell, run `source install/setup.bash`
afterwards.

**`make nano-image`**: builds the slim Jetson image (`runtime` target, `positronic-runtime:nano`) on the
Jetson. Only needed when the Dockerfile changes, and only by one person: tell the group when you run it.
Then run `make nano-service`, because the unit names the image: restarting the old unit would start the old
image. The old `positronic-dev:nano` stays on the Jetson as a rollback; remove it later with
`ssh nano docker rmi positronic-dev:nano`.

## Jetson boot service

The Jetson starts a container called `positronic-driver` at boot, with nobody logged in, through
a systemd unit: [`docker/jetson/positronic-driver.service`](../docker/jetson/positronic-driver.service).
For now it only keeps a ROS 2 container alive with the arm's serial port mapped; the driver
replaces that placeholder once it exists (#21-#23). It doesn't open the serial port and has its
own name, so `make nano-shell` works as before.

**Install or update it** (needs the Jetson's sudo password, once per change of the unit file):

```bash
make nano-service
```

It copies the unit to `/etc/systemd/system/`, enables it at boot and (re)starts it.
The file in the repo is the source of truth: never edit the copy on the Jetson.

**Check it:**

```bash
ssh nano systemctl status positronic-driver               # running? since when? last log lines
ssh nano journalctl -u positronic-driver -n 50            # its logs and every start, stop and crash
ssh nano "docker exec positronic-driver bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic list'"
```

| Command (on the Jetson) | Effect |
| --- | --- |
| `sudo systemctl stop positronic-driver` | Stops it until the next boot |
| `sudo systemctl disable positronic-driver` | No longer starts at boot (still running until stopped) |
| `sudo systemctl restart positronic-driver` | Restarts it with the current unit. After `make nano-image`, run `make nano-service` instead (the unit names the image) |

Why systemd and not a Docker restart policy: systemd starts a fresh container from the current
image on every boot, waits for the network, keeps start/crash/stop history in one log, and
stopping it never disables it for the next boot. Never add `--restart` to this container:
two supervisors fight over it.

## GPU for Gazebo and RViz

Gazebo needs OpenGL 3.3. How the container reaches the GPU depends on the machine, so `make` picks a
mode from `GPU` in your `.env` (each mode is a file in `docker/gpu/`):

| `GPU=` | Use it on | What it does |
| --- | --- | --- |
| `dri` | Intel, AMD, laptops with two GPUs (it uses the Intel one) | Passes `/dev/dri` into the container and adds your `video` and `render` groups. Their numbers are read from your machine, because they differ per machine |
| `nvidia` | NVIDIA-only machines. Needs the NVIDIA Container Toolkit on the host | Gives the container the NVIDIA GPU (`gpus: all`) and the variables laptops with two GPUs need |
| `sw` | A Mac VM, or any machine where the others crash Gazebo | Software rendering (llvmpipe): slow, but works everywhere |
| empty | | No GPU file; Mesa decides. This is how it was before |

1. `cp .env.example .env`, then set `GPU=` (one line, no quotes, no spaces; comments on their own lines).
2. `make doctor`: the `OpenGL` line must be `[ok]`. `[fail]` means Gazebo would crash; `[warn]` means
   software rendering without `GPU=sw` (works, but slow).
3. One run with another mode: `make GPU=sw shell`. Not `GPU=sw make shell`: a value in `.env` beats the environment.

A new mode is a new file `docker/gpu/<name>.yaml`; the Makefile finds it by name. The editor's dev container
terminal doesn't get the GPU (it runs the image directly): start Gazebo and RViz with `make shell` from a normal
terminal. The test data of the six team machines is in `scripts/gpu-test-*.txt`.

## Changing the defaults

Put per-machine values in `.env` (copy `.env.example`), or override one run with `make VAR=value target`:

| Variable | Default | Example |
| --- | --- | --- |
| `NANO` | `nano` (from `~/.ssh/config`) | `make NANO=10.20.118.0 nano-shell` if `nano.local` doesn't resolve |
| `NANO_DIR` | `positronic/<your username>` | `make NANO_DIR=positronic/test nano-build` for a separate copy |
| `ROS_DOMAIN_ID` | `17` | Must be the same on both sides, and different from other teams |
| `ARM_IP` | empty | `ARM_IP=10.20.118.0` in `.env`, or `make ARM_IP=10.20.118.0 doctor` |
| `GPU` | empty | `GPU=sw` in `.env`, or `make GPU=sw shell` (see above) |

## When something goes wrong

| Problem | Fix |
| --- | --- |
| Anything strange | `make doctor` first |
| `ros2 topic echo` on your machine shows nothing from the Jetson | Same `ROS_DOMAIN_ID` on both sides? |
| `positronic-arm` is already in use | Someone has the arm. Ask before running `ssh nano docker rm -f positronic-arm` |
| `Could not resolve hostname nano.local` | Use the IP: `NANO=<ip> make ...`, or put the IP in `~/.ssh/config` |
| `Permission denied` on the Jetson's `/ws` | The Jetson image is out of date: `make nano-image` |
| Build fails after pulling someone's Dockerfile change | `make image` (and `make nano-image` for the Jetson) |
| `make tidy` finds nothing | Run `make build` first |
| `positronic-driver` shows `failed` | `ssh nano journalctl -u positronic-driver -n 50`. Usual causes: image missing (`make nano-image`), serial port missing |
| `apt-get update` fails with `Release file ... is not valid yet` | The VM's clock is behind (a Mac VM after sleep): `sudo systemctl restart systemd-timesyncd`, then build again |
| CI fails but the same command passes on your laptop | The package is in a stage `ci` doesn't include (`sim` only?). Move it to `tools` or `core` |
| Gazebo closes with `OpenGL 3.3 is not supported` | `make doctor`, then set `GPU=` in `.env` (`sw` if nothing else works) |
| `GPU=sw make shell` seems to do nothing | A value in `.env` beats the environment: use `make GPU=sw shell` |
| `could not select device driver \"nvidia\"` with `GPU=nvidia` | The NVIDIA Container Toolkit isn't installed on the host: install it, or use `GPU=dri` or `sw` |
| Laptops don't see the Jetson's topics after a reboot | The service started before the Wi-Fi: `ssh -t nano sudo systemctl restart positronic-driver`, then open an issue |
