# Positronic: project context

Team context for Claude and for us. Last updated 12 Oct 2026 (end of Sprint 0).
When the repo is attached and disagrees with this file, the repo wins. Update this file at every sprint review.

---

## 1. The program: SEA:ME 2.0 (42 Porto, 2026/27)

Source: https://github.com/SEAME-pt/seame-2.0. A 26-week program plus a 12-week corporate capstone. Theme: "Physical AI", meaning software that makes real machines move, see, decide and cooperate.

| Phase | Dates | What the **arm** must show |
|---|---|---|
| Sprint 0 · Onboarding | 6–16 Oct 2026 | Tooling, Git, ROS 2 basics, hardware teardown, safety induction |
| **M1** · Architecture, RT control & simulation | 19 Oct – 11 Dec 2026 | Reach a **Cartesian goal pose through MoveIt 2**, in sim and on hardware, **with the pose error reported**. E-stop honoured through a lifecycle node; halt on joint limits and on comms timeout. |
| **M2** · Perception, edge AI & spatial | 14 Dec 2026 – 26 Feb 2027 | End-effector pose from FK verified against an external measurement. A pose estimator feeds **visual servoing**. **Pick-and-place** with tight alignment. A **demonstrated trajectory** (free-drive, DMP/ProMP) learned and replayed under variations. Compute split between Jetson and host, measured. |
| **M3** · Multi-agent ops | 1 Mar – 30 Apr 2027 | Serve a docking/loading request from a peer (likely the car) with visual alignment. On peer loss: abort and return home. SROS 2, signed OTA, interface contract with the other team. |
| **M4** · Corporate capstone | 10 May – 30 Jul 2027 | Arm project: #3, Quality Gate & Programming by Demonstration (injection-moulded parts). Possibly #5 (Car + Arm). |

Breaks: 21 Dec – 3 Jan, 8–12 Feb, 22–26 Mar.

**Deliverables common to every module** (M1 shown; M2 and M3 are similar):
- Architecture document: layer diagram, node graph, message contracts, timing budget per loop.
- Green CI: build, unit tests, static analysis, **coverage > 80 %**, plus a hardware-in-the-loop job on a self-hosted runner.
- Real-time profile: measured loop period and jitter, plotted.
- Sim-to-real report: the same motion in sim and on hardware, with the measured difference.
- E-stop under fault injection, shown live: facilitators kill a node, drop the link, or feed bad data.
- Sprint demo and retrospective every 2 weeks.

**Program rules:**
- Same milestones for every platform, proven differently on each.
- Simulation first.
- "Measured, not merely working."
- Team work with individual evaluation, and an individual oral defense.

**Reference architecture (M1).** Layers, top to bottom: Mission/behaviour → ROS 2 middleware (lifecycle nodes, actions, explicit QoS, TF2) → Hardware abstraction → Real-time layer → Sensors and actuators.
- For the arm, the **real-time layer is the arm's own servo controller**. We send it joint targets over serial; we don't write firmware.
- The **hardware abstraction is a ros2_control hardware interface** (our C++ plugin) plus a joint trajectory controller, with MoveIt 2 on top.
- The same interface must work for sim and real, so mission code never knows which one it's talking to.

**Competition:** 4 teams. Two are on drones; one other team has an arm like ours. We want to be the best team.

## 2. People

**Team Positronic (6).** No fixed areas of ownership. The **sprint leader rotates** each sprint, and roles are assigned at sprint planning. Shared goal: an outstanding project first, and maximum learning for everyone.
- Felipe Suassuna (GitHub `feazeved`)
- Leandro Gertrudes
- Wallace Lucas
- Rui Moura
- Pedro Neiva (GitHub `neivaeu`)
- Vinícius Polatto

**Background:**
- Five of us did the 42 common core's C/C++ track (C, C++, Unix, Git).
- **Rui Moura did the new Python/AI common core and has no C++ background.** He wants to learn C++. The team, Rui included, also wants him to work in Python, where he's strongest: M2 perception/ML, tooling, tests, launch files, data and analysis scripts.
- We're all mid level at GitHub and new to robotics, ROS 2, embedded and control.

**SEA:ME staff:**
- **Filipe**, coordinator. 25+ years of industry experience (Microsoft, Eclipse Foundation).
- **Pedro** and **Paulo**, facilitators. Industry engineers with embedded backgrounds. Facilitator Pedro is not our teammate Pedro Neiva.
- **João**, junior facilitator. A former SEA:ME participant.

## 3. Hardware

**Arm:** Elephant Robotics myCobot 280 Jetson Nano.
- 6 DOF, 280 mm reach, ~250 g payload.
- Hobby-grade servos: expect backlash and limited repeatability.
- Supports free-drive (moving it by hand), which matters for learning from demonstration in M2.
- Comes with the myCobot Adaptive Gripper. The lab has 3 units.

**Jetson Nano in the base** (recorded 7 Oct 2026). This is a **shared machine**: hostname `nano`, user `er`, on the `SEAME` Wi-Fi network.

| Item | Value | Why it matters |
|---|---|---|
| OS / kernel | Ubuntu 20.04.6, L4T R32.6.1 (JetPack 4.6), kernel 4.9, aarch64 | Old base. We run ROS 2 in Docker instead of natively. |
| CPU / RAM | 4× Cortex-A57, **4 GB shared with GPU** (~1.2 GB free when idle), 2 GB zram swap | Never run Gazebo, MoveIt or builds with high parallelism on it |
| GPU | CUDA 10.2 | Only usable from NVIDIA L4T r32 containers, **not from our Jazzy container** (see §7) |
| Serial to arm | `/dev/ttyTHS1`, **1 000 000 baud** | Our driver talks to the servo controller here. `er` is in `dialout`. |
| Python / pymycobot | Python 3.8.10, pymycobot 4.0.4b5, class `MyCobot280` | Reference implementation of the serial protocol. Units are **degrees and mm**. |
| End effector | M5Stack Atom (ESP32) at the tip, firmware 7.3 (MyStudio). It controls the gripper through the arm's serial link. | The gripper goes through the same serial protocol |
| Camera | `/dev/video0`, mounted near the tip of the arm. Use **MJPG** (30 fps at 2592×1944; YUYV is only 3 fps). | Needed for visual servoing in M2. Hand-eye calibration is required. |
| Docker | 24.0.7 (no compose plugin) | The Makefile uses plain `docker run` there |
| 40-pin header | Reads **right to left** in the case: pin 1 is on the far right. The sticker uses BCM numbering. UART2 and I2C2 are used internally. | See `docs/hardware_specs/pinouts.md` |

**Jetson rules:**
- No `apt upgrade` or `dist-upgrade`, and no JetPack update.
- Don't upgrade the system pymycobot.
- One person moves the arm at a time.
- Edit code on your own machine, never directly on the Jetson. `nano-sync` overwrites the Jetson copy.
- Turn off Wi-Fi power saving (`sudo iwconfig wlan0 power off`) if DDS messages drop.

**Safety:**
- **There is no e-stop in the cell yet.** SEA:ME expects a hardwired stop that our lifecycle node honours, and we have to design and build it (see §7, open questions).
- Until it exists, stop the arm with the power switch or a release/stop command, with one person keeping a hand on it.
- Do the safety induction before touching hardware.
- Log every crash or hardware fault as an issue with the `hardware` label. Spares are limited.

**Joint limits** (degrees, matching pymycobot): J1 ±168, J2 ±140, J3 ±150, J4 ±150, J5 −155/+160, J6 ±180.

## 4. Development environment

- **Everything builds and runs inside Docker**, through `make`. The host only needs Ubuntu, Docker, make and git. That's why it doesn't matter who's on which machine.
  - Felipe: Ubuntu 24.04 arm64 VM (UTM) on an M4 MacBook Air. The VM network must be in **Bridged** mode.
  - Everyone else: native Ubuntu laptops (amd64).
  - The image builds for both architectures.
- Dev image: `mimesis-dev:jazzy`, built from `ros:jazzy-ros-base`. It includes ros2_control, ros2_controllers, MoveIt 2, xacro, RViz, clang-format 18.1.8 (pinned via pip) and pymycobot. The container user matches the host uid/gid, so files the container writes stay yours.
- **Daily commands:** `make image | doctor | shell | build | test | format | format-check | tidy | clean`.
- **Arm commands:** `make nano-build` and `make nano-shell` rsync your repo to `~/mimesis/<you>` on the Jetson and run it in the `mimesis-dev:nano` container. That container is the arm lock: if `mimesis-arm` is already in use, someone else has the arm.
- `ROS_DOMAIN_ID=17` on every machine. It must differ from the other arm team's ID; agree on this with them.
- Docs: `README.md` and `docs/build.md`. Run `make doctor` first when something breaks.
- Network: VM ↔ Jetson ROS 2 messaging was tested and works. Expect ~300 ms SSH latency over Wi-Fi.

## 5. Current status (12 Oct 2026)

**Merged to `dev`:**
- Docker/Make tooling, `.clang-format` and `.clang-tidy`, `.gitignore`.
- Issue and PR templates, `release.yml`.
<<<<<<< HEAD
- Python CI (ruff + black on Python 3.8). #28 replaces it with `make format-check` + `make test` in the team image (black's double quotes conflict with ROS's flake8).
=======
- Python CI (ruff + black on Python 3.8). This will be replaced, see §7.
>>>>>>> dev
- `docs/build.md` and the Jetson Makefile targets.

**On branches, not merged:**
- `feat-mimesis-description`: robot description package (xacro URDF, metric meshes, collision hulls, inertials, ros2_control xacro with mock hardware, RViz launch). Its kinematics match Elephant's URDF. Joint directions, FK on the real arm, masses, velocity limits and the flange position are **not verified on hardware**.
- `7-add-specs-and-seame-readme`: Jetson specs, pinouts, and a copy of the SEA:ME docs.

**How we actually work right now:**
- We've **moved the real arm since day one** with pymycobot scripts (e.g. `~/Desktop/keyboard.py` on the Jetson), not through ROS.
- **Our simulation isn't working yet.** We've had problems getting it running.
- There is **no e-stop** yet.
- "Sim first" and "e-stop wins over our code" are targets we don't follow yet. Getting sim working and building the e-stop are top priorities for M1.

**Not started:** the hardware driver, MoveIt config, a working Gazebo setup, the e-stop, and the architecture doc. The team chose to first set up Claude and a well-defined plan and structure.

## 6. Conventions

<<<<<<< HEAD
**The full rules are in `docs/CONTRIBUTING.md`** (also uploaded to the Project). It covers the Claude usage rules, the workflow, the PR and review process, and the C++ and Python standards. It's final. Summary:
=======
**The full rules are in `docs/CONTRIBUTING.md`** (also uploaded to the Project). It covers the Claude usage rules, the workflow, the PR and review process, and the C++ and Python standards. Items marked [DISCUSS] there are still open. Summary:
>>>>>>> dev

- **Flow:** issue (template) → branch created from the issue (`<issue#>-<slug>`, base `dev`) → PR to `dev` → **2 approvals** (the pair partner plus one person from another pair) and green CI → **squash merge** by the author.
- **Keeping a branch current:** merge `dev` into it; no rebase or force-push.
- **Sprint end:** `dev` → `main` with a merge commit, tagged `v0.<sprint>.0`, with a GitHub Release.
- **Commits and PR titles:** Conventional Commits, `type(scope): imperative summary`.
- **Docs:**
  - ADRs in `docs/decisions/NNNN-title.md`.
  - `docs/M1_plan.md` (draft).
  - `docs/sprints/sprint_NN.md`.
  - `docs/standups/YYYY-MM-DD.md`.
  - The Claude setup files in `docs/claude/`.
  - `CLAUDE.md` at the repo root for Claude Code.
- **M1 organisation (draft):** three pairs, one per block, with one owner per story inside each pair:
  - A, motion and safety (C++ driver, ros2_control, lifecycle, e-stop).
  - B, planning and simulation (URDF, MoveIt, Gazebo, mission node).
  - C, quality and measurement (CI, HIL, coverage, measurements, docs).

  A Scrum Master rotates each sprint.
- **Code:**
<<<<<<< HEAD
  - **Style = ROS 2 Jazzy lint (`ament_lint_common`)**, the full set in every package: uncrustify, cpplint, cppcheck, flake8, pep257, copyright (MIT header on every file), lint_cmake, xmllint.
  - C++17. `make format` = clang-format (our `.clang-format`) then `ament_uncrustify --reformat` (final word). `make format-check` = `scripts/lint.sh`. Plus clang-tidy (`make tidy`, no new warnings per PR) and GoogleTest.
  - Python 3.12 checked by ament_flake8 (99 cols, single quotes, Google import order) and ament_pep257; pytest. No black or ruff.
=======
  - C++17 in ROS 2 style (clang-format 18.1.8, clang-tidy), GoogleTest.
  - Python 3.12 with ruff and pytest.
>>>>>>> dev
  - SI units on ROS interfaces (REP 103); unit conversion only in the driver.
  - Package names `positronic_<role>`.
  - Every node that can move the arm is a lifecycle node.

## 7. Technical decisions so far (each needs an ADR in the repo)

| # | Decision | Why | Revisit when |
|---|---|---|---|
| 1 | **ROS 2 Jazzy** everywhere (laptops, CI, Jetson container). Not Galactic (EOL 2022), not Humble, not Lyrical (yet). | Jazzy is LTS until May 2029, past our program's end. MoveIt 2, ros2_control, gz_ros2_control and almost every tutorial and answer target it. Lyrical (May 2026, LTS to 2031) has Ubuntu 26.04 as its main platform and a younger ecosystem, which is riskier for beginners. Every machine must run the same distro, because DDS compatibility across distros isn't guaranteed. | A package we need is Lyrical-only, or the facilitators require a change. Switching is just the `ROS_DISTRO` build arg. |
| 2 | **Docker for everything**, including on the Jetson. Laptops run the full dev image; the Jetson gets a slim runtime image (planned). | The Jetson's OS (20.04, JetPack 4.6) can't run Jazzy natively. Containers make the host OS irrelevant and the environment identical for all 6 of us and CI. | — |
| 3 | **Gazebo Harmonic** (Jazzy's official pairing) with `gz_ros2_control`, running on laptops or lab RTX workstations, never on the Jetson. | It's the officially supported ROS/Gazebo pair. The same URDF and controllers drive sim and real, which satisfies the "same interface" rule. Most MoveIt sim tutorials use it. Isaac Sim is possible later for M2 photorealism. | M2 needs photorealistic or contact-rich simulation |
<<<<<<< HEAD
| 4 | **ROS 2 lint is the style standard** (C++ and Python), and **CI runs inside the dev image** (`make build`, `make test`, `make format-check`, tidy, coverage), replacing the Python 3.8 / ubuntu-22.04 ruff+black job. | ROS tools enforce ROS style in `colcon test` anyway; CI must test the same environment the code runs in. | — |
=======
| 4 | **Python 3.12 + ruff**, and **CI runs inside the dev image** (build, test, format-check, tidy, coverage), replacing the Python 3.8 / ubuntu-22.04 job. | CI must test the same environment the code runs in. A Python 3.8 check against 3.12 code gives false results. | — |
>>>>>>> dev
| 5 | Workflow and standards in `docs/CONTRIBUTING.md`. | See that file | Retro feedback, or a PR changing it |

**Open technical questions** (add them to the plan):
- **E-stop design (M1 deliverable, top priority).** It has to work even when our software is dead, so a button read only by a ROS node isn't enough on its own. A typical design has two layers:
  - **Hardware:** a latching mushroom button that cuts servo power. This always wins, but the arm goes limp and can drop under gravity.
  - **Software:** the same button (second contact) or a GPIO/ROS signal makes the lifecycle driver stop the motion and deactivate.

  Decide the stop behaviour (cut power immediately, or stop the motion then cut power), the wiring, and what the facilitators accept. Ask the facilitators how this fits the other arm team and the shared lab before wiring anything.
- **Why simulation fails:** find the root cause and record it in an issue. We need a working Gazebo setup for M1's sim-to-real report.
- **GPU on the Jetson for M2:** CUDA 10.2 only works in L4T r32 (Ubuntu 18.04) containers, not in our Jazzy one. Options: run inference on a host GPU, use a separate L4T inference container bridged to ROS, or skip Jetson inference. Decide by measuring in M2.
- **Slim Jetson image:** today the Jetson runs the full dev image, MoveIt included. It needs a lean runtime target (ROS base, ros2_control, our driver).
- **Self-hosted HIL runner and a public repo:** never run hardware jobs on PRs from forks. Require approval for outside contributors' workflows, and trigger HIL only on pushes to `dev`/`main` or manually. *Why: in a public repo, anyone's fork could otherwise run code on our arm.*
- Rename `mimesis-*` image and container names, and the `NANO_DIR` path, to `positronic`.
- The ROS domain ID still needs to be agreed with the other arm team.
- Camera mount: hand-eye calibration procedure, and a 3D-printed mount (the lab has a Bambu P2S).

## 8. Glossary for newcomers

- **ROS 2:** middleware where programs (*nodes*) exchange messages over *topics*, *services* and *actions*.
- **DDS:** the network layer under ROS 2. It finds other nodes automatically on the same `ROS_DOMAIN_ID`.
- **URDF/xacro:** the XML description of the robot's links, joints, meshes and limits.
- **ros2_control:** the standard framework between controllers and hardware. We write a *hardware interface* plugin that reads and writes the servos; controllers (e.g. the joint trajectory controller) send it commands.
- **MoveIt 2:** a motion planner that turns "put the gripper here" (a Cartesian pose) into a collision-free joint trajectory.
- **FK / IK:** forward kinematics (joint angles → gripper pose) and inverse kinematics (pose → joint angles).
- **Lifecycle node:** a node with explicit states (unconfigured / inactive / active / finalized), so we control when it's allowed to move the arm. This is the basis of the e-stop.
- **TF2:** keeps track of every coordinate frame over time.
- **Visual servoing:** using the camera to correct the motion as it happens.
- **DMP / ProMP:** ways to encode a demonstrated trajectory so it can be replayed with a different start and goal.
- **HIL:** hardware-in-the-loop tests, meaning CI that runs on the real arm.
