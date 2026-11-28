-include .env

# GPU for Gazebo and RViz in the dev container: dri, nvidia or sw. Empty means no GPU override.
# Set it in .env (see .env.example); each value is a file in docker/gpu/.
# One-off override: "make GPU=sw shell". Not "GPU=sw make shell". A variable in .env beats the environment.
GPU ?=
# Make keeps trailing spaces in values ("GPU=sw " in .env would be "sw "). override: also
# strips a value given on the command line, which a plain assignment can't change.
override GPU := $(strip $(GPU))

COMPOSE ?= docker compose
# Inside a container (the dev container's terminal) run commands directly;
# on the host, run them in a throwaway dev container.
ifneq ($(wildcard /.dockerenv),)
  RUN :=
else
  ifneq ($(GPU),)
    ifeq ($(wildcard docker/gpu/$(GPU).yaml),)
      $(error GPU=$(GPU) is not valid: use one of $(basename $(notdir $(wildcard docker/gpu/*.yaml))))
    endif
    COMPOSE += -f compose.yaml -f docker/gpu/$(GPU).yaml
    ifeq ($(GPU),dri)
      # The container's user must be in the host's video and render groups to open /dev/dri.
      # Numbers, not names: the image has no "render" group, and the number differs per machine.
      export VIDEO_GID  := $(shell stat -c %g /dev/dri/card* 2>/dev/null | head -n 1)
      export RENDER_GID := $(shell stat -c %g /dev/dri/renderD* 2>/dev/null | head -n 1)
      ifeq ($(RENDER_GID),)
        $(error GPU=dri but this machine has no /dev/dri/renderD*; try GPU=sw)
      endif
    endif
  endif
  RUN := $(COMPOSE) run --rm dev
endif

USER_UID := $(shell id -u)
USER_GID := $(shell id -g)
ifeq ($(USER_UID),0)
  USER_UID := 1000
  USER_GID := 1000
endif
export USER_UID USER_GID

# Let's the container's user open windows (like RViz) on your desktop.
ALLOW_X = @if [ -n "$$DISPLAY" ] && command -v xhost > /dev/null; then xhost +SI:localuser:$$(id -un) > /dev/null; fi

# The packages that talk to the arm or command it (hardware, protocol, mission, safety). A warning
# here fails the build (CI runs this). Test files get only .clang-tidy's bugprone checks: the
# command-line checks are added after .clang-tidy, so they switch every other family of it off
# (ADR 0007). A new family in .clang-tidy goes here too (scripts/test_tidy_changed.py checks it).
TIDY_STRICT_FILES := ^/ws/src/(positronic_hardware|positronic_protocol|positronic_mission|positronic_safety)/
TIDY_TEST_FILES := ^/ws/src/[^/]+/test/
TIDY_TEST_CHECKS := -cert-*,-clang-analyzer-*,-concurrency-*,-cppcoreguidelines-*,-misc-*,-modernize-*,-performance-*,-portability-*,-readability-*
TIDY_OPTIONS = --strict='$(TIDY_STRICT_FILES)' --test-files='$(TIDY_TEST_FILES)' --test-checks='$(TIDY_TEST_CHECKS)' $(TIDY_VERBOSE)
# Only the files with warnings, then the summary; VERBOSE adds a line for every file (like make test).
TIDY_VERBOSE = $(if $(VERBOSE),--verbose)
# What make tidy-changed compares with. CI sets it to HEAD^1 (the PR's merge commit's base).
TIDY_BASE ?= origin/dev

.DEFAULT_GOAL := help
.PHONY: help image shell build test coverage format format-check tidy tidy-strict tidy-changed doctor clean fclean

# Full output for make test and make format-check: "make test VERBOSE" or "make test VERBOSE=1".
# Without =1, make reads VERBOSE as one more goal: it turns verbose on, and its rule does nothing.
ifneq ($(filter VERBOSE,$(MAKECMDGOALS)),)
  override VERBOSE := 1
endif
.PHONY: VERBOSE
VERBOSE:
	@:

help: ## Show this help
	@grep -h -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-14s %s\n", $$1, $$2}'

image: ## Build the dev image
	$(COMPOSE) build

shell: ## Shell in the dev container (GUI apps like rviz2 work from here)
	$(ALLOW_X)
	$(RUN) bash

build: ## colcon build
	$(RUN) bash -c 'colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

# colcon's per package stderr is hidden: scripts/summarize_tests.py shows each failure once, at the end.
# VERBOSE shows everything colcon prints, plus colcon test-result --verbose, before the summary.
# Packages run in parallel (colcon's default) and so do the tests inside each one (ctest -j).
# ROS_DOMAIN_ID is unset so the tests that talk over ROS (ament_add_ros_isolated_gtest) each get
# a free domain: with it set (17, the arm's) they all share it and see each other's nodes (#144).
# LOCALHOST: test nodes never discover anything on the network, like the arm's, whatever domain.
TEST_JOBS ?= 4
TEST_HANDLERS = $(if $(VERBOSE),,--event-handlers console_stderr-)
test: ## colcon test, then a summary per package and each failure (VERBOSE: full output)
	$(RUN) bash -c 'unset ROS_DOMAIN_ID; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; \
	  colcon test $(TEST_HANDLERS) --ctest-args -j$(TEST_JOBS); rc=$$?; \
	  $(if $(VERBOSE),colcon test-result --verbose;) python3 scripts/summarize_tests.py --colcon-rc $$rc'

# Own build in build_cov/ and install_cov/ (scripts/coverage.sh), then the gate: each package at 80 %.
# CI runs it on every PR and blocks the merge below 80 % (#46).
coverage: ## Line coverage of src/ (C++ and Python), fails if a package is below 80 %
	$(RUN) bash -c 'bash scripts/coverage.sh && python3 scripts/coverage_gate.py'

format: ## Format C/C++ in src/: clang-format, then ROS uncrustify (final word)
	$(RUN) bash scripts/lint.sh --fix

format-check: ## ROS 2 lint on src/, one line per tool (VERBOSE: each tool's full output)
	$(RUN) env VERBOSE=$(VERBOSE) bash scripts/lint.sh

# Every file in src/, with the same test-file rule as CI (ADR 0007), but warnings never fail
# (ADR 0004): --soft. It fails only when clang-tidy can't check a file (it doesn't compile).
tidy: ## clang-tidy on src/, warnings only, bugprone only on test files (VERBOSE: every file)
	$(RUN) python3 scripts/tidy_changed.py --all --soft --strict='^/ws/src/' \
	  --test-files='$(TIDY_TEST_FILES)' --test-checks='$(TIDY_TEST_CHECKS)' $(TIDY_VERBOSE)

# scripts/tidy_changed.py: the selection, then clang-tidy in parallel, one line per file.
tidy-strict: ## clang-tidy on the strict packages (TIDY_STRICT_FILES), any warning is an error (needs make build first)
	$(RUN) python3 scripts/tidy_changed.py --all $(TIDY_OPTIONS)

tidy-changed: ## tidy-strict on only the files your branch can affect, vs TIDY_BASE=origin/dev (needs make build first)
	$(RUN) python3 scripts/tidy_changed.py --base '$(TIDY_BASE)' $(TIDY_OPTIONS)

doctor: ## Check the environment
	$(ALLOW_X)
	$(RUN) bash scripts/doctor.sh

clean: ## Delete build/, install/, log/ and the coverage outputs
	rm -rf build install log build_cov install_cov coverage

fclean: clean ## Full clean: remove this project's containers, images and volumes
	@docker compose down --rmi local -v --remove-orphans

# ---------- Jetson (one shared machine, user er) ----------
NANO          ?= nano
NANO_DIR      ?= positronic/$(USER)
# Also written in docker/jetson/positronic-driver.service, change both together.
NANO_IMAGE    := positronic-runtime:nano
# The packages the Jetson builds, plus whatever they depend on (colcon --packages-up-to).
# MoveIt, Gazebo and RViz run on laptops only, so a package that needs them to build stays out
# of this list. CI builds exactly this list in the runtime image (make nano-packages).
NANO_PACKAGES := positronic_hardware positronic_bringup positronic_hil positronic_safety
ROS_DOMAIN_ID ?= 17
export ROS_DOMAIN_ID

NANO_RUN = docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) -v \$$HOME/$(NANO_DIR):/ws

.PHONY: nano-image nano-sync nano-build nano-shell nano-camera nano-service nano-packages

nano-image: ## Build the slim Jetson image (runtime target) ON the Jetson, once per Dockerfile change
	rsync -a --delete docker/ $(NANO):positronic-docker/
	ssh $(NANO) 'docker build --target runtime -t $(NANO_IMAGE) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) positronic-docker'

# /bags is kept on the Jetson: --delete would erase the sessions record_session.py wrote there (#128).
nano-sync: ## Copy this repo to ~/positronic/<you> on the Jetson
	ssh $(NANO) mkdir -p $(NANO_DIR)
	rsync -a --delete --exclude .git --exclude build --exclude install --exclude log --exclude /bags ./ $(NANO):$(NANO_DIR)/

nano-build: nano-sync ## Sync, then colcon build NANO_PACKAGES on the Jetson
	ssh $(NANO) "$(NANO_RUN) $(NANO_IMAGE) bash -c 'colcon build --symlink-install --parallel-workers 2 --packages-up-to $(NANO_PACKAGES)'"

nano-packages: ## Print the packages the Jetson builds (CI reads this)
	@echo $(NANO_PACKAGES)

# The wrist camera device on the Jetson. nano-shell maps it when it is plugged in, so
# driver.launch.py camera:=true can use it; without it the shell starts as before. --group-add:
# the gid of the device on the Jetson, whatever the image calls it.
NANO_CAMERA ?= /dev/video0
NANO_CAMERA_ARGS = \$$([ -e $(NANO_CAMERA) ] && echo --device $(NANO_CAMERA) --group-add \$$(stat -c %g $(NANO_CAMERA)))

nano-shell: nano-sync ## Sync, then shell on the Jetson with the arm's serial port (and the camera, if plugged)
	ssh -t $(NANO) "$(NANO_RUN) -it --name positronic-arm --device /dev/ttyTHS1 $(NANO_CAMERA_ARGS) $(NANO_IMAGE) bash"

# The camera alone (camera.launch.py), in its own container next to nano-shell's driver.
nano-camera: nano-sync ## Sync, then shell on the Jetson with the wrist camera only (NANO_CAMERA)
	ssh -t $(NANO) "$(NANO_RUN) -it --name positronic-camera --device $(NANO_CAMERA) \
	  --group-add \$$(stat -c %g $(NANO_CAMERA)) $(NANO_IMAGE) bash"

NANO_SERVICE := positronic-driver

nano-service: ## Install or update the boot service on the Jetson (asks for the Jetson sudo password)
	scp docker/jetson/$(NANO_SERVICE).service $(NANO):/tmp/
	ssh -t $(NANO) 'sudo install -m 644 /tmp/$(NANO_SERVICE).service /etc/systemd/system/ \
	  && sudo systemctl daemon-reload \
	  && sudo systemctl enable $(NANO_SERVICE) \
	  && sudo systemctl restart $(NANO_SERVICE) \
	  && systemctl --no-pager status $(NANO_SERVICE)'
