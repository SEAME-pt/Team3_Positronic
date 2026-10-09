COMPOSE ?= docker compose
RUN     := $(COMPOSE) run --rm dev

USER_UID := $(shell id -u)
USER_GID := $(shell id -g)
ifeq ($(USER_UID),0)
  USER_UID := 1000
  USER_GID := 1000
endif
export USER_UID USER_GID

CPP_FILES := find src -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) -print0

.DEFAULT_GOAL := help
.PHONY: help image shell build test format format-check tidy doctor clean

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-14s %s\n", $$1, $$2}'

image: ## Build the dev image
	$(COMPOSE) build

shell: ## Shell in the dev container (GUI apps like rviz2 work from here)
	@if [ -n "$$DISPLAY" ] && command -v xhost > /dev/null; then xhost +SI:localuser:$$(id -un) > /dev/null; fi
	$(RUN) bash

build: ## colcon build
	$(RUN) bash -c 'colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

test: ## colcon test and show results
	$(RUN) bash -c 'colcon test && colcon test-result --verbose'

format: ## Format C/C++ files in src/
	$(RUN) bash -c "$(CPP_FILES) | xargs -0 -r clang-format -i"

format-check: ## Fail if a C/C++ file in src/ is not formatted
	$(RUN) bash -c "$(CPP_FILES) | xargs -0 -r clang-format --dry-run -Werror"

tidy: ## clang-tidy on src/ (needs make build first)
	$(RUN) bash -c 'for d in build/*/; do \
	  if [ -f "$$d/compile_commands.json" ]; then run-clang-tidy -quiet -p "$$d" "^/ws/src/"; fi; \
	done'

doctor: ## Check the environment
	$(RUN) bash scripts/doctor.sh

clean: ## Delete build/, install/ and log/
	rm -rf build install log

# ---------- Jetson (one shared machine, user er) ----------
NANO          ?= nano
NANO_DIR      ?= mimesis/$(USER)
NANO_IMAGE    := mimesis-dev:nano
ROS_DOMAIN_ID ?= 17
export ROS_DOMAIN_ID

NANO_RUN = docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) -v \$$HOME/$(NANO_DIR):/ws

.PHONY: nano-image nano-sync nano-build nano-shell

nano-image: ## Build the Jetson image ON the Jetson (once per Dockerfile change)
	rsync -a --delete docker/ $(NANO):mimesis-docker/
	ssh $(NANO) 'docker build -t $(NANO_IMAGE) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) mimesis-docker'

nano-sync: ## Copy this repo to ~/mimesis/<you> on the Jetson
	ssh $(NANO) mkdir -p $(NANO_DIR)
	rsync -a --delete --exclude .git --exclude build --exclude install --exclude log ./ $(NANO):$(NANO_DIR)/

nano-build: nano-sync ## Sync, then colcon build on the Jetson
	ssh $(NANO) "$(NANO_RUN) $(NANO_IMAGE) bash -c 'colcon build --symlink-install --parallel-workers 2'"

nano-shell: nano-sync ## Sync, then shell on the Jetson with the arm's serial port
	ssh -t $(NANO) "$(NANO_RUN) -it --name mimesis-arm --device /dev/ttyTHS1 $(NANO_IMAGE) bash"
