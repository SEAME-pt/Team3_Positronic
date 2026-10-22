COMPOSE ?= docker compose
# Inside a container (the dev container's terminal) run commands directly;
# on the host, run them in a throwaway dev container.
ifneq ($(wildcard /.dockerenv),)
  RUN :=
else
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

.DEFAULT_GOAL := help
.PHONY: help image shell build test format format-check tidy doctor clean fclean

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-14s %s\n", $$1, $$2}'

image: ## Build the dev image
	$(COMPOSE) build

shell: ## Shell in the dev container (GUI apps like rviz2 work from here)
	$(ALLOW_X)
	$(RUN) bash

build: ## colcon build
	$(RUN) bash -c 'colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

test: ## colcon test and show results
	$(RUN) bash -c 'colcon test && colcon test-result --verbose'

format: ## Format C/C++ in src/: clang-format, then ROS uncrustify (final word)
	$(RUN) bash scripts/lint.sh --fix

format-check: ## ROS 2 lint on src/: uncrustify, cpplint, cppcheck, flake8, pep257, copyright
	$(RUN) bash scripts/lint.sh

tidy: ## clang-tidy on src/ (needs make build first)
	$(RUN) bash -c 'run-clang-tidy -quiet -p build "^/ws/src/"'

doctor: ## Check the environment
	$(ALLOW_X)
	$(RUN) bash scripts/doctor.sh

clean: ## Delete build/, install/ and log/
	rm -rf build install log

fclean: clean ## Full clean: remove this project's containers, images and volumes
	@docker compose down --rmi local -v --remove-orphans

# ---------- Jetson (one shared machine, user er) ----------
NANO          ?= nano
NANO_DIR      ?= positronic/$(USER)
NANO_IMAGE    := positronic-dev:nano
ROS_DOMAIN_ID ?= 17
export ROS_DOMAIN_ID

NANO_RUN = docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) -v \$$HOME/$(NANO_DIR):/ws

.PHONY: nano-image nano-sync nano-build nano-shell nano-service

nano-image: ## Build the Jetson image ON the Jetson (once per Dockerfile change)
	rsync -a --delete docker/ $(NANO):positronic-docker/
	ssh $(NANO) 'docker build -t $(NANO_IMAGE) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) positronic-docker'

nano-sync: ## Copy this repo to ~/positronic/<you> on the Jetson
	ssh $(NANO) mkdir -p $(NANO_DIR)
	rsync -a --delete --exclude .git --exclude build --exclude install --exclude log ./ $(NANO):$(NANO_DIR)/

nano-build: nano-sync ## Sync, then colcon build on the Jetson
	ssh $(NANO) "$(NANO_RUN) $(NANO_IMAGE) bash -c 'colcon build --symlink-install --parallel-workers 2'"

nano-shell: nano-sync ## Sync, then shell on the Jetson with the arm's serial port
	ssh -t $(NANO) "$(NANO_RUN) -it --name positronic-arm --device /dev/ttyTHS1 $(NANO_IMAGE) bash"

NANO_SERVICE := positronic-driver

nano-service: ## Install or update the boot service on the Jetson (asks for the Jetson sudo password)
	scp docker/jetson/$(NANO_SERVICE).service $(NANO):/tmp/
	ssh -t $(NANO) 'sudo install -m 644 /tmp/$(NANO_SERVICE).service /etc/systemd/system/ \
	  && sudo systemctl daemon-reload \
	  && sudo systemctl enable $(NANO_SERVICE) \
	  && sudo systemctl restart $(NANO_SERVICE) \
	  && systemctl --no-pager status $(NANO_SERVICE)'
