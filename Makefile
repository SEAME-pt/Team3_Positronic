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