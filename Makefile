# Simplified Makefile for Interceptor Drone System

PROJECT_DIR := $(shell pwd)
LOG_DIR := $(PROJECT_DIR)/logs
SERVICE_NAME := interceptor
# Persistent data (datasets, weights, batch results), mounted at /workspace/data.
DATA_DIR := $(HOME)/skyinterceptor-data

# The host: wsl (WSL2 on Windows) or native (Linux), read from the kernel's
# version string. It picks the compose overlay (docker-compose.<platform>.yml).
# Override it on the command line if the guess is wrong: make up PLATFORM=native
PLATFORM ?= $(shell grep -qi microsoft /proc/version 2>/dev/null && echo wsl || echo native)
# The compose command: the docker compose plugin when it is installed, else the
# old standalone docker-compose.
COMPOSE_CMD ?= $(shell docker compose version >/dev/null 2>&1 && echo "docker compose" || echo docker-compose)
COMPOSE := $(COMPOSE_CMD) -f docker-compose.yml -f docker-compose.$(PLATFORM).yml

.PHONY: help build up down shell test check clean logs status build-ws sim full teleop platform doctor

help:
	@echo "Interceptor Drone - Commands"
	@echo "============================"
	@echo "  make build          - Build the Docker image"
	@echo "  make up             - Start container"
	@echo "  make down           - Stop container"
	@echo "  make shell          - Enter container shell"
	@echo "  make build-ws       - Build ROS2 workspace"
	@echo "  make sim            - Launch simulation"
	@echo "  make teleop         - Fly the drone with the keyboard (run next to make sim)"
	@echo "  make full           - Launch full system"
	@echo "  make test           - Run project tests"
	@echo "  make check          - Run the same checks as CI (static + build + test)"
	@echo "  make clean          - Remove build artifacts"
	@echo "  make platform       - Show the detected host (wsl/native) and the compose command"
	@echo "  make doctor         - Check the host and the container: GPU, renderer, torch, mounts"
	@echo ""
	@echo "Host: $(PLATFORM) (override with PLATFORM=wsl|native), compose: $(COMPOSE_CMD)"

build:
	@mkdir -p $(LOG_DIR)
	$(COMPOSE) build $(SERVICE_NAME) 2>&1 | tee $(LOG_DIR)/docker-build.log

up:
	@mkdir -p $(DATA_DIR)
	@xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d $(SERVICE_NAME)

down:
	$(COMPOSE) down
	@xhost -local:docker 2>/dev/null || true

shell:
	$(COMPOSE) exec $(SERVICE_NAME) bash

build-ws:
	$(COMPOSE) exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
	"

test:
	$(COMPOSE) exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		colcon test --packages-select interceptor_drone interceptor_interfaces && \
		colcon test-result --verbose \
	"

# The local mirror of CI: static checks, a build with -Werror and the full
# test suite. It builds in a scratch workspace inside the container, so it
# tests this checkout even when the container's src is mounted from
# somewhere else, and it leaves the shared build volumes alone.
check:
	./scripts/check.sh

sim:
	$(COMPOSE) exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		ros2 launch interceptor_drone simulation.launch.py \
	"

teleop:
	$(COMPOSE) exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		ros2 run interceptor_drone drone_teleop_keyboard.py \
	"

full:
	$(COMPOSE) exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		ros2 launch interceptor_drone interceptor_full.launch.py \
	"

clean:
	$(COMPOSE) exec $(SERVICE_NAME) rm -rf build install log
	@echo "Cleaned build artifacts."

status:
	$(COMPOSE) ps

platform:
	@echo "platform: $(PLATFORM)"
	@echo "compose:  $(COMPOSE)"
	@echo "data:     $(DATA_DIR) -> /workspace/data"

# Checks the host and the running container, one line per check, so a broken
# setup says where it is broken. Run it after `make up`.
doctor:
	@echo "== host ($(PLATFORM))"
	@command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || echo "FAIL: nvidia-smi not found on the host"
	@$(COMPOSE_CMD) version >/dev/null 2>&1 && echo "compose: $(COMPOSE_CMD) ok" || echo "FAIL: no docker compose"
	@test -d $(DATA_DIR) && echo "data dir: $(DATA_DIR) ok" || echo "FAIL: $(DATA_DIR) missing (make up creates it)"
	@echo "== container"
	@docker exec $(SERVICE_NAME) true 2>/dev/null || { echo "FAIL: container $(SERVICE_NAME) not running (make up)"; exit 1; }
	@docker exec $(SERVICE_NAME) nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | sed 's/^/GPU in the container: /' || echo "FAIL: no GPU in the container (NVIDIA container toolkit?)"
	@docker exec $(SERVICE_NAME) bash -c "glxinfo -B 2>/dev/null | grep -E 'OpenGL renderer' || echo 'FAIL: glxinfo found no renderer (is DISPLAY set and xhost +local:docker done?)'"
	@docker exec $(SERVICE_NAME) python3 -c "import torch; print('torch CUDA:', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '')" 2>/dev/null || echo "FAIL: torch not importable"
	@docker exec $(SERVICE_NAME) bash -c "findmnt -n -o SOURCE -T /workspace/interceptor_ws/src | sed 's/^/src mounted from: /'; findmnt -n -o SOURCE -T /workspace/data | sed 's/^/data mounted from: /'"

