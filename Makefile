# Simplified Makefile for Interceptor Drone System

PROJECT_DIR := $(shell pwd)
LOG_DIR := $(PROJECT_DIR)/logs
SERVICE_NAME := interceptor

.PHONY: help build up down shell test clean logs status build-ws sim full

help:
	@echo "Interceptor Drone - Commands"
	@echo "============================"
	@echo "  make build          - Build the Docker image"
	@echo "  make up             - Start container"
	@echo "  make down           - Stop container"
	@echo "  make shell          - Enter container shell"
	@echo "  make build-ws       - Build ROS2 workspace"
	@echo "  make sim            - Launch simulation"
	@echo "  make full           - Launch full system"
	@echo "  make test           - Run project tests"
	@echo "  make clean          - Remove build artifacts"

build:
	@mkdir -p $(LOG_DIR)
	docker-compose build $(SERVICE_NAME) 2>&1 | tee $(LOG_DIR)/docker-build.log

up:
	@xhost +local:docker 2>/dev/null || true
	docker-compose up -d $(SERVICE_NAME)

down:
	docker-compose down
	@xhost -local:docker 2>/dev/null || true

shell:
	docker-compose exec $(SERVICE_NAME) bash

build-ws:
	docker-compose exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
	"

test:
	docker-compose exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		colcon test --packages-select interceptor_drone interceptor_interfaces && \
		colcon test-result --verbose \
	"

sim:
	docker-compose exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		ros2 launch interceptor_drone simulation.launch.py \
	"

full:
	docker-compose exec $(SERVICE_NAME) bash -c " \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		ros2 launch interceptor_drone interceptor_full.launch.py \
	"

clean:
	docker-compose exec $(SERVICE_NAME) rm -rf build install log
	@echo "Cleaned build artifacts."

status:
	docker-compose ps