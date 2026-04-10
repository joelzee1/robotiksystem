SHELL := /bin/bash

DOCKER_UID := $(shell id -u)
DOCKER_GID := $(shell id -g)
VIDEO_GID  := $(shell getent group video  | cut -d: -f3)
RENDER_GID := $(shell getent group render | cut -d: -f3)

export DOCKER_UID DOCKER_GID VIDEO_GID RENDER_GID

.PHONY: help build up down shell clean

help:
	@echo "labbar — ROS 2 Jazzy dev container"
	@echo ""
	@echo "  make build   # Build Docker image"
	@echo "  make up      # Start container + drop into shell"
	@echo "  make down    # Stop container"
	@echo "  make shell   # Shell into running container"
	@echo "  make clean   # Remove container, image, build/install/log"

build:
	@cd docker && DOCKER_BUILDKIT=1 docker compose build --build-arg BUILDKIT_INLINE_CACHE=1

up:
	@cd docker && docker compose up -d
	@cd docker && docker compose exec labbar bash

down:
	@cd docker && docker compose down --remove-orphans

shell:
	@cd docker && docker compose exec labbar bash

clean:
	@cd docker && docker compose down --volumes --rmi all
	@sudo rm -rf build install log
