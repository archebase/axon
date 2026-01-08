# Makefile for Axon
# Universal middleware plugin system for ROS1 and ROS2
# Supports building both ROS1 and ROS2 plugins and unified test programs

.PHONY: all build clean test help build-ros2 build-ros1 build-examples test-ros2 test-ros1 format format-check docker-build docker-build-ros2 docker-build-ros2-humble docker-build-ros2-jazzy docker-build-ros2-rolling docker-build-ros1 docker-test docker-clean docker-push docker-images
.DEFAULT_GOAL := help

# Use bash as the shell for all recipes (required for ROS setup.bash scripts)
SHELL := /bin/bash

# Detect number of CPU cores
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Build type
BUILD_TYPE ?= Release

# Colors for output
ifeq ($(shell test -t 1 && echo yes),yes)
  RED := \033[0;31m
  GREEN := \033[0;32m
  YELLOW := \033[0;33m
  BLUE := \033[0;34m
  NC := \033[0m
else
  RED :=
  GREEN :=
  YELLOW :=
  BLUE :=
  NC :=
endif

# Help target
help:
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║          Axon - Universal Middleware Plugin System        ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Build targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make build$(NC)              - Build ROS2 plugin and examples"
	@printf "%s\n" "  $(BLUE)make build-ros2$(NC)         - Build ROS2 plugin only"
	@printf "%s\n" "  $(BLUE)make build-ros1$(NC)         - Build ROS1 plugin only"
	@printf "%s\n" "  $(BLUE)make build-examples$(NC)     - Build unified test program (no ROS deps)"
	@printf "%s\n" "  $(BLUE)make build-all$(NC)          - Build everything (ROS1 + ROS2 + examples)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Test targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make test$(NC)               - Quick test of unified program"
	@printf "%s\n" "  $(BLUE)make test-ros2$(NC)          - Test ROS2 plugin"
	@printf "%s\n" "  $(BLUE)make test-ros1$(NC)          - Test ROS1 plugin"
	@echo ""
	@printf "%s\n" "$(YELLOW)Clean targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make clean$(NC)              - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make clean-ros2$(NC)         - Clean ROS2 build artifacts"
	@printf "%s\n" "  $(BLUE)make clean-ros1$(NC)         - Clean ROS1 build artifacts"
	@printf "%s\n" "  $(BLUE)make clean-examples$(NC)     - Clean examples build artifacts"
	@echo ""
	@printf "%s\n" "$(YELLOW)Run targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make run-ros2$(NC)           - Run unified test with ROS2 plugin"
	@printf "%s\n" "  $(BLUE)make run-ros1$(NC)           - Run unified test with ROS1 plugin"
	@echo ""
	@printf "%s\n" "$(YELLOW)Code formatting:$(NC)"
	@printf "%s\n" "  $(BLUE)make format$(NC)            - Format all C++ code using clang-format"
	@printf "%s\n" "  $(BLUE)make format-check$(NC)      - Check if code is properly formatted"
	@echo ""
	@printf "%s\n" "$(YELLOW)Docker targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make docker-build$(NC)      - Build all Docker images"
	@printf "%s\n" "  $(BLUE)make docker-build-ros2$(NC)  - Build all ROS2 Docker images (humble, jazzy, rolling)"
	@printf "%s\n" "  $(BLUE)make docker-build-humble$(NC) - Build ROS2 Humble Docker image"
	@printf "%s\n" "  $(BLUE)make docker-build-jazzy$(NC)  - Build ROS2 Jazzy Docker image"
	@printf "%s\n" "  $(BLUE)make docker-build-rolling$(NC) - Build ROS2 Rolling Docker image"
	@printf "%s\n" "  $(BLUE)make docker-build-ros1$(NC)  - Build ROS1 Noetic Docker image"
	@printf "%s\n" "  $(BLUE)make docker-test$(NC)       - Run tests in Docker containers"
	@printf "%s\n" "  $(BLUE)make docker-clean$(NC)      - Remove Docker images and containers"
	@printf "%s\n" "  $(BLUE)make docker-push$(NC)       - Push Docker images to registry"
	@printf "%s\n" "  $(BLUE)make docker-images$(NC)     - Show available Docker images"
	@echo ""
	@printf "%s\n" "$(YELLOW)Quick start:$(NC)"
	@printf "%s\n" "  1. $(BLUE)make build$(NC)           # Build ROS2 plugin and examples"
	@printf "%s\n" "  2. $(BLUE)make run-ros2$(NC)        # Run with ROS2"
	@echo ""

# Main build target (build ROS2 and examples by default)
build: build-ros2 build-examples
	@printf "%s\n" "$(GREEN)✓ Build complete!$(NC)"
	@printf "%s\n" "$(BLUE)To test: make run-ros2$(NC)"

# Build ROS2 plugin
build-ros2:
	@printf "%s\n" "$(YELLOW)Building ROS2 plugin...$(NC)"
	@if [ -f /opt/ros/humble/setup.bash ]; then \
		. /opt/ros/humble/setup.bash; \
		cd middlewares/ros2 && colcon build --packages-select ros2_plugin --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
	elif [ -f /opt/ros/iron/setup.bash ]; then \
		. /opt/ros/iron/setup.bash; \
		cd middlewares/ros2 && colcon build --packages-select ros2_plugin --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
	else \
		echo "$(RED)Error: ROS2 not found. Please install ROS2 (Humble/Iron/Jazzy)$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(GREEN)✓ ROS2 plugin built successfully$(NC)"

# Build ROS1 plugin
build-ros1:
	@printf "%s\n" "$(YELLOW)Building ROS1 plugin...$(NC)"
	@if [ -f /opt/ros/noetic/setup.bash ]; then \
		. /opt/ros/noetic/setup.bash; \
		cd middlewares/ros1 && catkin build --packages-select ros1_plugin; \
	elif [ -f /opt/ros/melodic/setup.bash ]; then \
		. /opt/ros/melodic/setup.bash; \
		cd middlewares/ros1 && catkin build --packages-select ros1_plugin; \
	else \
		echo "$(RED)Error: ROS1 not found. Please install ROS1 (Noetic/Melodic)$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(GREEN)✓ ROS1 plugin built successfully$(NC)"

# Build unified test program (NO ROS dependencies at compile time!)
build-examples:
	@printf "%s\n" "$(YELLOW)Building unified test program (no ROS deps)...$(NC)"
	@cd examples && ./build.sh
	@printf "%s\n" "$(GREEN)✓ Unified test program built successfully$(NC)"
	@printf "%s\n" "$(BLUE)The unified test program has ZERO compile-time ROS dependencies!$(NC)"

# Build everything
build-all: build-ros2 build-ros1 build-examples
	@printf "%s\n" "$(GREEN)✓ All components built successfully!$(NC)"
	@printf "%s\n" "$(BLUE)Unified test program works with both ROS1 and ROS2 plugins$(NC)"

# Quick test
test: build-examples
	@printf "%s\n" "$(YELLOW)Testing unified test program...$(NC)"
	@cd examples/build && ./test_unified.sh || true

# Test ROS2 plugin
test-ros2: build-ros2 build-examples
	@printf "%s\n" "$(YELLOW)Testing ROS2 plugin...$(NC)"
	@if [ -f middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so ]; then \
		cp middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so examples/build/; \
	elif [ -f middlewares/ros2/build/ros2_plugin/libros2_plugin.so ]; then \
		cp middlewares/ros2/build/ros2_plugin/libros2_plugin.so examples/build/; \
	fi
	@printf "%s\n" "$(GREEN)✓ ROS2 plugin ready for testing$(NC)"
	@printf "%s\n" "$(BLUE)To run manually: cd examples/build && ./run_ros2.sh$(NC)"

# Test ROS1 plugin
test-ros1: build-ros1 build-examples
	@printf "%s\n" "$(YELLOW)Testing ROS1 plugin...$(NC)"
	@if [ -f middlewares/ros1/devel/lib/libros1_plugin.so ]; then \
		cp middlewares/ros1/devel/lib/libros1_plugin.so examples/build/; \
	elif [ -f middlewares/ros1/build/libros1_plugin.so ]; then \
		cp middlewares/ros1/build/libros1_plugin.so examples/build/; \
	fi
	@printf "%s\n" "$(GREEN)✓ ROS1 plugin ready for testing$(NC)"
	@printf "%s\n" "$(BLUE)To run manually: cd examples/build && ./run_ros1.sh$(NC)"

# Run ROS2 test
run-ros2: build-ros2 build-examples
	@printf "%s\n" "$(YELLOW)Launching unified test with ROS2 plugin...$(NC)"
	@cd examples && ./run_ros2.sh

# Run ROS1 test
run-ros1: build-ros1 build-examples
	@printf "%s\n" "$(YELLOW)Launching unified test with ROS1 plugin...$(NC)"
	@cd examples && ./run_ros1.sh

# Clean all
clean: clean-ros2 clean-ros1 clean-examples
	@printf "%s\n" "$(GREEN)✓ All build artifacts cleaned$(NC)"

# Clean ROS2
clean-ros2:
	@printf "%s\n" "$(YELLOW)Cleaning ROS2 build artifacts...$(NC)"
	@cd middlewares/ros2 && rm -rf build install log || true
	@printf "%s\n" "$(GREEN)✓ ROS2 cleaned$(NC)"

# Clean ROS1
clean-ros1:
	@printf "%s\n" "$(YELLOW)Cleaning ROS1 build artifacts...$(NC)"
	@cd middlewares/ros1 && rm -rf build devel || true
	@printf "%s\n" "$(GREEN)✓ ROS1 cleaned$(NC)"

# Clean examples
clean-examples:
	@printf "%s\n" "$(YELLOW)Cleaning examples build artifacts...$(NC)"
	@cd examples && rm -rf build || true
	@printf "%s\n" "$(GREEN)✓ Examples cleaned$(NC)"

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=Debug build

# Release build (default)
release:
	@$(MAKE) BUILD_TYPE=Release build

# Show build info
info:
	@printf "%s\n" "$(BLUE)Axon Build Information:$(NC)"
	@echo ""
	@printf "%s\n" "Project: Universal middleware plugin system for ROS1 and ROS2"
	@echo ""
	@printf "%s\n" "$(YELLOW)ROS2 Status:$(NC)"
	@if [ -f /opt/ros/humble/setup.bash ]; then \
		echo "  ✓ ROS2 Humble installed"; \
	elif [ -f /opt/ros/iron/setup.bash ]; then \
		echo "  ✓ ROS2 Iron installed"; \
	else \
		echo "  ✗ ROS2 not found"; \
	fi
	@echo ""
	@printf "%s\n" "$(YELLOW)ROS1 Status:$(NC)"
	@if [ -f /opt/ros/noetic/setup.bash ]; then \
		echo "  ✓ ROS1 Noetic installed"; \
	elif [ -f /opt/ros/melodic/setup.bash ]; then \
		echo "  ✓ ROS1 Melodic installed"; \
	else \
		echo "  ✗ ROS1 not found"; \
	fi
	@echo ""
	@printf "%s\n" "$(YELLOW)Build Status:$(NC)"
	@if [ -d middlewares/ros2/build ]; then \
		echo "  ✓ ROS2 plugin built"; \
	else \
		echo "  ✗ ROS2 plugin not built"; \
	fi
	@if [ -d middlewares/ros1/build ]; then \
		echo "  ✓ ROS1 plugin built"; \
	else \
		echo "  ✗ ROS1 plugin not built"; \
	fi
	@if [ -f examples/build/subscriber_test ]; then \
		echo "  ✓ Unified test program built"; \
	else \
		echo "  ✗ Unified test program not built"; \
	fi
	@echo ""
	@printf "%s\n" "$(YELLOW)Key Feature:$(NC)"
	@printf "%s\n" "  The unified test program in examples/ has ZERO compile-time"
	@printf "%s\n" "  dependencies on ROS1 or ROS2 libraries! It loads plugins dynamically."
	@echo ""

# Install target (for setup)
install:
	@printf "%s\n" "$(YELLOW)Setting up project...$(NC)"
	@chmod +x scripts/*.sh || true
	@chmod +x middlewares/ros2/build.sh middlewares/ros1/build.sh middlewares/ros1/clean.sh || true
	@chmod +x examples/build.sh examples/run_ros2.sh examples/run_ros1.sh examples/test_unified.sh || true
	@printf "%s\n" "$(GREEN)✓ Setup complete$(NC)"
	@printf "%s\n" "$(BLUE)Run 'make help' for available targets$(NC)"

# Format all C++ code using clang-format
format:
	@printf "%s\n" "$(YELLOW)Formatting all C++ code...$(NC)"
	@if command -v clang-format >/dev/null 2>&1; then \
		find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" \) \
			! -path "*/build/*" \
			! -path "*/install/*" \
			! -path "*/log/*" \
			! -path "*/devel/*" \
			! -path "*/.*" \
			! -path "*/_deps/*" \
			-exec clang-format -i {} +; \
		printf "%s\n" "$(GREEN)✓ Code formatted successfully$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: clang-format not found$(NC)"; \
		printf "%s\n" "$(BLUE)Install with: sudo apt install clang-format$(NC)"; \
		exit 1; \
	fi

# Check if code is properly formatted (for CI/CD)
format-check:
	@printf "%s\n" "$(YELLOW)Checking code formatting...$(NC)"
	@if command -v clang-format >/dev/null 2>&1; then \
		find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" \) \
			! -path "*/build/*" \
			! -path "*/install/*" \
			! -path "*/log/*" \
			! -path "*/devel/*" \
			! -path "*/.*" \
			! -path "*/_deps/*" \
			-exec clang-format --dry-run --Werror {} +; \
		printf "%s\n" "$(GREEN)✓ All code is properly formatted$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: clang-format not found$(NC)"; \
		printf "%s\n" "$(BLUE)Install with: sudo apt install clang-format$(NC)"; \
		exit 1; \
	fi

# =============================================================================
# Docker Targets
# =============================================================================

# Docker registry and image tag
DOCKER_REGISTRY ?= ghcr.io
DOCKER_REPO ?= $(shell echo "$(USER)" | tr '[:upper:]' '[:lower:]')
DOCKER_TAG ?= latest

# Build all Docker images
docker-build: docker-build-ros2 docker-build-ros1
	@printf "%s\n" "$(GREEN)✓ All Docker images built successfully$(NC)"

# Build ROS2 Docker images
docker-build-ros2: docker-build-ros2-humble docker-build-ros2-jazzy docker-build-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ ROS2 Docker images built successfully$(NC)"

# Build ROS2 Humble Docker image
docker-build-ros2-humble:
	@printf "%s\n" "$(YELLOW)Building ROS2 Humble Docker image...$(NC)"
	@if [ -f docker/Dockerfile.ros2.humble ]; then \
		docker build -f docker/Dockerfile.ros2.humble -t $(DOCKER_REPO)/axon:ros2-humble docker/; \
		printf "%s\n" "$(GREEN)✓ ROS2 Humble image built successfully$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: docker/Dockerfile.ros2.humble not found$(NC)"; \
		exit 1; \
	fi

# Build ROS2 Jazzy Docker image
docker-build-ros2-jazzy:
	@printf "%s\n" "$(YELLOW)Building ROS2 Jazzy Docker image...$(NC)"
	@if [ -f docker/Dockerfile.ros2.jazzy ]; then \
		docker build -f docker/Dockerfile.ros2.jazzy -t $(DOCKER_REPO)/axon:ros2-jazzy docker/; \
		printf "%s\n" "$(GREEN)✓ ROS2 Jazzy image built successfully$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: docker/Dockerfile.ros2.jazzy not found$(NC)"; \
		exit 1; \
	fi

# Build ROS2 Rolling Docker image
docker-build-ros2-rolling:
	@printf "%s\n" "$(YELLOW)Building ROS2 Rolling Docker image...$(NC)"
	@if [ -f docker/Dockerfile.ros2.rolling ]; then \
		docker build -f docker/Dockerfile.ros2.rolling -t $(DOCKER_REPO)/axon:ros2-rolling docker/; \
		printf "%s\n" "$(GREEN)✓ ROS2 Rolling image built successfully$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: docker/Dockerfile.ros2.rolling not found$(NC)"; \
		exit 1; \
	fi

# Build ROS1 Docker image
docker-build-ros1:
	@printf "%s\n" "$(YELLOW)Building ROS1 Docker image...$(NC)"
	@if [ -f docker/Dockerfile.ros1 ]; then \
		docker build -f docker/Dockerfile.ros1 -t $(DOCKER_REPO)/axon:ros1-noetic docker/; \
		printf "%s\n" "$(GREEN)✓ ROS1 image built successfully$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: docker/Dockerfile.ros1 not found$(NC)"; \
		exit 1; \
	fi

# Run tests in Docker
docker-test:
	@printf "%s\n" "$(YELLOW)Running tests in Docker...$(NC)"
	@printf "%s\n" "Testing ROS2 Humble..."; \
	docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros2-humble \
		/workspace/axon/docker/scripts/run_integration.sh || true; \
	printf "%s\n" "Testing ROS2 Jazzy..."; \
	docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros2-jazzy \
		/workspace/axon/docker/scripts/run_integration.sh || true; \
	printf "%s\n" "Testing ROS2 Rolling..."; \
	docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros2-rolling \
		/workspace/axon/docker/scripts/run_integration.sh || true; \
	printf "%s\n" "Testing ROS1 Noetic..."; \
	docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros1-noetic \
		/workspace/axon/docker/scripts/run_integration.sh || true
	@printf "%s\n" "$(GREEN)✓ Docker tests completed$(NC)"

# Clean Docker images and containers
docker-clean:
	@printf "%s\n" "$(YELLOW)Cleaning Docker artifacts...$(NC)"
	@-docker ps -aq | xargs -r docker stop 2>/dev/null || true
	@-docker ps -aq | xargs -r docker rm 2>/dev/null || true
	@-docker images "$(DOCKER_REPO)/axon*" -q | xargs -r docker rmi -f 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Docker artifacts cleaned$(NC)"

# Push Docker images to registry
docker-push:
	@printf "%s\n" "$(YELLOW)Pushing Docker images to registry...$(NC)"
	@if docker images | grep -q "$(DOCKER_REPO)/axon:ros2-humble"; then \
		printf "%s\n" "Pushing ROS2 Humble..."; \
		docker tag $(DOCKER_REPO)/axon:ros2-humble $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros2-humble; \
		docker push $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros2-humble || true; \
	fi
	@if docker images | grep -q "$(DOCKER_REPO)/axon:ros2-jazzy"; then \
		printf "%s\n" "Pushing ROS2 Jazzy..."; \
		docker tag $(DOCKER_REPO)/axon:ros2-jazzy $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros2-jazzy; \
		docker push $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros2-jazzy || true; \
	fi
	@if docker images | grep -q "$(DOCKER_REPO)/axon:ros2-rolling"; then \
		printf "%s\n" "Pushing ROS2 Rolling..."; \
		docker tag $(DOCKER_REPO)/axon:ros2-rolling $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros2-rolling; \
		docker push $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros2-rolling || true; \
	fi
	@if docker images | grep -q "$(DOCKER_REPO)/axon:ros1-noetic"; then \
		printf "%s\n" "Pushing ROS1 Noetic..."; \
		docker tag $(DOCKER_REPO)/axon:ros1-noetic $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros1-noetic; \
		docker push $(DOCKER_REGISTRY)/$(DOCKER_REPO)/axon:ros1-noetic || true; \
	fi
	@printf "%s\n" "$(GREEN)✓ Docker images pushed successfully$(NC)"

# Show Docker images
docker-images:
	@printf "%s\n" "$(BLUE)Available Docker images:$(NC)"
	@echo ""
	@docker images | grep "axon" || echo "No axon images found"
	@echo ""
	@if [ -n "$$(docker images | grep 'axon.*ros[12]')" ]; then \
		printf "%s\n" "$(YELLOW)Image sizes:$(NC)"; \
		docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}" | grep 'axon.*ros[12]' || true; \
	fi

