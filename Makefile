# Makefile for Axon
# Universal middleware plugin system for ROS1 and ROS2
# Supports building both ROS1 and ROS2 plugins and unified test programs

.PHONY: all build clean test help build-core build-core-mcap build-core-uploader build-core-logging test-core-mcap test-core-uploader test-core-logging clean-core clean-core-coverage build-ros2 build-ros1 build-examples test-ros2 test-ros1 format format-check docker-build docker-build-ros2 docker-build-ros2-humble docker-build-ros2-jazzy docker-build-ros2-rolling docker-build-ros1 docker-test docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling docker-test-ros1 docker-clean docker-push docker-images
.DEFAULT_GOAL := help

# Use bash as the shell for all recipes (required for ROS setup.bash scripts)
SHELL := /bin/bash

# Detect number of CPU cores
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Build type
BUILD_TYPE ?= Release
CMAKE_OPTIONS ?=

# Unified build directory (all non-middleware builds go here)
BUILD_DIR := build

# Core library subdirectories
CORE_BUILD_DIR := $(BUILD_DIR)/core
CORE_UTILS_BUILD_DIR := $(BUILD_DIR)/core_utils
CORE_UPLOADER_BUILD_DIR := $(BUILD_DIR)/core_uploader
CORE_LOGGING_BUILD_DIR := $(BUILD_DIR)/core_logging
CORE_COVERAGE_DIR := $(BUILD_DIR)/coverage

# Examples build directory
EXAMPLES_BUILD_DIR := $(BUILD_DIR)/examples

# Axon recorder build directory
RECORDER_BUILD_DIR := $(BUILD_DIR)/axon_recorder

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
	@printf "%s\n" "  $(BLUE)make build$(NC)              - Build core libraries + ROS2 plugin + examples"
	@printf "%s\n" "  $(BLUE)make build-core$(NC)         - Build core C++ libraries (mcap + logging + utils, excludes uploader)"
	@printf "%s\n" "  $(BLUE)make build-core-mcap$(NC)    - Build axon_mcap library"
	@printf "%s\n" "  $(BLUE)make build-core-utils$(NC)   - Build axon_utils library"
	@printf "%s\n" "  $(BLUE)make build-core-uploader$(NC) - Build axon_uploader library (optional)"
	@printf "%s\n" "  $(BLUE)make build-core-logging$(NC)  - Build axon_logging library"
	@printf "%s\n" "  $(BLUE)make build-ros2$(NC)         - Build ROS2 plugin only"
	@printf "%s\n" "  $(BLUE)make build-ros1$(NC)         - Build ROS1 plugin only"
	@printf "%s\n" "  $(BLUE)make build-examples$(NC)     - Build unified test program (no ROS deps)"
	@printf "%s\n" "  $(BLUE)make build-all$(NC)          - Build everything (core + ROS1 + ROS2 + examples)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Test targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make test$(NC)               - Quick test of unified program"
	@printf "%s\n" "  $(BLUE)make test-core-mcap$(NC)     - Test axon_mcap library"
	@printf "%s\n" "  $(BLUE)make test-core-utils$(NC)   - Test axon_utils library"
	@printf "%s\n" "  $(BLUE)make test-core-uploader$(NC) - Test axon_uploader library"
	@printf "%s\n" "  $(BLUE)make test-core-logging$(NC)  - Test axon_logging library"
	@printf "%s\n" "  $(BLUE)make test-ros2$(NC)          - Test ROS2 plugin"
	@printf "%s\n" "  $(BLUE)make test-ros1$(NC)          - Test ROS1 plugin"
	@echo ""
	@printf "%s\n" "$(YELLOW)Clean targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make clean$(NC)              - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make clean-core$(NC)         - Clean core libraries build artifacts"
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
	@printf "%s\n" "  $(BLUE)make docker-test$(NC)       - Run tests in all Docker containers"
	@printf "%s\n" "  $(BLUE)make docker-test-humble$(NC) - Run tests in ROS2 Humble container"
	@printf "%s\n" "  $(BLUE)make docker-test-jazzy$(NC)  - Run tests in ROS2 Jazzy container"
	@printf "%s\n" "  $(BLUE)make docker-test-rolling$(NC) - Run tests in ROS2 Rolling container"
	@printf "%s\n" "  $(BLUE)make docker-test-ros1$(NC)   - Run tests in ROS1 Noetic container"
	@printf "%s\n" "  $(BLUE)make docker-clean$(NC)      - Remove Docker images and containers"
	@printf "%s\n" "  $(BLUE)make docker-push$(NC)       - Push Docker images to registry"
	@printf "%s\n" "  $(BLUE)make docker-images$(NC)     - Show available Docker images"
	@echo ""
	@printf "%s\n" "$(YELLOW)Quick start:$(NC)"
	@printf "%s\n" "  1. $(BLUE)make build$(NC)           # Build core + ROS2 plugin + examples"
	@printf "%s\n" "  2. $(BLUE)make run-ros2$(NC)        # Run with ROS2"
	@echo ""

# Main build target (build core, ROS2 and examples by default)
build: build-core build-ros2 build-examples
	@printf "%s\n" "$(GREEN)✓ Build complete!$(NC)"
	@printf "%s\n" "$(BLUE)To test: make run-ros2$(NC)"

# Build core C++ libraries (excludes uploader by default)
build-core: build-core-mcap build-core-logging build-core-utils
	@printf "%s\n" "$(GREEN)✓ Core libraries built successfully (excluded: axon_uploader)$(NC)"
	@printf "%s\n" "$(BLUE)To build uploader: make build-core-uploader$(NC)"

# Build axon_mcap library
build-core-mcap:
	@printf "%s\n" "$(YELLOW)Building axon_mcap library...$(NC)"
	@mkdir -p $(CORE_BUILD_DIR)
	@cd $(CORE_BUILD_DIR) && \
		cmake $(PWD)/core/axon_mcap \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_MCAP_BUILD_TESTS=ON \
			$(CMAKE_OPTIONS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_mcap built successfully$(NC)"

# Build axon_uploader library
build-core-uploader:
	@printf "%s\n" "$(YELLOW)Building axon_uploader library...$(NC)"
	@mkdir -p $(CORE_UPLOADER_BUILD_DIR)
	@cd $(CORE_UPLOADER_BUILD_DIR) && \
		cmake $(PWD)/core/axon_uploader \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_UPLOADER_BUILD_TESTS=ON \
			$(CMAKE_OPTIONS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_uploader built successfully$(NC)"

# Build axon_logging library
build-core-logging:
	@printf "%s\n" "$(YELLOW)Building axon_logging library...$(NC)"
	@mkdir -p $(CORE_LOGGING_BUILD_DIR)
	@cd $(CORE_LOGGING_BUILD_DIR) && \
		cmake $(PWD)/core/axon_logging \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_LOGGING_BUILD_TESTS=ON \
			$(CMAKE_OPTIONS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_logging built successfully$(NC)"

# Build axon_utils library
build-core-utils:
	@printf "%s\n" "$(YELLOW)Building axon_utils library...$(NC)"
	@mkdir -p $(CORE_UTILS_BUILD_DIR)
	@cd $(CORE_UTILS_BUILD_DIR) && \
		cmake $(PWD)/core/axon_utils \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			$(CMAKE_OPTIONS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_utils built successfully$(NC)"

# Test axon_mcap library
test-core-mcap: build-core-mcap
	@printf "%s\n" "$(YELLOW)Running axon_mcap tests...$(NC)"
	@cd $(CORE_BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_mcap tests passed$(NC)"

# Test axon_uploader library
test-core-uploader: build-core-uploader
	@printf "%s\n" "$(YELLOW)Running axon_uploader tests...$(NC)"
	@cd $(CORE_UPLOADER_BUILD_DIR) && \
		find . -name "*.gcda" -type f -delete 2>/dev/null || true && \
		if [ -f CMakeCache.txt ] && grep -q "AXON_UPLOADER_ENABLE_COVERAGE:BOOL=ON" CMakeCache.txt 2>/dev/null; then \
			printf "%s\n" "$(YELLOW)Coverage enabled - running tests sequentially...$(NC)"; \
			ctest --output-on-failure -j1; \
		else \
			ctest --output-on-failure; \
		fi
	@printf "%s\n" "$(GREEN)✓ axon_uploader tests passed$(NC)"

# Test axon_logging library
test-core-logging: build-core-logging
	@printf "%s\n" "$(YELLOW)Running axon_logging tests...$(NC)"
	@cd $(CORE_LOGGING_BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_logging tests passed$(NC)"

# Test axon_utils library
test-core-utils: build-core-utils
	@printf "%s\n" "$(YELLOW)Running axon_utils tests...$(NC)"
	@cd $(CORE_UTILS_BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_utils tests passed$(NC)"

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
	@mkdir -p $(EXAMPLES_BUILD_DIR)
	@cd $(EXAMPLES_BUILD_DIR) && \
		cmake $(PWD)/examples && \
		make -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ Unified test program built successfully$(NC)"
	@printf "%s\n" "$(BLUE)The unified test program has ZERO compile-time ROS dependencies!$(NC)"

# Build everything
build-all: build-core build-ros2 build-ros1 build-examples
	@printf "%s\n" "$(GREEN)✓ All components built successfully!$(NC)"
	@printf "%s\n" "$(BLUE)Unified test program works with both ROS1 and ROS2 plugins$(NC)"

# Quick test
test: build-examples
	@printf "%s\n" "$(YELLOW)Testing unified test program...$(NC)"
	@cd $(EXAMPLES_BUILD_DIR) && ./test_unified.sh || true

# Test ROS2 plugin
test-ros2: build-ros2 build-examples
	@printf "%s\n" "$(YELLOW)Testing ROS2 plugin...$(NC)"
	@if [ -f middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so ]; then \
		cp middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	elif [ -f middlewares/ros2/build/ros2_plugin/libros2_plugin.so ]; then \
		cp middlewares/ros2/build/ros2_plugin/libros2_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	fi
	@printf "%s\n" "$(GREEN)✓ ROS2 plugin ready for testing$(NC)"
	@printf "%s\n" "$(BLUE)To run manually: cd $(EXAMPLES_BUILD_DIR) && ./run_ros2.sh$(NC)"

# Test ROS1 plugin
test-ros1: build-ros1 build-examples
	@printf "%s\n" "$(YELLOW)Testing ROS1 plugin...$(NC)"
	@if [ -f middlewares/ros1/devel/lib/libros1_plugin.so ]; then \
		cp middlewares/ros1/devel/lib/libros1_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	elif [ -f middlewares/ros1/build/libros1_plugin.so ]; then \
		cp middlewares/ros1/build/libros1_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	fi
	@printf "%s\n" "$(GREEN)✓ ROS1 plugin ready for testing$(NC)"
	@printf "%s\n" "$(BLUE)To run manually: cd $(EXAMPLES_BUILD_DIR) && ./run_ros1.sh$(NC)"

# Run ROS2 test
run-ros2: build-ros2 build-examples
	@printf "%s\n" "$(YELLOW)Launching unified test with ROS2 plugin...$(NC)"
	@if [ -f /opt/ros/humble/setup.bash ]; then \
		. /opt/ros/humble/setup.bash; \
	elif [ -f /opt/ros/iron/setup.bash ]; then \
		. /opt/ros/iron/setup.bash; \
	fi
	@if [ -f middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so ]; then \
		cp middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	elif [ -f middlewares/ros2/build/ros2_plugin/libros2_plugin.so ]; then \
		cp middlewares/ros2/build/ros2_plugin/libros2_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	fi
	@cd $(EXAMPLES_BUILD_DIR) && ./subscriber_test ros2

# Run ROS1 test
run-ros1: build-ros1 build-examples
	@printf "%s\n" "$(YELLOW)Launching unified test with ROS1 plugin...$(NC)"
	@if [ -f /opt/ros/noetic/setup.bash ]; then \
		. /opt/ros/noetic/setup.bash; \
	elif [ -f /opt/ros/melodic/setup.bash ]; then \
		. /opt/ros/melodic/setup.bash; \
	fi
	@if [ -f middlewares/ros1/devel/lib/libros1_plugin.so ]; then \
		cp middlewares/ros1/devel/lib/libros1_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	elif [ -f middlewares/ros1/build/libros1_plugin.so ]; then \
		cp middlewares/ros1/build/libros1_plugin.so $(EXAMPLES_BUILD_DIR)/; \
	fi
	@cd $(EXAMPLES_BUILD_DIR) && ./subscriber_test ros1

# Clean all
clean: clean-core clean-ros2 clean-ros1 clean-examples
	@printf "%s\n" "$(YELLOW)Cleaning root build directory...$(NC)"
	@rm -rf $(BUILD_DIR) || true
	@printf "%s\n" "$(GREEN)✓ All build artifacts cleaned$(NC)"

# Clean core libraries
clean-core:
	@printf "%s\n" "$(YELLOW)Cleaning core libraries build artifacts...$(NC)"
	@rm -rf $(CORE_BUILD_DIR) $(CORE_UPLOADER_BUILD_DIR) $(CORE_LOGGING_BUILD_DIR) $(CORE_UTILS_BUILD_DIR) $(CORE_COVERAGE_DIR) || true
	@printf "%s\n" "$(GREEN)✓ Core libraries cleaned$(NC)"

# Clean core coverage data files
clean-core-coverage:
	@printf "%s\n" "$(YELLOW)Cleaning core coverage data files...$(NC)"
	@find $(CORE_BUILD_DIR) $(CORE_UPLOADER_BUILD_DIR) $(CORE_LOGGING_BUILD_DIR) -name "*.gcda" -type f -delete 2>/dev/null || true
	@find $(CORE_BUILD_DIR) $(CORE_UPLOADER_BUILD_DIR) $(CORE_LOGGING_BUILD_DIR) -name "*.gcno" -type f -delete 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Core coverage data files cleaned$(NC)"

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
	@rm -rf $(EXAMPLES_BUILD_DIR) || true
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

# Run tests in Docker (all versions)
docker-test: docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling docker-test-ros1
	@printf "%s\n" "$(GREEN)✓ All Docker tests completed$(NC)"

# Run tests in ROS2 Humble Docker container
docker-test-ros2-humble:
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Humble container...$(NC)"
	@docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros2-humble \
		/workspace/axon/docker/scripts/run_integration.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Humble tests completed$(NC)"

# Run tests in ROS2 Jazzy Docker container
docker-test-ros2-jazzy:
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Jazzy container...$(NC)"
	@docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros2-jazzy \
		/workspace/axon/docker/scripts/run_integration.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Jazzy tests completed$(NC)"

# Run tests in ROS2 Rolling Docker container
docker-test-ros2-rolling:
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Rolling container...$(NC)"
	@docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros2-rolling \
		/workspace/axon/docker/scripts/run_integration.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Rolling tests completed$(NC)"

# Run tests in ROS1 Noetic Docker container
docker-test-ros1:
	@printf "%s\n" "$(YELLOW)Running tests in ROS1 Noetic container...$(NC)"
	@docker run --rm \
		-v $(PWD):/workspace/axon \
		$(DOCKER_REPO)/axon:ros1-noetic \
		/workspace/axon/docker/scripts/run_integration.sh
	@printf "%s\n" "$(GREEN)✓ ROS1 Noetic tests completed$(NC)"

# Clean Docker images and containers
docker-clean:
	@printf "%s\n" "$(YELLOW)Cleaning Docker artifacts...$(NC)"
	@-docker ps -aq | xargs -r docker stop 2>/dev/null || true
	@-docker ps -aq | xargs -r docker rm 2>/dev/null || true
	@-docker images "$(DOCKER_REPO)/axon*" -q | xargs -r docker rmi -f 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Docker artifacts cleaned$(NC)"

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

