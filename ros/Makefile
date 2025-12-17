# Makefile for Axon by ArcheBase
# Supports ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling)
# Located in ros/ directory - paths are relative to project root

.PHONY: all build test clean install rust-build rust-test cpp-build cpp-test help
.DEFAULT_GOAL := help

# Use bash as the shell for all recipes (required for ROS setup.bash scripts)
SHELL := /bin/bash

# Project root (parent directory)
PROJECT_ROOT := $(shell cd .. && pwd)

# Detect ROS version
ROS_VERSION ?= $(shell if [ -n "$$ROS_VERSION" ]; then echo $$ROS_VERSION; elif [ -n "$$ROS_DISTRO" ]; then echo 1; else echo 2; fi)
ROS_DISTRO ?= $(shell echo $$ROS_DISTRO)

# Build directories (relative to this Makefile location)
BUILD_DIR := build
TEST_BUILD_DIR := axon_recorder/test/build
C_FFI_DIR := ../c
RUST_TARGET_DIR := $(C_FFI_DIR)/target

# Build type
BUILD_TYPE ?= Release
RUST_BUILD_TYPE ?= debug

# Detect number of CPU cores
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Colors for output (only use if stdout is a TTY)
# Check if stdout is a terminal, otherwise use empty strings
ifeq ($(shell test -t 1 && echo yes),yes)
  RED := \033[0;31m
  GREEN := \033[0;32m
  YELLOW := \033[0;33m
  BLUE := \033[0;34m
  NC := \033[0m # No Color
else
  RED :=
  GREEN :=
  YELLOW :=
  BLUE :=
  NC :=
endif

# Helper to print colored output (printf interprets escape sequences)
PRINTF := printf "%s\n"

# Help target
help:
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║     Axon by ArcheBase - Build System                     ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Available targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make build$(NC)          - Build the project (ROS1 or ROS2)"
	@printf "%s\n" "  $(BLUE)make test$(NC)           - Run all tests (Rust + C++)"
	@printf "%s\n" "  $(BLUE)make clean$(NC)          - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make install$(NC)        - Install the package"
	@echo ""
	@printf "%s\n" "$(YELLOW)Component-specific targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make rust-build$(NC)     - Build Rust bridge library only"
	@printf "%s\n" "  $(BLUE)make rust-test$(NC)      - Run Rust tests only"
	@printf "%s\n" "  $(BLUE)make cpp-build$(NC)      - Build C++ code only"
	@printf "%s\n" "  $(BLUE)make cpp-test$(NC)       - Run C++ tests only"
	@echo ""
	@printf "%s\n" "$(YELLOW)Build modes:$(NC)"
	@printf "%s\n" "  $(BLUE)make debug$(NC)          - Build in debug mode"
	@printf "%s\n" "  $(BLUE)make release$(NC)        - Build in release mode (default)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Code quality:$(NC)"
	@printf "%s\n" "  $(BLUE)make format$(NC)         - Format code (requires formatters)"
	@printf "%s\n" "  $(BLUE)make lint$(NC)           - Lint code (requires linters)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Docker:$(NC)"
	@printf "%s\n" "  $(BLUE)make docker-build$(NC)        - Build all Docker images"
	@printf "%s\n" "  $(BLUE)make docker-test$(NC)         - Run tests in Docker (auto-detect ROS)"
	@printf "%s\n" "  $(BLUE)make docker-test-all$(NC)     - Run tests in all Docker containers"
	@printf "%s\n" "  $(BLUE)make docker-test-compose$(NC) - Run tests using docker-compose"
	@echo ""
	@printf "%s\n" "$(YELLOW)Environment variables:$(NC)"
	@printf "%s\n" "  $(BLUE)ROS_VERSION$(NC)         - Set to 1 or 2 (auto-detected)"
	@printf "%s\n" "  $(BLUE)ROS_DISTRO$(NC)          - ROS distribution name"
	@printf "%s\n" "  $(BLUE)BUILD_TYPE$(NC)          - Release or Debug (default: Release)"
	@printf "%s\n" "  $(BLUE)RUST_BUILD_TYPE$(NC)     - release or debug (default: debug)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Current configuration:$(NC)"
	@echo "  ROS_VERSION: $(ROS_VERSION)"
	@echo "  ROS_DISTRO: $(if $(ROS_DISTRO),$(ROS_DISTRO),not set)"
	@echo "  BUILD_TYPE: $(BUILD_TYPE)"
	@echo "  RUST_BUILD_TYPE: $(RUST_BUILD_TYPE)"
	@echo "  CPU cores: $(NPROC)"
	@echo "  Project root: $(PROJECT_ROOT)"

# Main build target
all: rust-build cpp-build
	@printf "%s\n" "$(GREEN)✓ Build complete!$(NC)"

build: all

# Rust build (C FFI)
rust-build:
	@printf "%s\n" "$(YELLOW)Building C FFI library (Rust)...$(NC)"
	@cd $(C_FFI_DIR) && \
		if [ "$(RUST_BUILD_TYPE)" = "release" ]; then \
			cargo build --release; \
		else \
			cargo build; \
		fi
	@printf "%s\n" "$(GREEN)✓ C FFI library built$(NC)"

rust-test:
	@printf "%s\n" "$(YELLOW)Running C FFI tests (Rust)...$(NC)"
	@cd $(C_FFI_DIR) && \
		if [ "$(RUST_BUILD_TYPE)" = "release" ]; then \
			cargo test --release; \
		else \
			cargo test; \
		fi
	@printf "%s\n" "$(GREEN)✓ C FFI tests passed$(NC)"

# C++ build for ROS1
cpp-build-ros1:
	@printf "%s\n" "$(YELLOW)Building C++ code for ROS1...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		echo "Example: source /opt/ros/noetic/setup.bash"; \
		exit 1; \
	fi
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
			. /opt/ros/$(ROS_DISTRO)/setup.bash; \
		fi && \
		cmake ../axon_recorder -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ C++ code built for ROS1$(NC)"

# C++ build for ROS2
cpp-build-ros2:
	@printf "%s\n" "$(YELLOW)Building C++ code for ROS2...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		echo "Example: source /opt/ros/humble/setup.bash"; \
		exit 1; \
	fi
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
			. /opt/ros/$(ROS_DISTRO)/setup.bash; \
		fi && \
		cmake ../axon_recorder -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ C++ code built for ROS2$(NC)"

# C++ build (auto-detect ROS version)
cpp-build:
	@if [ "$(ROS_VERSION)" = "1" ]; then \
		$(MAKE) cpp-build-ros1; \
	else \
		$(MAKE) cpp-build-ros2; \
	fi

# C++ tests
cpp-test:
	@printf "%s\n" "$(YELLOW)Building and running C++ tests...$(NC)"
	@mkdir -p $(TEST_BUILD_DIR)
	@cd $(TEST_BUILD_DIR) && \
		cmake ../.. && \
		cmake --build . -j$(NPROC) && \
		ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ C++ tests passed$(NC)"

# Run all tests
test: rust-test cpp-test
	@printf "%s\n" "$(YELLOW)Running integration tests...$(NC)"
	@if [ -f axon_recorder/test/integration/test_rust_bridge.sh ]; then \
		bash axon_recorder/test/integration/test_rust_bridge.sh; \
	fi
	@printf "%s\n" "$(GREEN)✓ All tests passed!$(NC)"

# Clean targets
clean-rust:
	@printf "%s\n" "$(YELLOW)Cleaning C FFI build artifacts...$(NC)"
	@cd $(C_FFI_DIR) && cargo clean
	@printf "%s\n" "$(GREEN)✓ C FFI artifacts cleaned$(NC)"

clean-cpp:
	@printf "%s\n" "$(YELLOW)Cleaning C++ build artifacts...$(NC)"
	@rm -rf $(BUILD_DIR) $(TEST_BUILD_DIR)
	@printf "%s\n" "$(GREEN)✓ C++ artifacts cleaned$(NC)"

clean: clean-rust clean-cpp
	@printf "%s\n" "$(GREEN)✓ All build artifacts cleaned$(NC)"

# Install target
install: build
	@printf "%s\n" "$(YELLOW)Installing package...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set$(NC)"; \
		exit 1; \
	fi
	@cd $(BUILD_DIR) && \
		if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
			. /opt/ros/$(ROS_DISTRO)/setup.bash; \
		fi && \
		cmake --install .
	@printf "%s\n" "$(GREEN)✓ Package installed$(NC)"

# Docker build targets
docker-build-ros1:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS1...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros1 -t axon:ros1 . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros1: docker-build-ros1
	@printf "%s\n" "$(YELLOW)Running tests in ROS1 Docker container...$(NC)"
	@mkdir -p $(PROJECT_ROOT)/$(C_FFI_DIR)/target
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/$(C_FFI_DIR)/target:/workspace/axon/c/target \
		-v cargo-cache-ros1:/root/.cargo/registry \
		-v cargo-git-cache-ros1:/root/.cargo/git \
		-e ROS_DISTRO=noetic \
		-e ROS_VERSION=1 \
		axon:ros1 \
		/usr/local/bin/run_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS1 tests passed$(NC)"

docker-build-ros2-humble:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS2 Humble...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros2.humble -t axon:ros2-humble . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros2-humble: docker-build-ros2-humble
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Humble Docker container...$(NC)"
	@mkdir -p $(PROJECT_ROOT)/$(C_FFI_DIR)/target
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/$(C_FFI_DIR)/target:/workspace/axon/c/target \
		-v cargo-cache-ros2-humble:/root/.cargo/registry \
		-v cargo-git-cache-ros2-humble:/root/.cargo/git \
		-e ROS_DISTRO=humble \
		-e ROS_VERSION=2 \
		axon:ros2-humble \
		/usr/local/bin/run_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Humble tests passed$(NC)"

docker-build-ros2-jazzy:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS2 Jazzy...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros2.jazzy -t axon:ros2-jazzy . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros2-jazzy: docker-build-ros2-jazzy
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Jazzy Docker container...$(NC)"
	@mkdir -p $(PROJECT_ROOT)/$(C_FFI_DIR)/target
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/$(C_FFI_DIR)/target:/workspace/axon/c/target \
		-v cargo-cache-ros2-jazzy:/root/.cargo/registry \
		-v cargo-git-cache-ros2-jazzy:/root/.cargo/git \
		-e ROS_DISTRO=jazzy \
		-e ROS_VERSION=2 \
		axon:ros2-jazzy \
		/usr/local/bin/run_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Jazzy tests passed$(NC)"

docker-build-ros2-rolling:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS2 Rolling...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros2.rolling -t axon:ros2-rolling . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros2-rolling: docker-build-ros2-rolling
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Rolling Docker container...$(NC)"
	@mkdir -p $(PROJECT_ROOT)/$(C_FFI_DIR)/target
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/$(C_FFI_DIR)/target:/workspace/axon/c/target \
		-v cargo-cache-ros2-rolling:/root/.cargo/registry \
		-v cargo-git-cache-ros2-rolling:/root/.cargo/git \
		-e ROS_DISTRO=rolling \
		-e ROS_VERSION=2 \
		axon:ros2-rolling \
		/usr/local/bin/run_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Rolling tests passed$(NC)"

docker-build: docker-build-ros1 docker-build-ros2-humble docker-build-ros2-jazzy docker-build-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All Docker images built$(NC)"

# Run tests in all Docker containers
docker-test-all: docker-test-ros1 docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All Docker tests passed!$(NC)"

# Quick Docker test (single version)
docker-test:
	@if [ "$(ROS_VERSION)" = "1" ]; then \
		$(MAKE) docker-test-ros1; \
	else \
		$(MAKE) docker-test-ros2-humble; \
	fi

# Run tests using docker-compose (all versions in parallel)
docker-test-compose:
	@printf "%s\n" "$(YELLOW)Running tests in all Docker containers (docker-compose)...$(NC)"
	@cd $(PROJECT_ROOT)/docker && docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit
	@printf "%s\n" "$(GREEN)✓ All Docker tests passed!$(NC)"

# Build in Docker
docker-build-only:
	@printf "%s\n" "$(YELLOW)Building in Docker container...$(NC)"
	@mkdir -p $(PROJECT_ROOT)/$(C_FFI_DIR)/target
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/$(C_FFI_DIR)/target:/workspace/axon/c/target \
		-v cargo-cache-ros$(ROS_VERSION):/root/.cargo/registry \
		-v cargo-git-cache-ros$(ROS_VERSION):/root/.cargo/git \
		-e ROS_DISTRO=$(ROS_DISTRO) \
		-e ROS_VERSION=$(ROS_VERSION) \
		axon:ros$(ROS_VERSION) \
		/usr/local/bin/run_build.sh
	@printf "%s\n" "$(GREEN)✓ Docker build complete$(NC)"

# Development targets
dev-setup:
	@printf "%s\n" "$(YELLOW)Setting up development environment...$(NC)"
	@echo "Checking dependencies..."
	@command -v cargo >/dev/null 2>&1 || { echo "$(RED)Error: cargo not found. Install Rust toolchain.$(NC)"; exit 1; }
	@command -v cmake >/dev/null 2>&1 || { echo "$(RED)Error: cmake not found.$(NC)"; exit 1; }
	@command -v pkg-config >/dev/null 2>&1 || { echo "$(RED)Error: pkg-config not found.$(NC)"; exit 1; }
	@printf "%s\n" "$(GREEN)✓ Development environment ready$(NC)"

# Format code (if formatters are available)
format-rust:
	@printf "%s\n" "$(YELLOW)Formatting C FFI code (Rust)...$(NC)"
	@cd $(C_FFI_DIR) && cargo fmt
	@printf "%s\n" "$(GREEN)✓ C FFI code formatted$(NC)"

format-cpp:
	@printf "%s\n" "$(YELLOW)Formatting C++ code...$(NC)"
	@if command -v clang-format >/dev/null 2>&1; then \
		find ../cpp/src . -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i; \
		echo "$(GREEN)✓ C++ code formatted$(NC)"; \
	else \
		echo "$(YELLOW)clang-format not found, skipping$(NC)"; \
	fi

format: format-rust format-cpp

# Lint code
lint-rust:
	@printf "%s\n" "$(YELLOW)Linting C FFI code (Rust)...$(NC)"
	@cd $(C_FFI_DIR) && \
		if [ "$(RUST_BUILD_TYPE)" = "release" ]; then \
			cargo clippy --release -- -D warnings; \
		else \
			cargo clippy -- -D warnings; \
		fi
	@printf "%s\n" "$(GREEN)✓ C FFI linting passed$(NC)"

lint-cpp:
	@printf "%s\n" "$(YELLOW)Linting C++ code...$(NC)"
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --suppress=missingIncludeSystem ../cpp/src/ ./; \
		echo "$(GREEN)✓ C++ linting passed$(NC)"; \
	else \
		echo "$(YELLOW)cppcheck not found, skipping$(NC)"; \
	fi

lint: lint-rust lint-cpp

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=Debug RUST_BUILD_TYPE=debug build

# Release build (default)
release:
	@$(MAKE) BUILD_TYPE=Release RUST_BUILD_TYPE=release build

# Quick test (Rust only, fast)
quick-test: rust-test
	@printf "%s\n" "$(GREEN)✓ Quick test complete$(NC)"

# Full test suite
full-test: test lint
	@printf "%s\n" "$(GREEN)✓ Full test suite complete$(NC)"

# Check if ROS is sourced
check-ros:
	@if [ -z "$$ROS_DISTRO" ] && [ -z "$$ROS_VERSION" ]; then \
		echo "$(RED)Error: ROS environment not set.$(NC)"; \
		echo "For ROS1: source /opt/ros/noetic/setup.bash"; \
		echo "For ROS2: source /opt/ros/humble/setup.bash (or jazzy/rolling)"; \
		exit 1; \
	fi

# Build with ROS check
build-safe: check-ros build

# Test with ROS check
test-safe: check-ros test

# Verify build (build + test)
verify: build test
	@printf "%s\n" "$(GREEN)✓ Verification complete - build and tests passed!$(NC)"
