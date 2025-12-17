# Makefile for Axon C++ SDK (axon_arrow)
# Core library providing Arrow builders, batch management, and message conversion

.PHONY: all build test clean install help debug release format lint check-deps dev-setup verify

.DEFAULT_GOAL := help

# Use bash as the shell
SHELL := /bin/bash

# Project directories
PROJECT_ROOT := $(shell cd .. && pwd)
BUILD_DIR := build
TEST_BUILD_DIR := $(BUILD_DIR)

# Build configuration
BUILD_TYPE ?= Release
CMAKE_OPTIONS ?=

# Detect number of CPU cores
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Colors for output (only use if stdout is a TTY)
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
	@printf "%s\n" "$(GREEN)║     Axon C++ SDK (axon_arrow) - Build System             ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Available targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make build$(NC)          - Build the library"
	@printf "%s\n" "  $(BLUE)make test$(NC)           - Build and run all tests"
	@printf "%s\n" "  $(BLUE)make clean$(NC)          - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make install$(NC)        - Install the library (requires sudo)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Build modes:$(NC)"
	@printf "%s\n" "  $(BLUE)make debug$(NC)          - Build in debug mode"
	@printf "%s\n" "  $(BLUE)make release$(NC)        - Build in release mode (default)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Code quality:$(NC)"
	@printf "%s\n" "  $(BLUE)make format$(NC)         - Format code (requires clang-format)"
	@printf "%s\n" "  $(BLUE)make lint$(NC)           - Lint code (requires cppcheck)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Environment variables:$(NC)"
	@printf "%s\n" "  $(BLUE)BUILD_TYPE$(NC)          - Release or Debug (default: Release)"
	@printf "%s\n" "  $(BLUE)CMAKE_OPTIONS$(NC)       - Additional CMake options"
	@echo ""
	@printf "%s\n" "$(YELLOW)Current configuration:$(NC)"
	@echo "  BUILD_TYPE: $(BUILD_TYPE)"
	@echo "  CPU cores: $(NPROC)"
	@echo "  Project root: $(PROJECT_ROOT)"

# Main build target
all: build

build:
	@printf "%s\n" "$(YELLOW)Building axon_arrow library...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake ../axon_arrow \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_ARROW_BUILD_TESTS=ON \
			$(CMAKE_OPTIONS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ Build complete$(NC)"

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=Debug build

# Release build
release:
	@$(MAKE) BUILD_TYPE=Release build

# Run all tests
test: build
	@printf "%s\n" "$(YELLOW)Running all tests...$(NC)"
	@cd $(BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ All tests passed$(NC)"

# Clean build artifacts
clean:
	@printf "%s\n" "$(YELLOW)Cleaning build artifacts...$(NC)"
	@rm -rf $(BUILD_DIR)
	@printf "%s\n" "$(GREEN)✓ Build artifacts cleaned$(NC)"

# Install the library
install: build
	@printf "%s\n" "$(YELLOW)Installing axon_arrow library...$(NC)"
	@cd $(BUILD_DIR) && cmake --install .
	@printf "%s\n" "$(GREEN)✓ Library installed$(NC)"

# Format code
format:
	@printf "%s\n" "$(YELLOW)Formatting C++ code...$(NC)"
	@if command -v clang-format >/dev/null 2>&1; then \
		find axon_arrow -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i; \
		printf "%s\n" "$(GREEN)✓ Code formatted$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: clang-format not found$(NC)"; \
		exit 1; \
	fi

# Lint code
lint:
	@printf "%s\n" "$(YELLOW)Linting C++ code...$(NC)"
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --suppress=missingIncludeSystem \
			--error-exitcode=1 axon_arrow/; \
		printf "%s\n" "$(GREEN)✓ Linting passed$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: cppcheck not found$(NC)"; \
		exit 1; \
	fi

# Check dependencies
check-deps:
	@printf "%s\n" "$(YELLOW)Checking dependencies...$(NC)"
	@command -v cmake >/dev/null 2>&1 || { printf "%s\n" "$(RED)Error: cmake not found$(NC)"; exit 1; }
	@pkg-config --exists arrow || { printf "%s\n" "$(RED)Error: Apache Arrow not found$(NC)"; exit 1; }
	@pkg-config --exists yaml-cpp || { printf "%s\n" "$(RED)Error: yaml-cpp not found$(NC)"; exit 1; }
	@printf "%s\n" "$(GREEN)✓ All dependencies found$(NC)"

# Development setup
dev-setup: check-deps
	@printf "%s\n" "$(GREEN)✓ Development environment ready$(NC)"

# Verify (build + test)
verify: build test
	@printf "%s\n" "$(GREEN)✓ Verification complete - build and tests passed!$(NC)"
