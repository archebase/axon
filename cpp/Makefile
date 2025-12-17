# Makefile for Axon C++ SDK (axon_arrow)
# Core library providing Arrow builders, batch management, and message conversion

.PHONY: all build test clean install help debug release format lint check-deps dev-setup verify coverage coverage-html

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
COVERAGE_DIR := coverage

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
	@printf "%s\n" "  $(BLUE)make coverage$(NC)       - Run tests with coverage report (requires lcov)"
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
	@rm -rf $(BUILD_DIR) $(COVERAGE_DIR)
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

# =============================================================================
# Code Coverage
# =============================================================================
# Requires: lcov (brew install lcov / sudo apt install lcov)

# Build with coverage and run tests
# LCOV_IGNORE handles macOS LLVM/Clang compatibility issues:
#   - mismatch: source file checksum mismatches
#   - inconsistent: function end line < start line (LLVM gcov bug)
#   - unsupported: function begin/end line exclusions not supported
#   - unused: removing unused data
#   - format: line number 0 for compiler-generated functions (__cxx_global_var_init)
LCOV_IGNORE := --ignore-errors mismatch,mismatch \
               --ignore-errors inconsistent,inconsistent \
               --ignore-errors unsupported,unsupported \
               --ignore-errors unused,unused \
               --ignore-errors format,format

coverage:
	@printf "%s\n" "$(YELLOW)Building with coverage instrumentation...$(NC)"
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake ../axon_arrow \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_ARROW_BUILD_TESTS=ON \
			-DAXON_ARROW_ENABLE_COVERAGE=ON \
			$(CMAKE_OPTIONS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		cd $(BUILD_DIR) && \
		lcov --capture --directory . --output-file coverage_raw.info $(LCOV_IGNORE) && \
		lcov --remove coverage_raw.info \
			'/usr/*' \
			'/opt/*' \
			'/Library/*' \
			'*/test/*' \
			'*/_deps/*' \
			'*/c++/*' \
			--output-file coverage.info $(LCOV_IGNORE) && \
		echo "" && \
		echo "================================================================================" && \
		echo "                           COVERAGE SUMMARY" && \
		echo "================================================================================" && \
		lcov --list coverage.info $(LCOV_IGNORE) && \
		echo "================================================================================" && \
		printf "%s\n" "$(GREEN)✓ Coverage report generated$(NC)"; \
		printf "%s\n" "$(YELLOW)Run 'make coverage-html' to generate HTML report$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: lcov not found. Install with: brew install lcov (macOS) or sudo apt install lcov (Ubuntu)$(NC)"; \
		exit 1; \
	fi

# Generate HTML coverage report
coverage-html: coverage
	@printf "%s\n" "$(YELLOW)Generating HTML coverage report...$(NC)"
	@mkdir -p $(COVERAGE_DIR)
	@cd $(BUILD_DIR) && genhtml coverage.info --output-directory ../$(COVERAGE_DIR) $(LCOV_IGNORE)
	@printf "%s\n" "$(GREEN)✓ HTML report generated at $(COVERAGE_DIR)/index.html$(NC)"
	@if [ "$$(uname)" = "Darwin" ]; then \
		open $(COVERAGE_DIR)/index.html; \
	elif command -v xdg-open >/dev/null 2>&1; then \
		xdg-open $(COVERAGE_DIR)/index.html; \
	fi
