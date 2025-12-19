# =============================================================================
# ClangFormat.cmake - Automatic code formatting for C/C++ files
# =============================================================================
# This module provides clang-format integration for CMake projects.
#
# Usage:
#   include(ClangFormat)
#   axon_add_clang_format(target_name SOURCE_FILES file1.cpp file2.hpp ...)
#
# This creates two targets:
#   - format_<target_name>: Formats files in-place
#   - check_format_<target_name>: Checks formatting without modifying files
#
# The format target runs automatically before building when FORMAT_ON_BUILD=ON
# =============================================================================

option(FORMAT_ON_BUILD "Run clang-format automatically before building" ON)
option(FORMAT_CHECK_ONLY "Only check formatting, don't modify files" OFF)

find_program(CLANG_FORMAT_EXECUTABLE
    NAMES clang-format clang-format-14 clang-format-15 clang-format-16 clang-format-17
    DOC "Path to clang-format executable"
)

if(NOT CLANG_FORMAT_EXECUTABLE)
    message(STATUS "clang-format not found - formatting targets will be disabled")
    set(CLANG_FORMAT_FOUND FALSE)
else()
    message(STATUS "Found clang-format: ${CLANG_FORMAT_EXECUTABLE}")
    set(CLANG_FORMAT_FOUND TRUE)
    
    # Get version
    execute_process(
        COMMAND ${CLANG_FORMAT_EXECUTABLE} --version
        OUTPUT_VARIABLE CLANG_FORMAT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "clang-format version: ${CLANG_FORMAT_VERSION}")
endif()

# Function to add clang-format targets for a set of source files
function(axon_add_clang_format TARGET_NAME)
    cmake_parse_arguments(ARG "" "" "SOURCE_FILES" ${ARGN})
    
    if(NOT CLANG_FORMAT_FOUND)
        return()
    endif()
    
    if(NOT ARG_SOURCE_FILES)
        message(WARNING "axon_add_clang_format: No source files specified for ${TARGET_NAME}")
        return()
    endif()
    
    # Find .clang-format file by walking up from current source dir
    set(CLANG_FORMAT_FILE "")
    set(SEARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    while(NOT "${SEARCH_DIR}" STREQUAL "/")
        if(EXISTS "${SEARCH_DIR}/.clang-format")
            set(CLANG_FORMAT_FILE "${SEARCH_DIR}/.clang-format")
            break()
        endif()
        get_filename_component(SEARCH_DIR "${SEARCH_DIR}" DIRECTORY)
    endwhile()
    
    if(CLANG_FORMAT_FILE)
        message(STATUS "Using .clang-format: ${CLANG_FORMAT_FILE}")
    else()
        message(WARNING "No .clang-format file found - using clang-format defaults")
    endif()
    
    # Format target - modifies files in-place
    add_custom_target(format_${TARGET_NAME}
        COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${ARG_SOURCE_FILES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Formatting ${TARGET_NAME} source files with clang-format"
        VERBATIM
    )
    
    # Check format target - fails if files need formatting
    add_custom_target(check_format_${TARGET_NAME}
        COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror ${ARG_SOURCE_FILES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Checking ${TARGET_NAME} source file formatting"
        VERBATIM
    )
    
    # Optionally run format before building
    if(FORMAT_ON_BUILD AND TARGET ${TARGET_NAME})
        add_dependencies(${TARGET_NAME} format_${TARGET_NAME})
    endif()
endfunction()

# Convenience function to create a global format target
function(axon_add_format_all)
    if(NOT CLANG_FORMAT_FOUND)
        return()
    endif()
    
    # Create umbrella targets if they don't exist
    if(NOT TARGET format_all)
        add_custom_target(format_all
            COMMENT "Formatting all source files"
        )
    endif()
    
    if(NOT TARGET check_format_all)
        add_custom_target(check_format_all
            COMMENT "Checking all source file formatting"
        )
    endif()
endfunction()

# Add a target's format targets to the global format_all targets
function(axon_register_format_target TARGET_NAME)
    if(NOT CLANG_FORMAT_FOUND)
        return()
    endif()
    
    axon_add_format_all()
    
    if(TARGET format_${TARGET_NAME})
        add_dependencies(format_all format_${TARGET_NAME})
    endif()
    
    if(TARGET check_format_${TARGET_NAME})
        add_dependencies(check_format_all check_format_${TARGET_NAME})
    endif()
endfunction()
