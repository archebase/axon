# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

file(
    GLOB AXON_SYSTEM_TEST_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp"
)

foreach(TEST_SOURCE ${AXON_SYSTEM_TEST_SOURCES})
  get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

  add_executable(${TEST_NAME} ${TEST_SOURCE})
  target_link_libraries(${TEST_NAME} PRIVATE axon_system_core)
  target_compile_features(${TEST_NAME} PRIVATE cxx_std_17)
  axon_enable_warnings(${TEST_NAME})

  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
