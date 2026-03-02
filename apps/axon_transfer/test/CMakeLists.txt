find_package(GTest REQUIRED)

file(
    GLOB TEST_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp"
)

foreach(TEST_SOURCE ${TEST_SOURCES})
  get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

  add_executable(${TEST_NAME} ${TEST_SOURCE})
  target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
  target_link_libraries(${TEST_NAME} PRIVATE axon_transfer_core GTest::gtest GTest::gtest_main)

  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
