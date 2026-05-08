add_executable(bench_buffer_pool bench_buffer_pool.cpp)
target_link_libraries(bench_buffer_pool PRIVATE axon_memory Threads::Threads)
target_compile_features(bench_buffer_pool PRIVATE cxx_std_17)
set_target_properties(bench_buffer_pool PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
