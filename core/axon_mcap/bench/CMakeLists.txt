# Standalone micro-benchmarks for axon_mcap (zero dependencies beyond GTest's
# absence — just C++17 and the axon_mcap library itself).

add_executable(bench_compression bench_compression.cpp)
target_link_libraries(bench_compression PRIVATE axon_mcap Threads::Threads)
target_compile_features(bench_compression PRIVATE cxx_std_17)
set_target_properties(bench_compression PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

add_executable(bench_async_writer bench_async_writer.cpp)
target_link_libraries(bench_async_writer PRIVATE axon_mcap Threads::Threads)
target_compile_features(bench_async_writer PRIVATE cxx_std_17)
set_target_properties(bench_async_writer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
