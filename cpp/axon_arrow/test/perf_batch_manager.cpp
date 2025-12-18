//! BatchManager Performance Benchmarks
//!
//! Tests various scenarios for batch management performance:
//! - High-frequency small messages (1000Hz IMU simulation)
//! - Large binary data (camera frames)
//! - Mixed workloads (IMU + cameras)
//! - Queue throughput

#include <benchmark/benchmark.h>
#include "batch_manager.hpp"
#include "compression/image_compressor.hpp"
#include <arrow/api.h>
#include <arrow/builder.h>
#include <atomic>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

using namespace axon::core;
using namespace axon::compression;

// =============================================================================
// Test Data Generators
// =============================================================================

// Generate IMU-like sensor data schema
std::shared_ptr<arrow::Schema> create_imu_schema() {
    return arrow::schema({
        arrow::field("timestamp", arrow::int64()),
        arrow::field("accel_x", arrow::float64()),
        arrow::field("accel_y", arrow::float64()),
        arrow::field("accel_z", arrow::float64()),
        arrow::field("gyro_x", arrow::float64()),
        arrow::field("gyro_y", arrow::float64()),
        arrow::field("gyro_z", arrow::float64()),
    });
}

// Generate camera image schema
std::shared_ptr<arrow::Schema> create_image_schema() {
    return arrow::schema({
        arrow::field("timestamp", arrow::int64()),
        arrow::field("camera_id", arrow::int64()),
        arrow::field("width", arrow::int64()),
        arrow::field("height", arrow::int64()),
        arrow::field("rgb_data", arrow::binary()),
        arrow::field("depth_data", arrow::binary()),
    });
}

// Create single IMU row arrays
std::vector<std::shared_ptr<arrow::Array>> create_imu_row(int64_t timestamp) {
    std::vector<std::shared_ptr<arrow::Array>> arrays;
    
    // Timestamp
    auto ts_builder = std::make_shared<arrow::Int64Builder>();
    ts_builder->Append(timestamp);
    std::shared_ptr<arrow::Array> ts_array;
    ts_builder->Finish(&ts_array);
    arrays.push_back(ts_array);
    
    // Sensor values (6 doubles)
    for (int i = 0; i < 6; ++i) {
        auto builder = std::make_shared<arrow::DoubleBuilder>();
        builder->Append(static_cast<double>(timestamp) * 0.001 + i * 0.1);
        std::shared_ptr<arrow::Array> array;
        builder->Finish(&array);
        arrays.push_back(array);
    }
    
    return arrays;
}

// Create image row arrays with specified data sizes
std::vector<std::shared_ptr<arrow::Array>> create_image_row(
    int64_t timestamp,
    int camera_id,
    size_t rgb_size,
    size_t depth_size) {
    
    std::vector<std::shared_ptr<arrow::Array>> arrays;
    
    // Timestamp
    auto ts_builder = std::make_shared<arrow::Int64Builder>();
    ts_builder->Append(timestamp);
    std::shared_ptr<arrow::Array> ts_array;
    ts_builder->Finish(&ts_array);
    arrays.push_back(ts_array);
    
    // Camera ID
    auto cam_builder = std::make_shared<arrow::Int64Builder>();
    cam_builder->Append(camera_id);
    std::shared_ptr<arrow::Array> cam_array;
    cam_builder->Finish(&cam_array);
    arrays.push_back(cam_array);
    
    // Width
    auto w_builder = std::make_shared<arrow::Int64Builder>();
    w_builder->Append(1920);
    std::shared_ptr<arrow::Array> w_array;
    w_builder->Finish(&w_array);
    arrays.push_back(w_array);
    
    // Height
    auto h_builder = std::make_shared<arrow::Int64Builder>();
    h_builder->Append(1080);
    std::shared_ptr<arrow::Array> h_array;
    h_builder->Finish(&h_array);
    arrays.push_back(h_array);
    
    // RGB data (simulated compressed)
    std::vector<uint8_t> rgb_data(rgb_size, 128);
    auto rgb_builder = std::make_shared<arrow::BinaryBuilder>();
    rgb_builder->Append(rgb_data.data(), rgb_data.size());
    std::shared_ptr<arrow::Array> rgb_array;
    rgb_builder->Finish(&rgb_array);
    arrays.push_back(rgb_array);
    
    // Depth data (simulated compressed)
    std::vector<uint8_t> depth_data(depth_size, 64);
    auto depth_builder = std::make_shared<arrow::BinaryBuilder>();
    depth_builder->Append(depth_data.data(), depth_data.size());
    std::shared_ptr<arrow::Array> depth_array;
    depth_builder->Finish(&depth_array);
    arrays.push_back(depth_array);
    
    return arrays;
}

// =============================================================================
// BatchManager Benchmarks
// =============================================================================

// No-op write callback for benchmarking
static std::atomic<size_t> g_write_count{0};
static std::atomic<size_t> g_total_rows{0};

bool noop_write_callback(const std::shared_ptr<arrow::RecordBatch>& batch,
                        const std::string& /*path*/,
                        int64_t /*handle*/) {
    g_write_count++;
    g_total_rows += batch->num_rows();
    return true;
}

// Benchmark: High-frequency IMU data (1000Hz simulation)
static void BM_IMU_1000Hz(benchmark::State& state) {
    const size_t batch_size = state.range(0);
    auto schema = create_imu_schema();
    
    for (auto _ : state) {
        state.PauseTiming();
        g_write_count = 0;
        g_total_rows = 0;
        
        BatchManager manager(batch_size, 10000, noop_write_callback);
        manager.set_dataset("/tmp/bench.lance", 1);
        manager.initialize_schema(schema);
        manager.start();
        state.ResumeTiming();
        
        // Simulate 1 second of 1000Hz data
        for (int64_t i = 0; i < 1000; ++i) {
            auto row = create_imu_row(i * 1000000); // nanoseconds
            manager.add_row(row);
        }
        
        state.PauseTiming();
        manager.stop();
        state.ResumeTiming();
    }
    
    // Calculate throughput
    const size_t bytes_per_row = 7 * 8; // 7 columns * 8 bytes
    state.SetBytesProcessed(state.iterations() * 1000 * bytes_per_row);
    state.SetItemsProcessed(state.iterations() * 1000);
    state.counters["rows_per_sec"] = benchmark::Counter(1000, benchmark::Counter::kIsRate);
}

BENCHMARK(BM_IMU_1000Hz)
    ->Arg(100)   // batch_size = 100
    ->Arg(500)   // batch_size = 500
    ->Arg(1000)  // batch_size = 1000
    ->Unit(benchmark::kMillisecond);

// Benchmark: Camera frames at 30Hz with different compression ratios
static void BM_Camera_30Hz(benchmark::State& state) {
    const size_t rgb_size = state.range(0);
    const size_t depth_size = state.range(1);
    auto schema = create_image_schema();
    
    for (auto _ : state) {
        state.PauseTiming();
        g_write_count = 0;
        g_total_rows = 0;
        
        BatchManager manager(30, 1000, noop_write_callback);
        manager.set_dataset("/tmp/bench.lance", 1);
        manager.initialize_schema(schema);
        manager.start();
        state.ResumeTiming();
        
        // Simulate 1 second of 30Hz data from 3 cameras
        for (int frame = 0; frame < 30; ++frame) {
            for (int cam = 0; cam < 3; ++cam) {
                auto row = create_image_row(
                    frame * 33333333 + cam * 1000000,
                    cam,
                    rgb_size,
                    depth_size
                );
                manager.add_row(row);
            }
        }
        
        state.PauseTiming();
        manager.stop();
        state.ResumeTiming();
    }
    
    // 90 frames total (30 Hz * 3 cameras)
    const size_t total_bytes = 90 * (rgb_size + depth_size + 32);
    state.SetBytesProcessed(state.iterations() * total_bytes);
    state.SetItemsProcessed(state.iterations() * 90);
}

BENCHMARK(BM_Camera_30Hz)
    ->Args({230000, 150000})    // JPEG ~10:1 compression
    ->Args({575000, 375000})    // ZSTD ~4:1 compression
    ->Args({1150000, 750000})   // LZ4 ~2:1 compression
    ->Unit(benchmark::kMillisecond);

// Benchmark: Mixed workload (IMU + cameras)
static void BM_MixedWorkload(benchmark::State& state) {
    auto imu_schema = create_imu_schema();
    auto image_schema = create_image_schema();
    
    for (auto _ : state) {
        state.PauseTiming();
        g_write_count = 0;
        g_total_rows = 0;
        
        // Create two batch managers (real scenario)
        BatchManager imu_manager(1000, 1000, noop_write_callback);
        imu_manager.set_dataset("/tmp/imu.lance", 1);
        imu_manager.initialize_schema(imu_schema);
        
        BatchManager image_manager(30, 1000, noop_write_callback);
        image_manager.set_dataset("/tmp/images.lance", 2);
        image_manager.initialize_schema(image_schema);
        
        imu_manager.start();
        image_manager.start();
        state.ResumeTiming();
        
        // Simulate 1 second of mixed data
        for (int64_t ms = 0; ms < 1000; ++ms) {
            // IMU at every millisecond
            auto imu_row = create_imu_row(ms * 1000000);
            imu_manager.add_row(imu_row);
            
            // Camera at ~33ms intervals (30Hz)
            if (ms % 33 == 0) {
                for (int cam = 0; cam < 3; ++cam) {
                    auto img_row = create_image_row(
                        ms * 1000000 + cam * 1000,
                        cam,
                        230000,  // JPEG compressed RGB
                        150000   // ZSTD compressed depth
                    );
                    image_manager.add_row(img_row);
                }
            }
        }
        
        state.PauseTiming();
        imu_manager.stop();
        image_manager.stop();
        state.ResumeTiming();
    }
    
    // Calculate total throughput
    const size_t imu_bytes = 1000 * 7 * 8;
    const size_t image_bytes = 90 * (230000 + 150000 + 32);
    state.SetBytesProcessed(state.iterations() * (imu_bytes + image_bytes));
}

BENCHMARK(BM_MixedWorkload)->Unit(benchmark::kMillisecond);

// Benchmark: BatchQueue throughput
static void BM_BatchQueue_Throughput(benchmark::State& state) {
    const int num_producers = state.range(0);
    const int items_per_producer = 1000;
    
    for (auto _ : state) {
        BatchQueue queue;
        std::atomic<int> consumed{0};
        
        auto schema = arrow::schema({arrow::field("value", arrow::int64())});
        auto batch = arrow::RecordBatch::MakeEmpty(schema).ValueOrDie();
        
        // Consumer thread
        std::thread consumer([&]() {
            BatchQueue::BatchItem item;
            while (consumed < num_producers * items_per_producer) {
                if (queue.pop(item, 10)) {
                    consumed++;
                }
            }
        });
        
        // Producer threads
        std::vector<std::thread> producers;
        for (int p = 0; p < num_producers; ++p) {
            producers.emplace_back([&, p]() {
                for (int i = 0; i < items_per_producer; ++i) {
                    BatchQueue::BatchItem item(batch, "/test", p);
                    queue.push(std::move(item));
                }
            });
        }
        
        // Wait for completion
        for (auto& t : producers) {
            t.join();
        }
        consumer.join();
    }
    
    state.SetItemsProcessed(state.iterations() * num_producers * items_per_producer);
}

BENCHMARK(BM_BatchQueue_Throughput)
    ->Arg(1)   // 1 producer
    ->Arg(4)   // 4 producers
    ->Arg(8)   // 8 producers
    ->Unit(benchmark::kMillisecond);

// Benchmark: add_row latency
static void BM_AddRow_Latency(benchmark::State& state) {
    auto schema = create_imu_schema();
    
    BatchManager manager(10000, 60000, noop_write_callback);
    manager.set_dataset("/tmp/bench.lance", 1);
    manager.initialize_schema(schema);
    manager.start();
    
    int64_t timestamp = 0;
    for (auto _ : state) {
        auto row = create_imu_row(timestamp++);
        manager.add_row(row);
    }
    
    manager.stop();
    
    state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BM_AddRow_Latency)->Unit(benchmark::kMicrosecond);

// =============================================================================
// Compression Benchmarks
// =============================================================================

// Generate test image data (patterned for realistic compression)
std::vector<uint8_t> generate_test_image(int width, int height, int channels) {
    std::vector<uint8_t> data(width * height * channels);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> noise(-10, 10);
    
    // Create gradient pattern with noise (simulates real camera data)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int base = (x + y) % 256;
            for (int c = 0; c < channels; ++c) {
                int idx = (y * width + x) * channels + c;
                data[idx] = std::clamp(base + noise(rng) + c * 30, 0, 255);
            }
        }
    }
    return data;
}

// Benchmark: Compression codecs comparison
static void BM_Compression(benchmark::State& state) {
    const int width = 1920;
    const int height = 1080;
    const int channels = 3;
    auto test_data = generate_test_image(width, height, channels);
    
    CompressionCodec codec = static_cast<CompressionCodec>(state.range(0));
    auto compressor = CompressorFactory::create(codec);
    
    ImageData img(test_data.data(), test_data.size(), width, height, channels);
    std::vector<uint8_t> compressed;
    
    for (auto _ : state) {
        CompressionStats stats;
        compressor->compress(img, compressed, &stats);
        benchmark::DoNotOptimize(compressed.data());
    }
    
    state.SetBytesProcessed(state.iterations() * test_data.size());
    state.counters["ratio"] = static_cast<double>(test_data.size()) / compressed.size();
}

BENCHMARK(BM_Compression)
    ->Arg(static_cast<int>(CompressionCodec::NONE))
    ->Arg(static_cast<int>(CompressionCodec::LZ4))
    ->Arg(static_cast<int>(CompressionCodec::ZSTD))
    ->Arg(static_cast<int>(CompressionCodec::JPEG))
    ->Unit(benchmark::kMillisecond);

// Benchmark: Full pipeline (compress + batch)
static void BM_CompressAndBatch(benchmark::State& state) {
    const int width = 1920;
    const int height = 1080;
    auto test_image = generate_test_image(width, height, 3);
    auto test_depth = generate_test_image(width, height, 1);
    
    // Compress once to get typical sizes
    auto jpeg = std::make_unique<JpegCompressor>(85);
    auto zstd = std::make_unique<ZstdCompressor>(3);
    
    std::vector<uint8_t> compressed_rgb, compressed_depth;
    ImageData rgb_img(test_image.data(), test_image.size(), width, height, 3);
    ImageData depth_img(test_depth.data(), test_depth.size(), width, height, 1);
    
    jpeg->compress(rgb_img, compressed_rgb);
    zstd->compress(depth_img, compressed_depth);
    
    auto schema = create_image_schema();
    
    for (auto _ : state) {
        state.PauseTiming();
        BatchManager manager(30, 1000, noop_write_callback);
        manager.set_dataset("/tmp/bench.lance", 1);
        manager.initialize_schema(schema);
        manager.start();
        state.ResumeTiming();
        
        // Process 30 frames (1 second at 30Hz)
        for (int frame = 0; frame < 30; ++frame) {
            // Compress
            jpeg->compress(rgb_img, compressed_rgb);
            zstd->compress(depth_img, compressed_depth);
            
            // Create row with compressed data
            std::vector<std::shared_ptr<arrow::Array>> arrays;
            
            auto ts_builder = std::make_shared<arrow::Int64Builder>();
            ts_builder->Append(frame * 33333333);
            std::shared_ptr<arrow::Array> ts_array;
            ts_builder->Finish(&ts_array);
            arrays.push_back(ts_array);
            
            auto cam_builder = std::make_shared<arrow::Int64Builder>();
            cam_builder->Append(0);
            std::shared_ptr<arrow::Array> cam_array;
            cam_builder->Finish(&cam_array);
            arrays.push_back(cam_array);
            
            auto w_builder = std::make_shared<arrow::Int64Builder>();
            w_builder->Append(width);
            std::shared_ptr<arrow::Array> w_array;
            w_builder->Finish(&w_array);
            arrays.push_back(w_array);
            
            auto h_builder = std::make_shared<arrow::Int64Builder>();
            h_builder->Append(height);
            std::shared_ptr<arrow::Array> h_array;
            h_builder->Finish(&h_array);
            arrays.push_back(h_array);
            
            auto rgb_builder = std::make_shared<arrow::BinaryBuilder>();
            rgb_builder->Append(compressed_rgb.data(), compressed_rgb.size());
            std::shared_ptr<arrow::Array> rgb_array;
            rgb_builder->Finish(&rgb_array);
            arrays.push_back(rgb_array);
            
            auto depth_builder = std::make_shared<arrow::BinaryBuilder>();
            depth_builder->Append(compressed_depth.data(), compressed_depth.size());
            std::shared_ptr<arrow::Array> depth_array;
            depth_builder->Finish(&depth_array);
            arrays.push_back(depth_array);
            
            manager.add_row(arrays);
        }
        
        state.PauseTiming();
        manager.stop();
        state.ResumeTiming();
    }
    
    const size_t raw_bytes = 30 * (test_image.size() + test_depth.size());
    state.SetBytesProcessed(state.iterations() * raw_bytes);
    state.counters["frames_per_sec"] = benchmark::Counter(30, benchmark::Counter::kIsRate);
}

BENCHMARK(BM_CompressAndBatch)->Unit(benchmark::kMillisecond);

// =============================================================================
// CPU Usage Monitoring Benchmark
// =============================================================================

#ifdef __linux__
#include <sys/resource.h>
#include <fstream>

double get_cpu_usage() {
    static long prev_total = 0;
    static long prev_idle = 0;
    
    std::ifstream stat_file("/proc/stat");
    std::string line;
    std::getline(stat_file, line);
    
    // Parse CPU line: cpu user nice system idle iowait irq softirq
    long user, nice, system, idle, iowait, irq, softirq;
    sscanf(line.c_str(), "cpu %ld %ld %ld %ld %ld %ld %ld",
           &user, &nice, &system, &idle, &iowait, &irq, &softirq);
    
    long total = user + nice + system + idle + iowait + irq + softirq;
    long total_diff = total - prev_total;
    long idle_diff = idle - prev_idle;
    
    prev_total = total;
    prev_idle = idle;
    
    if (total_diff == 0) return 0.0;
    return 100.0 * (1.0 - static_cast<double>(idle_diff) / total_diff);
}
#elif __APPLE__
#include <mach/mach.h>

double get_cpu_usage() {
    // macOS implementation using Mach APIs
    mach_msg_type_number_t count = HOST_CPU_LOAD_INFO_COUNT;
    host_cpu_load_info_data_t cpu_load;
    
    if (host_statistics(mach_host_self(), HOST_CPU_LOAD_INFO,
                       (host_info_t)&cpu_load, &count) != KERN_SUCCESS) {
        return 0.0;
    }
    
    static uint64_t prev_total = 0;
    static uint64_t prev_idle = 0;
    
    uint64_t user = cpu_load.cpu_ticks[CPU_STATE_USER];
    uint64_t system = cpu_load.cpu_ticks[CPU_STATE_SYSTEM];
    uint64_t idle = cpu_load.cpu_ticks[CPU_STATE_IDLE];
    uint64_t nice = cpu_load.cpu_ticks[CPU_STATE_NICE];
    
    uint64_t total = user + system + idle + nice;
    uint64_t total_diff = total - prev_total;
    uint64_t idle_diff = idle - prev_idle;
    
    prev_total = total;
    prev_idle = idle;
    
    if (total_diff == 0) return 0.0;
    return 100.0 * (1.0 - static_cast<double>(idle_diff) / total_diff);
}
#else
double get_cpu_usage() { return 0.0; }
#endif

// Benchmark that reports CPU usage
static void BM_CPU_Usage_Recording(benchmark::State& state) {
    auto imu_schema = create_imu_schema();
    
    double total_cpu = 0;
    int samples = 0;
    
    for (auto _ : state) {
        BatchManager manager(1000, 1000, noop_write_callback);
        manager.set_dataset("/tmp/bench.lance", 1);
        manager.initialize_schema(imu_schema);
        manager.start();
        
        // Record CPU usage baseline
        get_cpu_usage();  // Reset
        
        // Simulate 100ms of recording at 1000Hz
        for (int64_t i = 0; i < 100; ++i) {
            auto row = create_imu_row(i * 1000000);
            manager.add_row(row);
            std::this_thread::sleep_for(std::chrono::microseconds(900));
        }
        
        total_cpu += get_cpu_usage();
        samples++;
        
        manager.stop();
    }
    
    state.counters["cpu_percent"] = total_cpu / samples;
}

BENCHMARK(BM_CPU_Usage_Recording)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(5);

BENCHMARK_MAIN();
