//! Write throughput benchmarks for Lance writer
//!
//! Tests various write patterns to establish baseline throughput:
//! - Small batches (1000Hz sensor data simulation)
//! - Large batches (camera frame simulation)
//! - Concurrent writes

use criterion::{
    black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput,
};
use std::sync::Arc;

use arrow_array::{
    ArrayRef, BinaryArray, Float64Array, Int64Array, RecordBatch, TimestampNanosecondArray,
};
use arrow_schema::{DataType, Field, Schema, TimeUnit};
use tempfile::TempDir;

use axon_lance::LanceWriter;

/// Create a schema for IMU sensor data
fn create_imu_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new(
            "timestamp",
            DataType::Timestamp(TimeUnit::Nanosecond, None),
            false,
        ),
        Field::new("accel_x", DataType::Float64, false),
        Field::new("accel_y", DataType::Float64, false),
        Field::new("accel_z", DataType::Float64, false),
        Field::new("gyro_x", DataType::Float64, false),
        Field::new("gyro_y", DataType::Float64, false),
        Field::new("gyro_z", DataType::Float64, false),
    ]))
}

/// Create a batch of IMU data
fn create_imu_batch(schema: Arc<Schema>, num_rows: usize) -> RecordBatch {
    let timestamps: Vec<i64> = (0..num_rows as i64).map(|i| i * 1_000_000).collect(); // 1ms intervals
    let accel_x: Vec<f64> = (0..num_rows).map(|i| (i as f64) * 0.001).collect();
    let accel_y: Vec<f64> = (0..num_rows).map(|i| (i as f64) * 0.002).collect();
    let accel_z: Vec<f64> = (0..num_rows).map(|_| 9.81).collect();
    let gyro_x: Vec<f64> = (0..num_rows).map(|i| (i as f64) * 0.0001).collect();
    let gyro_y: Vec<f64> = (0..num_rows).map(|i| (i as f64) * 0.0002).collect();
    let gyro_z: Vec<f64> = (0..num_rows).map(|i| (i as f64) * 0.0003).collect();

    RecordBatch::try_new(
        schema,
        vec![
            Arc::new(TimestampNanosecondArray::from(timestamps)) as ArrayRef,
            Arc::new(Float64Array::from(accel_x)),
            Arc::new(Float64Array::from(accel_y)),
            Arc::new(Float64Array::from(accel_z)),
            Arc::new(Float64Array::from(gyro_x)),
            Arc::new(Float64Array::from(gyro_y)),
            Arc::new(Float64Array::from(gyro_z)),
        ],
    )
    .unwrap()
}

/// Create a schema for camera image data
fn create_image_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new(
            "timestamp",
            DataType::Timestamp(TimeUnit::Nanosecond, None),
            false,
        ),
        Field::new("camera_id", DataType::Int64, false),
        Field::new("width", DataType::Int64, false),
        Field::new("height", DataType::Int64, false),
        Field::new("rgb_data", DataType::Binary, false),
        Field::new("depth_data", DataType::Binary, false),
    ]))
}

/// Create a batch of image data (simulated compressed data)
fn create_image_batch(
    schema: Arc<Schema>,
    num_frames: usize,
    rgb_size: usize,
    depth_size: usize,
) -> RecordBatch {
    let timestamps: Vec<i64> = (0..num_frames as i64)
        .map(|i| i * 33_333_333) // ~30Hz
        .collect();
    let camera_ids: Vec<i64> = (0..num_frames).map(|i| (i % 3) as i64).collect();
    let widths: Vec<i64> = vec![1920; num_frames];
    let heights: Vec<i64> = vec![1080; num_frames];

    // Simulate compressed image data
    let rgb_data: Vec<&[u8]> = (0..num_frames)
        .map(|_| vec![0u8; rgb_size].leak() as &[u8])
        .collect();
    let depth_data: Vec<&[u8]> = (0..num_frames)
        .map(|_| vec![0u8; depth_size].leak() as &[u8])
        .collect();

    RecordBatch::try_new(
        schema,
        vec![
            Arc::new(TimestampNanosecondArray::from(timestamps)) as ArrayRef,
            Arc::new(Int64Array::from(camera_ids)),
            Arc::new(Int64Array::from(widths)),
            Arc::new(Int64Array::from(heights)),
            Arc::new(BinaryArray::from(rgb_data)),
            Arc::new(BinaryArray::from(depth_data)),
        ],
    )
    .unwrap()
}

/// Benchmark small batch writes (simulating 1000Hz IMU data)
fn bench_imu_write_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("imu_write_throughput");

    // Test different batch sizes
    for batch_size in [100, 500, 1000, 5000].iter() {
        let schema = create_imu_schema();
        let batch = create_imu_batch(schema.clone(), *batch_size);
        let bytes_per_row = 7 * 8; // 7 float64 columns
        let total_bytes = (*batch_size * bytes_per_row) as u64;

        group.throughput(Throughput::Bytes(total_bytes));
        group.bench_with_input(
            BenchmarkId::new("batch_size", batch_size),
            batch_size,
            |b, _| {
                b.iter_batched(
                    || {
                        let temp_dir = TempDir::new().unwrap();
                        let path = temp_dir.path().join("imu_bench.lance");
                        let handle =
                            LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone())
                                .unwrap();
                        (temp_dir, handle, batch.clone())
                    },
                    |(temp_dir, handle, batch)| {
                        LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                        LanceWriter::close(handle).unwrap();
                        drop(temp_dir);
                    },
                    criterion::BatchSize::SmallInput,
                );
            },
        );
    }
    group.finish();
}

/// Benchmark large batch writes (simulating camera frames)
fn bench_image_write_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("image_write_throughput");
    group.sample_size(20); // Fewer samples for expensive benchmarks

    // Test with different compression ratios (simulated)
    // Real JPEG ~10:1, LZ4 ~2:1, ZSTD ~4:1
    let compression_scenarios = [
        ("jpeg_10x", 230_000, 150_000),    // ~230KB RGB, ~150KB depth (10:1)
        ("lz4_2x", 1_150_000, 750_000),    // ~1.15MB RGB, ~750KB depth (2:1)
        ("zstd_4x", 575_000, 375_000),     // ~575KB RGB, ~375KB depth (4:1)
        ("raw", 2_300_000, 1_500_000),     // ~2.3MB RGB, ~1.5MB depth (raw)
    ];

    for (name, rgb_size, depth_size) in compression_scenarios {
        let schema = create_image_schema();
        let num_frames = 30; // 1 second of 30Hz data
        let batch = create_image_batch(schema.clone(), num_frames, rgb_size, depth_size);
        let total_bytes = (num_frames * (rgb_size + depth_size + 32)) as u64;

        group.throughput(Throughput::Bytes(total_bytes));
        group.bench_with_input(
            BenchmarkId::new("compression", name),
            &(rgb_size, depth_size),
            |b, _| {
                b.iter_batched(
                    || {
                        let temp_dir = TempDir::new().unwrap();
                        let path = temp_dir.path().join("image_bench.lance");
                        let handle =
                            LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone())
                                .unwrap();
                        (temp_dir, handle, batch.clone())
                    },
                    |(temp_dir, handle, batch)| {
                        LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                        LanceWriter::close(handle).unwrap();
                        drop(temp_dir);
                    },
                    criterion::BatchSize::SmallInput,
                );
            },
        );
    }
    group.finish();
}

/// Benchmark multiple sequential writes (simulating continuous recording)
fn bench_continuous_write(c: &mut Criterion) {
    let mut group = c.benchmark_group("continuous_write");
    group.sample_size(10);

    let schema = create_imu_schema();
    let batch = create_imu_batch(schema.clone(), 1000); // 1 second at 1000Hz
    let bytes_per_batch = (1000 * 7 * 8) as u64;

    // Test writing multiple batches in sequence
    for num_batches in [10, 50, 100].iter() {
        let total_bytes = bytes_per_batch * (*num_batches as u64);

        group.throughput(Throughput::Bytes(total_bytes));
        group.bench_with_input(
            BenchmarkId::new("batches", num_batches),
            num_batches,
            |b, &num_batches| {
                b.iter_batched(
                    || {
                        let temp_dir = TempDir::new().unwrap();
                        let path = temp_dir.path().join("continuous_bench.lance");
                        let handle =
                            LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone())
                                .unwrap();
                        let batches: Vec<_> = (0..num_batches).map(|_| batch.clone()).collect();
                        (temp_dir, handle, batches)
                    },
                    |(temp_dir, handle, batches)| {
                        for batch in batches {
                            LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                        }
                        LanceWriter::close(handle).unwrap();
                        drop(temp_dir);
                    },
                    criterion::BatchSize::SmallInput,
                );
            },
        );
    }
    group.finish();
}

/// Benchmark mixed workload (IMU + images)
fn bench_mixed_workload(c: &mut Criterion) {
    let mut group = c.benchmark_group("mixed_workload");
    group.sample_size(10);

    let imu_schema = create_imu_schema();
    let image_schema = create_image_schema();

    // 1 second of data: 1000 IMU samples + 30 camera frames
    let imu_batch = create_imu_batch(imu_schema.clone(), 1000);
    let image_batch = create_image_batch(image_schema.clone(), 30, 230_000, 150_000);

    let imu_bytes = (1000 * 7 * 8) as u64;
    let image_bytes = (30 * (230_000 + 150_000 + 32)) as u64;
    let total_bytes = imu_bytes + image_bytes;

    group.throughput(Throughput::Bytes(total_bytes));
    group.bench_function("1_second_mixed", |b| {
        b.iter_batched(
            || {
                let temp_dir = TempDir::new().unwrap();
                let imu_path = temp_dir.path().join("imu.lance");
                let image_path = temp_dir.path().join("images.lance");

                let imu_handle =
                    LanceWriter::create_or_open(imu_path.to_str().unwrap(), imu_schema.clone())
                        .unwrap();
                let image_handle =
                    LanceWriter::create_or_open(image_path.to_str().unwrap(), image_schema.clone())
                        .unwrap();

                (
                    temp_dir,
                    imu_handle,
                    image_handle,
                    imu_batch.clone(),
                    image_batch.clone(),
                )
            },
            |(temp_dir, imu_handle, image_handle, imu_batch, image_batch)| {
                LanceWriter::write_batch(imu_handle, black_box(imu_batch)).unwrap();
                LanceWriter::write_batch(image_handle, black_box(image_batch)).unwrap();
                LanceWriter::close(imu_handle).unwrap();
                LanceWriter::close(image_handle).unwrap();
                drop(temp_dir);
            },
            criterion::BatchSize::SmallInput,
        );
    });
    group.finish();
}

/// Benchmark latency of individual writes
fn bench_write_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("write_latency");

    let schema = create_imu_schema();

    // Single row write latency
    let single_row_batch = create_imu_batch(schema.clone(), 1);

    group.bench_function("single_row", |b| {
        let temp_dir = TempDir::new().unwrap();
        let path = temp_dir.path().join("latency_bench.lance");
        let handle =
            LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone()).unwrap();

        b.iter(|| {
            LanceWriter::write_batch(handle, black_box(single_row_batch.clone())).unwrap();
        });

        LanceWriter::close(handle).unwrap();
    });

    // Small batch write latency (100 rows)
    let small_batch = create_imu_batch(schema.clone(), 100);

    group.bench_function("100_rows", |b| {
        let temp_dir = TempDir::new().unwrap();
        let path = temp_dir.path().join("latency_bench_100.lance");
        let handle =
            LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone()).unwrap();

        b.iter(|| {
            LanceWriter::write_batch(handle, black_box(small_batch.clone())).unwrap();
        });

        LanceWriter::close(handle).unwrap();
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_imu_write_throughput,
    bench_image_write_throughput,
    bench_continuous_write,
    bench_mixed_workload,
    bench_write_latency,
);

criterion_main!(benches);
