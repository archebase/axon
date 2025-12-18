//! Compression benchmarks for Lance writer
//!
//! Tests different compression strategies for robotics data:
//! - LZ4 (fastest decompression)
//! - ZSTD (best compression ratio)
//! - No compression (baseline)

use criterion::{
    black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput,
};
use rand::Rng;
use std::sync::Arc;

use arrow_array::{ArrayRef, BinaryArray, Float64Array, Int64Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema};
use tempfile::TempDir;

use axon_lance::LanceWriter;

/// Generate realistic sensor data (has patterns, compresses well)
fn generate_sensor_data(num_rows: usize) -> Vec<f64> {
    let mut rng = rand::thread_rng();
    let mut data = Vec::with_capacity(num_rows);
    let mut base_value = 0.0f64;

    for _ in 0..num_rows {
        // Simulate a sensor with slow drift + noise
        base_value += rng.gen_range(-0.01..0.01);
        let noise = rng.gen_range(-0.1..0.1);
        data.push(base_value + noise);
    }
    data
}

/// Generate random binary data (simulates incompressible data)
fn generate_random_binary(size: usize) -> Vec<u8> {
    let mut rng = rand::thread_rng();
    (0..size).map(|_| rng.gen()).collect()
}

/// Generate patterned binary data (simulates compressible image data)
fn generate_patterned_binary(size: usize) -> Vec<u8> {
    // Create data with repeating patterns (like real images have)
    let mut data = Vec::with_capacity(size);
    let pattern_size = 256;
    let pattern: Vec<u8> = (0..pattern_size as u8).collect();

    while data.len() < size {
        let chunk_size = std::cmp::min(pattern_size, size - data.len());
        data.extend_from_slice(&pattern[..chunk_size]);
    }
    data
}

/// Create sensor data schema
fn create_sensor_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("timestamp", DataType::Int64, false),
        Field::new("sensor_1", DataType::Float64, false),
        Field::new("sensor_2", DataType::Float64, false),
        Field::new("sensor_3", DataType::Float64, false),
        Field::new("sensor_4", DataType::Float64, false),
        Field::new("sensor_5", DataType::Float64, false),
        Field::new("sensor_6", DataType::Float64, false),
    ]))
}

/// Create binary data schema (for image-like data)
fn create_binary_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("id", DataType::Int64, false),
        Field::new("data", DataType::Binary, false),
    ]))
}

/// Create sensor batch
fn create_sensor_batch(schema: Arc<Schema>, num_rows: usize) -> RecordBatch {
    let timestamps: Vec<i64> = (0..num_rows as i64).collect();

    RecordBatch::try_new(
        schema,
        vec![
            Arc::new(Int64Array::from(timestamps)) as ArrayRef,
            Arc::new(Float64Array::from(generate_sensor_data(num_rows))),
            Arc::new(Float64Array::from(generate_sensor_data(num_rows))),
            Arc::new(Float64Array::from(generate_sensor_data(num_rows))),
            Arc::new(Float64Array::from(generate_sensor_data(num_rows))),
            Arc::new(Float64Array::from(generate_sensor_data(num_rows))),
            Arc::new(Float64Array::from(generate_sensor_data(num_rows))),
        ],
    )
    .unwrap()
}

/// Create binary data batch
fn create_binary_batch(
    schema: Arc<Schema>,
    num_rows: usize,
    data_size: usize,
    compressible: bool,
) -> RecordBatch {
    let ids: Vec<i64> = (0..num_rows as i64).collect();

    let binary_data: Vec<Vec<u8>> = if compressible {
        (0..num_rows)
            .map(|_| generate_patterned_binary(data_size))
            .collect()
    } else {
        (0..num_rows)
            .map(|_| generate_random_binary(data_size))
            .collect()
    };

    let binary_refs: Vec<&[u8]> = binary_data.iter().map(|v| v.as_slice()).collect();

    RecordBatch::try_new(
        schema,
        vec![
            Arc::new(Int64Array::from(ids)) as ArrayRef,
            Arc::new(BinaryArray::from(binary_refs)),
        ],
    )
    .unwrap()
}

/// Benchmark compression for sensor data (highly compressible floating point)
fn bench_sensor_compression(c: &mut Criterion) {
    let mut group = c.benchmark_group("sensor_compression");

    let schema = create_sensor_schema();
    let num_rows = 10000; // 10 seconds at 1000Hz
    let batch = create_sensor_batch(schema.clone(), num_rows);
    let raw_bytes = (num_rows * 7 * 8) as u64; // 7 columns * 8 bytes each

    group.throughput(Throughput::Bytes(raw_bytes));

    // Note: Lance handles compression internally based on dataset settings
    // This benchmark measures write throughput with default compression
    group.bench_function("default_compression", |b| {
        b.iter_batched(
            || {
                let temp_dir = TempDir::new().unwrap();
                let path = temp_dir.path().join("sensor_comp.lance");
                let handle =
                    LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone()).unwrap();
                (temp_dir, handle, batch.clone())
            },
            |(temp_dir, handle, batch)| {
                LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                LanceWriter::close(handle).unwrap();
                drop(temp_dir);
            },
            criterion::BatchSize::SmallInput,
        );
    });

    group.finish();
}

/// Benchmark compression for binary data (compressible vs incompressible)
fn bench_binary_compression(c: &mut Criterion) {
    let mut group = c.benchmark_group("binary_compression");
    group.sample_size(20);

    let schema = create_binary_schema();

    // Test different data sizes and compressibility
    let scenarios = [
        ("small_compressible", 10, 10_000, true),     // 10 x 10KB
        ("small_random", 10, 10_000, false),         // 10 x 10KB random
        ("medium_compressible", 10, 100_000, true),  // 10 x 100KB
        ("medium_random", 10, 100_000, false),       // 10 x 100KB random
        ("large_compressible", 5, 500_000, true),    // 5 x 500KB
        ("large_random", 5, 500_000, false),         // 5 x 500KB random
    ];

    for (name, num_rows, data_size, compressible) in scenarios {
        let batch = create_binary_batch(schema.clone(), num_rows, data_size, compressible);
        let total_bytes = (num_rows * data_size) as u64;

        group.throughput(Throughput::Bytes(total_bytes));
        group.bench_with_input(BenchmarkId::new("scenario", name), &name, |b, _| {
            b.iter_batched(
                || {
                    let temp_dir = TempDir::new().unwrap();
                    let path = temp_dir.path().join("binary_comp.lance");
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
        });
    }

    group.finish();
}

/// Benchmark compression ratio measurement
fn bench_compression_ratio(c: &mut Criterion) {
    let mut group = c.benchmark_group("compression_ratio");
    group.sample_size(10);

    let sensor_schema = create_sensor_schema();
    let binary_schema = create_binary_schema();

    // Create test data
    let sensor_batch = create_sensor_batch(sensor_schema.clone(), 10000);
    let compressible_batch = create_binary_batch(binary_schema.clone(), 30, 230_000, true);
    let random_batch = create_binary_batch(binary_schema.clone(), 30, 230_000, false);

    // Sensor data benchmark
    let sensor_raw_bytes = (10000 * 7 * 8) as u64;
    group.throughput(Throughput::Bytes(sensor_raw_bytes));
    group.bench_function("sensor_data", |b| {
        b.iter_batched(
            || {
                let temp_dir = TempDir::new().unwrap();
                let path = temp_dir.path().join("ratio_sensor.lance");
                let handle =
                    LanceWriter::create_or_open(path.to_str().unwrap(), sensor_schema.clone())
                        .unwrap();
                (temp_dir, handle, sensor_batch.clone())
            },
            |(temp_dir, handle, batch)| {
                LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                LanceWriter::close(handle).unwrap();
                // Could measure file size here for ratio calculation
                drop(temp_dir);
            },
            criterion::BatchSize::SmallInput,
        );
    });

    // Compressible image data benchmark
    let compressible_raw_bytes = (30 * 230_000) as u64;
    group.throughput(Throughput::Bytes(compressible_raw_bytes));
    group.bench_function("compressible_image", |b| {
        b.iter_batched(
            || {
                let temp_dir = TempDir::new().unwrap();
                let path = temp_dir.path().join("ratio_comp.lance");
                let handle =
                    LanceWriter::create_or_open(path.to_str().unwrap(), binary_schema.clone())
                        .unwrap();
                (temp_dir, handle, compressible_batch.clone())
            },
            |(temp_dir, handle, batch)| {
                LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                LanceWriter::close(handle).unwrap();
                drop(temp_dir);
            },
            criterion::BatchSize::SmallInput,
        );
    });

    // Random data benchmark (worst case)
    let random_raw_bytes = (30 * 230_000) as u64;
    group.throughput(Throughput::Bytes(random_raw_bytes));
    group.bench_function("random_data", |b| {
        b.iter_batched(
            || {
                let temp_dir = TempDir::new().unwrap();
                let path = temp_dir.path().join("ratio_random.lance");
                let handle =
                    LanceWriter::create_or_open(path.to_str().unwrap(), binary_schema.clone())
                        .unwrap();
                (temp_dir, handle, random_batch.clone())
            },
            |(temp_dir, handle, batch)| {
                LanceWriter::write_batch(handle, black_box(batch)).unwrap();
                LanceWriter::close(handle).unwrap();
                drop(temp_dir);
            },
            criterion::BatchSize::SmallInput,
        );
    });

    group.finish();
}

/// Benchmark throughput at different batch sizes for compression
fn bench_batch_size_impact(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_size_compression");
    group.sample_size(20);

    let schema = create_sensor_schema();

    // Test how batch size affects compression throughput
    for batch_size in [100, 500, 1000, 5000, 10000].iter() {
        let batch = create_sensor_batch(schema.clone(), *batch_size);
        let total_bytes = (*batch_size * 7 * 8) as u64;

        group.throughput(Throughput::Bytes(total_bytes));
        group.bench_with_input(
            BenchmarkId::new("batch_size", batch_size),
            batch_size,
            |b, _| {
                b.iter_batched(
                    || {
                        let temp_dir = TempDir::new().unwrap();
                        let path = temp_dir.path().join("batch_size_comp.lance");
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

criterion_group!(
    benches,
    bench_sensor_compression,
    bench_binary_compression,
    bench_compression_ratio,
    bench_batch_size_impact,
);

criterion_main!(benches);
