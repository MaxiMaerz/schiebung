use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::Isometry3;
use schiebung::{BufferTree, StampedIsometry, TransformType};
use std::hint::black_box;

// ============================================================================
// Setup Helpers
// ============================================================================

/// Create a simple 2-frame buffer (A -> B) with dynamic transforms
fn setup_simple_dynamic_buffer() -> BufferTree {
    let mut buffer = BufferTree::new();

    // Fill buffer with 100 timestamped transforms
    for i in 0..100 {
        let isometry = StampedIsometry::new(
            [i as f64, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            (i as i64) * 100_000_000,
        );
        buffer
            .update("frame_a", "frame_b", isometry, TransformType::Dynamic)
            .unwrap();
    }

    buffer
}

/// Create a simple 2-frame buffer (A -> B) with static transform
fn setup_simple_static_buffer() -> BufferTree {
    let mut buffer = BufferTree::new();

    let isometry = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
    buffer
        .update("frame_a", "frame_b", isometry, TransformType::Static)
        .unwrap();

    buffer
}

/// Create a deep tree: A -> N1 -> N2 -> ... -> N99 -> E (100 edges total)
fn setup_deep_tree() -> BufferTree {
    let mut buffer = BufferTree::new();

    for i in 0..100 {
        let isometry = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
        let target_name = if i == 99 {
            "E".to_string()
        } else {
            format!("N{}", i + 1)
        };
        let source_name = if i == 0 {
            "A".to_string()
        } else {
            format!("N{}", i)
        };

        buffer
            .update(&source_name, &target_name, isometry, TransformType::Static)
            .unwrap();
    }

    buffer
}

/// Create a wide tree: root with N children
fn setup_wide_tree(num_children: usize) -> BufferTree {
    let mut buffer = BufferTree::new();

    for i in 0..num_children {
        let isometry = StampedIsometry::new([i as f64, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
        buffer
            .update(
                "root",
                &format!("child_{}", i),
                isometry,
                TransformType::Static,
            )
            .unwrap();
    }

    buffer
}

// ============================================================================
// Update Benchmarks
// ============================================================================

fn bench_update_new_edge(c: &mut Criterion) {
    c.bench_function("update/new_edge_static", |b| {
        b.iter_batched(
            || BufferTree::new(),
            |mut buffer| {
                let isometry = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
                buffer
                    .update(
                        black_box("frame_a"),
                        black_box("frame_b"),
                        black_box(isometry),
                        TransformType::Static,
                    )
                    .unwrap();
            },
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function("update/existing_edge_dynamic", |b| {
        let mut buffer = BufferTree::new();
        let isometry = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
        buffer
            .update("frame_a", "frame_b", isometry, TransformType::Dynamic)
            .unwrap();

        let mut t = 1_000_000_000i64;
        b.iter(|| {
            t += 100_000_000;
            let isometry = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], t);
            buffer
                .update(
                    black_box("frame_a"),
                    black_box("frame_b"),
                    black_box(isometry),
                    TransformType::Dynamic,
                )
                .unwrap();
        })
    });
}

// ============================================================================
// Lookup Transform Benchmarks
// ============================================================================

fn bench_lookup_transform(c: &mut Criterion) {
    let buffer = setup_simple_dynamic_buffer();

    c.bench_function("lookup_transform/simple_interpolated", |b| {
        b.iter(|| {
            buffer
                .lookup_transform(
                    black_box("frame_a"),
                    black_box("frame_b"),
                    black_box(5_000_000_000),
                )
                .unwrap()
        })
    });

    let buffer_static = setup_simple_static_buffer();
    c.bench_function("lookup_transform/simple_static", |b| {
        b.iter(|| {
            buffer_static
                .lookup_transform(black_box("frame_a"), black_box("frame_b"), black_box(0))
                .unwrap()
        })
    });
}

fn bench_lookup_latest_transform(c: &mut Criterion) {
    let buffer = setup_simple_dynamic_buffer();

    c.bench_function("lookup_latest_transform/simple", |b| {
        b.iter(|| {
            buffer
                .lookup_latest_transform(black_box("frame_a"), black_box("frame_b"))
                .unwrap()
        })
    });
}

fn bench_lookup_deep_tree(c: &mut Criterion) {
    let buffer = setup_deep_tree();

    c.bench_function("lookup_latest_transform/deep_100_edges", |b| {
        b.iter(|| {
            buffer
                .lookup_latest_transform(black_box("A"), black_box("E"))
                .unwrap()
        })
    });

    c.bench_function("lookup_transform/deep_100_edges", |b| {
        b.iter(|| {
            buffer
                .lookup_transform(black_box("A"), black_box("E"), black_box(0))
                .unwrap()
        })
    });
}

fn bench_lookup_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("lookup_scaling");

    for depth in [2, 5, 10, 20, 50, 100].iter() {
        let mut buffer = BufferTree::new();
        for i in 0..*depth {
            let isometry = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
            let target = if i == depth - 1 {
                "end".to_string()
            } else {
                format!("n{}", i + 1)
            };
            let source = if i == 0 {
                "start".to_string()
            } else {
                format!("n{}", i)
            };
            buffer
                .update(&source, &target, isometry, TransformType::Static)
                .unwrap();
        }

        group.bench_with_input(BenchmarkId::from_parameter(depth), depth, |b, _| {
            b.iter(|| {
                buffer
                    .lookup_latest_transform(black_box("start"), black_box("end"))
                    .unwrap()
            })
        });
    }

    group.finish();
}

// ============================================================================
// Path Finding Benchmarks (Internal)
// ============================================================================

fn bench_path_finding(c: &mut Criterion) {
    // Note: find_path is private, so we benchmark it indirectly through lookup_latest_transform
    // but isolate just the path finding cost by using static transforms

    let buffer_simple = setup_simple_static_buffer();
    c.bench_function("path_finding/simple_2_nodes", |b| {
        b.iter(|| {
            // This isolates path finding + minimal transform computation
            buffer_simple
                .lookup_latest_transform(black_box("frame_a"), black_box("frame_b"))
                .unwrap()
        })
    });

    let buffer_deep = setup_deep_tree();
    c.bench_function("path_finding/deep_100_nodes", |b| {
        b.iter(|| {
            buffer_deep
                .lookup_latest_transform(black_box("A"), black_box("E"))
                .unwrap()
        })
    });

    // Wide tree: test LCA finding with many siblings
    let buffer_wide = setup_wide_tree(50);
    c.bench_function("path_finding/wide_50_siblings", |b| {
        b.iter(|| {
            buffer_wide
                .lookup_latest_transform(black_box("child_0"), black_box("child_49"))
                .unwrap()
        })
    });
}

// ============================================================================
// Baseline: Raw nalgebra Operations
// ============================================================================

fn bench_raw_nalgebra(c: &mut Criterion) {
    let transforms: Vec<Isometry3<f64>> = (0..100)
        .map(|i| Isometry3::translation(i as f64, 0.0, 0.0))
        .collect();

    c.bench_function("baseline/nalgebra_100_mults", |b| {
        b.iter(|| {
            let mut result = Isometry3::identity();
            for t in &transforms {
                result *= t;
            }
            black_box(result)
        })
    });

    c.bench_function("baseline/nalgebra_single_mult", |b| {
        let t1 = Isometry3::translation(1.0, 0.0, 0.0);
        let t2 = Isometry3::translation(0.0, 1.0, 0.0);
        b.iter(|| {
            let result = black_box(t1) * black_box(t2);
            black_box(result)
        })
    });
}

// ============================================================================
// TransformHistory Scaling Benchmarks
// ============================================================================

/// Create a buffer with a specified number of dynamic transforms
fn setup_buffer_with_history(num_entries: usize) -> BufferTree {
    let mut buffer = BufferTree::new();

    // Simulate 1kHz updates (1ms = 1_000_000 ns)
    for i in 0..num_entries {
        let timestamp = (i as i64) * 1_000_000; // 1ms intervals
        let isometry = StampedIsometry::new(
            [i as f64 * 0.001, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            timestamp,
        );
        buffer
            .update("frame_a", "frame_b", isometry, TransformType::Dynamic)
            .unwrap();
    }

    buffer
}

fn bench_history_update_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("history_update_scaling");

    // Test update performance as history grows
    // Sizes: 100, 1k, 10k, 60k (1 min @ 1kHz), 120k (2 min @ 1kHz)
    for size in [100, 1_000, 10_000, 60_000, 120_000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let mut buffer = setup_buffer_with_history(size);
            let mut t = (size as i64) * 1_000_000;

            b.iter(|| {
                t += 1_000_000; // Next millisecond
                let isometry =
                    StampedIsometry::new([t as f64 * 0.000001, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], t);
                buffer
                    .update(
                        black_box("frame_a"),
                        black_box("frame_b"),
                        black_box(isometry),
                        TransformType::Dynamic,
                    )
                    .unwrap();
            })
        });
    }

    group.finish();
}

fn bench_history_lookup_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("history_lookup_scaling");

    // Test lookup performance with different history sizes
    for size in [100, 1_000, 10_000, 60_000, 120_000].iter() {
        let buffer = setup_buffer_with_history(*size);

        // Lookup at middle of history (requires interpolation)
        let mid_timestamp = (*size as i64 / 2) * 1_000_000;

        group.bench_with_input(BenchmarkId::new("interpolated_mid", size), size, |b, _| {
            b.iter(|| {
                buffer
                    .lookup_transform(
                        black_box("frame_a"),
                        black_box("frame_b"),
                        black_box(mid_timestamp),
                    )
                    .unwrap()
            })
        });

        // Lookup at end of history (latest)
        group.bench_with_input(BenchmarkId::new("latest", size), size, |b, _| {
            b.iter(|| {
                buffer
                    .lookup_latest_transform(black_box("frame_a"), black_box("frame_b"))
                    .unwrap()
            })
        });

        // Lookup at beginning of history
        let start_timestamp = 0i64;
        group.bench_with_input(
            BenchmarkId::new("interpolated_start", size),
            size,
            |b, _| {
                b.iter(|| {
                    buffer
                        .lookup_transform(
                            black_box("frame_a"),
                            black_box("frame_b"),
                            black_box(start_timestamp),
                        )
                        .unwrap()
                })
            },
        );
    }

    group.finish();
}

fn bench_history_interpolation_positions(c: &mut Criterion) {
    let mut group = c.benchmark_group("history_interpolation_positions");

    // Use a large history (120k entries = 120 seconds @ 1kHz)
    let buffer = setup_buffer_with_history(120_000);
    let max_time = 120_000i64 * 1_000_000;

    // Test interpolation at different positions in the history
    for position_pct in [0, 10, 25, 50, 75, 90].iter() {
        let timestamp = (max_time * (*position_pct as i64)) / 100;

        group.bench_with_input(
            BenchmarkId::new("position_pct", position_pct),
            position_pct,
            |b, _| {
                b.iter(|| {
                    buffer
                        .lookup_transform(
                            black_box("frame_a"),
                            black_box("frame_b"),
                            black_box(timestamp),
                        )
                        .unwrap()
                })
            },
        );
    }

    group.finish();
}

// ============================================================================
// Criterion Groups
// ============================================================================

criterion_group!(update_benches, bench_update_new_edge,);

criterion_group!(
    lookup_benches,
    bench_lookup_transform,
    bench_lookup_latest_transform,
    bench_lookup_deep_tree,
    bench_lookup_scaling,
);

criterion_group!(path_benches, bench_path_finding,);

criterion_group!(
    history_benches,
    bench_history_update_scaling,
    bench_history_lookup_scaling,
    bench_history_interpolation_positions,
);

criterion_group!(baseline_benches, bench_raw_nalgebra,);

criterion_main!(
    update_benches,
    lookup_benches,
    path_benches,
    history_benches,
    baseline_benches
);
