# Core Library for Schiebung

This crate contains the pure Rust core functionality of the Schiebung library.
It provides a buffer for storing and retrieving transforms without any Python dependencies.

## Installation

```bash
git clone git@github.com:MaxiMaerz/schiebung.git
cd schiebung
cargo build
```

## Usage

This will create a local buffer, this buffer will NOT fill itself!

```rust
use schiebung_core::BufferTree;

let buffer = BufferTree::new();
let stamped_isometry = StampedIsometry {
    isometry: Isometry::from_parts(
        Translation3::new(
            1.0,
            2.0,
            3.0,
        ),
        UnitQuaternion::new_normalize(Quaternion::new(
            0.0,
            0.0,
            0.0,
            1.0,
        )),
    ),
    stamp: 1_000_000_000  // nanoseconds (1 second)
};
buffer.update("base_link", "target_link", stamped_isometry, TransformType::Static);

let transform = buffer.lookup_transform("base_link", "target_link", 1.0);
```

## How it works

### Update the Buffer

Each isometry sent to the buffer is, after validation, stored in an Directed Acyclic Graph (DAG). We use petgraph for the Graph implementation. We extend it with a Hashmap to lookup the human readable names of the nodes (Which is currently the biggest bottleneck).Currently we check if the graph becomes cyclic and if so, we reject the update.

If the update is the first update a TransformHistory is created and stored in the graph. We also check for cyclicality here, store all ancestors for this node and update the ancestors of the children of this node. This means the first update is rather expensive. All subsequent updates are just appended to existing TransformHistory.

We differentiate between static and dynamic transforms. Static transforms are stored in the graph as a single transform, dynamic transforms are stored in a TransformHistory.

Currently the size of the history is limited to a reasonable amount of 120 seconds, this can be changed in the config file.
There is **NO** safeguard against pushing to many updates to the buffer, e.g. if the time does not advance fast enough. This will lead to a large memory footprint. Reasonable frequencies of 1-1000 Hz have been tested more might be possible.

### Lookup a Transform

After the buffer is filled, any transform can be requested, we will walk from the "from" frame to the "to" frame and chain the isometries resulting in the connection isometry from the "from" frame to the "to" frame. We support two lookup types:

1. lookup_latest_transform: This will return the latest stored transform for any TransformHistory in the chain and timestamp it to the latest timestamp in the chain.
2. lookup_transform: Here a stamp must be provided and the transform will be interpolated based on the TransformHistory in the chain. nalgebra's lerp_slerp is used for interpolation.

lookup_latest_transform will work if a path "from" to "to" exists in the graph.

lookup_transform will fail if:

* A path "from" to "to" does not exist in the graph
* If any link's oldest transform is newer than the requested timestamp
* If any link's newest transform is older than the requested timestamp


### Observer

It is possible to register an observer to the buffer, on registration the buffer sends the latest transform for each link in the graph to the observer. Afterwards the observer is notified whenever a transform is updated or added to the buffer.

### Visualizer

The visualize methods converts the graph into a graphviz dot string, if graphviz is installed we can save the graph as a pdf.

## Performance

The performance can be tested via:

```bash
cd core/schiebung-core-rs
cargo bench --bench buffer_benchmark
```

### Summary

The performance was measured on a AMD Ryzen 7 PRO 8840HS under reasonable system load.

A matrix multiplication takes about 12ns with nalgebra, our call takes around 40 ns.
An interpolated lookup takes around 70 ns.

The path length should scale linearly with the number of nodes. Which makes sense since we multiply more matrices.
Runtime is not affected by the size of the transform history.

After some benchmarking, we found that the our bottleneck is the hash map lookup from frame names to nodes. We could provide a deeper API to avoid this, but the performance is already good enough for most use cases.

A selected but not exhaustive set of benchmarks is shown in the table below:

### Core Operations

| Operation                     | Time (ns) | Description                      |
|-----------|-----------|-------------|
| Update (new edge, static) | 498 | Creating a new transform link |
| Update (existing edge, dynamic) | 56 | Appending to existing history |
| Lookup (simple, interpolated) | 63 | Interpolated lookup, 2 frames |
| Lookup (latest) | 43 | Latest transform, 2 frames |
| Path finding (2 nodes) | 49 | Simple path traversal |
| Path finding (100 nodes) | 2,375 | Deep tree traversal |
| Lookup (100 edge chain) | 2,751 | Transform chain through 100 edges |

### TransformHistory Scaling

Update performance remains **constant** regardless of history size:

| History Size | Update Time (ns) | Notes        |
|--------------|------------------|-------|
| 100 | 55.5 | |
| 1,000 | 54.9 | |
| 10,000 | 52.8 | |
| 60,000 | 53.0 | 1 min @ 1kHz |
| 120,000 | 53.4 | 2 min @ 1kHz |

Lookup performance shows **minimal degradation** with history size:

| History Size | Interpolated Lookup (ns) | Latest Lookup (ns) |
|--------------|--------------------------|-------------------|
| 100 | 64 | 44 |
| 1,000 | 67 | 44 |
| 10,000 | 75 | 45 |
| 60,000 | 77 | 49 |
| 120,000      | 79                       | 44                 |
