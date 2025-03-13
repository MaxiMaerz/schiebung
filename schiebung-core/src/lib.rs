///! # Schiebung-core
///!
///! This crate contains the core functionality of the Schiebung library.
///! It provides a buffer for storing and retrieving transforms.
///!
///! NOTE: The Buffer must be filled manually and will not interface with ROS or any other system.
///!       Interfaces to ROS are provided by the `schiebung-ros2` crate.
///!
///! ## Usage
///!
///! ```rust
///! use schiebung_core::BufferTree;
///!
///! let buffer = BufferTree::new();
///!
///! let stamped_isometry = StampedIsometry {
///!     isometry: Isometry::from_parts(
///!         Translation3::new(
///!             1.0,
///!             2.0,
///!             3.0,
///!         ),
///!         UnitQuaternion::new_normalize(Quaternion::new(
///!             0.0,
///!             0.0,
///!             0.0,
///!             1.0,
///!         )),
///!     ),
///!     stamp: 1.0
///! };
///! buffer.update("base_link", "target_link", stamped_isometry, TransformType::Static);
///!
///! let transform = buffer.lookup_transform("base_link", "target_link", 1.0);
///! buffer.visualize();
///! ```
use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::Write;
use std::process::Command;

use log::info;
use nalgebra::geometry::Isometry3;
use petgraph::algo::is_cyclic_undirected;
use petgraph::graphmap::DiGraphMap;

pub mod types;
use crate::types::{StampedIsometry, TransformType};

mod config;
use crate::config::{get_config, BufferConfig};

/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// The graph is cyclic or the target has multiple incoming edges.
    InvalidGraph,
}

/// The TransformHistory keeps track of a single transform between two frames
/// Update pushes a new StampedTransform to the end, if the history reaches it's max length
/// The oldest transform is removed.
#[derive(Debug)]
struct TransformHistory {
    history: VecDeque<StampedIsometry>,
    kind: TransformType,
    max_history: usize,
}

impl TransformHistory {
    pub fn new(kind: TransformType, max_history: usize) -> Self {
        TransformHistory {
            history: VecDeque::new(),
            kind,
            max_history: max_history,
        }
    }

    pub fn update(&mut self, stamped_isometry: StampedIsometry) {
        self.history.push_back(stamped_isometry);
        if self.history.len() > self.max_history {
            self.history.pop_front();
        }
    }

    pub fn interpolate_isometry_at_time(&self, time: f64) -> Result<Isometry3<f64>, TfError> {
        match self.kind {
            TransformType::Static => {
                return Ok(self.history.back().unwrap().isometry);
            }
            TransformType::Dynamic => {
                if self.history.len() < 2 {
                    return Err(TfError::CouldNotFindTransform); // Not enough elements
                }

                let history = &self.history;
                let idx = history.binary_search_by(|entry| entry.stamp.partial_cmp(&time).unwrap());

                match idx {
                    Ok(i) => {
                        return Ok(history[i].isometry);
                    }
                    Err(i) => {
                        // Not found, i is the insertion point
                        if i == 0 {
                            return Err(TfError::AttemptedLookupInPast);
                        }
                        if i >= history.len() {
                            return Err(TfError::AttemptedLookUpInFuture);
                        } else {
                            let weight = (time - history[i - 1].stamp)
                                / (history[i].stamp - history[i - 1].stamp);
                            return Ok(history[i - 1]
                                .isometry
                                .lerp_slerp(&history[i].isometry, weight));
                        }
                    }
                }
            }
        }
    }
}

/// Need to index the strings via a hashmap
/// DiGrapMap does not support string indexing
struct NodeIndex {
    max_node_id: usize,
    node_ids: HashMap<String, usize>,
}

// DiGraphMap does not support strings and requires an external storage
// https://github.com/petgraph/petgraph/issues/325
impl NodeIndex {
    pub fn new() -> Self {
        NodeIndex {
            max_node_id: 0,
            node_ids: HashMap::new(),
        }
    }

    pub fn index(&mut self, node: String) -> usize {
        let ref mut max_node_id = self.max_node_id;

        let node_id = *self.node_ids.entry(node).or_insert_with(|| {
            let node_id = *max_node_id;
            *max_node_id += 1;
            node_id
        });

        node_id
    }

    pub fn contains(&self, node: &String) -> bool {
        self.node_ids.contains_key(node)
    }
}

/// The core BufferImplementation
/// The TF Graph is represented as a DiGraphMap:
/// This means the transforms build a acyclic direct graph
/// We check if the graph is acyclic or if the target has multiple incoming edges
/// We currently do NOT check if the graph is disconnected
/// The frame names are the nodes and the transform history is saved on the edges
pub struct BufferTree {
    graph: DiGraphMap<usize, TransformHistory>,
    index: NodeIndex,
    config: BufferConfig,
}

impl BufferTree {
    pub fn new() -> Self {
        BufferTree {
            graph: DiGraphMap::new(),
            index: NodeIndex::new(),
            config: get_config().unwrap(),
        }
    }

    /// Either update or push a transform to the graph
    /// Panics if the graph becomes cyclic
    pub fn update(
        &mut self,
        source: String,
        target: String,
        stamped_isometry: StampedIsometry,
        kind: TransformType,
    ) -> Result<(), TfError> {
        let source = self.index.index(source);
        let target = self.index.index(target);

        if !self.graph.contains_node(source) {
            self.graph.add_node(source);
        }
        if !self.graph.contains_node(target) {
            self.graph.add_node(target);
        }

        if !self.graph.contains_edge(source, target) {
            self.graph.add_edge(
                source,
                target,
                TransformHistory::new(kind, self.config.max_transform_history),
            );
            if is_cyclic_undirected(&self.graph)
                || self
                    .graph
                    .neighbors_directed(target, petgraph::Direction::Incoming)
                    .count()
                    > 1
            {
                // Remove the edge and nodes if they have no other edges
                self.graph.remove_edge(source, target);
                if self
                    .graph
                    .neighbors_directed(target, petgraph::Direction::Incoming)
                    .count()
                    < 1
                    && self
                        .graph
                        .neighbors_directed(target, petgraph::Direction::Outgoing)
                        .count()
                        < 1
                {
                    self.graph.remove_node(target);
                }
                if self
                    .graph
                    .neighbors_directed(source, petgraph::Direction::Incoming)
                    .count()
                    < 1
                    && self
                        .graph
                        .neighbors_directed(source, petgraph::Direction::Outgoing)
                        .count()
                        < 1
                {
                    self.graph.remove_node(source);
                }
                return Err(TfError::InvalidGraph);
            }
        }
        self.graph
            .edge_weight_mut(source, target)
            .unwrap()
            .update(stamped_isometry);
        Ok(())
    }

    /// Searches for a path in the graph
    /// We implement our own path search here because we have assumptions on the graph
    /// We have to consider that "form" and "to" are on different branches therefore we
    /// traverse the tree upwards from both nodes until we either hit the other node or the root
    /// Afterwards we prune the leftover path above the connection point
    pub fn find_path(&mut self, from: String, to: String) -> Option<Vec<usize>> {
        let mut path_1 = Vec::new();
        let mut path_2 = Vec::new();
        let mut from_idx = self.index.index(from);
        let mut to_idx = self.index.index(to);
        path_1.push(from_idx);
        path_2.push(to_idx);

        // Find all ancestors of from, return if to is an ancestor
        while let Some(parent) = self
            .graph
            .neighbors_directed(from_idx, petgraph::Direction::Incoming)
            .next()
        {
            // Break if to is ancestor
            if parent == to_idx {
                path_1.push(to_idx);
                return Some(path_1);
            }
            path_1.push(parent);
            from_idx = parent;
        }

        // Find all ancestors of to until one ancestor is in from
        while let Some(parent) = self
            .graph
            .neighbors_directed(to_idx, petgraph::Direction::Incoming)
            .next()
        {
            if path_1.contains(&parent) {
                // Remove elements above the common ancestor
                path_1.drain(path_1.iter().position(|x| *x == parent).unwrap() + 1..);
                break;
            }
            path_2.push(parent);
            to_idx = parent;
        }

        // Merge path on common ancestor
        path_2.reverse();
        path_1.append(&mut path_2);
        Some(path_1)
    }

    /// Lookup the latest transform without any checks
    /// This can be used for static transforms or if the user does not care if the
    /// transform is still valid.
    /// NOTE: This might give you outdated transforms!
    pub fn lookup_latest_transform(
        &mut self,
        source: String,
        target: String,
    ) -> Result<StampedIsometry, TfError> {
        let mut isometry = Isometry3::identity();
        if !self.index.contains(&source) || !self.index.contains(&target) {
            return Err(TfError::CouldNotFindTransform);
        }
        for pair in self.find_path(source, target).unwrap().windows(2) {
            let source_idx = pair[0];
            let target_idx = pair[1];

            if self.graph.contains_edge(source_idx, target_idx) {
                isometry *= self
                    .graph
                    .edge_weight(source_idx, target_idx)
                    .unwrap()
                    .history
                    .back()
                    .unwrap()
                    .isometry;
            } else {
                isometry *= self
                    .graph
                    .edge_weight(target_idx, source_idx)
                    .unwrap()
                    .history
                    .back()
                    .unwrap()
                    .isometry
                    .inverse();
            }
        }
        Ok(StampedIsometry {
            isometry,
            stamp: 0.0,
        })
    }

    /// Lookup the transform at time
    /// This will look for a transform at the provided time and can "time travel"
    /// If any edge contains a transform older then time a AttemptedLookupInPast is raised
    /// If the time is younger then any transform AttemptedLookUpInFuture is raised
    /// If there is no perfect match the transforms around this time are interpolated
    /// The interpolation is weighted with the distance to the time stamps
    pub fn lookup_transform(
        &mut self,
        source: String,
        target: String,
        time: f64,
    ) -> Result<StampedIsometry, TfError> {
        let mut isometry = Isometry3::identity();
        if !self.index.contains(&source) || !self.index.contains(&target) {
            return Err(TfError::CouldNotFindTransform);
        }
        for pair in self.find_path(source, target).unwrap().windows(2) {
            let source_idx = pair[0];
            let target_idx = pair[1];

            if self.graph.contains_edge(source_idx, target_idx) {
                isometry *= self
                    .graph
                    .edge_weight(source_idx, target_idx)
                    .unwrap()
                    .interpolate_isometry_at_time(time)?;
            } else {
                isometry *= self
                    .graph
                    .edge_weight(target_idx, source_idx)
                    .unwrap()
                    .interpolate_isometry_at_time(time)?
                    .inverse();
            }
        }
        Ok(StampedIsometry {
            isometry,
            stamp: time,
        })
    }

    /// Visualize the buffer tree as a DOT graph
    /// Can not use internal visualizer because we Store the nodes in self.index
    pub fn visualize(&self) -> String {
        // Create a mapping from index back to node name
        let reverse_index: HashMap<usize, &String> = self
            .index
            .node_ids
            .iter()
            .map(|(name, &id)| (id, name))
            .collect();

        // Convert the graph to DOT format manually
        let mut dot = String::from("digraph {\n");

        // Add nodes
        for node in self.graph.nodes() {
            let name = reverse_index.get(&node).unwrap();
            dot.push_str(&format!("    {} [label=\"{}\"]\n", node, name));
        }

        // Add edges with transform information
        for edge in self.graph.all_edges() {
            if let Some(latest) = edge.2.history.back() {
                let translation = latest.isometry.translation.vector;
                let rotation = latest.isometry.rotation.euler_angles();
                dot.push_str(&format!(
                    "    {} -> {} [label=\"t=[{:.3}, {:.3}, {:.3}]\\nr=[{:.3}, {:.3}, {:.3}]\\ntime={:.3}\"]\n",
                    edge.0, edge.1,
                    translation[0], translation[1], translation[2],
                    rotation.0, rotation.1, rotation.2,
                    latest.stamp
                ));
            } else {
                dot.push_str(&format!(
                    "    {} -> {} [label=\"No transforms\"]\n",
                    edge.0, edge.1
                ));
            }
        }

        dot.push_str("}");
        dot
    }

    /// Save the buffer tree as a PDF and dot file
    /// Runs graphiz to generate the PDF, fails if graphiz is not installed
    pub fn save_visualization(&self) -> std::io::Result<()> {
        let filename = &self.config.save_path;
        info!("Saving visualization to {}/graph.(dot/pdf)", filename);
        // Save DOT file
        let dot_content = self.visualize();
        let dot_filename = format!("{}/graph.dot", filename);
        let mut file = File::create(&dot_filename)?;
        file.write_all(dot_content.as_bytes())?;

        // Generate PDF using dot command
        let pdf_filename = format!("{}/graph.pdf", filename);
        let output = Command::new("dot")
            .args(["-Tpdf", &dot_filename, "-o", &pdf_filename])
            .output()?;

        if !output.status.success() {
            eprintln!(
                "Warning: Failed to generate PDF. Is Graphviz installed? Error: {}",
                String::from_utf8_lossy(&output.stderr)
            );
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::geometry::Isometry3;

    #[test]
    fn test_buffer_tree_update() {
        let mut buffer_tree = BufferTree::new();

        let source = "A".to_string();
        let target = "B".to_string();

        let stamped_isometry = StampedIsometry {
            isometry: Isometry3::identity(),
            stamp: 1.0,
        };

        // Add first transformation
        buffer_tree
            .update(
                source.clone(),
                target.clone(),
                stamped_isometry.clone(),
                TransformType::Static,
            )
            .unwrap();

        // Ensure the nodes exist
        let source_idx = buffer_tree.index.index(source.clone());
        let target_idx = buffer_tree.index.index(target.clone());

        assert!(buffer_tree.graph.contains_node(source_idx));
        assert!(buffer_tree.graph.contains_node(target_idx));

        // Ensure edge exists
        assert!(buffer_tree.graph.contains_edge(source_idx, target_idx));

        // Check that the transform history is updated
        let edge_weight = buffer_tree
            .graph
            .edge_weight(source_idx, target_idx)
            .unwrap();
        assert_eq!(edge_weight.history.len(), 1);
        assert_eq!(edge_weight.history.front().unwrap().stamp, 1.0);

        // Add another transformation
        let stamped_isometry_2 = StampedIsometry {
            isometry: Isometry3::identity(),
            stamp: 2.0,
        };
        buffer_tree
            .update(
                source.clone(),
                target.clone(),
                stamped_isometry_2.clone(),
                TransformType::Static,
            )
            .unwrap();

        // Ensure the history is updated
        let edge_weight = buffer_tree
            .graph
            .edge_weight(source_idx, target_idx)
            .unwrap();
        assert_eq!(edge_weight.history.len(), 2);
        assert_eq!(edge_weight.history.back().unwrap().stamp, 2.0);
    }

    #[test]
    fn test_buffer_tree_detects_cycles() {
        let mut buffer_tree = BufferTree::new();

        let a = "A".to_string();
        let b = "B".to_string();
        let c = "C".to_string();

        let stamped_isometry = StampedIsometry {
            isometry: Isometry3::identity(),
            stamp: 1.0,
        };

        // Add edges A → B and B → C
        buffer_tree
            .update(
                a.clone(),
                b.clone(),
                stamped_isometry.clone(),
                TransformType::Static,
            )
            .unwrap();
        buffer_tree
            .update(
                b.clone(),
                c.clone(),
                stamped_isometry.clone(),
                TransformType::Static,
            )
            .unwrap();

        // Creating a cycle C → A should panic
        let result = buffer_tree.update(
            c.clone(),
            a.clone(),
            stamped_isometry.clone(),
            TransformType::Static,
        );
        assert!(result.is_err());
        assert!(buffer_tree.graph.contains_node(buffer_tree.index.index(a)));
        assert!(buffer_tree.graph.contains_node(buffer_tree.index.index(b)));
        assert!(buffer_tree.graph.contains_node(buffer_tree.index.index(c)));
    }

    #[test]
    fn test_multiple_incoming_edges() {
        let mut buffer_tree = BufferTree::new();

        let a = "A".to_string();
        let b = "B".to_string();
        let c = "C".to_string();

        let stamped_isometry = StampedIsometry {
            isometry: Isometry3::identity(),
            stamp: 1.0,
        };

        buffer_tree
            .update(
                a.clone(),
                b.clone(),
                stamped_isometry.clone(),
                TransformType::Static,
            )
            .unwrap();
        let result = buffer_tree.update(
            c.clone(),
            b.clone(),
            stamped_isometry.clone(),
            TransformType::Static,
        );
        assert!(result.is_err());
        assert!(buffer_tree.graph.contains_node(buffer_tree.index.index(a)));
        assert!(buffer_tree.graph.contains_node(buffer_tree.index.index(b)));
        assert!(!buffer_tree.graph.contains_node(buffer_tree.index.index(c)));
    }

    #[test]
    fn test_find_path() {
        let mut buffer_tree = BufferTree::new();

        buffer_tree
            .update(
                "A".to_string(),
                "B".to_string(),
                StampedIsometry {
                    isometry: Isometry3::identity(),
                    stamp: 1.0,
                },
                TransformType::Dynamic,
            )
            .unwrap();

        buffer_tree
            .update(
                "A".to_string(),
                "C".to_string(),
                StampedIsometry {
                    isometry: Isometry3::identity(),
                    stamp: 2.0,
                },
                TransformType::Dynamic,
            )
            .unwrap();

        buffer_tree
            .update(
                "B".to_string(),
                "D".to_string(),
                StampedIsometry {
                    isometry: Isometry3::identity(),
                    stamp: 3.0,
                },
                TransformType::Dynamic,
            )
            .unwrap();

        buffer_tree
            .update(
                "B".to_string(),
                "E".to_string(),
                StampedIsometry {
                    isometry: Isometry3::identity(),
                    stamp: 3.0,
                },
                TransformType::Dynamic,
            )
            .unwrap();

        println!("{:?}", buffer_tree.visualize());

        let result = buffer_tree.find_path("D".to_string(), "B".to_string());
        assert_eq!(
            result,
            Some(vec![
                buffer_tree.index.index("D".to_string()),
                buffer_tree.index.index("B".to_string())
            ])
        );

        let result = buffer_tree.find_path("D".to_string(), "C".to_string());
        assert_eq!(
            result,
            Some(vec![
                buffer_tree.index.index("D".to_string()),
                buffer_tree.index.index("B".to_string()),
                buffer_tree.index.index("A".to_string()),
                buffer_tree.index.index("C".to_string()),
            ])
        );

        let result = buffer_tree.find_path("D".to_string(), "E".to_string());
        assert_eq!(
            result,
            Some(vec![
                buffer_tree.index.index("D".to_string()),
                buffer_tree.index.index("B".to_string()),
                buffer_tree.index.index("E".to_string()),
            ])
        );

        let result = buffer_tree.find_path("A".to_string(), "E".to_string());
        assert_eq!(
            result,
            Some(vec![
                buffer_tree.index.index("A".to_string()),
                buffer_tree.index.index("B".to_string()),
                buffer_tree.index.index("E".to_string()),
            ])
        );
        let _edge = buffer_tree.graph.edge_weight(
            buffer_tree.index.index("A".to_string()),
            buffer_tree.index.index("B".to_string()),
        );
    }

    #[test]
    fn test_robot_arm_transforms() {
        let mut buffer_tree = BufferTree::new();

        // Define test data as a vector of (source, target, translation, rotation, timestamp) tuples
        let transforms = vec![
            (
                "upper_arm_link",
                "forearm_link",
                [-0.425, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ),
            (
                "shoulder_link",
                "upper_arm_link",
                [0.0, 0.0, 0.0],
                [
                    0.5001990421112379,
                    0.49980087872426926,
                    -0.4998008786217583,
                    0.5001990420086454,
                ],
            ),
            (
                "base_link_inertia",
                "shoulder_link",
                [0.0, 0.0, 0.1625],
                [0.0, 0.0, 0.0, 1.0],
            ),
            (
                "forearm_link",
                "wrist_1_link",
                [-0.3922, 0.0, 0.1333],
                [0.0, 0.0, -0.7068251811053659, 0.7073882691671998],
            ),
            (
                "wrist_1_link",
                "wrist_2_link",
                [0.0, -0.0997, -2.044881182297852e-11],
                [0.7071067812590626, 0.0, 0.0, 0.7071067811140325],
            ),
            (
                "wrist_2_link",
                "wrist_3_link",
                [0.0, 0.0996, -2.042830148012698e-11],
                [
                    -0.7071067812590626,
                    8.659560562354933e-17,
                    8.880526795522719e-27,
                    0.7071067811140325,
                ],
            ),
        ];

        // Convert seconds and nanoseconds to floating point seconds
        let timestamp = 1741097108.0 + 171207063.0 * 1e-9;

        // Add all transforms to the buffer
        for (source, target, translation, rotation) in transforms {
            let stamped_isometry = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                    )),
                ),
                stamp: timestamp,
            };

            buffer_tree
                .update(
                    source.to_string(),
                    target.to_string(),
                    stamped_isometry,
                    TransformType::Dynamic,
                )
                .unwrap();
        }

        let transform = buffer_tree
            .lookup_latest_transform("base_link_inertia".to_string(), "shoulder_link".to_string());
        assert!(transform.is_ok());
        let transform = transform.unwrap();
        let translation = transform.isometry.translation.vector;
        let rotation = transform.isometry.rotation.into_inner();

        // Check translation components
        assert_relative_eq!(translation[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[1], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 0.1625, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation.w, 1.0, epsilon = 1e-3);
        assert_relative_eq!(rotation.i, 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation.j, 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation.k, 0.001, epsilon = 1e-3);

        // Test that we can find paths between arbitrary frames
        let path =
            buffer_tree.find_path("base_link_inertia".to_string(), "wrist_3_link".to_string());
        assert!(path.is_some());

        // Test that we can look up transforms
        let transform = buffer_tree
            .lookup_latest_transform("base_link_inertia".to_string(), "wrist_3_link".to_string());
        assert!(transform.is_ok());

        // Add these assertions
        let transform = transform.unwrap();
        let translation = transform.isometry.translation.vector;
        let rotation = transform.isometry.rotation.euler_angles();

        // Check translation components
        assert_relative_eq!(translation[0], -0.001, epsilon = 1e-3);
        assert_relative_eq!(translation[1], -0.233, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 1.079, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation.0, -1.571, epsilon = 1e-2);
        assert_relative_eq!(rotation.1, -0.002, epsilon = 1e-2);
        assert_relative_eq!(rotation.2, 3.142, epsilon = 1e-2);
    }

    #[test]
    fn test_robot_arm_transform_inverse() {
        let mut buffer_tree = BufferTree::new();

        // Define test data as a vector of (source, target, translation, rotation) tuples
        let transforms = vec![
            (
                "upper_arm_link",
                "forearm_link",
                [-0.425, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ),
            (
                "shoulder_link",
                "upper_arm_link",
                [0.0, 0.0, 0.0],
                [
                    0.5001990421112379,
                    0.49980087872426926,
                    -0.4998008786217583,
                    0.5001990420086454,
                ],
            ),
            (
                "base_link_inertia",
                "shoulder_link",
                [0.0, 0.0, 0.1625],
                [0.0, 0.0, 0.0, 1.0],
            ),
            (
                "forearm_link",
                "wrist_1_link",
                [-0.3922, 0.0, 0.1333],
                [0.0, 0.0, -0.7068251811053659, 0.7073882691671998],
            ),
            (
                "wrist_1_link",
                "wrist_2_link",
                [0.0, -0.0997, -2.044881182297852e-11],
                [0.7071067812590626, 0.0, 0.0, 0.7071067811140325],
            ),
            (
                "wrist_2_link",
                "wrist_3_link",
                [0.0, 0.0996, -2.042830148012698e-11],
                [
                    -0.7071067812590626,
                    8.659560562354933e-17,
                    8.880526795522719e-27,
                    0.7071067811140325,
                ],
            ),
        ];

        let timestamp = 1741097108.0 + 171207063.0 * 1e-9;

        // Add all transforms to the buffer
        for (source, target, translation, rotation) in transforms {
            let stamped_isometry = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                    )),
                ),
                stamp: timestamp,
            };

            buffer_tree
                .update(
                    source.to_string(),
                    target.to_string(),
                    stamped_isometry,
                    TransformType::Dynamic,
                )
                .unwrap();
        }

        println!("{}", buffer_tree.visualize());
        let transform = buffer_tree
            .lookup_latest_transform("wrist_3_link".to_string(), "base_link_inertia".to_string());
        assert!(transform.is_ok());
        let transform = transform.unwrap();
        let translation = transform.isometry.translation.vector;
        let rotation = transform.isometry.rotation.euler_angles();

        // Check translation components (should be inverse of original transform)
        assert_relative_eq!(translation[0], 0.001, epsilon = 1e-3);
        assert_relative_eq!(translation[1], 1.079, epsilon = 1e-3);
        assert_relative_eq!(translation[2], -0.233, epsilon = 1e-3);

        // Check euler angles (should be inverse of original transform)
        assert_relative_eq!(rotation.0, -1.571, epsilon = 1e-2);
        assert_relative_eq!(rotation.1, 0.00, epsilon = 1e-2);
        assert_relative_eq!(rotation.2, 3.14, epsilon = 1e-2);
    }

    #[test]
    fn test_robot_arm_transforms_interpolation() {
        let mut buffer_tree = BufferTree::new();

        // Define test data as a vector of (source, target, translation, rotation, timestamp) tuples
        let transforms = vec![
            (
                "upper_arm_link",
                "forearm_link",
                [-0.425, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ),
            (
                "shoulder_link",
                "upper_arm_link",
                [0.0, 0.0, 0.0],
                [
                    0.5001990421112379,
                    0.49980087872426926,
                    -0.4998008786217583,
                    0.5001990420086454,
                ],
            ),
            (
                "base_link_inertia",
                "shoulder_link",
                [0.0, 0.0, 0.1625],
                [0.0, 0.0, 0.0, 1.0],
            ),
            (
                "forearm_link",
                "wrist_1_link",
                [-0.3922, 0.0, 0.1333],
                [0.0, 0.0, -0.7068251811053659, 0.7073882691671998],
            ),
            (
                "wrist_1_link",
                "wrist_2_link",
                [0.0, -0.0997, -2.044881182297852e-11],
                [0.7071067812590626, 0.0, 0.0, 0.7071067811140325],
            ),
            (
                "wrist_2_link",
                "wrist_3_link",
                [0.0, 0.0996, -2.042830148012698e-11],
                [
                    -0.7071067812590626,
                    8.659560562354933e-17,
                    8.880526795522719e-27,
                    0.7071067811140325,
                ],
            ),
        ];

        // Convert seconds and nanoseconds to floating point seconds
        let timestamp_1 = 1.;
        let timestamp_2 = 2.;

        // Add all transforms to the buffer
        for (source, target, translation, rotation) in transforms {
            let stamped_isometry_1 = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                    )),
                ),
                stamp: timestamp_1,
            };
            let stamped_isometry_2 = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                    )),
                ),
                stamp: timestamp_2,
            };

            buffer_tree
                .update(
                    source.to_string(),
                    target.to_string(),
                    stamped_isometry_1,
                    TransformType::Dynamic,
                )
                .unwrap();
            buffer_tree
                .update(
                    source.to_string(),
                    target.to_string(),
                    stamped_isometry_2,
                    TransformType::Dynamic,
                )
                .unwrap();
        }

        let transform = buffer_tree.lookup_transform(
            "base_link_inertia".to_string(),
            "shoulder_link".to_string(),
            1.5,
        );
        assert!(transform.is_ok());
        let transform = transform.unwrap();
        let translation = transform.isometry.translation.vector;
        let rotation = transform.isometry.rotation.into_inner();

        // Check translation components
        assert_relative_eq!(translation[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[1], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 0.1625, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation.w, 1.0, epsilon = 1e-3);
        assert_relative_eq!(rotation.i, 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation.j, 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation.k, 0.001, epsilon = 1e-3);

        // Test that we can find paths between arbitrary frames
        let path =
            buffer_tree.find_path("base_link_inertia".to_string(), "wrist_3_link".to_string());
        assert!(path.is_some());

        // Test that we can look up transforms
        let transform = buffer_tree
            .lookup_latest_transform("base_link_inertia".to_string(), "wrist_3_link".to_string());
        assert!(transform.is_ok());

        // Add these assertions
        let transform = transform.unwrap();
        let translation = transform.isometry.translation.vector;
        let rotation = transform.isometry.rotation.euler_angles();

        // Check translation components
        assert_relative_eq!(translation[0], -0.001, epsilon = 1e-3);
        assert_relative_eq!(translation[1], -0.233, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 1.079, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation.0, -1.571, epsilon = 1e-2);
        assert_relative_eq!(rotation.1, -0.002, epsilon = 1e-2);
        assert_relative_eq!(rotation.2, 3.142, epsilon = 1e-2);

        // Check if correct error raised
        match buffer_tree.lookup_transform(
            "base_link_inertia".to_string(),
            "shoulder_link".to_string(),
            0.,
        ) {
            Err(TfError::AttemptedLookupInPast) => {
                // The function returned the expected error variant
                assert!(true);
            }
            _ => {
                // The function did not return the expected error variant
                assert!(false, "Expected TfError::AttemptedLookupInPast");
            }
        }
        match buffer_tree.lookup_transform(
            "base_link_inertia".to_string(),
            "shoulder_link".to_string(),
            3.,
        ) {
            Err(TfError::AttemptedLookUpInFuture) => {
                // The function returned the expected error variant
                assert!(true);
            }
            _ => {
                // The function did not return the expected error variant
                assert!(false, "Expected TfError::AttemptedLookupInPast");
            }
        }
        match buffer_tree.lookup_transform("XXXXX".to_string(), "shoulder_link".to_string(), 3.) {
            Err(TfError::CouldNotFindTransform) => {
                // The function returned the expected error variant
                assert!(true);
            }
            _ => {
                // The function did not return the expected error variant
                assert!(false, "Expected TfError::AttemptedLookupInPast");
            }
        }
    }

    /// This test is generated using the following python code:
    /// It tests if the interpolation yields the same result as the ROS TF2 Buffer.
    ///
    /// import random
    ///
    /// import yaml
    /// from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3
    /// from std_msgs.msg import Header
    /// from rclpy.time import Time
    /// import rclpy
    /// from rclpy.node import Node
    /// from tf2_ros.buffer import Buffer
    /// from tf2_ros.transform_listener import TransformListener
    /// from tf2_ros import TransformBroadcaster
    /// def generate_random_transform(mock_time: float, frame_id: str, child_frame_id: str):
    ///     random_numbers = [random.random() for _ in range(4)]
    ///     # Calculate the sum of these numbers
    ///     total_sum = sum(random_numbers)
    ///     # Normalize the numbers so that their sum is 1
    ///     normalized_numbers = [x / total_sum for x in random_numbers]
    ///
    ///     return TransformStamped(
    ///         header=Header(
    ///             frame_id=frame_id,
    ///             stamp=Time(seconds=mock_time).to_msg(),
    ///         ),
    ///         child_frame_id=child_frame_id,
    ///         transform=Transform(
    ///             translation=Vector3(x=random.uniform(-1, 1), y=random.uniform(-1, 1), z=random.uniform(-1, 1)),
    ///             rotation=Quaternion(x=normalized_numbers[0], y=normalized_numbers[1], z=normalized_numbers[2], w=normalized_numbers[3]),
    ///         ),
    ///     )
    ///
    /// def generate_ro_time_from_float(mock_time: float):
    ///     return Time(seconds=mock_time)
    ///
    /// def transform_stamped_to_dict(transform: TransformStamped) -> dict:
    ///     return {
    ///         'header': {
    ///             'frame_id': transform.header.frame_id,
    ///             'stamp': transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9,
    ///         },
    ///         'child_frame_id': transform.child_frame_id,
    ///         'transform': {
    ///             'translation': {
    ///                 'x': transform.transform.translation.x,
    ///                 'y': transform.transform.translation.y,
    ///                 'z': transform.transform.translation.z,
    ///             },
    ///             'rotation': {
    ///                 'x': transform.transform.rotation.x,
    ///                 'y': transform.transform.rotation.y,
    ///                 'z': transform.transform.rotation.z,
    ///                 'w': transform.transform.rotation.w,
    ///             }
    ///         }
    ///     }
    /// class FramePublisher(Node):
    ///
    ///     def __init__(self):
    ///         super().__init__('turtle_tf2_frame_publisher')
    ///         # Initialize the transform broadcaster
    ///         self.tf_broadcaster = TransformBroadcaster(self)
    ///         self.tf_buffer = Buffer()
    ///
    ///         self.tf_listener = TransformListener(self.tf_buffer, self)
    ///
    ///
    ///     def publish_transforms(self):
    ///         transforms_t0 = [
    ///             generate_random_transform(mock_time=0.0, frame_id="a", child_frame_id="b"),
    ///             generate_random_transform(mock_time=0.0, frame_id="b", child_frame_id="c"),
    ///             generate_random_transform(mock_time=0.0, frame_id="c", child_frame_id="d"),
    ///             generate_random_transform(mock_time=0.0, frame_id="d", child_frame_id="e"),
    ///             generate_random_transform(mock_time=0.0, frame_id="e", child_frame_id="f"),
    ///         ]
    ///         transforms_t1 = [
    ///             generate_random_transform(mock_time=1.0, frame_id="a", child_frame_id="b"),
    ///             generate_random_transform(mock_time=1.0, frame_id="b", child_frame_id="c"),
    ///             generate_random_transform(mock_time=1.0, frame_id="c", child_frame_id="d"),
    ///             generate_random_transform(mock_time=1.0, frame_id="d", child_frame_id="e"),
    ///             generate_random_transform(mock_time=1.0, frame_id="e", child_frame_id="f"),
    ///         ]
    ///         input_yaml = []
    ///
    ///         for transform in transforms_t0 + transforms_t1:
    ///             input_yaml.append(transform_stamped_to_dict(transform))
    ///
    ///         print(yaml.dump(input_yaml))
    ///         # Fill the buffer
    ///         for transform in transforms_t0 + transforms_t1:
    ///             self.tf_broadcaster.sendTransform(transform)
    ///
    ///     def request_transform(self):
    ///         # Get the transform from the buffer
    ///         t_1 = self.tf_buffer.lookup_transform("a", "f", generate_ro_time_from_float(0.2))
    ///         t_2 = self.tf_buffer.lookup_transform("a", "f", generate_ro_time_from_float(0.5))
    ///         t_3 = self.tf_buffer.lookup_transform("a", "f", generate_ro_time_from_float(0.8))
    ///
    ///         t_4 = self.tf_buffer.lookup_transform("f", "a", generate_ro_time_from_float(0.2))
    ///         t_5 = self.tf_buffer.lookup_transform("f", "a", generate_ro_time_from_float(0.5))
    ///         t_6 = self.tf_buffer.lookup_transform("f", "a", generate_ro_time_from_float(0.8))
    ///
    ///         print(yaml.dump(transform_stamped_to_dict(t_1)))
    ///         print(yaml.dump(transform_stamped_to_dict(t_2)))
    ///         print(yaml.dump(transform_stamped_to_dict(t_3)))
    ///
    ///         print(yaml.dump(transform_stamped_to_dict(t_4)))
    ///         print(yaml.dump(transform_stamped_to_dict(t_5)))
    ///         print(yaml.dump(transform_stamped_to_dict(t_6)))
    ///
    /// def main():
    ///     rclpy.init()
    ///     node = FramePublisher()
    ///     node.publish_transforms()
    ///     while rclpy.ok():
    ///         rclpy.spin_once(node)
    ///         try:
    ///             node.request_transform()
    ///         except Exception as e:
    ///             print(e)
    ///             continue
    ///         break
    ///
    ///     rclpy.shutdown()
    ///
    ///
    /// if __name__ == "__main__":
    ///     main()
    #[test]
    fn test_complex_interpolation() {
        let mut buffer_tree = BufferTree::new();

        // First set of transforms at t=0.0
        let transforms_t0 = vec![
            (
                "a",
                "b",
                [0.9542820082386645, -0.6552492462418078, 0.7161777435789107],
                [
                    0.5221303556354912,
                    0.35012976926397515,
                    0.06385453213291199,
                    0.06388534296762166,
                ],
            ),
            (
                "b",
                "c",
                [
                    -0.19846060797892018,
                    0.37060239713344223,
                    -0.9325041671812722,
                ],
                [
                    0.17508543470264146,
                    0.015141878067977513,
                    0.7464281310309472,
                    0.0633445561984338,
                ],
            ),
            (
                "c",
                "d",
                [-0.794492125974928, 0.3998294717449842, 0.10994520945722774],
                [
                    0.09927023004042039,
                    0.3127284173757304,
                    0.09323219806580624,
                    0.49476915451804293,
                ],
            ),
            (
                "d",
                "e",
                [
                    -0.10568484318994975,
                    -0.25311133155256416,
                    -0.5050832697305845,
                ],
                [
                    0.34253037231148725,
                    0.18360347226679302,
                    0.03909759741077618,
                    0.43476855801094355,
                ],
            ),
            (
                "e",
                "f",
                [
                    0.08519341627411214,
                    -0.21820466927246485,
                    -0.49430885607234565,
                ],
                [
                    0.5030721633460956,
                    0.42228251371020586,
                    0.05757558742063205,
                    0.017069735523066495,
                ],
            ),
        ];

        // Second set of transforms at t=1.0
        let transforms_t1 = vec![
            (
                "a",
                "b",
                [-0.2577564261850547, 0.7493551580360949, 0.9508883926449649],
                [
                    0.22516451641196783,
                    0.39948597131211394,
                    0.2540343540211825,
                    0.12131515825473572,
                ],
            ),
            (
                "b",
                "c",
                [
                    0.8409405814571027,
                    -0.9879602392577504,
                    -0.13140102332772097,
                ],
                [
                    0.1398908842037251,
                    0.2758514837076157,
                    0.24490871323462493,
                    0.33934891885403434,
                ],
            ),
            (
                "c",
                "d",
                [
                    0.22500109579960625,
                    -0.1414475909286277,
                    -0.14392029811070084,
                ],
                [
                    0.19694092483717301,
                    0.27122448763510776,
                    0.4097865936798704,
                    0.12204799384784887,
                ],
            ),
            (
                "d",
                "e",
                [
                    -0.20684779237257978,
                    -0.7643987654163593,
                    -0.6253015724407152,
                ],
                [
                    0.27849097201454626,
                    0.15911896201926773,
                    0.19901604722897315,
                    0.3633740187372129,
                ],
            ),
            (
                "e",
                "f",
                [-0.09213549320472025, 0.7601862256435243, -0.84895940549366],
                [
                    0.002094505867313596,
                    0.13339467043347925,
                    0.22297487081296374,
                    0.6415359528862433,
                ],
            ),
        ];

        // Add transforms at t=0.0
        for (source, target, translation, rotation) in transforms_t0 {
            let stamped_isometry = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                    )),
                ),
                stamp: 0.0,
            };
            buffer_tree
                .update(
                    source.to_string(),
                    target.to_string(),
                    stamped_isometry,
                    TransformType::Dynamic,
                )
                .unwrap();
        }

        // Add transforms at t=1.0
        for (source, target, translation, rotation) in transforms_t1 {
            let stamped_isometry = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                    )),
                ),
                stamp: 1.0,
            };
            buffer_tree
                .update(
                    source.to_string(),
                    target.to_string(),
                    stamped_isometry,
                    TransformType::Dynamic,
                )
                .unwrap();
        }

        // Look up transform at t=0.2
        let result = buffer_tree
            .lookup_transform("a".to_string(), "f".to_string(), 0.2)
            .unwrap();

        // Expected values
        let expected_translation =
            nalgebra::Vector3::new(-0.02688966809486315, 0.8302180267299373, 1.6491944090937691);
        let expected_rotation =
            nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                -0.23762484510717535,
                0.7704449853702972,
                -0.44625068910795557,
                -0.38834170517242694,
            ));

        // Assert translation components
        assert_relative_eq!(
            result.isometry.translation.vector,
            expected_translation,
            epsilon = 1e-6
        );

        // Assert rotation components
        assert_relative_eq!(result.isometry.rotation, expected_rotation, epsilon = 1e-6);
        // Additional test cases at different timestamps
        let test_cases = vec![
            // a->f at t=0.2
            (
                0.2,
                "a",
                "f",
                [-0.02688966809486315, 0.8302180267299373, 1.6491944090937691],
                [
                    -0.23762484510717535,
                    0.7704449853702972,
                    -0.44625068910795557,
                    -0.38834170517242694,
                ],
            ),
            // a->f at t=0.5
            (
                0.5,
                "a",
                "f",
                [-0.7313014953477409, 0.8588360737131203, 1.3897218882465063],
                [
                    -0.20299191732296193,
                    0.9561102276829774,
                    0.10847159958471206,
                    -0.1813323636450122,
                ],
            ),
            // a->f at t=0.8
            (
                0.8,
                "a",
                "f",
                [-1.5366396114062963, 0.5615052687815749, 1.2753385241243729],
                [
                    0.025710201700027795,
                    0.8191599958838035,
                    0.5182799902870279,
                    0.2443393917080692,
                ],
            ),
            // f->a at t=0.2
            (
                0.2,
                "f",
                "a",
                [1.7623488465323582, 0.4146044950680975, 0.36339631387666715],
                [
                    0.23762484510717535,
                    0.7704449853702972,
                    -0.44625068910795557,
                    -0.38834170517242694,
                ],
            ),
            // // f->a at t=0.5
            (
                0.5,
                "f",
                "a",
                [0.8453152942269395, 1.4598104847572575, 0.5984342964929825],
                [
                    0.20299191732296193,
                    0.9561102276829774,
                    0.10847159958471206,
                    -0.1813323636450122,
                ],
            ),
            // // f->a at t=0.8
            (
                0.8,
                "f",
                "a",
                [-0.43273825025921875, 1.1678464326290772, 1.6588882210342657],
                [
                    -0.025710201700027795,
                    0.8191599958838035,
                    0.5182799902870279,
                    0.2443393917080692,
                ],
            ),
        ];

        // Test each case
        for (time, source, target, translation, rotation) in test_cases {
            let result = buffer_tree
                .lookup_transform(source.to_string(), target.to_string(), time)
                .unwrap();

            let expected_translation =
                nalgebra::Vector3::new(translation[0], translation[1], translation[2]);
            let expected_rotation =
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    rotation[0], // w
                    rotation[1], // x
                    rotation[2], // y
                    rotation[3], // z
                ));

            // Assert translation components
            assert_relative_eq!(
                result.isometry.translation.vector,
                expected_translation,
                epsilon = 1e-6,
                max_relative = 1e-6
            );

            // Assert rotation components
            assert_relative_eq!(
                result.isometry.rotation,
                expected_rotation,
                epsilon = 1e-6,
                max_relative = 1e-6
            );
        }
    }
}
