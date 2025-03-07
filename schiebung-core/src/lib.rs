use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::Write;
use std::process::Command;
use dirs::home_dir;

use nalgebra::geometry::Isometry3;
use petgraph::algo::is_cyclic_undirected;
use petgraph::graphmap::DiGraphMap;
use schiebung_types::{StampedIsometry, TransformType};

/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
}

#[derive(Debug)]
pub struct BufferConfig {
    max_transform_history: usize,
    save_path: String,
}
impl BufferConfig {
    pub fn new() -> Self{
        BufferConfig {
            max_transform_history: 1000,
            save_path: home_dir().unwrap().display().to_string(),
        }
    }
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
/// We check if the graph is acyclic every time a node is added
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
            config: BufferConfig::new(),
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
    ) {
        let source = self.index.index(source);
        let target = self.index.index(target);

        if !self.graph.contains_node(source) {
            self.graph.add_node(source);
        }
        if !self.graph.contains_node(target) {
            self.graph.add_node(target);
        }

        if !self.graph.contains_edge(source, target) {
            self.graph
                .add_edge(source, target, TransformHistory::new(kind, self.config.max_transform_history));
            if is_cyclic_undirected(&self.graph) {
                panic!("Cyclic graph detected");
            }
        }
        self.graph
            .edge_weight_mut(source, target)
            .unwrap()
            .update(stamped_isometry);
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
                    .edge_weight(source_idx, target_idx)
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
                dot.push_str(&format!("    {} -> {} [label=\"No transforms\"]\n", edge.0, edge.1));
            }
        }
        
        dot.push_str("}");
        dot
    }

    /// Save the buffer tree as a PDF and dot file
    /// Runs graphiz to generate the PDF, fails if graphiz is not installed
    pub fn save_visualization(&self) -> std::io::Result<()> {
        let filename = &self.config.save_path;
        // Save DOT file
        let dot_content = self.visualize();
        let dot_filename = format!("{}.dot", filename);
        let mut file = File::create(&dot_filename)?;
        file.write_all(dot_content.as_bytes())?;

        // Generate PDF using dot command
        let pdf_filename = format!("{}.pdf", filename);
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
        buffer_tree.update(
            source.clone(),
            target.clone(),
            stamped_isometry.clone(),
            TransformType::Static,
        );

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
        buffer_tree.update(
            source.clone(),
            target.clone(),
            stamped_isometry_2.clone(),
            TransformType::Static,
        );

        // Ensure the history is updated
        let edge_weight = buffer_tree
            .graph
            .edge_weight(source_idx, target_idx)
            .unwrap();
        assert_eq!(edge_weight.history.len(), 2);
        assert_eq!(edge_weight.history.back().unwrap().stamp, 2.0);
    }

    #[test]
    #[should_panic(expected = "Cyclic graph detected")]
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
        buffer_tree.update(
            a.clone(),
            b.clone(),
            stamped_isometry.clone(),
            TransformType::Static,
        );
        buffer_tree.update(
            b.clone(),
            c.clone(),
            stamped_isometry.clone(),
            TransformType::Static,
        );

        // Creating a cycle C → A should panic
        buffer_tree.update(
            c.clone(),
            a.clone(),
            stamped_isometry.clone(),
            TransformType::Static,
        );
    }

    #[test]
    fn test_find_path() {
        let mut buffer_tree = BufferTree::new();

        buffer_tree.update(
            "A".to_string(),
            "B".to_string(),
            StampedIsometry {
                isometry: Isometry3::identity(),
                stamp: 1.0,
            },
            TransformType::Dynamic,
        );

        buffer_tree.update(
            "A".to_string(),
            "C".to_string(),
            StampedIsometry {
                isometry: Isometry3::identity(),
                stamp: 2.0,
            },
            TransformType::Dynamic,
        );

        buffer_tree.update(
            "B".to_string(),
            "D".to_string(),
            StampedIsometry {
                isometry: Isometry3::identity(),
                stamp: 3.0,
            },
            TransformType::Dynamic,
        );

        buffer_tree.update(
            "B".to_string(),
            "E".to_string(),
            StampedIsometry {
                isometry: Isometry3::identity(),
                stamp: 3.0,
            },
            TransformType::Dynamic,
        );

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
        let edge = buffer_tree.graph.edge_weight(
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

            buffer_tree.update(
                source.to_string(),
                target.to_string(),
                stamped_isometry,
                TransformType::Dynamic,
            );
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

            buffer_tree.update(
                source.to_string(),
                target.to_string(),
                stamped_isometry,
                TransformType::Dynamic,
            );
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
    }    #[test]

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

            buffer_tree.update(
                source.to_string(),
                target.to_string(),
                stamped_isometry_1,
                TransformType::Dynamic,
            );
            buffer_tree.update(
                source.to_string(),
                target.to_string(),
                stamped_isometry_2,
                TransformType::Dynamic,
            );
        }

        let transform = buffer_tree
            .lookup_transform("base_link_inertia".to_string(), "shoulder_link".to_string(), 1.5);
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
        match buffer_tree.lookup_transform("base_link_inertia".to_string(), "shoulder_link".to_string(), 0.) {
            Err(TfError::AttemptedLookupInPast) => {
                // The function returned the expected error variant
                assert!(true);
            }
            _ => {
                // The function did not return the expected error variant
                assert!(false, "Expected TfError::AttemptedLookupInPast");
            }
        }
        match buffer_tree.lookup_transform("base_link_inertia".to_string(), "shoulder_link".to_string(), 3.) {
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
}
