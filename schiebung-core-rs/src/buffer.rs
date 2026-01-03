use rustc_hash::FxHashMap;
use std::collections::VecDeque;
use std::fs::File;
use std::io::Write;
use std::process::Command;

use nalgebra::geometry::Isometry3;
use petgraph::algo::is_cyclic_undirected;
use petgraph::graphmap::DiGraphMap;

use crate::config::{get_config, BufferConfig};
use crate::error::TfError;
use crate::types::{StampedIsometry, TransformType};

/// The TransformHistory keeps track of a single transform between two frames
/// Update pushes a new StampedTransform to the end, if the history reaches it's max length
/// The oldest transform is removed.
#[derive(Debug)]
struct TransformHistory {
    history: VecDeque<StampedIsometry>,
    kind: TransformType,
    buffer_window: f64,
}

impl TransformHistory {
    pub fn new(kind: TransformType, buffer_window: f64) -> Self {
        TransformHistory {
            history: VecDeque::new(),
            kind,
            buffer_window,
        }
    }

    pub fn update(&mut self, stamped_isometry: StampedIsometry) {
        self.history.push_back(stamped_isometry);
        if (self.history.back().unwrap().stamp - self.history.front().unwrap().stamp)
            > self.buffer_window
        {
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
                    return Err(TfError::CouldNotFindTransform(format!(
                        "Not enough history to interpolate. Len: {}",
                        self.history.len()
                    ))); // Not enough elements
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
                            return Err(TfError::AttemptedLookupInPast(format!(
                                "Time {} is before the oldest transform at {}",
                                time, history[0].stamp
                            )));
                        }
                        if i >= history.len() {
                            return Err(TfError::AttemptedLookUpInFuture(format!(
                                "Time {} is after the newest transform at {}",
                                time,
                                history[history.len() - 1].stamp
                            )));
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

/// Node representation with name and ancestors
#[derive(Debug, Clone)]
pub struct Node {
    pub name: String,
    pub ancestors: Vec<String>,
    pub ancestor_ids: Vec<usize>,
}

/// Need to index the strings via a hashmap
/// DiGrapMap does not support string indexing
/// We use FxHashMap to improve the lookup speed
struct NodeIndex {
    node_map: FxHashMap<String, usize>,
    nodes: Vec<Node>,
}

// DiGraphMap does not support strings and requires an external storage
// https://github.com/petgraph/petgraph/issues/325
impl NodeIndex {
    pub fn new() -> Self {
        NodeIndex {
            node_map: FxHashMap::default(),
            nodes: Vec::new(),
        }
    }

    pub fn get(&self, node: &str) -> Option<usize> {
        self.node_map.get(node).cloned()
    }

    pub fn index(&mut self, node: &str) -> usize {
        if let Some(&id) = self.node_map.get(node) {
            return id;
        }

        let node_id = self.nodes.len();
        self.node_map.insert(node.to_string(), node_id);
        self.nodes.push(Node {
            name: node.to_string(),
            ancestors: Vec::new(),
            ancestor_ids: Vec::new(),
        });
        node_id
    }

    #[allow(dead_code)]
    pub fn contains(&self, node: &str) -> bool {
        self.node_map.contains_key(node)
    }

    pub fn get_node(&self, id: usize) -> Option<&Node> {
        self.nodes.get(id)
    }

    pub fn get_node_mut(&mut self, id: usize) -> Option<&mut Node> {
        self.nodes.get_mut(id)
    }
}

/// Trait to observe changes in the buffer
pub trait BufferObserver: Send + Sync {
    fn on_update(&self, from: &str, to: &str, transform: &StampedIsometry, kind: TransformType);
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
    observers: Vec<Box<dyn BufferObserver>>,
}

impl BufferTree {
    pub fn new() -> Self {
        BufferTree {
            graph: DiGraphMap::new(),
            index: NodeIndex::new(),
            config: get_config().unwrap(),
            observers: Vec::new(),
        }
    }

    /// Register a new observer
    /// The observer will be notified about all current transforms in the buffer
    pub fn register_observer(&mut self, observer: Box<dyn BufferObserver>) {
        // Notify the new observer about all existing transforms
        for (from_idx, to_idx, history) in self.graph.all_edges() {
            let from_node = self.index.get_node(from_idx);
            let to_node = self.index.get_node(to_idx);

            if let (Some(from_node), Some(to_node)) = (from_node, to_node) {
                for item in &history.history {
                    observer.on_update(&from_node.name, &to_node.name, item, history.kind);
                }
            }
        }
        self.observers.push(observer);
    }

    /// Recursively update the ancestors of a node and its children
    fn update_subtree_ancestors(
        &mut self,
        root_id: usize,
        ancestors: Vec<String>,
        ancestor_ids: Vec<usize>,
    ) {
        let mut my_name = String::new();
        if let Some(node) = self.index.get_node_mut(root_id) {
            node.ancestors = ancestors.clone();
            node.ancestor_ids = ancestor_ids.clone();
            my_name = node.name.clone();
        }

        let children: Vec<usize> = self
            .graph
            .neighbors_directed(root_id, petgraph::Direction::Outgoing)
            .collect();

        let mut child_ancestors = ancestors;
        child_ancestors.push(my_name);

        let mut child_ancestor_ids = ancestor_ids;
        child_ancestor_ids.push(root_id);

        for child_id in children {
            self.update_subtree_ancestors(
                child_id,
                child_ancestors.clone(),
                child_ancestor_ids.clone(),
            );
        }
    }

    pub fn update(
        &mut self,
        from: &str,
        to: &str,
        stamped_isometry: StampedIsometry,
        kind: TransformType,
    ) -> Result<(), TfError> {
        let from_idx = self.index.index(from);
        let to_idx = self.index.index(to);

        if !self.graph.contains_node(from_idx) {
            self.graph.add_node(from_idx);
        }
        if !self.graph.contains_node(to_idx) {
            self.graph.add_node(to_idx);
        }

        if !self.graph.contains_edge(from_idx, to_idx) {
            self.graph.add_edge(
                from_idx,
                to_idx,
                TransformHistory::new(kind, self.config.buffer_window),
            );
            if is_cyclic_undirected(&self.graph)
                || self
                    .graph
                    .neighbors_directed(to_idx, petgraph::Direction::Incoming)
                    .count()
                    > 1
            {
                // Remove the edge and nodes if they have no other edges
                self.graph.remove_edge(from_idx, to_idx);
                if self
                    .graph
                    .neighbors_directed(to_idx, petgraph::Direction::Incoming)
                    .count()
                    < 1
                    && self
                        .graph
                        .neighbors_directed(to_idx, petgraph::Direction::Outgoing)
                        .count()
                        < 1
                {
                    self.graph.remove_node(to_idx);
                }
                if self
                    .graph
                    .neighbors_directed(from_idx, petgraph::Direction::Incoming)
                    .count()
                    < 1
                    && self
                        .graph
                        .neighbors_directed(from_idx, petgraph::Direction::Outgoing)
                        .count()
                        < 1
                {
                    self.graph.remove_node(from_idx);
                }
                return Err(TfError::InvalidGraph(format!(
                    "Graph cycle or multiple parents detected when adding edge {} -> {}",
                    from_idx, to_idx
                )));
            }

            // Update ancestors
            let mut new_ancestors = Vec::new();
            let mut new_ancestor_ids = Vec::new();
            if let Some(from_node) = self.index.get_node(from_idx) {
                new_ancestors = from_node.ancestors.clone();
                new_ancestors.push(from_node.name.clone());

                new_ancestor_ids = from_node.ancestor_ids.clone();
                new_ancestor_ids.push(from_idx);
            }
            self.update_subtree_ancestors(to_idx, new_ancestors, new_ancestor_ids);
        }
        // Notify observers
        for observer in &self.observers {
            observer.on_update(from, to, &stamped_isometry, kind);
        }

        self.graph
            .edge_weight_mut(from_idx, to_idx)
            .unwrap()
            .update(stamped_isometry);
        Ok(())
    }

    /// Searches for a path in the graph
    /// We implement our own path search here because we have assumptions on the graph
    /// We have to consider that "form" and "to" are on different branches therefore we
    /// traverse the tree upwards from both nodes until we either hit the other node or the root
    /// Afterwards we prune the leftover path above the connection point
    #[allow(dead_code)]
    fn find_path(&self, from: &str, to: &str) -> Option<Vec<usize>> {
        let from_idx = self.index.get(from)?;
        let to_idx = self.index.get(to)?;
        self.find_path_by_id(from_idx, to_idx)
    }

    fn find_path_by_id(&self, from_idx: usize, to_idx: usize) -> Option<Vec<usize>> {
        let from_node = self.index.get_node(from_idx)?;
        let to_node = self.index.get_node(to_idx)?;

        // Build full paths from root to each node using pre-computed ancestors
        let ancestor_ids_from = &from_node.ancestor_ids;
        let ancestor_ids_to = &to_node.ancestor_ids;

        // Pre-allocate with exact capacity to avoid reallocation
        let mut path_from_root = Vec::with_capacity(ancestor_ids_from.len() + 1);
        path_from_root.extend_from_slice(ancestor_ids_from);
        path_from_root.push(from_idx);

        let mut path_to_root = Vec::with_capacity(ancestor_ids_to.len() + 1);
        path_to_root.extend_from_slice(ancestor_ids_to);
        path_to_root.push(to_idx);

        // Find the split point (LCA)
        let mut split_idx = 0;
        while split_idx < path_from_root.len() && split_idx < path_to_root.len() {
            if path_from_root[split_idx] != path_to_root[split_idx] {
                break;
            }
            split_idx += 1;
        }

        if split_idx == 0 {
            // No common ancestor
            return None;
        }

        // Pre-allocate result_path with exact size
        let up_len = path_from_root.len() - split_idx;
        let down_len = path_to_root.len() - split_idx + 1;
        let mut result_path = Vec::with_capacity(up_len + down_len);

        // Add part from 'from' up to LCA (reversed)
        for i in (split_idx..path_from_root.len()).rev() {
            result_path.push(path_from_root[i]);
        }

        // Add part from LCA down to 'to' (use extend_from_slice for better performance)
        result_path.extend_from_slice(&path_to_root[split_idx - 1..]);

        Some(result_path)
    }

    /// Lookup the latest transform without any checks
    /// This can be used for static transforms or if the user does not care if the
    /// transform is still valid.
    /// NOTE: This might give you outdated transforms!
    pub fn lookup_latest_transform(
        &self,
        from: &str,
        to: &str,
    ) -> Result<StampedIsometry, TfError> {
        // Get node IDs upfront to avoid redundant hash lookups
        let from_idx = self.index.get(from).ok_or_else(|| {
            TfError::CouldNotFindTransform(format!("Source frame '{}' does not exist", from))
        })?;
        let to_idx = self.index.get(to).ok_or_else(|| {
            TfError::CouldNotFindTransform(format!("Target frame '{}' does not exist", to))
        })?;

        let path = self.find_path_by_id(from_idx, to_idx);
        match path {
            Some(path) => {
                let isometry = self.compute_transform_along_path(&path, |history| {
                    let latest_transform = history
                        .history
                        .back()
                        .ok_or(TfError::CouldNotFindTransform(format!("")))?;
                    Ok(latest_transform.isometry)
                })?;

                Ok(StampedIsometry {
                    isometry,
                    stamp: 0.0,
                })
            }
            None => Err(TfError::CouldNotFindTransform(format!(
                "Could not find path between '{}' and '{}'",
                from, to
            ))),
        }
    }

    /// Lookup the transform at time
    /// This will look for a transform at the provided time and can "time travel"
    /// If any edge contains a transform older then time a AttemptedLookupInPast is raised
    /// If the time is younger then any transform AttemptedLookUpInFuture is raised
    /// If there is no perfect match the transforms around this time are interpolated
    /// The interpolation is weighted with the distance to the time stamps
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: f64,
    ) -> Result<StampedIsometry, TfError> {
        // Get node IDs upfront to avoid redundant hash lookups
        let from_idx = self.index.get(from).ok_or_else(|| {
            TfError::CouldNotFindTransform(format!("Source frame '{}' does not exist", from))
        })?;
        let to_idx = self.index.get(to).ok_or_else(|| {
            TfError::CouldNotFindTransform(format!("Target frame '{}' does not exist", to))
        })?;

        let path = self.find_path_by_id(from_idx, to_idx);

        match path {
            Some(path) => {
                let isometry = self.compute_transform_along_path(&path, |history| {
                    history.interpolate_isometry_at_time(time)
                })?;

                Ok(StampedIsometry {
                    isometry,
                    stamp: time,
                })
            }
            None => Err(TfError::CouldNotFindTransform(format!(
                "Could not find path between '{}' and '{}'",
                from, to
            ))),
        }
    }

    /// Visualize the buffer tree as a DOT graph
    /// Can not use internal visualizer because we Store the nodes in self.index
    pub fn visualize(&self) -> String {
        // Create a mapping from index back to node name
        // Convert the graph to DOT format manually
        let mut dot = String::from("digraph {\n");

        // Add nodes
        for node in self.graph.nodes() {
            let name = &self.index.get_node(node).unwrap().name;
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
        println!("Saving visualization to {}/graph.(dot/pdf)", filename);
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

    /// Helper function to compute transforms along a path
    /// This function handles the common logic of iterating through a path and computing
    /// the cumulative transform. The transform_getter function determines how to get
    /// the transform for each edge (latest vs interpolated at time).
    fn compute_transform_along_path<F>(
        &self,
        path: &[usize],
        transform_getter: F,
    ) -> Result<Isometry3<f64>, TfError>
    where
        F: Fn(&TransformHistory) -> Result<Isometry3<f64>, TfError>,
    {
        let mut isometry = Isometry3::identity();

        // Use direct indexing instead of windows(2) for better performance
        for i in 0..path.len().saturating_sub(1) {
            let from_idx = path[i];
            let to_idx = path[i + 1];

            // Try forward edge first, then reverse - avoids redundant contains_edge check
            if let Some(edge_weight) = self.graph.edge_weight(from_idx, to_idx) {
                isometry *= transform_getter(edge_weight)?;
            } else if let Some(edge_weight) = self.graph.edge_weight(to_idx, from_idx) {
                isometry *= transform_getter(edge_weight)?.inverse();
            } else {
                return Err(TfError::CouldNotFindTransform(format!(
                    "Edge transform not found for edge {} -> {}",
                    from_idx, to_idx
                )));
            }
        }

        Ok(isometry)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BufferTree, StampedIsometry, TransformType};
    use approx::assert_relative_eq;
    use nalgebra::geometry::Isometry3;

    #[test]
    fn test_buffer_tree_update() {
        let mut buffer_tree = BufferTree::new();

        let source = "A";
        let target = "B";

        let stamped_isometry = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0);

        // Add first transformation
        buffer_tree
            .update(source, target, stamped_isometry, TransformType::Static)
            .unwrap();

        // Test lookup
        let result = buffer_tree.lookup_latest_transform(&source, &target);
        assert!(result.is_ok());
    }

    #[test]
    fn test_separation_works() {
        // This test verifies that the core logic works without PyO3
        let mut buffer_tree = BufferTree::new();

        let transform = StampedIsometry::new([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 1.0);

        let result =
            buffer_tree.update("base_link", "target_link", transform, TransformType::Static);

        assert!(result.is_ok());

        let lookup_result = buffer_tree.lookup_latest_transform("base_link", "target_link");

        assert!(lookup_result.is_ok());
        let transform_result = lookup_result.unwrap();
        assert_eq!(transform_result.stamp(), 0.0); // Latest transform has stamp 0.0
    }

    #[test]
    fn test_buffer_tree_detects_cycles() {
        let mut buffer_tree = BufferTree::new();

        let a = "A";
        let b = "B";
        let c = "C";

        let stamped_isometry = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0);

        // Add edges A → B and B → C
        buffer_tree
            .update(a, b, stamped_isometry.clone(), TransformType::Static)
            .unwrap();
        buffer_tree
            .update(b, c, stamped_isometry.clone(), TransformType::Static)
            .unwrap();

        // Creating a cycle C → A should fail
        let result = buffer_tree.update(c, a, stamped_isometry.clone(), TransformType::Static);
        assert!(result.is_err());
        assert!(matches!(result, Err(TfError::InvalidGraph(_))));
    }

    #[test]
    fn test_multiple_incoming_edges() {
        let mut buffer_tree = BufferTree::new();

        let a = "A";
        let b = "B";
        let c = "C";

        let stamped_isometry = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0);

        buffer_tree
            .update(a, b, stamped_isometry.clone(), TransformType::Static)
            .unwrap();
        let result = buffer_tree.update(c, b, stamped_isometry.clone(), TransformType::Static);
        assert!(result.is_err());
        assert!(matches!(result, Err(TfError::InvalidGraph(_))));
    }

    #[test]
    fn test_find_path() {
        let mut buffer_tree = BufferTree::new();

        buffer_tree
            .update(
                "A",
                "B",
                StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0),
                TransformType::Dynamic,
            )
            .unwrap();

        buffer_tree
            .update(
                "A",
                "C",
                StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 2.0),
                TransformType::Dynamic,
            )
            .unwrap();

        buffer_tree
            .update(
                "B",
                "D",
                StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 3.0),
                TransformType::Dynamic,
            )
            .unwrap();

        buffer_tree
            .update(
                "B",
                "E",
                StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 3.0),
                TransformType::Dynamic,
            )
            .unwrap();

        println!("{:?}", buffer_tree.visualize());

        // Test path finding - these tests verify the internal find_path method works
        // Note: find_path is private, so we test it indirectly through lookup operations
        let result = buffer_tree.lookup_latest_transform("D", "B");
        assert!(result.is_ok());

        let result = buffer_tree.lookup_latest_transform("D", "C");
        assert!(result.is_ok());

        let result = buffer_tree.lookup_latest_transform("D", "E");
        assert!(result.is_ok());

        let result = buffer_tree.lookup_latest_transform("A", "E");
        assert!(result.is_ok());
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
            let stamped_isometry = StampedIsometry::new(translation, rotation, timestamp);

            buffer_tree
                .update(source, target, stamped_isometry, TransformType::Dynamic)
                .unwrap();
        }

        let transform = buffer_tree.lookup_latest_transform("base_link_inertia", "shoulder_link");
        assert!(transform.is_ok());
        let transform = transform.unwrap();
        let translation = transform.translation();
        let rotation = transform.rotation();

        // Check translation components
        assert_relative_eq!(translation[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[1], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 0.1625, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation[3], 1.0, epsilon = 1e-3);
        assert_relative_eq!(rotation[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation[1], 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation[2], 0.001, epsilon = 1e-3);

        // Test that we can find paths between arbitrary frames
        let transform = buffer_tree.lookup_latest_transform("base_link_inertia", "wrist_3_link");
        assert!(transform.is_ok());

        // Add these assertions
        let transform = transform.unwrap();
        let translation = transform.translation();
        let rotation = transform.euler_angles();

        // Check translation components
        assert_relative_eq!(translation[0], -0.001, epsilon = 1e-3);
        assert_relative_eq!(translation[1], -0.233, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 1.079, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation[0], -1.571, epsilon = 1e-2);
        assert_relative_eq!(rotation[1], -0.002, epsilon = 1e-2);
        assert_relative_eq!(rotation[2], 3.142, epsilon = 1e-2);
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
            let stamped_isometry = StampedIsometry::new(translation, rotation, timestamp);

            buffer_tree
                .update(source, target, stamped_isometry, TransformType::Dynamic)
                .unwrap();
        }

        println!("{}", buffer_tree.visualize());
        let transform = buffer_tree.lookup_latest_transform("wrist_3_link", "base_link_inertia");
        assert!(transform.is_ok());
        let transform = transform.unwrap();
        let translation = transform.translation();
        let rotation = transform.euler_angles();

        // Check translation components (should be inverse of original transform)
        assert_relative_eq!(translation[0], 0.001, epsilon = 1e-3);
        assert_relative_eq!(translation[1], 1.079, epsilon = 1e-3);
        assert_relative_eq!(translation[2], -0.233, epsilon = 1e-3);

        // Check euler angles (should be inverse of original transform)
        assert_relative_eq!(rotation[0], -1.571, epsilon = 1e-2);
        assert_relative_eq!(rotation[1], 0.00, epsilon = 1e-2);
        assert_relative_eq!(rotation[2], 3.14, epsilon = 1e-2);
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
            let stamped_isometry_1 = StampedIsometry::new(translation, rotation, timestamp_1);
            let stamped_isometry_2 = StampedIsometry::new(translation, rotation, timestamp_2);

            buffer_tree
                .update(source, target, stamped_isometry_1, TransformType::Dynamic)
                .unwrap();
            buffer_tree
                .update(source, target, stamped_isometry_2, TransformType::Dynamic)
                .unwrap();
        }

        let transform = buffer_tree.lookup_transform("base_link_inertia", "shoulder_link", 1.5);
        assert!(transform.is_ok());
        let transform = transform.unwrap();
        let translation = transform.translation();
        let rotation = transform.rotation();

        // Check translation components
        assert_relative_eq!(translation[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[1], 0.0, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 0.1625, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation[3], 1.0, epsilon = 1e-3);
        assert_relative_eq!(rotation[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation[1], 0.0, epsilon = 1e-3);
        assert_relative_eq!(rotation[2], 0.001, epsilon = 1e-3);

        // Test that we can find paths between arbitrary frames
        let transform = buffer_tree.lookup_latest_transform("base_link_inertia", "wrist_3_link");
        assert!(transform.is_ok());

        // Add these assertions
        let transform = transform.unwrap();
        let translation = transform.translation();
        let rotation = transform.euler_angles();

        // Check translation components
        assert_relative_eq!(translation[0], -0.001, epsilon = 1e-3);
        assert_relative_eq!(translation[1], -0.233, epsilon = 1e-3);
        assert_relative_eq!(translation[2], 1.079, epsilon = 1e-3);

        // Check quaternion components (w, x, y, z)
        assert_relative_eq!(rotation[0], -1.571, epsilon = 1e-2);
        assert_relative_eq!(rotation[1], -0.002, epsilon = 1e-2);
        assert_relative_eq!(rotation[2], 3.142, epsilon = 1e-2);

        // Check if correct error raised
        match buffer_tree.lookup_transform("base_link_inertia", "shoulder_link", 0.) {
            Err(TfError::AttemptedLookupInPast(_)) => {
                // The function returned the expected error variant
                assert!(true);
            }
            err => {
                // The function did not return the expected error variant
                assert!(
                    false,
                    "Expected TfError::AttemptedLookupInPast, got {:?}",
                    err
                );
            }
        }
        match buffer_tree.lookup_transform("base_link_inertia", "shoulder_link", 3.) {
            Err(TfError::AttemptedLookUpInFuture(_)) => {
                // The function returned the expected error variant
                assert!(true);
            }
            err => {
                // The function did not return the expected error variant
                assert!(
                    false,
                    "Expected TfError::AttemptedLookUpInFuture, got {:?}",
                    err
                );
            }
        }
        match buffer_tree.lookup_transform("XXXXX", "shoulder_link", 3.) {
            Err(TfError::CouldNotFindTransform(_)) => {
                // The function returned the expected error variant
                assert!(true);
            }
            _ => {
                // The function did not return the expected error variant
                assert!(false, "Expected TfError::CouldNotFindTransform");
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
                        rotation[3], // w
                        rotation[0], // x
                        rotation[1], // y
                        rotation[2], // z
                    )),
                ),
                stamp: 0.0,
            };
            buffer_tree
                .update(source, target, stamped_isometry, TransformType::Dynamic)
                .unwrap();
        }

        // Add transforms at t=1.0
        for (source, target, translation, rotation) in transforms_t1 {
            let stamped_isometry = StampedIsometry {
                isometry: Isometry3::from_parts(
                    nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        rotation[3], // w
                        rotation[0], // x
                        rotation[1], // y
                        rotation[2], // z
                    )),
                ),
                stamp: 1.0,
            };
            buffer_tree
                .update(source, target, stamped_isometry, TransformType::Dynamic)
                .unwrap();
        }

        // Look up transform at t=0.2
        let result = buffer_tree.lookup_transform("a", "f", 0.2).unwrap();

        // Expected values
        let expected_translation = [-0.02688966809486315, 0.8302180267299373, 1.6491944090937691];
        let _expected_rotation = [
            -0.23762484510717535,
            0.7704449853702972,
            -0.44625068910795557,
            -0.38834170517242694,
        ];

        // Assert translation components
        assert_relative_eq!(
            result.translation()[0],
            expected_translation[0],
            epsilon = 1e-6
        );
        assert_relative_eq!(
            result.translation()[1],
            expected_translation[1],
            epsilon = 1e-6
        );
        assert_relative_eq!(
            result.translation()[2],
            expected_translation[2],
            epsilon = 1e-6
        );

        // Additional test cases at different timestamps
        let test_cases = vec![
            // a->f at t=0.2
            (
                0.2,
                "a",
                "f",
                [-0.02688966809486315, 0.8302180267299373, 1.6491944090937691],
                [
                    0.7704449853702972,   // x
                    -0.44625068910795557, // y
                    -0.38834170517242694, // z
                    -0.23762484510717535, // w
                ],
            ),
            // a->f at t=0.5
            (
                0.5,
                "a",
                "f",
                [-0.7313014953477409, 0.8588360737131203, 1.3897218882465063],
                [
                    0.9561102276829774,   // x
                    0.10847159958471206,  // y
                    -0.1813323636450122,  // z
                    -0.20299191732296193, // w
                ],
            ),
            // a->f at t=0.8
            (
                0.8,
                "a",
                "f",
                [-1.5366396114062963, 0.5615052687815749, 1.2753385241243729],
                [
                    0.8191599958838035,   // x
                    0.5182799902870279,   // y
                    0.2443393917080692,   // z
                    0.025710201700027795, // w
                ],
            ),
            // f->a at t=0.2
            (
                0.2,
                "f",
                "a",
                [1.7623488465323582, 0.4146044950680975, 0.36339631387666715],
                [
                    0.7704449853702972,   // x
                    -0.44625068910795557, // y
                    -0.38834170517242694, // z
                    0.23762484510717535,  // w
                ],
            ),
            // // f->a at t=0.5
            (
                0.5,
                "f",
                "a",
                [0.8453152942269395, 1.4598104847572575, 0.5984342964929825],
                [
                    0.9561102276829774,  // x
                    0.10847159958471206, // y
                    -0.1813323636450122, // z
                    0.20299191732296193, // w
                ],
            ),
            // // f->a at t=0.8
            (
                0.8,
                "f",
                "a",
                [-0.43273825025921875, 1.1678464326290772, 1.6588882210342657],
                [
                    0.8191599958838035,    // x
                    0.5182799902870279,    // y
                    0.2443393917080692,    // z
                    -0.025710201700027795, // w
                ],
            ),
        ];

        // Test each case
        for (time, source, target, translation, rotation) in test_cases {
            let result = buffer_tree.lookup_transform(source, target, time).unwrap();

            // Assert translation components
            assert_relative_eq!(
                result.translation()[0],
                translation[0],
                epsilon = 1e-6,
                max_relative = 1e-6
            );
            assert_relative_eq!(
                result.translation()[1],
                translation[1],
                epsilon = 1e-6,
                max_relative = 1e-6
            );
            assert_relative_eq!(
                result.translation()[2],
                translation[2],
                epsilon = 1e-6,
                max_relative = 1e-6
            );

            // Assert rotation components (handle quaternion sign ambiguity)
            let result_rot = result.rotation();
            let expected_rot = rotation;

            // Check if the quaternions are equivalent (q and -q represent the same rotation)
            let mut matches = true;
            let mut neg_matches = true;

            for i in 0..4 {
                if !approx::relative_eq!(result_rot[i], expected_rot[i], epsilon = 1e-6) {
                    matches = false;
                }
                if !approx::relative_eq!(result_rot[i], -expected_rot[i], epsilon = 1e-6) {
                    neg_matches = false;
                }
            }

            assert!(
                matches || neg_matches,
                "Rotation mismatch: result={:?}, expected={:?}",
                result_rot,
                expected_rot
            );
        }
    }

    #[test]
    fn test_lookup_latest_transform_no_path() {
        let mut buffer_tree = BufferTree::new();

        // Add some transforms but leave frames disconnected
        buffer_tree
            .update(
                "A",
                "B",
                StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0),
                TransformType::Static,
            )
            .unwrap();

        buffer_tree
            .update(
                "C",
                "D",
                StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0),
                TransformType::Static,
            )
            .unwrap();

        // Try to lookup transform between disconnected frames
        let result = buffer_tree.lookup_latest_transform("A", "C");
        assert!(result.is_err());
        assert!(matches!(result, Err(TfError::CouldNotFindTransform(_))));

        // Try to lookup transform between non-existent frames
        let result = buffer_tree.lookup_latest_transform("X", "Y");
        assert!(result.is_err());
        assert!(matches!(result, Err(TfError::CouldNotFindTransform(_))));
    }

    #[test]
    fn test_transform_history_buffer_window() {
        let buffer_window = 1.0; // 1 second window
        let mut history = TransformHistory::new(TransformType::Dynamic, buffer_window);

        // Add transforms at different times
        let transforms = vec![
            (0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            (0.2, [0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            (0.4, [0.4, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            (0.6, [0.6, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            (0.8, [0.8, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            (1.2, [1.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), // This should cause older transforms to be removed
        ];

        // Add all transforms
        for (time, translation, rotation) in transforms {
            let stamped_isometry = StampedIsometry::new(translation, rotation, time);
            history.update(stamped_isometry);
        }

        // Check that oldest transforms (0.0, 0.2) were removed due to buffer window
        assert!(history.history.len() < 6);
        assert!(history.history.front().unwrap().stamp >= 0.2);

        // Check that newest transforms are still present
        assert_eq!(history.history.back().unwrap().stamp, 1.2);

        // Verify interpolation still works within the valid time range
        let result = history.interpolate_isometry_at_time(1.0);
        assert!(result.is_ok());
        let interpolated = result.unwrap();
        assert_relative_eq!(interpolated.translation.vector[0], 1.0, epsilon = 1e-6);

        // Verify interpolation fails for times outside the buffer window
        let result = history.interpolate_isometry_at_time(0.1);
        assert!(matches!(result, Err(TfError::AttemptedLookupInPast(_))));
    }

    #[test]
    fn test_ancestor_updates() {
        let mut buffer_tree = BufferTree::new();

        let a = "A";
        let b = "B";
        let c = "C";
        let r = "R";

        let tf = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0);

        // A -> B -> C
        buffer_tree
            .update(a, b, tf.clone(), TransformType::Static)
            .unwrap();
        buffer_tree
            .update(b, c, tf.clone(), TransformType::Static)
            .unwrap();

        // Check ancestors
        let get_ancestors = |buffer: &BufferTree, node: &str| -> Vec<String> {
            let id = buffer.index.get(node).unwrap();
            buffer.index.get_node(id).unwrap().ancestors.clone()
        };

        assert_eq!(get_ancestors(&buffer_tree, a), Vec::<String>::new());
        assert_eq!(get_ancestors(&buffer_tree, b), vec![a.to_string()]);
        assert_eq!(
            get_ancestors(&buffer_tree, c),
            vec![a.to_string(), b.to_string()]
        );

        // Add R -> A
        buffer_tree
            .update(r, a, tf.clone(), TransformType::Static)
            .unwrap();

        // Check updates
        assert_eq!(get_ancestors(&buffer_tree, a), vec![r.to_string()]);
        assert_eq!(
            get_ancestors(&buffer_tree, b),
            vec![r.to_string(), a.to_string()]
        );
        assert_eq!(
            get_ancestors(&buffer_tree, c),
            vec![r.to_string(), a.to_string(), b.to_string()]
        );
    }
}
