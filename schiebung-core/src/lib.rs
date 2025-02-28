use std::cmp::Ordering;
use std::collections::{HashMap, VecDeque};

use nalgebra::geometry::Isometry3;
use petgraph::algo::is_cyclic_undirected;
use petgraph::dot::{Config, Dot};
use petgraph::graphmap::DiGraphMap;


#[derive(Clone, Debug)]
pub enum TransformType {
    /// Does not change over time
    Static,
    /// Changes over time
    Dynamic,
}

/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    CouldNotAcquireLock,
}

#[derive(Clone, Debug)]
pub struct StampedIsometry {
    pub isometry: Isometry3<f64>,
    /// The time at which this isometry was recorded in seconds
    pub stamp: f64,
}

impl PartialEq for StampedIsometry {
    fn eq(&self, other: &Self) -> bool {
        self.stamp == other.stamp
    }
}

impl Eq for StampedIsometry {}

impl Ord for StampedIsometry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.stamp.partial_cmp(&other.stamp).unwrap()
    }
}

impl PartialOrd for StampedIsometry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug)]
pub struct TransformHistory {
    history: VecDeque<StampedIsometry>,
    kind: TransformType,
    max_history: usize,
}

impl TransformHistory {
    pub fn new(kind: TransformType) -> Self {
        TransformHistory {
            history: VecDeque::new(),
            kind,
            max_history: 100,
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

pub struct BufferTree {
    graph: DiGraphMap<usize, TransformHistory>,
    index: NodeIndex,
}

impl BufferTree {
    pub fn new() -> Self {
        BufferTree {
            graph: DiGraphMap::new(),
            index: NodeIndex::new(),
        }
    }

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
                .add_edge(source, target, TransformHistory::new(kind));
            if is_cyclic_undirected(&self.graph) {
                panic!("Cyclic graph detected");
            }
        }
        self.graph
            .edge_weight_mut(source, target)
            .unwrap()
            .update(stamped_isometry);
    }

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

    pub fn lookup_latest_transform(
        &mut self,
        source: String,
        target: String,
    ) -> Option<StampedIsometry> {
        let mut isometry = Isometry3::identity();
        if !self.index.contains(&source) | self.index.contains(&target) {
            return None
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
                    .edge_weight(source_idx, target_idx)
                    .unwrap()
                    .history
                    .back()
                    .unwrap()
                    .isometry
                    .inverse();
            }
        }
        Some(StampedIsometry {
            isometry,
            stamp: 0.0,
        })
    }

    pub fn lookup_transform(
        &mut self,
        source: String,
        target: String,
        time: f64,
    ) -> Option<StampedIsometry> {
        let mut isometry = Isometry3::identity();
        for pair in self.find_path(source, target).unwrap().windows(2) {
            let source_idx = pair[0];
            let target_idx = pair[1];

            if self.graph.contains_edge(source_idx, target_idx) {
                isometry *= self
                    .graph
                    .edge_weight(source_idx, target_idx)
                    .unwrap()
                    .interpolate_isometry_at_time(time)
                    .unwrap();
            } else {
                isometry *= self
                    .graph
                    .edge_weight(source_idx, target_idx)
                    .unwrap()
                    .interpolate_isometry_at_time(time)
                    .unwrap()
                    .inverse();
            }
        }
        Some(StampedIsometry {
            isometry,
            stamp: time,
        })
    }

    pub fn visualize(&self) -> Dot<&DiGraphMap<usize, TransformHistory>> {
        Dot::with_config(&self.graph, &[Config::GraphContentOnly])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
