use std::cmp::Ordering;
use std::collections::{VecDeque, HashMap};

use nalgebra::geometry::Isometry3;
use petgraph::algo::is_cyclic_undirected;
use petgraph::graphmap::DiGraphMap;


pub enum TransformType {
    /// Does not change over time
    Static,
    /// Changes over time
    Dynamic
}


#[derive(Clone, Debug)]
struct StampedIsometry {
    isometry: Isometry3<f64>,
    /// The time at which this isometry was recorded in seconds
    stamp: f64
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


struct TransformHistory {
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
}

struct NodeIndex {
    max_node_id: usize,
    node_ids: HashMap<String, usize>
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

        let node_id = *self.node_ids.entry(node)
            .or_insert_with(|| {
                let node_id = *max_node_id;
                *max_node_id += 1;
                node_id
            });

        node_id
    }
}


struct BufferTree {
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

    pub fn update(&mut self, source: String, target: String, stamped_isometry: StampedIsometry, kind: TransformType) {
        let source = self.index.index(source);
        let target = self.index.index(target);

        if !self.graph.contains_node(source) {
            self.graph.add_node(source);
        }
        if !self.graph.contains_node(target) {
            self.graph.add_node(target);
        }

        if !self.graph.contains_edge(source, target) {
            self.graph.add_edge(source, target, TransformHistory::new(kind));
            if is_cyclic_undirected(&self.graph) {
                panic!("Cyclic graph detected");
            }
        }
        self.graph.edge_weight_mut(source, target).unwrap().update(stamped_isometry);
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
        buffer_tree.update(source.clone(), target.clone(), stamped_isometry.clone(), TransformType::Static);

        // Ensure the nodes exist
        let source_idx = buffer_tree.index.index(source.clone());
        let target_idx = buffer_tree.index.index(target.clone());

        assert!(buffer_tree.graph.contains_node(source_idx));
        assert!(buffer_tree.graph.contains_node(target_idx));

        // Ensure edge exists
        assert!(buffer_tree.graph.contains_edge(source_idx, target_idx));

        // Check that the transform history is updated
        let edge_weight = buffer_tree.graph.edge_weight(source_idx, target_idx).unwrap();
        assert_eq!(edge_weight.history.len(), 1);
        assert_eq!(edge_weight.history.front().unwrap().stamp, 1.0);

        // Add another transformation
        let stamped_isometry_2 = StampedIsometry {
            isometry: Isometry3::identity(),
            stamp: 2.0,
        };
        buffer_tree.update(source.clone(), target.clone(), stamped_isometry_2.clone(), TransformType::Static);

        // Ensure the history is updated
        let edge_weight = buffer_tree.graph.edge_weight(source_idx, target_idx).unwrap();
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
        buffer_tree.update(a.clone(), b.clone(), stamped_isometry.clone(), TransformType::Static);
        buffer_tree.update(b.clone(), c.clone(), stamped_isometry.clone(), TransformType::Static);

        // Creating a cycle C → A should panic
        buffer_tree.update(c.clone(), a.clone(), stamped_isometry.clone(), TransformType::Static);
    }
}