use std::hash::{Hash, Hasher};

use super::point::PointKey;

#[derive(Clone)]
pub struct Joint<K: PointKey> {
    pub point1: K,
    pub point2: K,
    pub length: f32,
}

impl<K: PointKey> Joint<K> {
    pub fn new(point1: K, point2: K, length: f32) -> Self {
        Joint {
            point1,
            point2,
            length,
        }
    }
}

impl<K: PointKey> Hash for Joint<K> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.point1.hash(state);
        self.point2.hash(state);
    }
}

impl<K: PointKey> PartialEq for Joint<K> {
    fn eq(&self, other: &Self) -> bool {
        self.point1 == other.point1 && self.point2 == other.point2 && self.length == other.length
    }
}

impl<K: PointKey> Eq for Joint<K> {}
