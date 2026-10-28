use std::hash::{Hash, Hasher};

use super::point::PointKey;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct JointKey<K: PointKey> {
    pub a: K,
    pub b: K,
}
impl<K: PointKey> JointKey<K> {
    pub fn new(a: K, b: K) -> Self {
        if a < b {
            JointKey { a: a, b: b }
        } else {
            JointKey { a: b, b: a }
        }
    }
}
impl<K: PointKey> Hash for JointKey<K> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.a.hash(state);
        self.b.hash(state);
    }
}

#[derive(Clone)]
pub struct Joint<K: PointKey> {
    pub key: JointKey<K>,
    pub length: f32,
}

impl<K: PointKey> Joint<K> {
    pub fn new(a: K, b: K, length: f32) -> Self {
        Joint {
            key: JointKey::new(a, b),
            length,
        }
    }
}
