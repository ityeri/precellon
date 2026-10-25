use glam::Vec2;
use std::hash::Hash;

pub trait PointKey: Copy + Eq + Hash + Send + Sync {}
impl<T: Copy + Hash + Eq + Send + Sync> PointKey for T {}

#[derive(Clone)]
pub struct Point<K: PointKey> {
    pub key: K,
    pub position: Vec2,
    pub prev_position: Vec2,
}

impl<K: PointKey> Point<K> {
    pub fn new(key: K, position: Vec2) -> Self {
        Point {
            key,
            position,
            prev_position: position,
        }
    }
}
