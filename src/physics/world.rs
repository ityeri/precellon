use glam::Vec2;
use std::collections::{HashMap, HashSet};

use super::{
    joint::Joint,
    point::{Point, PointKey},
};

#[derive(Clone)]
pub struct PhysicsConfig {
    pub stiffness: f32,
    pub attenuation_force: f32,
    pub gravity_force: Vec2,
}

pub struct World<K: PointKey> {
    pub points: HashMap<K, Point<K>>,
    pub joints: HashSet<Joint<K>>,
    pub kinematics: HashMap<K, Vec2>,
    pub config: PhysicsConfig,
}

impl<K: PointKey> World<K> {
    pub fn update(self, dt: f32, substeps: u32) -> Self {
        self.kinematic_update(substeps).substep_update(dt, substeps)
    }

    pub fn kinematic_update(self, substeps: u32) -> Self {
        World {
            points: self
                .points
                .iter()
                .map(|(&key, point)| {
                    (
                        key,
                        if let Some(&kinematic_position) = self.kinematics.get(&key) {
                            Point {
                                key,
                                position: kinematic_position,
                                prev_position: point.position
                                    + (kinematic_position - point.position) / substeps as f32,
                            }
                        } else {
                            point.clone()
                        },
                    )
                })
                .collect::<HashMap<K, Point<K>>>(),
            ..self
        }
    }

    fn substep_update(self, dt: f32, substeps: u32) -> Self {
        (0..substeps).fold(self, |acc, _| World {
            points: acc
                .points
                .iter()
                .map(|(&key, point)| {
                    (
                        key,
                        if acc.kinematics.contains_key(&key) {
                            point.clone()
                        } else {
                            acc.update_point(point, dt / substeps as f32, 1.0 / substeps as f32)
                        },
                    )
                })
                .collect::<HashMap<K, Point<K>>>(),
            ..acc
        })
    }

    fn update_point(&self, point: &Point<K>, dt: f32, step_dt: f32) -> Point<K> {
        let connected_joints = self
            .joints
            .iter()
            .filter(|j| j.point1 == point.key || j.point2 == point.key);

        let position_correction = connected_joints
            .map(|joint| {
                let other_point = if joint.point1 != point.key {
                    self.points.get(&joint.point1).unwrap()
                } else {
                    self.points.get(&joint.point2).unwrap()
                };

                let distance_error = point.position.distance(other_point.position) - joint.length;
                let direction =
                    (other_point.position - point.position).normalize_or(Vec2::new(1.0, 0.0));

                direction * distance_error * 0.5 * self.config.stiffness
            })
            .sum::<Vec2>();

        let velocity = point.position - point.prev_position;
        let acc = self.config.gravity_force;
        let new_velocity =
            (velocity + acc * dt.powi(2)) * (1.0 - self.config.attenuation_force).max(0.0);

        Point {
            key: point.key,
            // TODO step_dt is really necessary?
            position: point.position + new_velocity + position_correction * step_dt,
            prev_position: point.position,
        }
    }
}
