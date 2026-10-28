use glam::{Vec2, vec2};
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use crate::physics::joint::JointKey;

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
    pub joints: HashMap<JointKey<K>, Joint<K>>,
    pub kinematics: HashMap<K, Vec2>,
    pub config: PhysicsConfig,
}

impl<K: PointKey> World<K> {
    pub fn update(self, dt: f32, substeps: u32) -> Self {
        self.solve_joints_with_substeps(substeps)
            .kinematic_update()
            .update_points(dt)
    }

    pub fn kinematic_update(self) -> Self {
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
                                    + (kinematic_position - point.position),
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

    fn solve_joints_with_substeps(self, substeps: u32) -> Self {
        (0..substeps).fold(self, |acc, _| acc.solve_joints())
    }
    fn solve_joints(self) -> Self {
        self.joints
            .clone()
            .keys()
            .fold(self, |acc, &joint_key| acc.solve_joint(joint_key))
    }
    fn solve_joint(mut self, joint_key: JointKey<K>) -> Self {
        let joint = self.joints.get(&joint_key).unwrap();
        let point1 = self.points.get(&joint_key.a).unwrap();
        let point2 = self.points.get(&joint_key.b).unwrap();

        let point1_movement_weight = if self.kinematics.contains_key(&point1.key) {
            0.0
        } else {
            1.0
        };
        let point2_movement_weight = if self.kinematics.contains_key(&point2.key) {
            0.0
        } else {
            1.0
        };

        let distance_error = point1.position.distance(point2.position) - joint.length;
        let direction = (point2.position - point1.position).normalize_or(vec2(1.0, 0.0));

        self.points.get_mut(&joint_key.a).unwrap().position +=
            direction * distance_error * 0.5 * point1_movement_weight;
        self.points.get_mut(&joint_key.b).unwrap().position -=
            direction * distance_error * 0.5 * point2_movement_weight;

        self
    }

    fn update_points(self, dt: f32) -> Self {
        World {
            points: self
                .points
                .iter()
                .map(|(&key, point)| {
                    (
                        key,
                        if self.kinematics.contains_key(&key) {
                            point.clone()
                        } else {
                            let velocity = point.position - point.prev_position;
                            let acc = self.config.gravity_force;
                            let new_velocity = velocity + acc * dt.powi(2);

                            let new_position = point.position + new_velocity;

                            Point {
                                key: point.key,
                                position: new_position,
                                prev_position: point.position,
                            }
                        },
                    )
                })
                .collect::<HashMap<K, Point<K>>>(),
            joints: self.joints,
            kinematics: self.kinematics,
            config: self.config,
        }
    }

    // fn update_point(&self, point: &Point<K>, dt: f32) -> Point<K> {
    //     let connected_joints = self
    //         .joints
    //         .iter()
    //         .filter(|j| j.point1 == point.key || j.point2 == point.key);
    //
    //     let position_correction; = connected_joints
    //         .map(|joint| {
    //             let other_point = if joint.point1 != point.key {
    //                 self.points.get(&joint.point1).unwrap()
    //             } else {
    //                 self.points.get(&joint.point2).unwrap()
    //             };
    //
    //             let distance_error = point.position.distance(other_point.position) - joint.length;
    //             let direction =
    //                 (other_point.position - point.position).normalize_or(Vec2::new(1.0, 0.0));
    //
    //             direction * distance_error * 0.5 * self.config.stiffness
    //         })
    //         .sum::<Vec2>();
    //
    //     let velocity = point.position - point.prev_position;
    //     let acc = self.config.gravity_force;
    //     let new_velocity = velocity + acc * dt.powi(2);
    //
    //     let new_position = point.position + new_velocity + position_correction;
    //
    //     Point {
    //         key: point.key,
    //         position: new_position,
    //         // prev_position: point.position + (new_position - point.position) * 0.01,
    //         prev_position: point.position,
    //     }
    // }
}
