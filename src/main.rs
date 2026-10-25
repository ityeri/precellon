mod physics;

use glam::Vec2;
use macroquad::input::{
    KeyCode, MouseButton, is_key_pressed, is_mouse_button_down, is_mouse_button_pressed,
    is_mouse_button_released, mouse_position, mouse_wheel,
};
use macroquad::{
    color,
    shapes::{draw_circle, draw_line},
    time::get_frame_time,
    window::{Conf, next_frame, screen_height, screen_width},
};
use physics::{
    joint::Joint,
    point::{Point, PointKey},
    world::{PhysicsConfig, World},
};
use scrollrs::Projector;
use std::collections::{HashMap, HashSet};
use uuid::Uuid;

fn window_conf() -> Conf {
    Conf {
        window_title: "precellon".to_owned(),
        window_width: 500,
        window_height: 500,
        fullscreen: false,
        window_resizable: true,
        ..Default::default()
    }
}

#[derive(Clone)]
struct JointCreatingContext<K: PointKey> {
    source_point_key: K,
    target_point_key: K,
    joint: Joint<K>,
    is_mounted: bool,
}

#[macroquad::main(window_conf)]
async fn main() {
    println!("Hello, world!");

    let mut adaptor = scrollrs::ScrollAdaptor::new(0.0, 0.0, None, None, None);
    adaptor = adaptor.set_zoom(0.5);
    let wheel_sensitivity = 0.03;

    let mut current_mouse_pos = {
        let mouse_position = mouse_position();
        Vec2::new(mouse_position.0, mouse_position.1)
    };
    let mut prev_mouse_pos: Vec2;

    let fixed_point_key = Uuid::new_v4();
    let mut world = World::<Uuid> {
        points: HashMap::from([(
            fixed_point_key,
            Point::new(fixed_point_key, Vec2::new(0.0, 0.0)),
        )]),
        joints: HashSet::from([]),
        kinematics: HashMap::from([(fixed_point_key, Vec2::ZERO)]),
        config: PhysicsConfig {
            stiffness: 1.0,
            attenuation_force: 0.0001,
            gravity_force: Vec2::new(0.0, -10.0),
        },
    };
    let mut is_running = true;

    // let mut controlling_point_key: Option<Uuid> = None;
    let mut joint_creating_context: Option<JointCreatingContext<Uuid>> = None;
    let mut dragging_point: Option<(Uuid, bool)> = None;

    loop {
        let dt = get_frame_time();

        let screen_width = screen_width();
        let screen_height = screen_height();

        prev_mouse_pos = current_mouse_pos;
        current_mouse_pos = {
            let mouse_position = mouse_position();
            Vec2::new(mouse_position.0, mouse_position.1)
        };
        let unprojected_mouse_pos = adaptor.unproject(current_mouse_pos.as_dvec2()).as_vec2();

        adaptor = adaptor.resized(screen_width as f64, screen_height as f64, None, None);

        if is_key_pressed(KeyCode::Space) {
            is_running = !is_running;
        }
        if is_mouse_button_pressed(MouseButton::Right) {
            joint_creating_context = if let Some(context) = joint_creating_context {
                world.kinematics.remove(&context.target_point_key);
                None
            } else {
                let clicked_point_key = world
                    .points
                    .values()
                    .find(|point| {
                        current_mouse_pos
                            .as_dvec2()
                            .distance(adaptor.project(point.position.as_dvec2()))
                            < 10.0
                    })
                    .map(|point| point.key);

                let source_point_key = if let Some(key) = clicked_point_key {
                    key
                } else {
                    let key = Uuid::new_v4();
                    world
                        .points
                        .insert(key, Point::new(key, unprojected_mouse_pos));
                    key
                };

                let target_point_key = Uuid::new_v4();
                world.points.insert(
                    target_point_key,
                    Point::new(target_point_key, unprojected_mouse_pos),
                );
                world
                    .kinematics
                    .insert(target_point_key, unprojected_mouse_pos);

                let joint = Joint::new(source_point_key, target_point_key, 0.0);
                world.joints.insert(joint.clone());

                Some(JointCreatingContext {
                    source_point_key,
                    target_point_key,
                    joint: joint,
                    is_mounted: false,
                })
            };
        }

        joint_creating_context = if let Some(context) = joint_creating_context.take() {
            Some(
                if let Some(near_point_key) = world
                    .points
                    .values()
                    .find(|point| {
                        current_mouse_pos
                            .as_dvec2()
                            .distance(adaptor.project(point.position.as_dvec2()))
                            < 10.0
                            && point.key != context.source_point_key
                            && point.key != context.target_point_key
                    })
                    .map(|p| p.key)
                {
                    if !context.is_mounted {
                        world.kinematics.remove(&context.target_point_key);
                        world.points.remove(&context.target_point_key);
                    }
                    world.joints.remove(&context.joint);

                    let source_point_pos = world.points[&context.source_point_key].position;
                    let target_point_pos = world.points[&near_point_key].position;

                    let new_context = JointCreatingContext {
                        source_point_key: context.source_point_key,
                        target_point_key: near_point_key,
                        joint: Joint {
                            point1: context.source_point_key,
                            point2: near_point_key,
                            length: source_point_pos.distance(target_point_pos),
                        },
                        is_mounted: true,
                    };

                    world.joints.insert(new_context.joint.clone());

                    new_context
                } else {
                    world
                        .kinematics
                        .insert(context.target_point_key, unprojected_mouse_pos);

                    if let Some(old_joint) = world.joints.take(&context.joint) {
                        let source_point_pos = world.points[&context.source_point_key].position;
                        let target_point_pos = world.points[&context.target_point_key].position;

                        let new_context = JointCreatingContext {
                            joint: Joint {
                                length: source_point_pos.distance(target_point_pos),
                                ..old_joint
                            },
                            is_mounted: false,
                            ..context
                        };

                        world.joints.insert(new_context.joint.clone());

                        new_context
                    } else {
                        context
                    }
                },
            )
        } else {
            joint_creating_context
        };

        if is_mouse_button_pressed(MouseButton::Left) {
            let nearby = world.points.values().find(|point| {
                current_mouse_pos
                    .as_dvec2()
                    .distance(adaptor.project(point.position.as_dvec2()))
                    < 10.0
            });
            if let Some(point) = nearby {
                let key = point.key;
                let was_kinematic = world.kinematics.contains_key(&key);
                world.kinematics.insert(key, unprojected_mouse_pos);
                dragging_point = Some((key, was_kinematic));
            }
        }

        if is_mouse_button_released(MouseButton::Left) {
            if let Some((key, was_kinematic)) = dragging_point.take() {
                if !was_kinematic {
                    world.kinematics.remove(&key);
                }
            }
        }

        if let Some((key, _)) = dragging_point {
            world.kinematics.insert(key, unprojected_mouse_pos);
        } else if is_mouse_button_down(MouseButton::Left) {
            let mouse_delta = current_mouse_pos - prev_mouse_pos;
            let camera_delta = adaptor.unproject_scalev(mouse_delta.as_dvec2());
            adaptor = adaptor.moved(-camera_delta);
        }

        let (_wheel_x, wheel_y) = mouse_wheel();
        let zoom_amount = wheel_y as f64 * wheel_sensitivity;
        adaptor = adaptor.zoom(zoom_amount);

        adaptor = adaptor.updated(dt as f64);

        world.kinematics.insert(fixed_point_key, Vec2::ZERO);

        world.points.values().for_each(|point| {
            let projected_pos = adaptor.project(point.position.as_dvec2()).as_vec2();

            draw_circle(projected_pos.x, projected_pos.y, 4.0, color::WHITE);
        });

        world.joints.iter().for_each(|joint| {
            let point1 = world.points.get(&joint.point1).unwrap();
            let point2 = world.points.get(&joint.point2).unwrap();
            let projected_pos1 = adaptor.project(point1.position.as_dvec2()).as_vec2();
            let projected_pos2 = adaptor.project(point2.position.as_dvec2()).as_vec2();

            draw_line(
                projected_pos1.x,
                projected_pos1.y,
                projected_pos2.x,
                projected_pos2.y,
                1.0,
                color::WHITE,
            );
        });

        if is_running {
            world = world.update(dt, 64);
        } else {
            world = world.kinematic_update(64);
        }

        next_frame().await
    }
}
