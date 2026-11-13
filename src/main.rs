mod physics;

use glam::{Vec2, vec2};
use macroquad::input::{
    KeyCode, MouseButton, is_key_down, is_key_pressed, is_mouse_button_down,
    is_mouse_button_pressed, is_mouse_button_released, mouse_position, mouse_wheel,
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

use crate::physics::joint::JointKey;

const POINT_SNAP_RADIUS: f32 = 10.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "precellon".to_owned(),
        window_width: 500,
        window_height: 500,
        fullscreen: false,
        window_resizable: true,
        high_dpi: true,
        ..Default::default()
    }
}

#[derive(Clone)]
struct StructureCreatingContext<K: PointKey> {
    tail_point_keys: Vec<K>,
    head_point_key: K,
    head_joint_key: JointKey<K>,
    attach_info: Option<AttachInfo>,
}
#[derive(Clone)]
struct AttachInfo {
    is_joint_dependent: bool,
}
impl AttachInfo {
    pub fn non_dependent() -> Self {
        Self {
            is_joint_dependent: false,
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    println!("Hello, world!");

    let mut adapter = scrollrs::ScrollAdapter::new(0.0, 0.0, None, None, None);
    adapter = adapter.set_zoom(0.5);
    let wheel_sensitivity = 0.03;

    let mut simulation_time_elapsed = 0.0f32;

    let mut current_mouse_pos = {
        let position = mouse_position();
        Vec2::new(position.0, position.1)
    };
    let mut prev_mouse_pos: Vec2;

    // let ground_size = 10
    // let ground_point_keys = (0..ground_size).map(||)
    // TODO

    let mut world = World::<Uuid> {
        points: HashMap::from([]),
        joints: HashMap::from([]),
        kinematics: HashMap::from([]),
        config: PhysicsConfig {
            stiffness: 1.0,
            attenuation_force: 0.0001,
            gravity_force: Vec2::new(0.0, -10.0),
        },
    };
    // let fixed_point_key1 = Uuid::new_v4();
    // let fixed_point_key2 = Uuid::new_v4();
    // let moving_point_key1 = Uuid::new_v4();
    // let moving_joint_key1 = JointKey::new(fixed_point_key1, moving_point_key1);
    // let moving_point_key2 = Uuid::new_v4();
    // let moving_joint_key2 = JointKey::new(fixed_point_key2, moving_point_key2);
    //
    // let mut world = World::<Uuid> {
    //     points: HashMap::from([
    //         (fixed_point_key1, Point::new(fixed_point_key1, Vec2::ZERO)),
    //         (fixed_point_key2, Point::new(fixed_point_key2, Vec2::ZERO)),
    //         (moving_point_key1, Point::new(moving_point_key1, Vec2::ZERO)),
    //         (moving_point_key2, Point::new(moving_point_key2, Vec2::ZERO)),
    //     ]),
    //     joints: HashMap::from([
    //         (
    //             moving_joint_key1,
    //             Joint::new(fixed_point_key1, moving_point_key1, 1.0),
    //         ),
    //         (
    //             moving_joint_key2,
    //             Joint::new(fixed_point_key2, moving_point_key2, 1.0),
    //         ),
    //     ]),
    //     kinematics: HashMap::new(),
    //     config: PhysicsConfig {
    //         stiffness: 1.0,
    //         attenuation_force: 0.0001,
    //         gravity_force: Vec2::new(0.0, -10.0),
    //     },
    // };
    let mut is_running = true;

    // let mut controlling_point_key: Option<Uuid> = None;
    let mut structure_creating_context: Option<StructureCreatingContext<Uuid>> = None;
    let mut dragging_point_key: Option<Uuid> = None;

    loop {
        let dt = get_frame_time();

        let screen_width = screen_width();
        let screen_height = screen_height();

        prev_mouse_pos = current_mouse_pos;
        current_mouse_pos = {
            let mouse_position = mouse_position();
            Vec2::new(mouse_position.0, mouse_position.1)
        };
        let world_mouse_pos = adapter.unproject(current_mouse_pos.as_dvec2()).as_vec2();

        adapter = adapter.resized(screen_width as f64, screen_height as f64, None, None);

        if is_key_pressed(KeyCode::Space) {
            is_running = !is_running;
        }

        if is_mouse_button_pressed(MouseButton::Right) {
            structure_creating_context = match structure_creating_context {
                Some(ctx) => {
                    world.kinematics.remove(&ctx.head_point_key);
                    ctx.tail_point_keys.iter().for_each(|key| {
                        world.kinematics.remove(&key);
                    });

                    None
                }
                None => {
                    let nerby_point = world
                        .points
                        .values()
                        .filter(|point| {
                            adapter
                                .project(point.position.as_dvec2())
                                .distance(current_mouse_pos.as_dvec2())
                                < POINT_SNAP_RADIUS as f64
                        })
                        .next();
                    let (source_point, is_attached) = if let Some(nerby_point) = nerby_point {
                        (nerby_point.clone(), true)
                    } else {
                        (Point::new(Uuid::new_v4(), world_mouse_pos), false)
                    };
                    let dest_point = Point::new(Uuid::new_v4(), world_mouse_pos);
                    let joint = Joint::new(source_point.key, dest_point.key, 0.0);

                    world.points.insert(source_point.key, source_point.clone());
                    world.points.insert(dest_point.key, dest_point.clone());
                    world.joints.insert(joint.key, joint.clone());
                    if !is_attached {
                        world.kinematics.insert(source_point.key, world_mouse_pos);
                    }

                    Some(StructureCreatingContext {
                        tail_point_keys: vec![source_point.key],
                        head_point_key: dest_point.key,
                        head_joint_key: joint.key,
                        attach_info: if is_attached {
                            Some(AttachInfo::non_dependent())
                        } else {
                            None
                        },
                    })
                }
            }
        }
        if is_mouse_button_pressed(MouseButton::Left) {
            match structure_creating_context {
                Some(ctx) => {
                    let new_point_key = Uuid::new_v4();
                    let joint = Joint::new(ctx.head_point_key, new_point_key, 0.0);
                    world
                        .points
                        .insert(new_point_key, Point::new(new_point_key, world_mouse_pos));
                    world.joints.insert(joint.key, joint.clone());
                    if ctx.attach_info.is_none() {
                        world.kinematics.insert(ctx.head_point_key, world_mouse_pos);
                    }

                    structure_creating_context = Some(StructureCreatingContext {
                        tail_point_keys: [ctx.tail_point_keys.as_slice(), &[ctx.head_point_key]]
                            .concat(),
                        head_point_key: new_point_key,
                        head_joint_key: joint.key,
                        attach_info: None,
                    })
                }
                None => {
                    let nerby_point_key = world
                        .points
                        .keys()
                        .filter(|point_key| {
                            adapter
                                .project(world.points[point_key].position.as_dvec2())
                                .distance(current_mouse_pos.as_dvec2())
                                < POINT_SNAP_RADIUS as f64
                        })
                        .next();

                    match nerby_point_key {
                        Some(&nerby_point_key) => {
                            if is_key_down(KeyCode::LeftShift) {
                                world
                                    .joints
                                    .clone()
                                    .keys()
                                    .filter(|joint_key| {
                                        joint_key.a == nerby_point_key
                                            || joint_key.b == nerby_point_key
                                    })
                                    .for_each(|joint_key| {
                                        world.joints.remove(joint_key);
                                    });
                                world.points.remove(&nerby_point_key);
                            } else {
                                dragging_point_key = Some(nerby_point_key);
                            }
                        }
                        None => {}
                    }
                }
            }
        }
        if is_mouse_button_released(MouseButton::Left) {
            if let Some(point_key) = dragging_point_key {
                world.kinematics.remove(&point_key);
                dragging_point_key = None;
            }
        }

        if is_mouse_button_down(MouseButton::Left) {
            if dragging_point_key.is_none() {
                let mouse_delta = current_mouse_pos - prev_mouse_pos;
                let camera_delta = adapter.unproject_scalev(mouse_delta.as_dvec2());
                adapter = adapter.moved(-camera_delta);
            }
        }

        structure_creating_context = match structure_creating_context {
            Some(ctx) => Some(match ctx.attach_info.as_ref() {
                Some(attach_info) => {
                    let attached_point = world.points[&ctx.head_point_key].clone();

                    if adapter
                        .project(attached_point.position.as_dvec2())
                        .distance(current_mouse_pos.as_dvec2())
                        < POINT_SNAP_RADIUS as f64
                    {
                        ctx
                    } else {
                        if !attach_info.is_joint_dependent {
                            world.joints.remove(&ctx.head_joint_key);
                        }

                        let source_point_key = ctx.tail_point_keys.last().unwrap().clone();
                        let new_point = Point::new(Uuid::new_v4(), world_mouse_pos);
                        let new_joint = Joint::new(
                            source_point_key,
                            new_point.key,
                            world.points[&source_point_key]
                                .position
                                .distance(new_point.position),
                        );
                        world.points.insert(new_point.key, new_point.clone());
                        world.joints.insert(new_joint.key, new_joint.clone());

                        StructureCreatingContext {
                            tail_point_keys: ctx.tail_point_keys,
                            head_point_key: new_point.key,
                            head_joint_key: new_joint.key,
                            attach_info: None,
                        }
                    }
                }
                None => {
                    let nerby_point = world
                        .points
                        .values()
                        .filter(|point| {
                            adapter
                                .project(point.position.as_dvec2())
                                .distance(current_mouse_pos.as_dvec2())
                                < POINT_SNAP_RADIUS as f64
                                && point.key != ctx.head_point_key
                        })
                        .next();

                    match nerby_point {
                        Some(nerby_point) => {
                            let nerby_point = nerby_point.clone();

                            world.joints.remove(&ctx.head_joint_key);
                            world.points.remove(&ctx.head_point_key);

                            let source_point_key = ctx.tail_point_keys.last().unwrap().clone();
                            let new_joint = Joint::new(
                                source_point_key,
                                nerby_point.key,
                                world.points[&source_point_key]
                                    .position
                                    .distance(nerby_point.position),
                            );
                            let is_joint_dependent = if world.joints.contains_key(&new_joint.key) {
                                true
                            } else {
                                world.joints.insert(new_joint.key, new_joint.clone());
                                false
                            };

                            StructureCreatingContext {
                                tail_point_keys: ctx.tail_point_keys,
                                head_point_key: nerby_point.key,
                                head_joint_key: new_joint.key,
                                attach_info: Some(AttachInfo { is_joint_dependent }),
                            }
                        }
                        None => {
                            world.points.get_mut(&ctx.head_point_key).unwrap().position =
                                world_mouse_pos;

                            let joint = world.joints[&ctx.head_joint_key].clone();
                            world.joints.get_mut(&ctx.head_joint_key).unwrap().length = world
                                .points[&joint.key.a]
                                .position
                                .distance(world.points[&joint.key.b].position);
                            ctx
                        }
                    }
                }
            }),
            None => None,
        };

        if let Some(dragging_point_key) = dragging_point_key {
            world.kinematics.insert(dragging_point_key, world_mouse_pos);
        }

        // world.kinematics.insert(fixed_point_key1, vec2(-2.0, 0.0));
        // world.kinematics.insert(fixed_point_key2, vec2(0.0, 2.0));
        //
        // world.joints.get_mut(&moving_joint_key1).unwrap().length =
        //     2.0 + (simulation_time_elapsed * 3.0).sin();
        // world.joints.get_mut(&moving_joint_key2).unwrap().length =
        //     2.0 + (simulation_time_elapsed * 3.0).cos();

        let (_wheel_x, wheel_y) = mouse_wheel();
        let zoom_amount = wheel_y as f64 * wheel_sensitivity;
        adapter = adapter.zoom(zoom_amount);

        adapter = adapter.updated(dt as f64);

        world.points.values().for_each(|point| {
            let projected_pos = adapter.project(point.position.as_dvec2()).as_vec2();

            draw_circle(projected_pos.x, projected_pos.y, 4.0, color::WHITE);
        });

        world.joints.keys().for_each(|joint| {
            let point1 = world.points.get(&joint.a).unwrap();
            let point2 = world.points.get(&joint.b).unwrap();
            let projected_pos1 = adapter.project(point1.position.as_dvec2()).as_vec2();
            let projected_pos2 = adapter.project(point2.position.as_dvec2()).as_vec2();

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
            world = world.update(dt, 128);
            simulation_time_elapsed += dt;
        } else {
            world = world.kinematic_update();
        }

        next_frame().await
    }
}
