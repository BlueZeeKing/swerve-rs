use std::fs::File;

use rapier2d::prelude::*;
use swerve_rs::drivetrain::Drivetrain;

fn main() {
    let mut bodies = RigidBodySet::new();

    let mut physics_pipeline = PhysicsPipeline::new();

    let gravity = vector![0.0, 0.0];
    let integration_params = IntegrationParameters::default();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    let body = RigidBodyBuilder::dynamic().build();
    let collider = ColliderBuilder::ball(1.0).build();

    let mut drivetrain = Drivetrain::default();
    drivetrain.set_input(vector![1.0, 1.0]);

    let body_handle = bodies.insert(body);
    colliders.insert_with_parent(collider, body_handle, &mut bodies);

    let file = File::create("./data/plot.csv").unwrap();

    let mut csv = csv::Writer::from_writer(file);

    csv.write_record(&["Time", "X", "Y", "Speed", "New Input"])
        .unwrap();

    (0..200).for_each(|t| {
        physics_pipeline.step(
            &gravity,
            &integration_params,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            None,
            &(),
            &(),
        );

        let body = &mut bodies[body_handle];

        if t == 100 {
            drivetrain.set_input(vector![1.0, -1.0]);
        }

        body.set_linvel(drivetrain.current_value(), true);

        let pos = body.translation();

        csv.write_record(&[
            t.to_string(),
            pos.x.to_string(),
            pos.y.to_string(),
            body.linvel().magnitude().to_string(),
            (t >= 100).to_string(),
        ])
        .unwrap();
    });
}
