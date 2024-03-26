use std::f32::consts::PI;

use nalgebra::{Rotation2, Vector2};
use robotrs::{
    control::ControlSafe,
    math::filter::{Filter, SlewRateLimiter},
    FailableDefault,
};

use crate::{swerve_module::SwerveModule, types::SwerveState};

/// Meters per second
const MAX_VELOCITY_LIMIT: f32 = 4.8;
/// Radians per second
const MAX_ROTATION_LIMIT: f32 = 20.0 * (PI / 180.0);
/// Meters
const LOW_SPEED_CUTOFF: f32 = 0.01;

const MAX_ACCEL: f32 = 5.0;
const MAX_ANGLE_ACCEL: f32 = 5.0;

const TRACK_WIDTH: f32 = 0.7239;
const WHEEL_BASE: f32 = 0.6096;

pub struct Drivetrain {
    modules: [SwerveModule; 4],
    positions: [Vector2<f32>; 4],

    x_limit: SlewRateLimiter,
    y_limit: SlewRateLimiter,
    angle_limit: SlewRateLimiter,
}

impl Drivetrain {
    pub fn set_input_raw(&mut self, drive: Vector2<f32>, turn_rate: f32) -> anyhow::Result<()> {
        for (module, position) in self.modules.iter_mut().zip(self.positions) {
            module.set_target(
                (drive + Rotation2::new(90.0_f32.to_radians()) * position.scale(turn_rate)).into(),
            )?;
        }

        Ok(())
    }

    pub fn set_input(&mut self, drive: Vector2<f32>, turn_rate: f32) -> anyhow::Result<()> {
        let drive = Vector2::new(self.x_limit.apply(drive.x)?, self.y_limit.apply(drive.y)?);
        let turn_rate = self.angle_limit.apply(turn_rate)?;

        self.set_input_raw(drive, turn_rate)
    }

    pub fn brake(&mut self) -> anyhow::Result<()> {
        for (module, position) in self.modules.iter_mut().zip(self.positions.iter()) {
            let mut state: SwerveState = (*position).into();
            state.stop();

            module.set_target(state)?;
        }

        Ok(())
    }
}

impl FailableDefault for Drivetrain {
    fn failable_default() -> anyhow::Result<Self> {
        Ok(Self {
            angle_limit: SlewRateLimiter::new(MAX_ANGLE_ACCEL)?,
            x_limit: SlewRateLimiter::new(MAX_ACCEL)?,
            y_limit: SlewRateLimiter::new(MAX_ACCEL)?,

            modules: [
                SwerveModule::new(1, 2, Rotation2::new(0.0))?, // front right
                SwerveModule::new(3, 4, Rotation2::new(-PI / 2.0))?, // front left
                SwerveModule::new(5, 6, Rotation2::new(PI))?,  // rear left
                SwerveModule::new(7, 8, Rotation2::new(PI / 2.0))?, // rear right
            ],
            positions: [
                Vector2::new(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                Vector2::new(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                Vector2::new(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                Vector2::new(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            ],
        })
    }
}

impl ControlSafe for Drivetrain {
    fn stop(&mut self) {
        for module in &mut self.modules {
            module.stop();
        }
    }
}
