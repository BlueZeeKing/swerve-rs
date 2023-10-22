use std::f32::consts::PI as f32PI;
use std::f64::consts::PI;

use robotrs::{
    control::ControlSafe,
    math::filter::{Filter, SlewRateLimiter},
    FailableDefault,
};

use crate::{
    swerve_module::SwerveModule,
    types::{normalize_angle, Vector},
};

/// Meters per second
const MAX_VELOCITY_LIMIT: f64 = 4.8;
/// Radians per second
const MAX_ROTATION_LIMIT: f64 = 20.0 * (PI / 180.0);

pub struct Drivetrain {
    modules: [SwerveModule; 4],
    positions: [Vector; 4],

    gyro: navx::NavX,

    angle_limit: SlewRateLimiter,
    mag_limit: SlewRateLimiter,
    last_drive: Vector,
}

impl Drivetrain {
    fn set_input_raw(&mut self, drive: Vector, turn_rate: f32) -> anyhow::Result<()> {
        for (module, position) in self.modules.iter_mut().zip(self.positions) {
            module.set_target(
                (drive.field_relative(self.gyro.get_yaw()) + position.rotate_90().scale(turn_rate))
                    .into(),
            )?;
        }

        Ok(())
    }

    pub fn set_input(&mut self, drive: Vector, turn_rate: f32) -> anyhow::Result<()> {
        // TODO: Add case for starting from zero

        // deconstruct target and current vectors to get their magnitude and angle
        let (target_angle, target_mag) = drive.deconstruct();
        let (last_angle, last_mag) = self.last_drive.deconstruct();

        // deconstruct target and current vectors to get their magnitude and angle
        let (target_angle, target_mag) = if (target_angle - last_angle).abs() > f32PI / 2.0 {
            (normalize_angle(target_angle + f32PI), target_mag * -1.0)
        } else {
            (target_angle, target_mag)
        };

        // deconstruct target and current vectors to get their magnitude and angle
        let target_mag = target_mag * (1.0 - (target_angle - last_angle).abs() / f32PI).powi(3);

        // limit the angle rate of change based on the current speed
        self.angle_limit
            .set_limit((1.0 - (last_mag / 2.0).cbrt()) as f64 * MAX_ROTATION_LIMIT);

        // calculate the next values by stepping the last values to the target values
        let angle = self.angle_limit.apply(target_angle as f64)?;
        let mag = self.mag_limit.apply(target_mag as f64)?;

        let drive = ((angle.cos() * mag) as f32, (angle.sin() * mag) as f32).into();

        self.set_input_raw(drive, turn_rate)
    }
}

impl FailableDefault for Drivetrain {
    fn failable_default() -> anyhow::Result<Self> {
        Ok(Self {
            last_drive: (0.0, 0.0).into(),
            angle_limit: SlewRateLimiter::new(MAX_ROTATION_LIMIT)?,
            mag_limit: SlewRateLimiter::new(MAX_VELOCITY_LIMIT)?,

            gyro: navx::NavX::new(),

            modules: [
                SwerveModule::new(1, 2)?,
                SwerveModule::new(3, 4)?,
                SwerveModule::new(5, 6)?,
                SwerveModule::new(7, 8)?,
            ],
            positions: [
                // FIXME: Make this the right numbers
                (1.0, 1.0).into(),
                (1.0, -1.0).into(),
                (-1.0, 1.0).into(),
                (-1.0, -1.0).into(),
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
