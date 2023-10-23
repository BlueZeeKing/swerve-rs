use std::f32::consts::PI as f32PI;
use std::f64::consts::PI;

use robotrs::{
    control::ControlSafe,
    math::filter::{Filter, SlewRateLimiter},
    FailableDefault,
};

use crate::{
    swerve_module::SwerveModule,
    types::{optimize_angle, Vector},
};

/// Meters per second
const MAX_VELOCITY_LIMIT: f64 = 4.8;
/// Radians per second
const MAX_ROTATION_LIMIT: f64 = 20.0 * (PI / 180.0);
/// Meters
const LOW_SPEED_CUTOFF: f32 = 0.01;
/// Radians
const ANGLE_OFFSETS: [f32; 4] = [0.0, 0.0, 0.0, 0.0]; // TODO: Make correct

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
        // deconstruct target and current vectors to get their magnitude and angle
        let (target_angle, target_mag) = drive.deconstruct();
        let (last_angle, last_mag) = self.last_drive.deconstruct();

        // Correct angles when moving very slowly
        let last_angle = if last_mag < LOW_SPEED_CUTOFF {
            target_angle
        } else {
            last_angle
        };

        let target_angle = if target_mag < LOW_SPEED_CUTOFF {
            last_angle
        } else {
            target_angle
        };

        let (target_angle, last_angle) = optimize_angle(target_angle, last_angle);

        // if the angle change is greater than 90 degrees, drive backwards
        let (target_angle, target_mag) = if (target_angle - last_angle).abs() > f32PI / 2.0 {
            (target_angle + f32PI, target_mag * -1.0)
        } else {
            (target_angle, target_mag)
        };

        let (target_angle, last_angle) = optimize_angle(target_angle, last_angle);

        // if we aren't traveling in the correct direction set the target speed to 0
        let target_mag = target_mag * (1.0 - (target_angle - last_angle).abs() / f32PI).powi(3);

        // limit the angle rate of change based on the current speed
        self.angle_limit
            .set_limit((1.0 - (last_mag / 4.8).cbrt()) as f64 * MAX_ROTATION_LIMIT);

        // calculate the next values by stepping the last values to the target values
        let angle = self.angle_limit.apply(target_angle as f64)?;
        let mag = self.mag_limit.apply(target_mag as f64)?;

        let drive = ((angle.cos() * mag) as f32, (angle.sin() * mag) as f32).into();

        self.last_drive = drive;

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
                SwerveModule::new(1, 2, ANGLE_OFFSETS[0])?,
                SwerveModule::new(3, 4, ANGLE_OFFSETS[1])?,
                SwerveModule::new(5, 6, ANGLE_OFFSETS[2])?,
                SwerveModule::new(7, 8, ANGLE_OFFSETS[3])?,
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
