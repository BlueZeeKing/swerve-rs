use std::f32::consts::PI;

use math::{
    kinematics::{module_positions_from_dimensions, Kinematics, SwerveKinematics},
    normalize_angle,
    odometry::Odometry,
};
use nalgebra::{Rotation2, Vector2, Vector3};
use navx::NavX;
use robotrs::{
    control::ControlSafe,
    math::filter::{Filter, SlewRateLimiter},
    scheduler::spawn,
    yield_now, FailableDefault,
};
use utils::error::log;

use crate::swerve_module::SwerveModule;

/// Meters per second
const MAX_VELOCITY_LIMIT: f32 = 1.0;
/// Radians per second
const MAX_ROTATION_LIMIT: f32 = 1.0;

const MAX_ACCEL: f32 = 5.0;
const MAX_ANGLE_ACCEL: f32 = 5.0;

const TRACK_WIDTH: f32 = 0.7239;
const WHEEL_BASE: f32 = 0.6096;

pub struct Drivetrain {
    modules: [SwerveModule; 4],

    kinematics: SwerveKinematics,
    odometry: Odometry<SwerveKinematics>,
    gyro: NavX,

    x_limit: SlewRateLimiter,
    y_limit: SlewRateLimiter,
    angle_limit: SlewRateLimiter,
}

impl Drivetrain {
    pub fn get_pose(&self) -> Vector3<f32> {
        self.odometry.get_pose()
    }

    pub fn set_input_raw(&mut self, drive: Vector2<f32>, turn_rate: f32) -> anyhow::Result<()> {
        let drive = Rotation2::new(-self.get_heading()).matrix() * drive;

        for (module, state) in self.modules.iter_mut().zip(
            self.kinematics
                .inverse(drive.fixed_resize(turn_rate))
                .into_iter(),
        ) {
            module.set_target(state)?;
        }

        Ok(())
    }

    pub fn get_heading(&self) -> f32 {
        normalize_angle(-self.gyro.heading().to_radians())
    }

    pub fn set_input(&mut self, drive: Vector2<f32>, turn_rate: f32) -> anyhow::Result<()> {
        let drive = Vector2::new(self.x_limit.apply(drive.x)?, self.y_limit.apply(drive.y)?)
            .scale(MAX_VELOCITY_LIMIT);
        let turn_rate = self.angle_limit.apply(turn_rate)? * MAX_ROTATION_LIMIT;

        self.set_input_raw(drive, turn_rate)
    }

    pub fn brake(&mut self) -> anyhow::Result<()> {
        for (module, state) in self
            .modules
            .iter_mut()
            .zip(self.kinematics.brake().into_iter())
        {
            module.set_target(state)?;
        }

        Ok(())
    }
}

impl FailableDefault for Drivetrain {
    fn failable_default() -> anyhow::Result<Self> {
        let kinematics =
            SwerveKinematics::new(module_positions_from_dimensions(TRACK_WIDTH, WHEEL_BASE));

        let (front_left, mut front_left_state) =
            SwerveModule::new(3, 4, Rotation2::new(-PI / 2.0))?;
        let (front_right, mut front_right_state) = SwerveModule::new(1, 2, Rotation2::new(0.0))?;
        let (rear_left, mut rear_left_state) = SwerveModule::new(5, 6, Rotation2::new(PI))?;
        let (rear_right, mut rear_right_state) = SwerveModule::new(7, 8, Rotation2::new(PI / 2.0))?;

        let odometry = Odometry::new(kinematics.clone(), Vector3::new(0.0, 0.0, 0.0));
        let odometry2 = odometry.clone();

        let gyro = NavX::new(hal::spi::RioSPI::new(hal::spi::Port::MXP)?, 60);
        let gyro2 = gyro.clone();

        spawn(async move {
            loop {
                let _ = log(async {
                    odometry2.update(
                        [
                            front_left_state()?,
                            front_right_state()?,
                            rear_left_state()?,
                            rear_right_state()?,
                        ],
                        -gyro2.heading().to_radians(),
                    );

                    anyhow::Ok(())
                })
                .await;

                yield_now().await;
            }
        })
        .detach();

        Ok(Self {
            angle_limit: SlewRateLimiter::new(MAX_ANGLE_ACCEL)?,
            x_limit: SlewRateLimiter::new(MAX_ACCEL)?,
            y_limit: SlewRateLimiter::new(MAX_ACCEL)?,

            odometry,
            kinematics,
            gyro,

            modules: [front_left, front_right, rear_left, rear_right],
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
