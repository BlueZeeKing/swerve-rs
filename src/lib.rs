use std::future::pending;

use anyhow::bail;
use drivetrain::Drivetrain;
use nalgebra::Vector2;
use robotrs::{
    control::ControlLock, hid::controller::XboxController, robot::AsyncRobot, yield_now, Deadzone,
    FailableDefault,
};
use utils::trigger::TriggerExt;

pub mod drivetrain;
pub mod swerve_module;
pub mod types;

pub struct Robot {
    drivetrain: ControlLock<Drivetrain>,
    controller: XboxController,
}

impl AsyncRobot for Robot {
    async fn get_auto_future(&self) -> anyhow::Result<()> {
        Ok(())
    }

    async fn get_enabled_future(&self) -> anyhow::Result<()> {
        Ok(())
    }

    async fn get_teleop_future(&self) -> anyhow::Result<()> {
        let mut drivetrain = self.drivetrain.lock().await;
        loop {
            drivetrain.set_input(
                Vector2::new(
                    -self.controller.left_y()?.deadzone(0.1),
                    -self.controller.left_x()?.deadzone(0.1),
                ),
                -self.controller.right_x()?.deadzone(0.1),
            )?;

            yield_now().await;
        }
    }

    fn configure_bindings<'a>(
        &'a self,
        scheduler: &'a robotrs::scheduler::RobotScheduler<'a, Self>,
    ) -> anyhow::Result<()> {
        self.controller.x().while_pressed(scheduler, || async {
            let mut drivetrain = self.drivetrain.lock().await;

            drivetrain.brake()?;

            pending::<()>().await;

            anyhow::Ok(())
        });

        Ok(())
    }
}

impl FailableDefault for Robot {
    fn failable_default() -> anyhow::Result<Self> {
        Ok(Self {
            drivetrain: FailableDefault::failable_default()?,
            controller: XboxController::new(0)?,
        })
    }
}
