#![feature(async_fn_in_trait, return_position_impl_trait_in_trait)]

use anyhow::bail;
use drivetrain::Drivetrain;
use robotrs::{
    control::ControlLock, hid::controller::XboxController, robot::AsyncRobot, scheduler::Spawner,
    yield_now, FailableDefault,
};

pub mod drivetrain;
pub mod swerve_module;
pub mod types;

pub struct Robot {
    drivetrain: ControlLock<Drivetrain>,
    controller: XboxController,
}

impl AsyncRobot for Robot {
    async fn get_auto_future(self: std::rc::Rc<Self>) -> anyhow::Result<()> {
        bail!("Not created");
    }

    async fn get_enabled_future(self: std::rc::Rc<Self>) -> anyhow::Result<()> {
        Ok(())
    }

    async fn get_teleop_future(self: std::rc::Rc<Self>) -> anyhow::Result<()> {
        loop {
            let mut drivetrain = self.drivetrain.lock().await;

            drivetrain.set_input(
                (self.controller.left_x()?, self.controller.left_y()?).into(),
                self.controller.right_x()?,
            )?;
                
            yield_now();
        }
    }

    fn create_bindings(self: std::rc::Rc<Self>, executor: &Spawner) {
        executor.spawn(async move {
            loop {
                let released = self.controller.x().await?;

                let mut drivetrain = self.drivetrain.lock().await;

                drivetrain.brake()?;

                released.await?;

                drop(drivetrain)
            }
        })
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
