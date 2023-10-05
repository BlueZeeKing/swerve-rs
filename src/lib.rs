#![feature(async_fn_in_trait, return_position_impl_trait_in_trait)]

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
        todo!()
    }

    async fn get_enabled_future(self: std::rc::Rc<Self>) -> anyhow::Result<()> {
        todo!()
    }

    async fn get_teleop_future(self: std::rc::Rc<Self>) -> anyhow::Result<()> {
        let mut drivetrain = self.drivetrain.lock().await;

        loop {
            drivetrain.set_input(
                (self.controller.left_x()?, self.controller.left_y()?).into(),
                self.controller.right_x()?,
            )?;
            yield_now();
        }
    }

    fn create_bindings(self: std::rc::Rc<Self>, _executor: &Spawner) {
        todo!()
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
