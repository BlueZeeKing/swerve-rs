use drivetrain::Drivetrain;
use nalgebra::Vector2;
use robotrs::{
    hid::controller::XboxController, robot::AsyncRobot, scheduler::guard, yield_now, Deadzone,
    FailableDefault,
};
use utils::{periodic, subsystem::Subsystem, tracing::info, trigger::TriggerExt, wait};

pub mod drivetrain;
pub mod swerve_module;

pub struct Robot {
    drivetrain: Subsystem<Drivetrain>,
    controller: XboxController,
}

impl AsyncRobot for Robot {
    async fn get_auto_future(&'static self) -> anyhow::Result<()> {
        Ok(())
    }

    async fn get_enabled_future(&'static self) -> anyhow::Result<()> {
        Ok(())
    }

    async fn get_teleop_future(&'static self) -> anyhow::Result<()> {
        periodic!([drivetrain = self.drivetrain => 1], async {
            drivetrain
                .set_input(
                    Vector2::new(
                        -self.controller.left_y().unwrap().deadzone(0.1),
                        -self.controller.left_x().unwrap().deadzone(0.1),
                    ),
                    -self.controller.right_x().unwrap().deadzone(0.1),
                )
                .unwrap();

            yield_now().await;
        })
    }

    fn configure_bindings(
        &'static self,
        _scheduler: &robotrs::scheduler::RobotScheduler<Self>,
    ) -> anyhow::Result<()> {
        self.controller.x().while_pressed(move || async move {
            let mut drivetrain = self.drivetrain.lock(2).await;
            drivetrain.brake()?;

            wait!();

            anyhow::Ok(())
        });

        Ok(())
    }
}

impl FailableDefault for Robot {
    fn failable_default() -> anyhow::Result<Self> {
        Ok(Self {
            drivetrain: Subsystem::new(Drivetrain::failable_default()?),
            controller: XboxController::new(0)?,
        })
    }
}
