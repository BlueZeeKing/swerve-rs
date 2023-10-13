use robotrs::{control::ControlSafe, FailableDefault};

use crate::{swerve_module::SwerveModule, types::Vector};

pub struct Drivetrain {
    modules: [SwerveModule; 4],
    positions: [Vector; 4],
    gyro: navx::NavX,
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
        // TODO: Ratelimiting
        self.set_input_raw(drive, turn_rate)
    }
}

impl FailableDefault for Drivetrain {
    fn failable_default() -> anyhow::Result<Self> {
        Ok(Self {
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
