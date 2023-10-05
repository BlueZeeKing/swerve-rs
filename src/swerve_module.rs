use revlib::{
    encoder::{absolute::SparkMaxAbsoluteEncoder, relative::SparkMaxRelativeEncoder, Encoder},
    SparkMax,
};
use robotrs::{
    control::ControlSafe,
    motor::{IdleMode, SetIdleMode},
};

use std::f32::consts::PI;

use crate::types::SwerveState;

const TURN_POSITION_CONVERSION_FACTOR: f32 = 0.0;
const TURN_VELOCITY_CONVERSION_FACTOR: f32 = 0.0;
const DRIVE_POSITION_CONVERSION_FACTOR: f32 = 0.0;
const DRIVE_VELOCITY_CONVERSION_FACTOR: f32 = 0.0;

const TURN_P: f32 = 0.0;
const TURN_I: f32 = 0.0;
const TURN_D: f32 = 0.0;
const TURN_F: f32 = 0.0;
const DRIVE_P: f32 = 0.0;
const DRIVE_I: f32 = 0.0;
const DRIVE_D: f32 = 0.0;
const DRIVE_F: f32 = 0.0;

const TURN_MAX_OUTPUT: f32 = 0.0;
const TURN_MIN_OUTPUT: f32 = 0.0;
const DRIVE_MAX_OUTPUT: f32 = 0.0;
const DRIVE_MIN_OUTPUT: f32 = 0.0;

const TURN_MAX_CURRENT: u8 = 0;
const DRIVE_MAX_CURRENT: u8 = 0;

const TURN_IDLE_MODE: IdleMode = IdleMode::Brake;
const DRIVE_IDLE_MODE: IdleMode = IdleMode::Brake;

pub struct SwerveModule {
    turn: SparkMax,
    drive: SparkMax,
    turn_encoder: SparkMaxAbsoluteEncoder,
    drive_encoder: SparkMaxRelativeEncoder,
    current_state: SwerveState,
}

impl SwerveModule {
    pub fn new(drive_id: i32, turn_id: i32) -> anyhow::Result<Self> {
        let mut turn = SparkMax::new(turn_id, revlib::MotorType::Brushless);
        let mut drive = SparkMax::new(drive_id, revlib::MotorType::Brushless);

        turn.reset_settings()?;
        drive.reset_settings()?;

        turn.set_pid_range(TURN_MIN_OUTPUT..=TURN_MAX_OUTPUT)?;
        drive.set_pid_range(DRIVE_MIN_OUTPUT..=DRIVE_MAX_OUTPUT)?;

        let mut turn_encoder = turn.get_absolute_encoder()?;
        let mut drive_encoder = drive.get_relative_encoder()?;

        turn_encoder.set_inverted(true)?;

        turn_encoder.set_position_conversion_factor(TURN_POSITION_CONVERSION_FACTOR)?;
        turn_encoder.set_velocity_conversion_factor(TURN_VELOCITY_CONVERSION_FACTOR)?;
        drive_encoder.set_position_conversion_factor(DRIVE_POSITION_CONVERSION_FACTOR)?;
        drive_encoder.set_velocity_conversion_factor(DRIVE_VELOCITY_CONVERSION_FACTOR)?;

        turn.set_pid(TURN_P, TURN_D, TURN_I, TURN_F)?;
        drive.set_pid(DRIVE_P, DRIVE_D, DRIVE_I, DRIVE_F)?;

        turn.set_wrapping(true, 0.0, 2.0 * PI)?;

        let starting_turn = turn_encoder.get_position()?;

        turn.set_pid_input(&turn_encoder)?;
        drive.set_pid_input(&drive_encoder)?;

        turn.set_smart_current_limit(TURN_MAX_CURRENT)?;
        drive.set_smart_current_limit(DRIVE_MAX_CURRENT)?;

        turn.set_idle_mode(TURN_IDLE_MODE)?;
        drive.set_idle_mode(DRIVE_IDLE_MODE)?;

        turn.write_settings()?;
        drive.write_settings()?;

        drive_encoder.set_position(0.0)?;

        Ok(Self {
            turn,
            drive,
            turn_encoder,
            drive_encoder,
            current_state: SwerveState::new(starting_turn, 0.0),
        })
    }

    pub fn set_target(&mut self, state: SwerveState) -> anyhow::Result<()> {
        let state = state.optimize(self.current_state);
        self.current_state = state;

        self.turn
            .set_refrence(state.get_angle(), revlib::ControlType::Position)?;
        self.drive
            .set_refrence(state.get_drive(), revlib::ControlType::Velocity)?;

        Ok(())
    }
}

impl ControlSafe for SwerveModule {
    fn stop(&mut self) {
        self.turn.stop();
        self.drive.stop();
    }
}
