use std::f32::consts::PI;

use rapier2d::na::Vector2;

use crate::types::{normalize_angle, optimize_angle};

const LIMIT: f32 = 0.2;
const ANGLE_LIMIT: f32 = 10.0 * (PI / 180.0);

pub struct Drivetrain {
    target: Vector2<f32>,
    last: Vector2<f32>,
}

impl Drivetrain {
    pub fn set_input(&mut self, drive: Vector2<f32>) {
        self.target = drive;
    }

    pub fn current_value(&mut self) -> Vector2<f32> {
        // deconstruct target and current vectors to get their magnitude and angle
        let (target_angle, target_mag) = deconstruct(self.target);
        let (last_angle, last_mag) = deconstruct(self.last);

        let last_angle = if last_mag < 0.01 {
            target_angle
        } else {
            last_angle
        };

        let target_angle = if target_mag < 0.01 {
            last_angle
        } else {
            target_angle
        };

        let (target_angle, last_angle) = optimize_angle(target_angle, last_angle);

        // if the angle change is greater than 90 degrees, drive backwards
        let (target_angle, target_mag) = if (target_angle - last_angle).abs() > PI / 2.0 {
            (target_angle + PI, target_mag * -1.0)
        } else {
            (target_angle, target_mag)
        };

        let (target_angle, last_angle) = optimize_angle(target_angle, last_angle);

        // if we aren't traveling in the correct direction set the target speed to 0
        let target_mag = target_mag * (1.0 - (target_angle - last_angle).abs() / PI).powi(3);

        // limit the angle rate of change based on the current speed
        let angle_limit = (1.0 - last_mag / 2.0).powi(3) * ANGLE_LIMIT;

        // calculate the next values by stepping the last values to the target values
        let (angle, mag) = (
            if (target_angle - last_angle).abs() < LIMIT {
                target_angle
            } else if target_angle > last_angle {
                last_angle + angle_limit
            } else if target_angle < last_angle {
                last_angle - angle_limit
            } else {
                last_angle
            },
            if (target_mag - last_mag).abs() < LIMIT {
                target_mag
            } else if target_mag > last_mag {
                last_mag + LIMIT
            } else if target_mag < last_mag {
                last_mag - LIMIT
            } else {
                last_mag
            },
        );

        self.last = Vector2::new(angle.cos() * mag, angle.sin() * mag);

        self.last
    }
}

fn deconstruct(vector: Vector2<f32>) -> (f32, f32) {
    (theta(vector), vector.magnitude())
}

fn theta(vector: Vector2<f32>) -> f32 {
    if vector.x == 0.0 {
        return normalize_angle(PI / 2.0 * vector.y.signum());
    }

    let tan_res = (vector.y / vector.x).atan();

    let angle = if vector.x < 0.0 {
        tan_res + PI
    } else {
        tan_res
    };

    normalize_angle(angle)
}

impl Default for Drivetrain {
    fn default() -> Self {
        Self {
            last: Vector2::zeros(),
            target: Vector2::zeros(),
        }
    }
}
