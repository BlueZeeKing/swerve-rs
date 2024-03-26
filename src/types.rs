use std::f32::consts::PI;

use nalgebra::Vector2;

#[derive(Clone, Copy, Debug)]
pub struct SwerveState {
    pub angle: f32,
    pub drive: f32,
}

impl From<Vector2<f32>> for SwerveState {
    fn from(value: Vector2<f32>) -> Self {
        Self {
            drive: (value.x.powi(2) + value.y.powi(2)).sqrt(),
            angle: normalize_angle(value.y.atan2(value.x)),
        }
    }
}

impl From<SwerveState> for Vector2<f32> {
    fn from(value: SwerveState) -> Self {
        Vector2::new(
            value.angle.cos() * value.drive,
            value.angle.sin() * value.drive,
        )
    }
}

impl SwerveState {
    pub fn new(angle: f32, drive: f32) -> Self {
        Self { angle, drive }
    }

    pub fn get_angle(&self) -> f32 {
        self.angle
    }

    pub fn get_drive(&self) -> f32 {
        self.drive
    }

    pub fn stop(&mut self) {
        self.drive = 0.0
    }

    pub fn optimize(self, old: SwerveState) -> SwerveState {
        let new_angle = normalize_angle(self.angle);
        let old_angle = normalize_angle(old.angle);
        let diff = new_angle - old_angle;

        if diff.abs() < PI / 2.0 {
            self
        } else {
            Self {
                angle: normalize_angle(new_angle - PI),
                drive: -1.0 * self.drive,
            }
        }
    }
}

pub fn normalize_angle(angle: f32) -> f32 {
    if angle > 2.0 * PI {
        angle % (2.0 * PI)
    } else if angle < 0.0 {
        2.0 * PI - (-angle % (2.0 * PI))
    } else {
        angle
    }
}

pub fn optimize_angle(a: f32, b: f32) -> (f32, f32) {
    let a = normalize_angle(a);
    let b = normalize_angle(b);

    let b1 = b + 2.0 * PI;
    let b2 = b - 2.0 * PI;

    let diff = (a - b).abs();
    let diff1 = (a - b1).abs();
    let diff2 = (a - b2).abs();

    if diff < diff1 && diff < diff2 {
        (a, b)
    } else if diff1 < diff && diff1 < diff2 {
        (a, b1)
    } else {
        (a, b2)
    }
}
