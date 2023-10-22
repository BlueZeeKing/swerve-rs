use std::{f32::consts::PI, ops::Add};

#[derive(Clone, Copy)]
pub struct Vector(f32, f32);

impl Add for Vector {
    type Output = Vector;

    fn add(self, rhs: Self) -> Self::Output {
        Vector(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl From<(f32, f32)> for Vector {
    fn from(value: (f32, f32)) -> Self {
        Self(value.0, value.1)
    }
}

impl From<Vector> for (f32, f32) {
    fn from(value: Vector) -> Self {
        (value.0, value.1)
    }
}

impl Vector {
    pub fn rotate_90(self) -> Self {
        Self(self.1 * -1.0, self.0)
    }

    pub fn scale(self, scalar: f32) -> Self {
        Self(self.0 * scalar, self.1 * scalar)
    }

    // multiply the vector by the matrix:
    //
    // cos theta, -sin theta
    // sin theta, cos theta
    pub fn field_relative(self, orientation: f32) -> Self {
        let (x, y) = Vector::from(self).into();
        let (sin, cos) = (-1.0 * orientation).sin_cos();

        (x * cos - y * sin, x * sin - y * cos).into()
    }
}

#[derive(Clone, Copy)]
pub struct SwerveState {
    angle: f32,
    drive: f32,
}

impl From<Vector> for SwerveState {
    fn from(value: Vector) -> Self {
        Self {
            drive: (value.0.powi(2) + value.1.powi(2)).sqrt(),
            angle: normalize_angle((value.1 / value.0).atan()),
        }
    }
}

impl From<SwerveState> for Vector {
    fn from(value: SwerveState) -> Self {
        Self(
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

    pub fn optimize(self, old: SwerveState) -> SwerveState {
        let new_angle = normalize_angle(self.angle);
        let old_angle = normalize_angle(old.angle);
        let diff = new_angle - old_angle;

        if diff.abs() < PI / 2.0 {
            self
        } else if diff < 0.0 {
            Self {
                angle: new_angle + 2.0 * PI,
                drive: -1.0 * self.drive,
            }
        } else {
            Self {
                angle: new_angle - 2.0 * PI,
                drive: -1.0 * self.drive,
            }
        }
    }
}

pub fn normalize_angle(angle: f32) -> f32 {
    if angle > 2.0 * PI {
        angle % (2.0 * PI)
    } else if angle < 0.0 {
        angle % (-2.0 * PI)
    } else {
        angle
    }
}
