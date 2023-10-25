use std::{f32::consts::PI, ops::Add};

use units::{
    angle::Radian,
    rate::MeterPerSecond,
    ratio::{Fraction, Ratio},
    Unit,
};

#[derive(Clone, Copy)]
pub struct Vector<U: Unit>(U, U);

impl<U: Unit> Add for Vector<U> {
    type Output = Vector<U>;

    fn add(self, rhs: Self) -> Self::Output {
        Vector(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl<U: Unit> From<(U, U)> for Vector<U> {
    fn from(value: (U, U)) -> Self {
        Self(value.0, value.1)
    }
}

impl<U: Unit> Vector<U> {
    pub fn new(x: U, y: U) -> Self {
        Self(x, y)
    }

    pub fn rotate_90(self) -> Self {
        Self(self.1.scale(Fraction::new(-1.0)), self.0)
    }

    pub fn scale(self, scalar: f32) -> Self {
        Self(
            self.0.scale(Fraction::new(scalar)),
            self.1.scale(Fraction::new(scalar)),
        )
    }

    pub fn x(self) -> U {
        self.0
    }

    pub fn y(self) -> U {
        self.1
    }

    // multiply the vector by the matrix:
    //
    // cos theta, -sin theta
    // sin theta, cos theta
    pub fn field_relative(self, orientation: Radian) -> Self {
        let vector = Vector::from(self);
        let (x, y) = (vector.0.raw(), vector.1.raw());
        let (sin, cos) = (-1.0 * orientation.raw()).sin_cos();

        (U::new(x * cos - y * sin), U::new(x * sin - y * cos)).into()
    }

    pub fn theta(&self) -> Radian {
        if self.0.raw() == 0.0 {
            return Radian::new(PI / 2.0 * self.1.raw().signum());
        }

        let tan_res = (self.1.raw() / self.0.raw()).atan();

        let angle = if self.0.raw() < 0.0 {
            tan_res + PI
        } else {
            tan_res
        };

        Radian::new(angle).normalize()
    }

    pub fn magnitude(&self) -> U {
        U::new((self.0.raw().powi(2) + self.1.raw().powi(2)).sqrt())
    }

    /// Into theta, magnitude
    pub fn deconstruct(self) -> (Radian, U) {
        (self.theta(), self.magnitude())
    }
}

#[derive(Clone, Copy)]
pub struct SwerveState {
    angle: Radian,
    drive: MeterPerSecond,
}

impl From<Vector<MeterPerSecond>> for SwerveState {
    fn from(value: Vector<MeterPerSecond>) -> Self {
        Self {
            drive: MeterPerSecond::new((value.0.raw().powi(2) + value.1.raw().powi(2)).sqrt()),
            angle: value.theta(),
        }
    }
}

impl From<SwerveState> for Vector<MeterPerSecond> {
    fn from(value: SwerveState) -> Self {
        Self(
            MeterPerSecond::new(value.angle.raw().cos() * value.drive.raw()),
            MeterPerSecond::new(value.angle.raw().sin() * value.drive.raw()),
        )
    }
}

impl SwerveState {
    pub fn new(angle: Radian, drive: MeterPerSecond) -> Self {
        Self { angle, drive }
    }

    pub fn get_angle(&self) -> Radian {
        self.angle
    }

    pub fn get_drive(&self) -> MeterPerSecond {
        self.drive
    }

    pub fn stop(&mut self) {
        self.drive = MeterPerSecond::new(0.0)
    }

    pub fn optimize(self, old: SwerveState) -> SwerveState {
        let new_angle = self.angle.normalize();
        let old_angle = old.angle.normalize();
        let diff = (new_angle - old_angle).raw();

        if diff.abs() < PI / 2.0 {
            self
        } else if diff < 0.0 {
            Self {
                angle: new_angle + Radian::new(2.0 * PI),
                drive: self.drive.scale(Fraction::new(-1.0)),
            }
        } else {
            Self {
                angle: new_angle - Radian::new(2.0 * PI),
                drive: self.drive.scale(Fraction::new(-1.0)),
            }
        }
    }
}

pub trait RadianExt: Sized {
    fn normalize(self) -> Self;
    fn optimize(self, other: Self) -> (Self, Self);
}

impl RadianExt for Radian {
    fn normalize(self) -> Self {
        let angle = self.raw();

        Radian::new(if angle > 2.0 * PI {
            angle % (2.0 * PI)
        } else if angle < 0.0 {
            2.0 * PI - (-angle % (2.0 * PI))
        } else {
            angle
        })
    }

    fn optimize(self, other: Self) -> (Self, Self) {
        let a = self.normalize().raw();
        let b = other.normalize().raw();

        let b1 = b + 2.0 * PI;
        let b2 = b - 2.0 * PI;

        let diff = (a - b).abs();
        let diff1 = (a - b1).abs();
        let diff2 = (a - b2).abs();

        let val = if diff < diff1 && diff < diff2 {
            (a, b)
        } else if diff1 < diff && diff1 < diff2 {
            (a, b1)
        } else {
            (a, b2)
        };

        (Radian::new(val.0), Radian::new(val.1))
    }
}
