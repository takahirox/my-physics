//! Small deterministic math layer. Keeping it local makes the floating-point
//! policy and serialization surface explicit.

use core::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    pub const FORWARD: Self = Self::new(0.0, 0.0, -1.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(self.y * rhs.z - self.z * rhs.y, self.z * rhs.x - self.x * rhs.z, self.x * rhs.y - self.y * rhs.x)
    }
    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }
    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }
    pub fn normalized(self) -> Self {
        let n = self.length();
        if n > 1.0e-12 { self / n } else { Self::ZERO }
    }
    pub fn lerp(self, rhs: Self, t: f64) -> Self {
        self + (rhs - self) * t
    }
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, r: Self) -> Self {
        Self::new(self.x + r.x, self.y + r.y, self.z + r.z)
    }
}
impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, r: Self) -> Self {
        Self::new(self.x - r.x, self.y - r.y, self.z - r.z)
    }
}
impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}
impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for Vec3 {
    fn add_assign(&mut self, r: Self) {
        *self = *self + r;
    }
}
impl SubAssign for Vec3 {
    fn sub_assign(&mut self, r: Self) {
        *self = *self - r;
    }
}
impl MulAssign<f64> for Vec3 {
    fn mul_assign(&mut self, s: f64) {
        *self = *self * s;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let h = angle * 0.5;
        let s = h.sin();
        let a = axis.normalized();
        Self::new(h.cos(), a.x * s, a.y * s, a.z * s)
    }
    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }
    pub fn normalized(self) -> Self {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n > 1.0e-12 { Self::new(self.w / n, self.x / n, self.y / n, self.z / n) } else { Self::IDENTITY }
    }
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v) * 2.0;
        v + t * self.w + qv.cross(t)
    }
    pub fn integrate_world_angular_velocity(self, omega: Vec3, dt: f64) -> Self {
        let angle = omega.length() * dt;
        if angle < 1.0e-14 {
            return self;
        }
        (Self::from_axis_angle(omega.normalized(), angle) * self).normalized()
    }
}

impl Mul for Quat {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        Self::new(
            self.w * b.w - self.x * b.x - self.y * b.y - self.z * b.z,
            self.w * b.x + self.x * b.w + self.y * b.z - self.z * b.y,
            self.w * b.y - self.x * b.z + self.y * b.w + self.z * b.x,
            self.w * b.z + self.x * b.y - self.y * b.x + self.z * b.w,
        )
    }
}

pub fn clamp01(v: f64) -> f64 {
    v.clamp(0.0, 1.0)
}
pub fn smoothstep(a: f64, b: f64, x: f64) -> f64 {
    let t = clamp01((x - a) / (b - a));
    t * t * (3.0 - 2.0 * t)
}
