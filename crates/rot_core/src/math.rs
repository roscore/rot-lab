//! Minimal vector / quaternion / matrix types.
//!
//! Conventions used everywhere in this crate:
//! - Right-handed frames, column vectors, active rotations: `v' = R v`.
//! - Hamilton quaternions `q = w + xi + yj + zk` with `ij = k`.
//!   `q.rotate(v) == q.to_mat() * v`, and `(a * b).to_mat() == a.to_mat() * b.to_mat()`.

use core::f64::consts::PI;
use core::ops::{Add, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    pub const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    pub const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }
    pub fn from_array(a: [f64; 3]) -> Self {
        Vec3::new(a[0], a[1], a[2])
    }
    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
    pub fn get(self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn scale(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
    /// Unit vector, or `None` for a (near) zero vector.
    pub fn normalized(self) -> Option<Vec3> {
        let n = self.norm();
        if n > 1e-12 && n.is_finite() {
            Some(self.scale(1.0 / n))
        } else {
            None
        }
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

/// Hamilton unit quaternion (scalar first in memory).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Component order of a quaternion written as four numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QuatOrder {
    /// `[w, x, y, z]` — MuJoCo, Eigen constructor, scipy `scalar_first=True`, Isaac Sim.
    Wxyz,
    /// `[x, y, z, w]` — ROS / tf2 / geometry_msgs, scipy default, PyBullet.
    Xyzw,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat::new(1.0, 0.0, 0.0, 0.0);

    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Quat { w, x, y, z }
    }

    pub fn from_array(a: [f64; 4], order: QuatOrder) -> Self {
        match order {
            QuatOrder::Wxyz => Quat::new(a[0], a[1], a[2], a[3]),
            QuatOrder::Xyzw => Quat::new(a[3], a[0], a[1], a[2]),
        }
    }

    pub fn to_array(self, order: QuatOrder) -> [f64; 4] {
        match order {
            QuatOrder::Wxyz => [self.w, self.x, self.y, self.z],
            QuatOrder::Xyzw => [self.x, self.y, self.z, self.w],
        }
    }

    pub fn vec(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn dot(self, o: Quat) -> f64 {
        self.w * o.w + self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Unit quaternion, or `None` when the norm is ~0 or not finite.
    pub fn normalized(self) -> Option<Quat> {
        let n = self.norm();
        if n > 1e-12 && n.is_finite() {
            Some(Quat::new(self.w / n, self.x / n, self.y / n, self.z / n))
        } else {
            None
        }
    }

    pub fn conj(self) -> Quat {
        Quat::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Inverse of a unit quaternion (= conjugate).
    pub fn inverse(self) -> Quat {
        self.conj()
    }

    #[allow(clippy::should_implement_trait)]
    pub fn neg(self) -> Quat {
        Quat::new(-self.w, -self.x, -self.y, -self.z)
    }

    /// Representative with `w >= 0` (q and -q are the same rotation).
    pub fn canonical(self) -> Quat {
        if self.w < 0.0
            || (self.w == 0.0
                && (self.x < 0.0 || (self.x == 0.0 && (self.y < 0.0 || (self.y == 0.0 && self.z < 0.0)))))
        {
            self.neg()
        } else {
            self
        }
    }

    /// Rotation of `angle` radians about `axis` (need not be unit). Zero axis → identity.
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Quat {
        match axis.normalized() {
            Some(a) => {
                let (s, c) = (angle * 0.5).sin_cos();
                Quat::new(c, a.x * s, a.y * s, a.z * s)
            }
            None => Quat::IDENTITY,
        }
    }

    /// Exponential map: rotation vector (axis * angle) → quaternion.
    pub fn from_rotvec(v: Vec3) -> Quat {
        let angle = v.norm();
        if angle < 1e-12 {
            // first-order expansion keeps tiny rotations exact enough
            Quat::new(1.0, v.x * 0.5, v.y * 0.5, v.z * 0.5).normalized().unwrap_or(Quat::IDENTITY)
        } else {
            Quat::from_axis_angle(v, angle)
        }
    }

    /// `(unit axis, angle)` with angle in `[0, pi]`. Identity returns axis +X, angle 0.
    pub fn to_axis_angle(self) -> (Vec3, f64) {
        let q = self.canonical();
        let s = q.vec().norm();
        let angle = 2.0 * s.atan2(q.w);
        match q.vec().normalized() {
            Some(axis) if s > 1e-15 => (axis, angle),
            _ => (Vec3::X, 0.0),
        }
    }

    /// Logarithm map (so(3) rotation vector), magnitude in `[0, pi]`.
    pub fn to_rotvec(self) -> Vec3 {
        let q = self.canonical();
        let s = q.vec().norm();
        if s < 1e-12 {
            // angle ≈ 2s, axis*angle ≈ 2 * vec
            return q.vec().scale(2.0 / q.w.max(1e-300));
        }
        let angle = 2.0 * s.atan2(q.w);
        q.vec().scale(angle / s)
    }

    /// Rotate a vector: `q v q*`.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let u = self.vec();
        let t = u.cross(v).scale(2.0);
        v + t.scale(self.w) + u.cross(t)
    }

    pub fn to_mat(self) -> Mat3 {
        let Quat { w, x, y, z } = self;
        Mat3::new([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ])
    }

    /// Quaternion from a rotation matrix (Shepperd's method; assumes `m` is a rotation).
    pub fn from_mat(m: &Mat3) -> Quat {
        let r = &m.m;
        let tr = r[0][0] + r[1][1] + r[2][2];
        let q = if tr > r[0][0] && tr > r[1][1] && tr > r[2][2] {
            let s = (1.0 + tr).sqrt() * 2.0; // 4w
            Quat::new(0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
        } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
            let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0; // 4x
            Quat::new((r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s)
        } else if r[1][1] > r[2][2] {
            let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0; // 4y
            Quat::new((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s)
        } else {
            let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0; // 4z
            Quat::new((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s)
        };
        q.normalized().unwrap_or(Quat::IDENTITY).canonical()
    }

    /// Geodesic distance on SO(3) in radians, in `[0, pi]`.
    pub fn angle_to(self, o: Quat) -> f64 {
        // angle of the relative rotation self^-1 * o
        let r = self.conj() * o;
        let s = r.vec().norm();
        2.0 * s.atan2(r.w.abs())
    }

    /// Spherical linear interpolation along the shortest arc; `t=0 → self`, `t=1 → o` (same rotation).
    pub fn slerp(self, o: Quat, t: f64) -> Quat {
        let mut b = o;
        let mut d = self.dot(b);
        if d < 0.0 {
            b = b.neg();
            d = -d;
        }
        if d > 0.9995 {
            let q = Quat::new(
                self.w + (b.w - self.w) * t,
                self.x + (b.x - self.x) * t,
                self.y + (b.y - self.y) * t,
                self.z + (b.z - self.z) * t,
            );
            return q.normalized().unwrap_or(self);
        }
        let theta = d.min(1.0).acos();
        let s = theta.sin();
        let wa = ((1.0 - t) * theta).sin() / s;
        let wb = (t * theta).sin() / s;
        Quat::new(
            wa * self.w + wb * b.w,
            wa * self.x + wb * b.x,
            wa * self.y + wb * b.y,
            wa * self.z + wb * b.z,
        )
    }

    /// True when both quaternions encode the same rotation (q ~ ±o).
    pub fn same_rotation(self, o: Quat, tol: f64) -> bool {
        self.angle_to(o) <= tol
    }
}

impl Mul for Quat {
    type Output = Quat;
    /// Hamilton product. `(a * b).rotate(v) == a.rotate(b.rotate(v))`.
    fn mul(self, b: Quat) -> Quat {
        let a = self;
        Quat::new(
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        )
    }
}

/// Row-major 3x3 matrix: `m[row][col]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    pub m: [[f64; 3]; 3],
}

impl Default for Mat3 {
    fn default() -> Self {
        Mat3::IDENTITY
    }
}

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3 { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] };

    pub const fn new(m: [[f64; 3]; 3]) -> Self {
        Mat3 { m }
    }

    /// From 9 numbers in row-major order.
    pub fn from_row_major(a: &[f64]) -> Mat3 {
        Mat3::new([[a[0], a[1], a[2]], [a[3], a[4], a[5]], [a[6], a[7], a[8]]])
    }

    pub fn to_row_major(&self) -> [f64; 9] {
        let r = &self.m;
        [r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]]
    }

    pub fn from_cols(c0: Vec3, c1: Vec3, c2: Vec3) -> Mat3 {
        Mat3::new([[c0.x, c1.x, c2.x], [c0.y, c1.y, c2.y], [c0.z, c1.z, c2.z]])
    }

    pub fn col(&self, j: usize) -> Vec3 {
        Vec3::new(self.m[0][j], self.m[1][j], self.m[2][j])
    }

    pub fn transpose(&self) -> Mat3 {
        let r = &self.m;
        Mat3::new([[r[0][0], r[1][0], r[2][0]], [r[0][1], r[1][1], r[2][1]], [r[0][2], r[1][2], r[2][2]]])
    }

    pub fn det(&self) -> f64 {
        let r = &self.m;
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    }

    pub fn mul_vec(&self, v: Vec3) -> Vec3 {
        let r = &self.m;
        Vec3::new(
            r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z,
            r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
            r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z,
        )
    }

    pub fn scale(&self, s: f64) -> Mat3 {
        let mut o = *self;
        for row in o.m.iter_mut() {
            for v in row.iter_mut() {
                *v *= s;
            }
        }
        o
    }

    pub fn add(&self, b: &Mat3) -> Mat3 {
        let mut o = *self;
        for i in 0..3 {
            for j in 0..3 {
                o.m[i][j] += b.m[i][j];
            }
        }
        o
    }

    /// Inverse, or `None` if singular.
    pub fn inverse(&self) -> Option<Mat3> {
        let d = self.det();
        if d.abs() < 1e-14 || !d.is_finite() {
            return None;
        }
        let r = &self.m;
        let c = |a: usize, b: usize, c: usize, e: usize| r[a][b] * r[c][e] - r[a][e] * r[c][b];
        // adjugate / det
        let adj = [
            [c(1, 1, 2, 2), -c(0, 1, 2, 2), c(0, 1, 1, 2)],
            [-c(1, 0, 2, 2), c(0, 0, 2, 2), -c(0, 0, 1, 2)],
            [c(1, 0, 2, 1), -c(0, 0, 2, 1), c(0, 0, 1, 1)],
        ];
        Some(Mat3::new(adj).scale(1.0 / d))
    }

    /// Frobenius norm of `R^T R - I` (0 for an exact rotation / reflection).
    pub fn orthogonality_error(&self) -> f64 {
        let p = self.transpose() * *self;
        let mut e = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                let t = p.m[i][j] - if i == j { 1.0 } else { 0.0 };
                e += t * t;
            }
        }
        e.sqrt()
    }

    /// True when `m` is orthonormal with det +1 within `tol`.
    pub fn is_rotation(&self, tol: f64) -> bool {
        self.orthogonality_error() <= tol && (self.det() - 1.0).abs() <= tol
    }

    pub fn max_abs_diff(&self, o: &Mat3) -> f64 {
        let mut e: f64 = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                e = e.max((self.m[i][j] - o.m[i][j]).abs());
            }
        }
        e
    }

    /// Single-axis rotation matrix (axis 0 = X, 1 = Y, 2 = Z).
    pub fn axis_rotation(axis: usize, angle: f64) -> Mat3 {
        let (s, c) = angle.sin_cos();
        match axis {
            0 => Mat3::new([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]),
            1 => Mat3::new([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]),
            _ => Mat3::new([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]),
        }
    }
}

impl Mul for Mat3 {
    type Output = Mat3;
    fn mul(self, b: Mat3) -> Mat3 {
        let mut o = [[0.0; 3]; 3];
        for (i, row) in o.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = self.m[i][0] * b.m[0][j] + self.m[i][1] * b.m[1][j] + self.m[i][2] * b.m[2][j];
            }
        }
        Mat3::new(o)
    }
}

/// Why a 3x3 matrix could not be projected to a rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OrthoError {
    /// Contains NaN / infinity.
    NotFinite,
    /// det ≈ 0: rank deficient.
    Singular,
    /// det < 0: the closest orthogonal matrix is a reflection, not a rotation.
    Reflection,
    /// Iteration did not converge.
    NoConvergence,
}

/// Nearest rotation (in Frobenius norm) via iterative polar decomposition:
/// `X <- (X + X^-T) / 2`, which converges quadratically to the orthogonal polar factor.
pub fn nearest_rotation(m: &Mat3) -> Result<Mat3, OrthoError> {
    if m.m.iter().flatten().any(|v| !v.is_finite()) {
        return Err(OrthoError::NotFinite);
    }
    let d = m.det();
    let scale = m.m.iter().flatten().fold(0.0f64, |a, v| a.max(v.abs())).max(1e-300);
    if d.abs() < 1e-12 * scale * scale * scale {
        return Err(OrthoError::Singular);
    }
    if d < 0.0 {
        return Err(OrthoError::Reflection);
    }
    let mut x = *m;
    for _ in 0..100 {
        let inv_t = match x.inverse() {
            Some(i) => i.transpose(),
            None => return Err(OrthoError::Singular),
        };
        let next = x.add(&inv_t).scale(0.5);
        let delta = next.max_abs_diff(&x);
        x = next;
        if delta < 1e-15 {
            break;
        }
    }
    if x.orthogonality_error() > 1e-9 {
        return Err(OrthoError::NoConvergence);
    }
    Ok(x)
}

/// Wrap an angle to `(-pi, pi]`.
pub fn wrap_pi(a: f64) -> f64 {
    let mut r = a % (2.0 * PI);
    if r <= -PI {
        r += 2.0 * PI;
    } else if r > PI {
        r -= 2.0 * PI;
    }
    r
}
