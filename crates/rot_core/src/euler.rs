//! Euler angles: all 12 axis sequences, intrinsic or extrinsic.
//!
//! - **Intrinsic** `A-B-C` with angles `(a, b, c)`: rotate about the body axis A by `a`, then about the
//!   *new* body axis B by `b`, then about the newest C by `c`: `R = R_A(a) R_B(b) R_C(c)`.
//! - **Extrinsic** `A-B-C` with angles `(a, b, c)`: rotate about the fixed world axis A by `a`, then
//!   world B by `b`, then world C by `c`: `R = R_C(c) R_B(b) R_A(a)`.
//!
//! Hence extrinsic `A-B-C (a,b,c)` == intrinsic `C-B-A (c,b,a)`.
//! URDF `rpy` = extrinsic X-Y-Z (roll, pitch, yaw) = intrinsic Z-Y-X (yaw, pitch, roll).

use crate::math::{wrap_pi, Mat3, Quat, Vec3};
use core::f64::consts::{FRAC_PI_2, PI};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame {
    Intrinsic,
    Extrinsic,
}

/// An axis sequence such as XYZ or ZYZ (axis indices 0 = X, 1 = Y, 2 = Z).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EulerSeq(pub [usize; 3]);

impl EulerSeq {
    /// The 6 Tait-Bryan followed by the 6 proper Euler sequences.
    pub const ALL: [EulerSeq; 12] = [
        EulerSeq([0, 1, 2]),
        EulerSeq([0, 2, 1]),
        EulerSeq([1, 0, 2]),
        EulerSeq([1, 2, 0]),
        EulerSeq([2, 0, 1]),
        EulerSeq([2, 1, 0]),
        EulerSeq([0, 1, 0]),
        EulerSeq([0, 2, 0]),
        EulerSeq([1, 0, 1]),
        EulerSeq([1, 2, 1]),
        EulerSeq([2, 0, 2]),
        EulerSeq([2, 1, 2]),
    ];

    /// Parse "XYZ", "zyx", "x-y-z", ... (adjacent axes must differ).
    pub fn parse(s: &str) -> Option<EulerSeq> {
        let mut out = [0usize; 3];
        let mut n = 0;
        for ch in s.chars() {
            let a = match ch.to_ascii_uppercase() {
                'X' => 0,
                'Y' => 1,
                'Z' => 2,
                '-' | ' ' | '_' => continue,
                _ => return None,
            };
            if n == 3 {
                return None;
            }
            out[n] = a;
            n += 1;
        }
        let seq = EulerSeq(out);
        (n == 3 && seq.is_valid()).then_some(seq)
    }

    pub fn is_valid(self) -> bool {
        let [a, b, c] = self.0;
        a < 3 && b < 3 && c < 3 && a != b && b != c
    }

    /// True for proper Euler sequences (first axis == last axis, e.g. ZYZ).
    pub fn is_proper(self) -> bool {
        self.0[0] == self.0[2]
    }

    pub fn name(self) -> String {
        self.0.iter().map(|&a| ['X', 'Y', 'Z'][a]).collect()
    }

    pub fn reversed(self) -> EulerSeq {
        EulerSeq([self.0[2], self.0[1], self.0[0]])
    }
}

/// Euler angles (radians) with their convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Euler {
    pub seq: EulerSeq,
    pub frame: Frame,
    pub angles: [f64; 3],
}

/// Result of decomposing a rotation into Euler angles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EulerResult {
    pub angles: [f64; 3],
    /// Angular distance (radians) of the middle angle from the singular configuration.
    /// Near 0 means gimbal lock: only a sum/difference of the outer angles is defined.
    pub singularity_margin: f64,
    /// True when the decomposition hit the numerical singularity and fixed one outer angle to 0.
    pub gimbal_lock: bool,
}

impl EulerResult {
    /// Gimbal-lock warning threshold used by the UI (0.1 degree by default).
    pub fn near_gimbal_lock(&self, threshold_rad: f64) -> bool {
        self.gimbal_lock || self.singularity_margin < threshold_rad
    }
}

fn axis_vec(a: usize) -> Vec3 {
    match a {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

/// Elementary rotation about one coordinate axis as a quaternion.
pub fn axis_quat(axis: usize, angle: f64) -> Quat {
    Quat::from_axis_angle(axis_vec(axis), angle)
}

impl Euler {
    pub fn new(seq: EulerSeq, frame: Frame, angles: [f64; 3]) -> Euler {
        Euler { seq, frame, angles }
    }

    pub fn to_quat(&self) -> Quat {
        let [a, b, c] = self.seq.0;
        let q1 = axis_quat(a, self.angles[0]);
        let q2 = axis_quat(b, self.angles[1]);
        let q3 = axis_quat(c, self.angles[2]);
        let q = match self.frame {
            Frame::Intrinsic => q1 * q2 * q3,
            Frame::Extrinsic => q3 * q2 * q1,
        };
        q.normalized().unwrap_or(Quat::IDENTITY).canonical()
    }

    pub fn to_mat(&self) -> Mat3 {
        self.to_quat().to_mat()
    }

    /// Partial rotation after `progress` in `[0, 3]` elementary steps (used for step-by-step animation).
    pub fn partial_quat(&self, progress: f64) -> Quat {
        let f = |i: f64| (progress - i).clamp(0.0, 1.0);
        let e = Euler { angles: [self.angles[0] * f(0.0), self.angles[1] * f(1.0), self.angles[2] * f(2.0)], ..*self };
        e.to_quat()
    }

    /// Decompose rotation `m` in the given convention.
    pub fn from_mat(m: &Mat3, seq: EulerSeq, frame: Frame) -> (Euler, EulerResult) {
        let res = match frame {
            Frame::Intrinsic => intrinsic_from_mat(m, seq),
            Frame::Extrinsic => {
                // extrinsic A-B-C (a,b,c) == intrinsic C-B-A (c,b,a)
                let r = intrinsic_from_mat(m, seq.reversed());
                EulerResult { angles: [r.angles[2], r.angles[1], r.angles[0]], ..r }
            }
        };
        (Euler { seq, frame, angles: res.angles }, res)
    }

    pub fn from_quat(q: Quat, seq: EulerSeq, frame: Frame) -> (Euler, EulerResult) {
        Euler::from_mat(&q.to_mat(), seq, frame)
    }
}

/// Numerical gimbal threshold for the decomposition (cos/sin of middle angle).
const GIMBAL_EPS: f64 = 1e-9;

fn intrinsic_from_mat(m: &Mat3, seq: EulerSeq) -> EulerResult {
    let [i, j, k0] = seq.0;
    let proper = seq.is_proper();
    let k = if proper { 3 - i - j } else { k0 };
    // Relabel axes (i, j, k) -> (X, Y, Z). For an odd permutation the relabelling is a reflection,
    // which flips the sign of every rotation angle.
    let p = [i, j, k];
    let even = matches!((i, j, k), (0, 1, 2) | (1, 2, 0) | (2, 0, 1));
    let s = if even { 1.0 } else { -1.0 };
    let mut r = [[0.0; 3]; 3];
    for (a, row) in r.iter_mut().enumerate() {
        for (b, v) in row.iter_mut().enumerate() {
            *v = m.m[p[a]][p[b]];
        }
    }
    let (mut angles, margin, lock) = if proper { decompose_xyx(&r) } else { decompose_xyz(&r) };
    for a in angles.iter_mut() {
        *a *= s;
    }
    if proper && angles[1] < 0.0 {
        // keep the middle angle in [0, pi]: R_i(a) R_j(-b) R_i(c) == R_i(a+pi) R_j(b) R_i(c+pi)
        angles = [wrap_pi(angles[0] + PI), -angles[1], wrap_pi(angles[2] + PI)];
    }
    EulerResult { angles, singularity_margin: margin, gimbal_lock: lock }
}

/// `R = Rx(a) Ry(b) Rz(c)`.
fn decompose_xyz(r: &[[f64; 3]; 3]) -> ([f64; 3], f64, bool) {
    let cb = r[0][0].hypot(r[0][1]);
    let b = r[0][2].atan2(cb);
    let margin = FRAC_PI_2 - b.abs();
    if cb > GIMBAL_EPS {
        let a = (-r[1][2]).atan2(r[2][2]);
        let c = (-r[0][1]).atan2(r[0][0]);
        ([a, b, c], margin, false)
    } else {
        // b = ±90°: only a ± c is observable. Choose c = 0; then R = Rx(a) Ry(b).
        let a = r[2][1].atan2(r[1][1]);
        ([a, b, 0.0], margin, true)
    }
}

/// `R = Rx(a) Ry(b) Rx(c)`.
fn decompose_xyx(r: &[[f64; 3]; 3]) -> ([f64; 3], f64, bool) {
    let sb = r[0][1].hypot(r[0][2]);
    let b = sb.atan2(r[0][0]);
    let margin = b.min(core::f64::consts::PI - b);
    if sb > GIMBAL_EPS {
        let a = r[1][0].atan2(-r[2][0]);
        let c = r[0][1].atan2(r[0][2]);
        ([a, b, c], margin, false)
    } else {
        // b = 0 or 180°: choose c = 0.
        let a = r[2][1].atan2(r[1][1]);
        ([a, b, 0.0], margin, true)
    }
}
