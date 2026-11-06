//! Rotation representations as flat number lists, 6D, batch conversion and code snippets.

use crate::euler::{Euler, EulerSeq, Frame};
use crate::math::{nearest_rotation, Mat3, OrthoError, Quat, QuatOrder, Vec3};
use core::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AngleUnit {
    Deg,
    Rad,
}

impl AngleUnit {
    pub fn to_rad(self, v: f64) -> f64 {
        match self {
            AngleUnit::Deg => v.to_radians(),
            AngleUnit::Rad => v,
        }
    }
    pub fn from_rad(self, v: f64) -> f64 {
        match self {
            AngleUnit::Deg => v.to_degrees(),
            AngleUnit::Rad => v,
        }
    }
}

/// 6D continuous representation (Zhou et al. 2019): the first two columns of R,
/// laid out as `[r00, r10, r20, r01, r11, r21]`.
pub fn mat_to_6d(m: &Mat3) -> [f64; 6] {
    let c0 = m.col(0);
    let c1 = m.col(1);
    [c0.x, c0.y, c0.z, c1.x, c1.y, c1.z]
}

/// Gram-Schmidt the two 3-vectors of a 6D representation back to a rotation matrix.
pub fn mat_from_6d(v: &[f64; 6]) -> Option<Mat3> {
    let a1 = Vec3::new(v[0], v[1], v[2]);
    let a2 = Vec3::new(v[3], v[4], v[5]);
    let b1 = a1.normalized()?;
    let b2 = (a2 - b1.scale(b1.dot(a2))).normalized()?;
    let b3 = b1.cross(b2);
    Some(Mat3::from_cols(b1, b2, b3))
}

/// A representation of a rotation as a list of numbers (used by batch mode and parsing).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Repr {
    Quat(QuatOrder),
    Euler { seq: EulerSeq, frame: Frame, unit: AngleUnit },
    /// 9 numbers, row-major.
    Matrix,
    /// `axis_x, axis_y, axis_z, angle`.
    AxisAngle(AngleUnit),
    /// so(3) rotation vector, radians.
    RotVec,
    SixD,
}

impl Repr {
    pub fn len(&self) -> usize {
        match self {
            Repr::Quat(_) | Repr::AxisAngle(_) => 4,
            Repr::Euler { .. } | Repr::RotVec => 3,
            Repr::Matrix => 9,
            Repr::SixD => 6,
        }
    }

    pub fn is_empty(&self) -> bool {
        false
    }

    /// CSV header column names.
    pub fn header(&self) -> Vec<String> {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        match self {
            Repr::Quat(QuatOrder::Wxyz) => s(&["qw", "qx", "qy", "qz"]),
            Repr::Quat(QuatOrder::Xyzw) => s(&["qx", "qy", "qz", "qw"]),
            Repr::Euler { seq, unit, .. } => {
                let u = if *unit == AngleUnit::Deg { "deg" } else { "rad" };
                seq.name().chars().enumerate().map(|(i, c)| format!("{}{}_{}", c.to_ascii_lowercase(), i + 1, u)).collect()
            }
            Repr::Matrix => s(&["r00", "r01", "r02", "r10", "r11", "r12", "r20", "r21", "r22"]),
            Repr::AxisAngle(u) => {
                let a = if *u == AngleUnit::Deg { "angle_deg" } else { "angle_rad" };
                s(&["ax", "ay", "az", a])
            }
            Repr::RotVec => s(&["rx", "ry", "rz"]),
            Repr::SixD => s(&["r00", "r10", "r20", "r01", "r11", "r21"]),
        }
    }

    /// Convert a rotation to this representation.
    pub fn from_quat(&self, q: Quat) -> Vec<f64> {
        let q = q.normalized().unwrap_or(Quat::IDENTITY);
        match *self {
            Repr::Quat(order) => q.canonical().to_array(order).to_vec(),
            Repr::Euler { seq, frame, unit } => {
                let (e, _) = Euler::from_quat(q, seq, frame);
                e.angles.iter().map(|&a| unit.from_rad(a)).collect()
            }
            Repr::Matrix => q.to_mat().to_row_major().to_vec(),
            Repr::AxisAngle(unit) => {
                let (ax, ang) = q.to_axis_angle();
                vec![ax.x, ax.y, ax.z, unit.from_rad(ang)]
            }
            Repr::RotVec => q.to_rotvec().to_array().to_vec(),
            Repr::SixD => mat_to_6d(&q.to_mat()).to_vec(),
        }
    }

    /// Parse numbers in this representation into a unit quaternion.
    /// Quaternions are normalized; matrices are projected to the nearest rotation.
    pub fn to_quat(&self, v: &[f64]) -> Result<Quat, String> {
        if v.len() != self.len() {
            return Err(format!("expected {} numbers, got {}", self.len(), v.len()));
        }
        if v.iter().any(|x| !x.is_finite()) {
            return Err("non-finite value".into());
        }
        match *self {
            Repr::Quat(order) => {
                let q = Quat::from_array([v[0], v[1], v[2], v[3]], order);
                q.normalized().ok_or_else(|| "zero quaternion".to_string())
            }
            Repr::Euler { seq, frame, unit } => {
                Ok(Euler::new(seq, frame, [unit.to_rad(v[0]), unit.to_rad(v[1]), unit.to_rad(v[2])]).to_quat())
            }
            Repr::Matrix => {
                let m = Mat3::from_row_major(v);
                let r = nearest_rotation(&m).map_err(ortho_msg)?;
                Ok(Quat::from_mat(&r))
            }
            Repr::AxisAngle(unit) => {
                let axis = Vec3::new(v[0], v[1], v[2]);
                let ang = unit.to_rad(v[3]);
                if axis.normalized().is_none() {
                    if ang.abs() < 1e-12 {
                        return Ok(Quat::IDENTITY);
                    }
                    return Err("zero axis".into());
                }
                Ok(Quat::from_axis_angle(axis, ang))
            }
            Repr::RotVec => Ok(Quat::from_rotvec(Vec3::new(v[0], v[1], v[2]))),
            Repr::SixD => {
                let a = [v[0], v[1], v[2], v[3], v[4], v[5]];
                mat_from_6d(&a).map(|m| Quat::from_mat(&m)).ok_or_else(|| "degenerate 6D vectors".to_string())
            }
        }
    }
}

pub fn ortho_msg(e: OrthoError) -> String {
    match e {
        OrthoError::NotFinite => "matrix has non-finite values".into(),
        OrthoError::Singular => "matrix is singular".into(),
        OrthoError::Reflection => "det < 0: reflection, not a rotation".into(),
        OrthoError::NoConvergence => "orthonormalization did not converge".into(),
    }
}

/// Split a line of numbers separated by commas, semicolons, whitespace or brackets.
/// Returns `Ok(None)` for blank lines and `#` comments.
pub fn parse_numbers(line: &str) -> Result<Option<Vec<f64>>, String> {
    let t = line.trim();
    if t.is_empty() || t.starts_with('#') || t.starts_with("//") {
        return Ok(None);
    }
    let mut out = Vec::new();
    for tok in t.split(|c: char| c == ',' || c == ';' || c.is_whitespace() || "[](){}".contains(c)) {
        if tok.is_empty() {
            continue;
        }
        match tok.parse::<f64>() {
            Ok(v) if v.is_finite() => out.push(v),
            _ => return Err(format!("not a number: '{}'", tok)),
        }
    }
    if out.is_empty() {
        return Ok(None);
    }
    Ok(Some(out))
}

/// One converted line of batch input.
#[derive(Clone, Debug, PartialEq)]
pub struct BatchLine {
    /// 1-based line number in the input.
    pub line: usize,
    pub result: Result<Vec<f64>, String>,
}

/// Convert many lines. Blank lines / comments are skipped. If the first non-empty line fails to
/// parse as numbers it is treated as a header and skipped.
pub fn convert_batch(text: &str, input: &Repr, output: &Repr) -> Vec<BatchLine> {
    let mut out = Vec::new();
    let mut first = true;
    for (idx, line) in text.lines().enumerate() {
        let parsed = parse_numbers(line);
        let was_first = first;
        match &parsed {
            Ok(None) => continue,
            _ => first = false,
        }
        let result = match parsed {
            Ok(Some(v)) => input.to_quat(&v).map(|q| output.from_quat(q)),
            Err(_) if was_first && line.chars().any(|c| c.is_ascii_alphabetic()) => continue,
            Err(e) => Err(e),
            Ok(None) => unreachable!(),
        };
        out.push(BatchLine { line: idx + 1, result });
    }
    out
}

/// Format a number compactly: fixed `digits` decimals with trailing zeros trimmed, no `-0`.
pub fn fmt_num(v: f64, digits: usize) -> String {
    let s = format!("{:.*}", digits, v);
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    if s == "-0" {
        "0".into()
    } else {
        s
    }
}

/// CSV text with a header row; failed lines become a row with an `error` column.
pub fn batch_to_csv(lines: &[BatchLine], output: &Repr, digits: usize) -> String {
    let mut s = String::from("line,");
    s.push_str(&output.header().join(","));
    s.push_str(",error\n");
    for l in lines {
        s.push_str(&l.line.to_string());
        s.push(',');
        match &l.result {
            Ok(v) => {
                s.push_str(&v.iter().map(|x| fmt_num(*x, digits)).collect::<Vec<_>>().join(","));
                s.push_str(",\n");
            }
            Err(e) => {
                s.push_str(&vec![""; output.len()].join(","));
                s.push_str(",\"");
                s.push_str(&e.replace('"', "'"));
                s.push_str("\"\n");
            }
        }
    }
    s
}

fn join(v: &[f64], digits: usize) -> String {
    v.iter().map(|x| fmt_num(*x, digits)).collect::<Vec<_>>().join(", ")
}

/// Code snippets for copy buttons.
pub mod snippet {
    use super::*;

    /// URDF `<origin>` with fixed-axis roll/pitch/yaw (radians).
    pub fn urdf_origin(q: Quat, digits: usize) -> String {
        let (e, _) = Euler::from_quat(q, EulerSeq([0, 1, 2]), Frame::Extrinsic);
        let a = e.angles;
        format!(
            "<origin xyz=\"0 0 0\" rpy=\"{} {} {}\"/>",
            fmt_num(a[0], digits),
            fmt_num(a[1], digits),
            fmt_num(a[2], digits)
        )
    }

    pub fn python_list(v: &[f64], digits: usize) -> String {
        format!("[{}]", join(v, digits))
    }

    pub fn numpy_array(v: &[f64], digits: usize) -> String {
        format!("np.array([{}])", join(v, digits))
    }

    /// 3x3 numpy matrix literal.
    pub fn numpy_matrix(m: &Mat3, digits: usize) -> String {
        let rows: Vec<String> = m.m.iter().map(|r| format!("[{}]", join(r, digits))).collect();
        format!("np.array([{},\n          {},\n          {}])", rows[0], rows[1], rows[2])
    }

    /// `geometry_msgs/Quaternion` YAML (ROS order x, y, z, w).
    pub fn ros_quaternion_yaml(q: Quat, digits: usize) -> String {
        let q = q.normalized().unwrap_or(Quat::IDENTITY);
        format!(
            "orientation:\n  x: {}\n  y: {}\n  z: {}\n  w: {}",
            fmt_num(q.x, digits),
            fmt_num(q.y, digits),
            fmt_num(q.z, digits),
            fmt_num(q.w, digits)
        )
    }
}

/// Shareable URL fragment state: `q=w,x,y,z` (always wxyz) plus optional settings.
pub mod fragment {
    use super::*;

    #[derive(Clone, Debug, PartialEq)]
    pub struct State {
        pub q: Quat,
        pub seq: EulerSeq,
        pub frame: Frame,
        pub unit: AngleUnit,
        pub order: QuatOrder,
    }

    impl Default for State {
        fn default() -> Self {
            State {
                q: Quat::IDENTITY,
                seq: EulerSeq([0, 1, 2]),
                frame: Frame::Extrinsic,
                unit: AngleUnit::Deg,
                order: QuatOrder::Wxyz,
            }
        }
    }

    pub fn encode(s: &State) -> String {
        let q = s.q.canonical();
        format!(
            "q={},{},{},{}&seq={}&f={}&u={}&o={}",
            fmt_num(q.w, 9),
            fmt_num(q.x, 9),
            fmt_num(q.y, 9),
            fmt_num(q.z, 9),
            s.seq.name(),
            if s.frame == Frame::Intrinsic { "i" } else { "e" },
            if s.unit == AngleUnit::Deg { "deg" } else { "rad" },
            if s.order == QuatOrder::Wxyz { "wxyz" } else { "xyzw" }
        )
    }

    /// Parse a fragment (with or without leading `#`). Unknown / bad keys are ignored.
    pub fn decode(frag: &str) -> State {
        let mut s = State::default();
        for kv in frag.trim_start_matches('#').split('&') {
            let (k, v) = match kv.split_once('=') {
                Some(p) => p,
                None => continue,
            };
            match k {
                "q" => {
                    let n: Vec<f64> = v.split(',').filter_map(|t| t.parse().ok()).collect();
                    if n.len() == 4 {
                        if let Some(q) = Quat::new(n[0], n[1], n[2], n[3]).normalized() {
                            s.q = q;
                        }
                    }
                }
                "seq" => {
                    if let Some(q) = EulerSeq::parse(v) {
                        s.seq = q;
                    }
                }
                "f" => s.frame = if v == "i" { Frame::Intrinsic } else { Frame::Extrinsic },
                "u" => s.unit = if v == "rad" { AngleUnit::Rad } else { AngleUnit::Deg },
                "o" => s.order = if v == "xyzw" { QuatOrder::Xyzw } else { QuatOrder::Wxyz },
                _ => {}
            }
        }
        s
    }
}

/// Wrap degrees to (-180, 180].
pub fn wrap_deg(a: f64) -> f64 {
    crate::math::wrap_pi(a * PI / 180.0) * 180.0 / PI
}
