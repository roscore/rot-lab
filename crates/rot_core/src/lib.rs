//! rot_core — 3D rotation conversions for robotics conventions (no dependencies).
//!
//! See [`math`] for the frame / quaternion conventions and [`euler`] for intrinsic vs extrinsic.

pub mod euler;
pub mod math;
pub mod repr;

pub use euler::{Euler, EulerResult, EulerSeq, Frame};
pub use math::{nearest_rotation, Mat3, OrthoError, Quat, QuatOrder, Vec3};
pub use repr::{AngleUnit, Repr};

/// Common conventions offered as one-click presets.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Preset {
    pub id: &'static str,
    pub seq: EulerSeq,
    pub frame: Frame,
    pub order: QuatOrder,
    pub note_en: &'static str,
    pub note_ko: &'static str,
}

pub const PRESETS: [Preset; 4] = [
    Preset {
        id: "urdf",
        seq: EulerSeq([0, 1, 2]),
        frame: Frame::Extrinsic,
        order: QuatOrder::Xyzw,
        note_en: "URDF rpy: fixed-axis roll(X), pitch(Y), yaw(Z) in radians = extrinsic X-Y-Z = intrinsic Z-Y-X. R = Rz(yaw) Ry(pitch) Rx(roll).",
        note_ko: "URDF rpy: 고정축 기준 roll(X) → pitch(Y) → yaw(Z), 라디안 = extrinsic X-Y-Z = intrinsic Z-Y-X. R = Rz(yaw) Ry(pitch) Rx(roll).",
    },
    Preset {
        id: "ros",
        seq: EulerSeq([0, 1, 2]),
        frame: Frame::Extrinsic,
        order: QuatOrder::Xyzw,
        note_en: "ROS tf2 / geometry_msgs: quaternion stored as x, y, z, w (scalar last). tf2 setRPY uses the same fixed-axis rpy as URDF.",
        note_ko: "ROS tf2 / geometry_msgs: 쿼터니언을 x, y, z, w 순서(스칼라가 마지막)로 저장. tf2 setRPY는 URDF와 같은 고정축 rpy.",
    },
    Preset {
        id: "mujoco",
        seq: EulerSeq([0, 1, 2]),
        frame: Frame::Intrinsic,
        order: QuatOrder::Wxyz,
        note_en: "MuJoCo: quat = w, x, y, z (scalar first). MJCF euler defaults to eulerseq=\"xyz\" (lowercase = intrinsic / moving axes) in degrees unless angle=\"radian\".",
        note_ko: "MuJoCo: quat = w, x, y, z (스칼라가 처음). MJCF euler 기본값은 eulerseq=\"xyz\" (소문자 = intrinsic, 움직이는 축), angle=\"radian\"이 아니면 도(degree) 단위.",
    },
    Preset {
        id: "aero",
        seq: EulerSeq([2, 1, 0]),
        frame: Frame::Intrinsic,
        order: QuatOrder::Wxyz,
        note_en: "Aerospace yaw-pitch-roll: intrinsic Z-Y'-X'' (yaw ψ, pitch θ, roll φ). Same rotation as URDF rpy with the angles listed in reverse.",
        note_ko: "항공 yaw-pitch-roll: intrinsic Z-Y'-X'' (yaw ψ, pitch θ, roll φ). URDF rpy와 같은 회전이며 각도 나열 순서만 반대.",
    },
];

#[cfg(test)]
mod tests;
