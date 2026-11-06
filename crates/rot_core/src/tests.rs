use super::repr::{convert_batch, fmt_num, fragment, mat_from_6d, mat_to_6d, parse_numbers, snippet};
use super::*;
use core::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

const TOL: f64 = 1e-9;

/// Small deterministic PRNG (xorshift64*), so tests need no dependencies.
struct Rng(u64);
impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let v = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D);
        (v >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
    /// Uniform random rotation (Shoemake).
    fn quat(&mut self) -> Quat {
        let (u1, u2, u3) = (self.next_f64(), self.range(0.0, 2.0 * PI), self.range(0.0, 2.0 * PI));
        let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
        Quat::new(b * u3.cos(), a * u2.sin(), a * u2.cos(), b * u3.sin())
    }
}

fn assert_same_rot(a: Quat, b: Quat, ctx: &str) {
    let d = a.angle_to(b);
    assert!(d < 1e-8, "{ctx}: rotations differ by {d} rad: {a:?} vs {b:?}");
}

fn assert_vec(a: Vec3, b: Vec3, ctx: &str) {
    assert!((a - b).norm() < TOL, "{ctx}: {a:?} != {b:?}");
}

#[test]
fn quat_matrix_consistency() {
    let mut rng = Rng(7);
    for _ in 0..200 {
        let a = rng.quat();
        let b = rng.quat();
        let v = Vec3::new(rng.range(-2.0, 2.0), rng.range(-2.0, 2.0), rng.range(-2.0, 2.0));
        assert_vec(a.rotate(v), a.to_mat().mul_vec(v), "rotate vs matrix");
        assert!(((a * b).to_mat()).max_abs_diff(&(a.to_mat() * b.to_mat())) < TOL);
        assert_vec((a * b).rotate(v), a.rotate(b.rotate(v)), "composition");
        assert_same_rot(Quat::from_mat(&a.to_mat()), a, "mat round trip");
        assert!(a.to_mat().is_rotation(1e-9));
        let inv = a.to_mat().inverse().unwrap();
        assert!(inv.max_abs_diff(&a.to_mat().transpose()) < TOL, "inverse == transpose");
    }
}

#[test]
fn known_90_deg_about_each_axis() {
    let h = FRAC_1_SQRT_2;
    // 90° about +Z maps +X -> +Y (right-handed, active rotation)
    let qz = Quat::from_axis_angle(Vec3::Z, FRAC_PI_2);
    assert_vec(qz.rotate(Vec3::X), Vec3::Y, "Rz90 x");
    assert_vec(qz.rotate(Vec3::Y), -Vec3::X, "Rz90 y");
    assert!((qz.w - h).abs() < TOL && (qz.z - h).abs() < TOL);
    let qx = Quat::from_axis_angle(Vec3::X, FRAC_PI_2);
    assert_vec(qx.rotate(Vec3::Y), Vec3::Z, "Rx90 y");
    let qy = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);
    assert_vec(qy.rotate(Vec3::Z), Vec3::X, "Ry90 z");
    // matrices
    let mz = qz.to_mat();
    assert!(mz.max_abs_diff(&Mat3::new([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])) < TOL);
    for axis in 0..3 {
        let q = euler::axis_quat(axis, FRAC_PI_2);
        assert!(q.to_mat().max_abs_diff(&Mat3::axis_rotation(axis, FRAC_PI_2)) < TOL);
        // every Euler convention whose first axis is `axis` gives (90°, 0, 0) or an equivalent
        for seq in EulerSeq::ALL {
            for frame in [Frame::Intrinsic, Frame::Extrinsic] {
                let (e, _) = Euler::from_quat(q, seq, frame);
                assert_same_rot(e.to_quat(), q, "90deg decomposition");
            }
        }
    }
    // axis-angle / rotvec
    let (ax, ang) = qz.to_axis_angle();
    assert_vec(ax, Vec3::Z, "axis");
    assert!((ang - FRAC_PI_2).abs() < TOL);
    assert_vec(qz.to_rotvec(), Vec3::new(0.0, 0.0, FRAC_PI_2), "rotvec");
}

#[test]
fn euler_round_trip_all_24_conventions() {
    let mut rng = Rng(12345);
    let qs: Vec<Quat> = (0..500).map(|_| rng.quat()).collect();
    for seq in EulerSeq::ALL {
        for frame in [Frame::Intrinsic, Frame::Extrinsic] {
            for &q in &qs {
                let (e, res) = Euler::from_quat(q, seq, frame);
                assert!(!res.gimbal_lock);
                assert_same_rot(e.to_quat(), q, &format!("{} {:?}", seq.name(), frame));
                // middle angle range
                let b = e.angles[1];
                if seq.is_proper() {
                    assert!((0.0..=PI).contains(&b));
                } else {
                    assert!((-FRAC_PI_2..=FRAC_PI_2).contains(&b));
                }
            }
            // angles -> quat -> angles reproduces the angles inside the principal range
            for _ in 0..200 {
                let mid = if seq.is_proper() { rng.range(0.05, PI - 0.05) } else { rng.range(-1.5, 1.5) };
                let a = [rng.range(-3.1, 3.1), mid, rng.range(-3.1, 3.1)];
                let q = Euler::new(seq, frame, a).to_quat();
                let (e, _) = Euler::from_quat(q, seq, frame);
                for i in 0..3 {
                    assert!((e.angles[i] - a[i]).abs() < 1e-7, "{} {:?}: {:?} vs {:?}", seq.name(), frame, e.angles, a);
                }
            }
        }
    }
}

#[test]
fn euler_definition_matches_matrix_products() {
    let mut rng = Rng(99);
    for seq in EulerSeq::ALL {
        let a = [rng.range(-3.0, 3.0), rng.range(-1.0, 1.0), rng.range(-3.0, 3.0)];
        let [i, j, k] = seq.0;
        let ri = Mat3::axis_rotation(i, a[0]);
        let rj = Mat3::axis_rotation(j, a[1]);
        let rk = Mat3::axis_rotation(k, a[2]);
        let intr = Euler::new(seq, Frame::Intrinsic, a).to_mat();
        let extr = Euler::new(seq, Frame::Extrinsic, a).to_mat();
        assert!(intr.max_abs_diff(&(ri * rj * rk)) < TOL);
        assert!(extr.max_abs_diff(&(rk * rj * ri)) < TOL);
    }
}

#[test]
fn intrinsic_equals_reversed_extrinsic() {
    let mut rng = Rng(4242);
    for seq in EulerSeq::ALL {
        for _ in 0..50 {
            let a = [rng.range(-3.0, 3.0), rng.range(-1.5, 1.5), rng.range(-3.0, 3.0)];
            let intr = Euler::new(seq, Frame::Intrinsic, a).to_quat();
            let extr = Euler::new(seq.reversed(), Frame::Extrinsic, [a[2], a[1], a[0]]).to_quat();
            assert_same_rot(intr, extr, &seq.name());
        }
    }
    // the classic example: intrinsic XYZ == extrinsic ZYX with reversed angles
    let a = [0.3, -0.7, 1.1];
    let xyz = EulerSeq::parse("XYZ").unwrap();
    let zyx = EulerSeq::parse("ZYX").unwrap();
    assert_same_rot(
        Euler::new(xyz, Frame::Intrinsic, a).to_quat(),
        Euler::new(zyx, Frame::Extrinsic, [a[2], a[1], a[0]]).to_quat(),
        "XYZ intrinsic vs ZYX extrinsic",
    );
}

#[test]
fn gimbal_lock_reproduces_matrix() {
    let mut rng = Rng(2024);
    for seq in EulerSeq::ALL {
        let mids: &[f64] = if seq.is_proper() { &[0.0, PI] } else { &[FRAC_PI_2, -FRAC_PI_2] };
        for frame in [Frame::Intrinsic, Frame::Extrinsic] {
            for &mid in mids {
                for _ in 0..20 {
                    let a = [rng.range(-3.0, 3.0), mid, rng.range(-3.0, 3.0)];
                    let m = Euler::new(seq, frame, a).to_mat();
                    let (e, res) = Euler::from_mat(&m, seq, frame);
                    assert!(res.near_gimbal_lock(1e-6), "{} {:?} should flag gimbal lock", seq.name(), frame);
                    assert!(e.to_mat().max_abs_diff(&m) < 1e-7, "{} {:?} gimbal matrix mismatch", seq.name(), frame);
                }
            }
        }
    }
    // URDF pitch = 90°: roll and yaw collapse, but the matrix is reproduced
    let urdf = Euler::new(EulerSeq([0, 1, 2]), Frame::Extrinsic, [0.4, FRAC_PI_2, -0.9]);
    let (e, res) = Euler::from_mat(&urdf.to_mat(), urdf.seq, urdf.frame);
    assert!(res.gimbal_lock);
    assert!(e.to_mat().max_abs_diff(&urdf.to_mat()) < 1e-9);
}

#[test]
fn urdf_rpy_known_examples() {
    let urdf = EulerSeq::parse("xyz").unwrap();
    // yaw 90°: rpy="0 0 1.5707963" → quat wxyz (√½, 0, 0, √½), ROS xyzw (0, 0, √½, √½)
    let q = Euler::new(urdf, Frame::Extrinsic, [0.0, 0.0, FRAC_PI_2]).to_quat();
    let h = FRAC_1_SQRT_2;
    let w = q.to_array(QuatOrder::Wxyz);
    let expect = [h, 0.0, 0.0, h];
    for i in 0..4 {
        assert!((w[i] - expect[i]).abs() < TOL);
    }
    let x = q.to_array(QuatOrder::Xyzw);
    assert!((x[2] - h).abs() < TOL && (x[3] - h).abs() < TOL);

    // rpy = (r, p, y) is R = Rz(y) Ry(p) Rx(r) == intrinsic ZYX (y, p, r) == aerospace preset
    let (r, p, y) = (0.1, 0.2, 0.3);
    let m = Mat3::axis_rotation(2, y) * Mat3::axis_rotation(1, p) * Mat3::axis_rotation(0, r);
    let urdf_q = Euler::new(urdf, Frame::Extrinsic, [r, p, y]).to_quat();
    let aero = PRESETS.iter().find(|p| p.id == "aero").unwrap();
    let aero_q = Euler::new(aero.seq, aero.frame, [y, p, r]).to_quat();
    assert!(urdf_q.to_mat().max_abs_diff(&m) < TOL);
    assert_same_rot(urdf_q, aero_q, "urdf vs aero");
    // numeric reference for rpy = (0.1, 0.2, 0.3)
    let wxyz = urdf_q.to_array(QuatOrder::Wxyz);
    let reference = [0.983_347_443, 0.034_270_799, 0.106_020_511, 0.143_572_175];
    for i in 0..4 {
        assert!((wxyz[i] - reference[i]).abs() < 1e-6, "{wxyz:?}");
    }
    // snippet
    let s = snippet::urdf_origin(urdf_q, 4);
    assert_eq!(s, "<origin xyz=\"0 0 0\" rpy=\"0.1 0.2 0.3\"/>");
}

#[test]
fn quat_order_xyzw_vs_wxyz() {
    let ros_identity = Quat::from_array([0.0, 0.0, 0.0, 1.0], QuatOrder::Xyzw);
    assert_eq!(ros_identity, Quat::IDENTITY);
    let wrong = Quat::from_array([1.0, 0.0, 0.0, 0.0], QuatOrder::Xyzw);
    assert!(wrong.angle_to(Quat::IDENTITY) > 3.0, "x=1 is a 180° flip about X");
    let mj_identity = Quat::from_array([1.0, 0.0, 0.0, 0.0], QuatOrder::Wxyz);
    assert_eq!(mj_identity, Quat::IDENTITY);
    let q = Quat::new(0.1, 0.2, 0.3, 0.4);
    assert_eq!(q.to_array(QuatOrder::Wxyz), [0.1, 0.2, 0.3, 0.4]);
    assert_eq!(q.to_array(QuatOrder::Xyzw), [0.2, 0.3, 0.4, 0.1]);
    assert_eq!(Quat::from_array(q.to_array(QuatOrder::Xyzw), QuatOrder::Xyzw), q);
    // canonical sign
    assert!(Quat::new(-0.5, 0.5, 0.5, 0.5).canonical().w > 0.0);
    // ROS YAML is x, y, z, w
    let yaml = snippet::ros_quaternion_yaml(Quat::new(0.5, 0.5, -0.5, 0.5), 3);
    assert_eq!(yaml, "orientation:\n  x: 0.5\n  y: -0.5\n  z: 0.5\n  w: 0.5");
}

#[test]
fn slerp_endpoints_and_halfway() {
    let mut rng = Rng(77);
    for _ in 0..100 {
        let a = rng.quat();
        let b = rng.quat();
        assert_same_rot(a.slerp(b, 0.0), a, "t=0");
        assert_same_rot(a.slerp(b, 1.0), b, "t=1");
        let m = a.slerp(b, 0.5);
        let d = a.angle_to(b);
        assert!((a.angle_to(m) - d / 2.0).abs() < 1e-8);
        assert!((m.angle_to(b) - d / 2.0).abs() < 1e-8);
        let t = rng.next_f64();
        assert!((a.angle_to(a.slerp(b, t)) - t * d).abs() < 1e-7);
        // shortest path even when b is given with the opposite sign
        assert!((a.angle_to(a.slerp(b.neg(), 0.5)) - d / 2.0).abs() < 1e-8);
    }
    // halfway between identity and 90° about Z is 45° about Z
    let m = Quat::IDENTITY.slerp(Quat::from_axis_angle(Vec3::Z, FRAC_PI_2), 0.5);
    assert_same_rot(m, Quat::from_axis_angle(Vec3::Z, PI / 4.0), "45deg");
}

#[test]
fn geodesic_inverse_and_relative() {
    let a = Quat::from_axis_angle(Vec3::Z, 0.3);
    let b = Quat::from_axis_angle(Vec3::Z, 1.0);
    assert!((a.angle_to(b) - 0.7).abs() < TOL);
    assert!((a.angle_to(b.neg()) - 0.7).abs() < TOL, "sign invariant");
    assert_same_rot(a * a.inverse(), Quat::IDENTITY, "inverse");
    let rel = a.inverse() * b;
    assert_same_rot(a * rel, b, "relative");
    assert!((Quat::IDENTITY.angle_to(Quat::from_axis_angle(Vec3::X, PI)) - PI).abs() < TOL);
}

#[test]
fn six_d_round_trip() {
    let mut rng = Rng(31337);
    for _ in 0..200 {
        let q = rng.quat();
        let m = q.to_mat();
        let v = mat_to_6d(&m);
        assert!(mat_from_6d(&v).unwrap().max_abs_diff(&m) < TOL);
        // non-orthogonal, unnormalized input still yields a rotation
        let mut noisy = v;
        for x in noisy.iter_mut() {
            *x = *x * 2.5 + rng.range(-0.1, 0.1);
        }
        let r = mat_from_6d(&noisy).unwrap();
        assert!(r.is_rotation(1e-9));
    }
    assert!(mat_from_6d(&[0.0; 6]).is_none());
    assert!(mat_from_6d(&[1.0, 0.0, 0.0, 2.0, 0.0, 0.0]).is_none(), "parallel columns");
}

#[test]
fn nearest_rotation_polar() {
    let mut rng = Rng(5);
    for _ in 0..100 {
        let r = rng.quat().to_mat();
        let mut noisy = r;
        for row in noisy.m.iter_mut() {
            for v in row.iter_mut() {
                *v += rng.range(-0.02, 0.02);
            }
        }
        assert!(!noisy.is_rotation(1e-6));
        let n = nearest_rotation(&noisy).unwrap();
        assert!(n.is_rotation(1e-9));
        assert!(n.max_abs_diff(&r) < 0.1);
        // exact rotations are fixed points; scaled rotations map back
        assert!(nearest_rotation(&r).unwrap().max_abs_diff(&r) < 1e-12);
        assert!(nearest_rotation(&r.scale(3.0)).unwrap().max_abs_diff(&r) < 1e-12);
    }
    let reflect = Mat3::new([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    assert_eq!(nearest_rotation(&reflect), Err(OrthoError::Reflection));
    let singular = Mat3::new([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
    assert_eq!(nearest_rotation(&singular), Err(OrthoError::Singular));
}

#[test]
fn axis_angle_and_rotvec() {
    let mut rng = Rng(8);
    for _ in 0..200 {
        let q = rng.quat();
        let (ax, ang) = q.to_axis_angle();
        assert!((ax.norm() - 1.0).abs() < TOL && (0.0..=PI).contains(&ang));
        assert_same_rot(Quat::from_axis_angle(ax, ang), q, "axis-angle");
        assert_same_rot(Quat::from_rotvec(q.to_rotvec()), q, "rotvec");
    }
    assert_eq!(Quat::IDENTITY.to_rotvec(), Vec3::default());
    let tiny = Quat::from_rotvec(Vec3::new(1e-14, 0.0, 0.0));
    assert!((tiny.to_rotvec().x - 1e-14).abs() < 1e-20);
}

#[test]
fn repr_round_trip_every_representation() {
    let mut rng = Rng(1);
    let mut reprs = vec![
        Repr::Quat(QuatOrder::Wxyz),
        Repr::Quat(QuatOrder::Xyzw),
        Repr::Matrix,
        Repr::AxisAngle(AngleUnit::Deg),
        Repr::AxisAngle(AngleUnit::Rad),
        Repr::RotVec,
        Repr::SixD,
    ];
    for seq in EulerSeq::ALL {
        reprs.push(Repr::Euler { seq, frame: Frame::Intrinsic, unit: AngleUnit::Deg });
        reprs.push(Repr::Euler { seq, frame: Frame::Extrinsic, unit: AngleUnit::Rad });
    }
    for _ in 0..50 {
        let q = rng.quat();
        for r in &reprs {
            let v = r.from_quat(q);
            assert_eq!(v.len(), r.len());
            assert_eq!(r.header().len(), r.len());
            assert_same_rot(r.to_quat(&v).unwrap(), q, &format!("{r:?}"));
        }
    }
}

#[test]
fn batch_parsing_and_csv() {
    assert_eq!(parse_numbers("  1, 2;3 [4]").unwrap(), Some(vec![1.0, 2.0, 3.0, 4.0]));
    assert_eq!(parse_numbers("1\t2  3e-1").unwrap(), Some(vec![1.0, 2.0, 0.3]));
    assert_eq!(parse_numbers("   ").unwrap(), None);
    assert_eq!(parse_numbers("# comment").unwrap(), None);
    assert!(parse_numbers("1, two, 3").is_err());

    let text = "qw,qx,qy,qz\n1 0 0 0\n\n0.7071068, 0, 0, 0.7071068\n1, 2\nfoo bar baz\n0 0 0 0\n";
    let input = Repr::Quat(QuatOrder::Wxyz);
    let out = Repr::Euler { seq: EulerSeq([0, 1, 2]), frame: Frame::Extrinsic, unit: AngleUnit::Deg };
    let lines = convert_batch(text, &input, &out);
    assert_eq!(lines.len(), 5, "header skipped, blank skipped: {lines:?}");
    assert_eq!(lines[0].line, 2);
    let yaw = lines[1].result.as_ref().unwrap();
    assert!((yaw[2] - 90.0).abs() < 1e-4);
    assert_eq!(lines[2].line, 5);
    assert!(lines[2].result.as_ref().unwrap_err().contains("expected 4"));
    assert!(lines[3].result.is_err(), "non-header text later is an error");
    assert!(lines[4].result.as_ref().unwrap_err().contains("zero"));

    let csv = repr::batch_to_csv(&lines, &out, 4);
    let rows: Vec<&str> = csv.lines().collect();
    assert_eq!(rows[0], "line,x1_deg,y2_deg,z3_deg,error");
    assert_eq!(rows[1], "2,0,0,0,");
    assert_eq!(rows[2], "4,0,0,90,");
    assert!(rows[3].starts_with("5,,,,\"expected 4"));

    // matrices that are slightly off are projected; reflections are rejected
    let m = convert_batch("1 0 0 0 1 0 0 0 1.01\n-1 0 0 0 1 0 0 0 1", &Repr::Matrix, &Repr::Quat(QuatOrder::Xyzw));
    assert_eq!(m[0].result.as_ref().unwrap(), &vec![0.0, 0.0, 0.0, 1.0]);
    assert!(m[1].result.as_ref().unwrap_err().contains("reflection"));
}

#[test]
fn snippets_and_fragment() {
    assert_eq!(fmt_num(-0.0000001, 4), "0");
    assert_eq!(fmt_num(1.5, 4), "1.5");
    assert_eq!(fmt_num(2.0, 4), "2");
    assert_eq!(snippet::python_list(&[1.0, 0.25], 3), "[1, 0.25]");
    assert_eq!(snippet::numpy_array(&[0.0, -1.0], 3), "np.array([0, -1])");
    assert!(snippet::numpy_matrix(&Mat3::IDENTITY, 2).starts_with("np.array([[1, 0, 0],"));

    let s = fragment::State {
        q: Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0), 1.0),
        seq: EulerSeq::parse("ZYZ").unwrap(),
        frame: Frame::Intrinsic,
        unit: AngleUnit::Rad,
        order: QuatOrder::Xyzw,
    };
    let enc = fragment::encode(&s);
    let dec = fragment::decode(&format!("#{enc}"));
    assert_same_rot(dec.q, s.q, "fragment q");
    assert_eq!((dec.seq, dec.frame, dec.unit, dec.order), (s.seq, s.frame, s.unit, s.order));
    let junk = fragment::decode("#q=a,b&seq=XXY&zzz");
    assert_eq!(junk, fragment::State::default());
}

#[test]
fn euler_seq_parsing() {
    assert_eq!(EulerSeq::ALL.len(), 12);
    for s in EulerSeq::ALL {
        assert!(s.is_valid());
        assert_eq!(EulerSeq::parse(&s.name()), Some(s));
    }
    assert_eq!(EulerSeq::parse("z-y-x"), Some(EulerSeq([2, 1, 0])));
    assert_eq!(EulerSeq::parse("XXY"), None);
    assert_eq!(EulerSeq::parse("XY"), None);
    assert_eq!(EulerSeq::parse("XYZX"), None);
}

#[test]
fn partial_animation_steps() {
    let e = Euler::new(EulerSeq([2, 1, 0]), Frame::Intrinsic, [0.5, 0.4, 0.3]);
    assert_same_rot(e.partial_quat(0.0), Quat::IDENTITY, "start");
    assert_same_rot(e.partial_quat(1.0), euler::axis_quat(2, 0.5), "after step 1");
    assert_same_rot(e.partial_quat(3.0), e.to_quat(), "end");
}
