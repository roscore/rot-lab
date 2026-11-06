//! rot-lab web UI. All state lives in this page; nothing is sent over the network.

mod dom;
mod i18n;
mod viz;

use dom::{el, get, msg, on, on_id, set, show, text};
use i18n::{tr, trf, Lang};
use rot_core::repr::{batch_to_csv, convert_batch, fmt_num, fragment, mat_from_6d, mat_to_6d, parse_numbers, snippet};
use rot_core::{nearest_rotation, AngleUnit, Euler, EulerSeq, Frame, Mat3, OrthoError, Quat, QuatOrder, Repr, Vec3, PRESETS};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{HtmlCanvasElement, PointerEvent};

const DIGITS: usize = 6;
/// Show a "near gimbal lock" warning within this distance of the singularity.
const GIMBAL_WARN_DEG: f64 = 1.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Src {
    None,
    Quat,
    Euler,
    Matrix,
    AxisAngle,
    RotVec,
    SixD,
}

struct Anim {
    start_ms: f64,
    progress: f64,
}

struct App {
    q: Quat,
    order: QuatOrder,
    seq: EulerSeq,
    frame: Frame,
    unit: AngleUnit,
    canonical: bool,
    lang: Lang,
    cam: viz::Cam,
    cam2: viz::Cam,
    drag: Option<(f64, f64)>,
    anim: Option<Anim>,
    pending_fix: Option<Mat3>,
    batch_csv: String,
    preset: Option<&'static str>,
    /// Representation currently used by the compose inputs.
    compose_fmt: Repr,
}

thread_local! {
    static APP: RefCell<App> = RefCell::new(App {
        q: Quat::IDENTITY,
        order: QuatOrder::Wxyz,
        seq: EulerSeq([0, 1, 2]),
        frame: Frame::Extrinsic,
        unit: AngleUnit::Deg,
        canonical: true,
        lang: Lang::En,
        cam: viz::Cam::default(),
        cam2: viz::Cam::default(),
        drag: None,
        anim: None,
        pending_fix: None,
        batch_csv: String::new(),
        preset: None,
        compose_fmt: Repr::Quat(QuatOrder::Wxyz),
    });
}

fn with<R>(f: impl FnOnce(&mut App) -> R) -> R {
    APP.with(|a| f(&mut a.borrow_mut()))
}

fn t(key: &str) -> &'static str {
    tr(with(|a| a.lang), key)
}

fn tf(key: &str, args: &[(&str, &str)]) -> String {
    trf(with(|a| a.lang), key, args)
}

fn f(v: f64) -> String {
    fmt_num(v, DIGITS)
}

// ---------------------------------------------------------------- parsing helpers

/// Read `n` numeric inputs `prefix0..prefix{n-1}`; marks bad fields.
fn read_fields(prefix: &str, n: usize) -> Result<Vec<f64>, String> {
    let mut out = Vec::with_capacity(n);
    let mut err = None;
    for i in 0..n {
        let id = format!("{prefix}{i}");
        let s = get(&id);
        let s = s.trim();
        let v = s.parse::<f64>().ok().filter(|v| v.is_finite());
        dom::set_class(&el(&id), "bad", v.is_none());
        match v {
            Some(v) => out.push(v),
            None if err.is_none() => {
                err = Some(if s.is_empty() {
                    tf("need_n", &[("n", &n.to_string())])
                } else {
                    tf("bad_num", &[("s", s)])
                })
            }
            None => {}
        }
    }
    match err {
        Some(e) => Err(e),
        None => Ok(out),
    }
}

fn clear_bad(prefix: &str, n: usize) {
    for i in 0..n {
        dom::set_class(&el(&format!("{prefix}{i}")), "bad", false);
    }
}

fn fill(prefix: &str, vals: &[f64]) {
    for (i, v) in vals.iter().enumerate() {
        set(&format!("{prefix}{i}"), &f(*v));
    }
}

fn axis_name(a: usize) -> char {
    ['X', 'Y', 'Z'][a]
}

fn euler_conv_name(seq: EulerSeq, frame: Frame, unit: AngleUnit) -> String {
    let fr = if frame == Frame::Intrinsic { "intrinsic" } else { "extrinsic" };
    let u = if unit == AngleUnit::Deg { "deg" } else { "rad" };
    format!("{} {} ({})", seq.name(), fr, u)
}

fn unit_name(u: AngleUnit) -> &'static str {
    if u == AngleUnit::Deg {
        "deg"
    } else {
        "rad"
    }
}

// ---------------------------------------------------------------- rendering

fn display_quat(a: &App, q: Quat) -> Quat {
    if a.canonical {
        q.canonical()
    } else {
        q
    }
}

fn render_quat(a: &App) {
    let labels = match a.order {
        QuatOrder::Wxyz => ["w", "x", "y", "z"],
        QuatOrder::Xyzw => ["x", "y", "z", "w"],
    };
    for (i, l) in labels.iter().enumerate() {
        text(&format!("ql{i}"), l);
    }
    fill("q", &display_quat(a, a.q).to_array(a.order));
    clear_bad("q", 4);
}

fn render_euler_labels(a: &App) {
    for i in 0..3 {
        let ax = axis_name(a.seq.0[i]);
        let prime = if a.frame == Frame::Intrinsic { ["", "′", "″"][i] } else { "" };
        text(&format!("el{i}"), &format!("{}: {}{} ({})", i + 1, ax, prime, unit_name(a.unit)));
    }
    let [i, j, k] = a.seq.0.map(axis_name);
    let formula = match a.frame {
        Frame::Intrinsic => format!("R = R{i}(θ1) · R{j}(θ2) · R{k}(θ3)   [= extrinsic {k}{j}{i} (θ3, θ2, θ1)]"),
        Frame::Extrinsic => format!("R = R{k}(θ3) · R{j}(θ2) · R{i}(θ1)   [= intrinsic {k}{j}{i} (θ3, θ2, θ1)]"),
    };
    text("e-formula", &formula);
}

fn gimbal_message(lang: Lang, margin: f64, lock: bool) -> Option<String> {
    let deg = margin.to_degrees();
    if lock || deg < 1e-3 {
        Some(tr(lang, "gimbal").to_string())
    } else if deg < GIMBAL_WARN_DEG {
        Some(trf(lang, "gimbal_near", &[("d", &fmt_num(deg, 3))]))
    } else {
        None
    }
}

fn render_euler(a: &App) {
    render_euler_labels(a);
    let (e, res) = Euler::from_quat(a.q, a.seq, a.frame);
    let vals: Vec<f64> = e.angles.iter().map(|&x| a.unit.from_rad(x)).collect();
    fill("e", &vals);
    clear_bad("e", 3);
    match gimbal_message(a.lang, res.singularity_margin, res.gimbal_lock) {
        Some(m) => msg("e-msg", "warn", &m),
        None => msg("e-msg", "", ""),
    }
}

fn render_matrix(a: &App) {
    fill("m", &a.q.to_mat().to_row_major());
    clear_bad("m", 9);
    msg("m-msg", "", "");
    show("m-fix", false);
}

fn render_aa(a: &App) {
    let (axis, ang) = a.q.to_axis_angle();
    fill("a", &[axis.x, axis.y, axis.z, a.unit.from_rad(ang)]);
    text("al3", &format!("θ ({})", unit_name(a.unit)));
    clear_bad("a", 4);
    msg("a-msg", "", "");
}

fn render_rv(a: &App) {
    fill("r", &a.q.to_rotvec().to_array());
    clear_bad("r", 3);
    msg("r-msg", "", "");
}

fn render_6d(a: &App) {
    fill("s", &mat_to_6d(&a.q.to_mat()));
    clear_bad("s", 6);
    msg("s-msg", "", "");
}

/// Re-render every representation except the one the user is typing in.
fn render_all(a: &App, src: Src) {
    if src != Src::Quat {
        render_quat(a);
        msg("q-msg", "", "");
    }
    if src != Src::Euler {
        render_euler(a);
    }
    if src != Src::Matrix {
        render_matrix(a);
    }
    if src != Src::AxisAngle {
        render_aa(a);
    }
    if src != Src::RotVec {
        render_rv(a);
    }
    if src != Src::SixD {
        render_6d(a);
    }
    draw_main(a);
    render_conv_notes(a);
    update_fragment(a);
}

fn render_conv_notes(a: &App) {
    let conv = euler_conv_name(a.seq, a.frame, a.unit);
    let note = trf(a.lang, "conv_note", &[("conv", &conv), ("unit", unit_name(a.unit))]);
    text("b-conv", &note);
    let order = if a.order == QuatOrder::Wxyz { "wxyz" } else { "xyzw" };
    text("c-conv", &format!("{note} Quaternion order: {order}."));
}

fn update_fragment(a: &App) {
    let s = fragment::State { q: a.q, seq: a.seq, frame: a.frame, unit: a.unit, order: a.order };
    let frag = format!("#{}", fragment::encode(&s));
    if let Ok(h) = dom::win().history() {
        let _ = h.replace_state_with_url(&JsValue::NULL, "", Some(&frag));
    }
}

fn draw_main(a: &App) {
    let canvas: HtmlCanvasElement = match el("viz").dyn_into() {
        Ok(c) => c,
        Err(_) => return,
    };
    let mut highlight = None;
    let mut bodies = Vec::new();
    let q_draw = if let Some(anim) = &a.anim {
        let p = anim.progress;
        let e = Euler::from_quat(a.q, a.seq, a.frame).0;
        // ghost of the final orientation
        bodies.push(viz::Body { q: a.q, alpha: 0.18, label: "", show_box: false });
        let step = (p.floor() as usize).min(2);
        let axis_idx = a.seq.0[step];
        let base = match axis_idx {
            0 => Vec3::X,
            1 => Vec3::Y,
            _ => Vec3::Z,
        };
        let ang = format!("{} {}", fmt_num(a.unit.from_rad(e.angles[step]), 2), unit_name(a.unit));
        let axis_label;
        let axis_world = if a.frame == Frame::Intrinsic {
            axis_label = trf(a.lang, "body_axis", &[("a", &format!("{}{}", axis_name(axis_idx), ["", "′", "″"][step]))]);
            e.partial_quat(step as f64).rotate(base)
        } else {
            axis_label = trf(a.lang, "world_axis", &[("a", &axis_name(axis_idx).to_string())]);
            base
        };
        if p < 3.0 {
            let msg_s = trf(
                a.lang,
                "step",
                &[("i", &(step + 1).to_string()), ("ang", &ang), ("axis", &axis_label)],
            );
            text("viz-step", &msg_s);
            highlight = Some((axis_world, axis_label));
        } else {
            text("viz-step", &trf(a.lang, "step_done", &[("formula", &get_formula())]));
        }
        e.partial_quat(p)
    } else {
        text("viz-step", "");
        a.q
    };
    bodies.push(viz::Body { q: q_draw, alpha: 1.0, label: "", show_box: true });
    viz::draw(&canvas, &a.cam, &viz::Scene { bodies: &bodies, highlight });
}

fn get_formula() -> String {
    el("e-formula").text_content().unwrap_or_default().split("   ").next().unwrap_or("").trim_start_matches("R = ").to_string()
}

// ---------------------------------------------------------------- converter input handlers

fn set_rotation(q: Quat, src: Src) {
    with(|a| {
        a.q = q;
        a.anim = None;
        a.pending_fix = None;
        render_all(a, src);
    });
    set_anim_button(false);
    render_compose();
}

fn on_quat_input() {
    let lang = with(|a| a.lang);
    let order = with(|a| a.order);
    match read_fields("q", 4) {
        Err(e) => msg("q-msg", "err", &e),
        Ok(v) => {
            let raw = Quat::from_array([v[0], v[1], v[2], v[3]], order);
            match raw.normalized() {
                None => msg("q-msg", "err", tr(lang, "q_zero")),
                Some(q) => {
                    let n = raw.norm();
                    if (n - 1.0).abs() > 1e-3 {
                        msg("q-msg", "warn", &trf(lang, "q_norm", &[("n", &fmt_num(n, 6))]));
                    } else {
                        msg("q-msg", "", "");
                    }
                    set_rotation(q, Src::Quat);
                }
            }
        }
    }
}

fn on_euler_input() {
    let (seq, frame, unit, lang) = with(|a| (a.seq, a.frame, a.unit, a.lang));
    match read_fields("e", 3) {
        Err(e) => msg("e-msg", "err", &e),
        Ok(v) => {
            let e = Euler::new(seq, frame, [unit.to_rad(v[0]), unit.to_rad(v[1]), unit.to_rad(v[2])]);
            let q = e.to_quat();
            set_rotation(q, Src::Euler);
            // gimbal check on the typed middle angle
            let (_, res) = Euler::from_quat(q, seq, frame);
            match gimbal_message(lang, res.singularity_margin, res.gimbal_lock) {
                Some(m) => msg("e-msg", "warn", &m),
                None => msg("e-msg", "", ""),
            }
        }
    }
}

fn on_matrix_input() {
    let lang = with(|a| a.lang);
    show("m-fix", false);
    match read_fields("m", 9) {
        Err(e) => msg("m-msg", "err", &e),
        Ok(v) => {
            let m = Mat3::from_row_major(&v);
            if m.is_rotation(1e-6) {
                msg("m-msg", "", "");
                with(|a| a.pending_fix = None);
                set_rotation(Quat::from_mat(&m), Src::Matrix);
                return;
            }
            let base = trf(lang, "m_bad", &[("e", &fmt_num(m.orthogonality_error(), 6)), ("d", &fmt_num(m.det(), 6))]);
            match nearest_rotation(&m) {
                Ok(r) => {
                    with(|a| a.pending_fix = Some(r));
                    msg("m-msg", "err", &format!("{base} {}", tr(lang, "m_offer")));
                    show("m-fix", true);
                }
                Err(e) => {
                    with(|a| a.pending_fix = None);
                    let k = match e {
                        OrthoError::Reflection => "m_reflection",
                        OrthoError::Singular => "m_singular",
                        _ => "m_other",
                    };
                    msg("m-msg", "err", &format!("{base} {}", tr(lang, k)));
                }
            }
        }
    }
}

fn on_matrix_fix() {
    if let Some(r) = with(|a| a.pending_fix.take()) {
        set_rotation(Quat::from_mat(&r), Src::None);
        msg("m-msg", "ok", t("m_fixed"));
    }
}

fn on_aa_input() {
    let (unit, lang) = with(|a| (a.unit, a.lang));
    match read_fields("a", 4) {
        Err(e) => msg("a-msg", "err", &e),
        Ok(v) => {
            let axis = Vec3::new(v[0], v[1], v[2]);
            let ang = unit.to_rad(v[3]);
            let n = axis.norm();
            if n < 1e-12 {
                if ang.abs() < 1e-12 {
                    msg("a-msg", "", "");
                    set_rotation(Quat::IDENTITY, Src::AxisAngle);
                } else {
                    msg("a-msg", "err", tr(lang, "a_zero"));
                }
                return;
            }
            set_rotation(Quat::from_axis_angle(axis, ang), Src::AxisAngle);
            if (n - 1.0).abs() > 1e-3 {
                msg("a-msg", "warn", &trf(lang, "a_norm", &[("n", &fmt_num(n, 6))]));
            } else {
                msg("a-msg", "", "");
            }
        }
    }
}

fn on_rv_input() {
    match read_fields("r", 3) {
        Err(e) => msg("r-msg", "err", &e),
        Ok(v) => {
            msg("r-msg", "", "");
            set_rotation(Quat::from_rotvec(Vec3::new(v[0], v[1], v[2])), Src::RotVec);
        }
    }
}

fn on_6d_input() {
    let lang = with(|a| a.lang);
    match read_fields("s", 6) {
        Err(e) => msg("s-msg", "err", &e),
        Ok(v) => {
            let arr = [v[0], v[1], v[2], v[3], v[4], v[5]];
            match mat_from_6d(&arr) {
                None => msg("s-msg", "err", tr(lang, "s_bad")),
                Some(m) => {
                    set_rotation(Quat::from_mat(&m), Src::SixD);
                    let back = mat_to_6d(&m);
                    let off = arr.iter().zip(back.iter()).any(|(x, y)| (x - y).abs() > 1e-6);
                    if off {
                        msg("s-msg", "warn", tr(lang, "s_gs"));
                    } else {
                        msg("s-msg", "", "");
                    }
                }
            }
        }
    }
}

fn apply_settings_from_controls() {
    with(|a| {
        a.order = if get("q-order") == "xyzw" { QuatOrder::Xyzw } else { QuatOrder::Wxyz };
        a.seq = EulerSeq::parse(&get("e-seq")).unwrap_or(a.seq);
        a.frame = if get("e-frame") == "i" { Frame::Intrinsic } else { Frame::Extrinsic };
        a.unit = if get("e-unit") == "rad" { AngleUnit::Rad } else { AngleUnit::Deg };
        a.canonical = dom::checked("q-canon");
        a.anim = None;
        render_all(a, Src::None);
    });
    set_anim_button(false);
    render_compose();
}

fn sync_controls(a: &App) {
    set("q-order", if a.order == QuatOrder::Xyzw { "xyzw" } else { "wxyz" });
    set("e-seq", &a.seq.name());
    set("e-frame", if a.frame == Frame::Intrinsic { "i" } else { "e" });
    set("e-unit", unit_name(a.unit));
}

fn apply_preset(id: &str) {
    let p = match PRESETS.iter().find(|p| p.id == id) {
        Some(p) => *p,
        None => return,
    };
    with(|a| {
        a.seq = p.seq;
        a.frame = p.frame;
        a.order = p.order;
        a.unit = if matches!(id, "urdf" | "ros") { AngleUnit::Rad } else { AngleUnit::Deg };
        a.preset = Some(p.id);
        a.anim = None;
        sync_controls(a);
        render_all(a, Src::None);
    });
    render_preset_note();
    for b in dom::query_all("[data-preset]") {
        dom::set_class(&b, "on", b.get_attribute("data-preset").as_deref() == Some(id));
    }
    set_anim_button(false);
    render_compose();
}

fn render_preset_note() {
    let (preset, lang) = with(|a| (a.preset, a.lang));
    match preset.and_then(|id| PRESETS.iter().find(|p| p.id == id)) {
        Some(p) => {
            text("preset-note", if lang == Lang::Ko { p.note_ko } else { p.note_en });
            show("preset-note", true);
        }
        None => show("preset-note", false),
    }
}

fn on_snippet(kind: &str) {
    let s = with(|a| {
        let q = display_quat(a, a.q);
        match kind {
            "urdf" => snippet::urdf_origin(q, DIGITS),
            "py" => snippet::python_list(&q.to_array(a.order), DIGITS),
            "np" => snippet::numpy_array(&q.to_array(a.order), DIGITS),
            "npmat" => snippet::numpy_matrix(&q.to_mat(), DIGITS),
            _ => snippet::ros_quaternion_yaml(q, DIGITS),
        }
    });
    let label = match kind {
        "py" | "np" => {
            let o = with(|a| a.order);
            format!(" ({})", if o == QuatOrder::Wxyz { "wxyz" } else { "xyzw" })
        }
        _ => String::new(),
    };
    set("snip-out", &s);
    dom::copy(&s);
    text("snip-msg", &format!("{}{}", t("copied"), label));
}

// ---------------------------------------------------------------- animation

fn set_anim_button(running: bool) {
    text("e-anim", t(if running { "stop" } else { "animate" }));
    dom::set_class(&el("e-anim"), "on", running);
}

fn toggle_anim() {
    let running = with(|a| a.anim.is_some());
    if running {
        with(|a| {
            a.anim = None;
            draw_main(a);
        });
        set_anim_button(false);
        return;
    }
    let now = js_sys::Date::now();
    with(|a| a.anim = Some(Anim { start_ms: now, progress: 0.0 }));
    set_anim_button(true);
    // requestAnimationFrame loop
    let cb: FrameCb = Rc::new(RefCell::new(None));
    let cb2 = cb.clone();
    *cb2.borrow_mut() = Some(Closure::<dyn FnMut()>::new(move || {
        let keep = with(|a| {
            let anim = match &mut a.anim {
                Some(x) => x,
                None => return false,
            };
            // per step: 1.1 s motion + 0.5 s hold
            let tsec = (js_sys::Date::now() - anim.start_ms) / 1000.0;
            let (step_len, move_len) = (1.6, 1.1);
            let step = (tsec / step_len).floor();
            let frac = ((tsec - step * step_len) / move_len).clamp(0.0, 1.0);
            let eased = frac * frac * (3.0 - 2.0 * frac);
            anim.progress = (step + eased).min(3.0);
            let done = tsec > 3.0 * step_len + 1.5;
            draw_main(a);
            if done {
                a.anim = None;
                draw_main(a);
            }
            !done
        });
        if keep {
            request_frame(cb.borrow().as_ref().unwrap());
        } else {
            set_anim_button(false);
            cb.borrow_mut().take();
        }
    }));
    request_frame(cb2.borrow().as_ref().unwrap());
}

type FrameCb = Rc<RefCell<Option<Closure<dyn FnMut()>>>>;

fn request_frame(c: &Closure<dyn FnMut()>) {
    let _ = dom::win().request_animation_frame(c.as_ref().unchecked_ref());
}

// ---------------------------------------------------------------- canvas orbit

fn attach_orbit(id: &'static str, second: bool) {
    let canvas = el(id);
    let c2 = canvas.clone();
    on(canvas.as_ref(), "pointerdown", move |e| {
        let e: PointerEvent = e.unchecked_into();
        let _ = c2.set_pointer_capture(e.pointer_id());
        with(|a| a.drag = Some((e.client_x() as f64, e.client_y() as f64)));
    });
    on(canvas.as_ref(), "pointermove", move |e| {
        let e: PointerEvent = e.unchecked_into();
        let moved = with(|a| {
            let (x0, y0) = a.drag?;
            let (x, y) = (e.client_x() as f64, e.client_y() as f64);
            a.drag = Some((x, y));
            let cam = if second { &mut a.cam2 } else { &mut a.cam };
            cam.az -= (x - x0) * 0.01;
            cam.el = (cam.el + (y - y0) * 0.01).clamp(-1.5, 1.5);
            Some(())
        });
        if moved.is_some() {
            if second {
                render_compose();
            } else {
                with(|a| draw_main(a));
            }
        }
    });
    for ev in ["pointerup", "pointercancel", "lostpointercapture"] {
        on(canvas.as_ref(), ev, move |_| with(|a| a.drag = None));
    }
}

// ---------------------------------------------------------------- compose & compare

fn compose_repr(a: &App) -> Repr {
    match get("c-fmt").as_str() {
        "euler" => Repr::Euler { seq: a.seq, frame: a.frame, unit: a.unit },
        "rotvec" => Repr::RotVec,
        _ => Repr::Quat(a.order),
    }
}

fn parse_rot(s: &str, r: &Repr) -> Result<Quat, String> {
    match parse_numbers(s) {
        Ok(Some(v)) => r.to_quat(&v),
        Ok(None) => Err(format!("expected {} numbers", r.len())),
        Err(e) => Err(e),
    }
}

fn fmt_list(v: &[f64]) -> String {
    v.iter().map(|x| fmt_num(*x, 5)).collect::<Vec<_>>().join(", ")
}

fn render_compose() {
    reexpress_compose_inputs();
    let (repr, order, seq, frame, unit, lang, cam2) = with(|a| (compose_repr(a), a.order, a.seq, a.frame, a.unit, a.lang, a.cam2));
    let ta = get("c-t").parse::<f64>().unwrap_or(0.5);
    text("c-tval", &fmt_num(ta, 2));
    let qa = parse_rot(&get("ca"), &repr);
    let qb = parse_rot(&get("cb"), &repr);
    let body = el("c-body");
    body.set_inner_html("");
    let (a, b) = match (qa, qb) {
        (Ok(a), Ok(b)) => {
            msg("c-msg", "", "");
            (a, b)
        }
        (ea, eb) => {
            let mut m = String::new();
            if let Err(e) = ea {
                m.push_str(&format!("A: {e}. "));
            }
            if let Err(e) = eb {
                m.push_str(&format!("B: {e}."));
            }
            msg("c-msg", "err", &m);
            return;
        }
    };
    let euler_r = Repr::Euler { seq, frame, unit };
    let qname = if order == QuatOrder::Wxyz { "wxyz" } else { "xyzw" };
    let rows: Vec<(String, Quat)> = vec![
        ("A · B".into(), a * b),
        ("B · A".into(), b * a),
        ("A⁻¹".into(), a.inverse()),
        ("A⁻¹ · B".into(), a.inverse() * b),
        (format!("slerp(A, B, {})", fmt_num(ta, 2)), a.slerp(b, ta)),
    ];
    let d = dom::doc();
    let add_row = |cells: &[String]| {
        let tr_ = d.create_element("tr").unwrap();
        for c in cells {
            let td = d.create_element("td").unwrap();
            td.set_text_content(Some(c));
            let _ = tr_.append_child(&td);
        }
        let _ = body.append_child(&tr_);
    };
    add_row(&["".into(), format!("quat ({qname})"), format!("Euler {}", euler_conv_name(seq, frame, unit))]);
    for (name, q) in rows {
        let q = q.normalized().unwrap_or(Quat::IDENTITY);
        add_row(&[name, fmt_list(&q.canonical().to_array(order)), fmt_list(&euler_r.from_quat(q))]);
    }
    let ang = a.angle_to(b).to_degrees();
    add_row(&[tr(lang, "geodesic").into(), format!("{} °", fmt_num(ang, 4)), String::new()]);

    if let Ok(canvas) = el("c-viz").dyn_into::<HtmlCanvasElement>() {
        let bodies = [
            viz::Body { q: a, alpha: 0.35, label: "A", show_box: false },
            viz::Body { q: b, alpha: 0.35, label: "B", show_box: false },
            viz::Body { q: a.slerp(b, ta), alpha: 1.0, label: "", show_box: true },
        ];
        viz::draw(&canvas, &cam2, &viz::Scene { bodies: &bodies, highlight: None });
    }
}

fn fill_compose_from_converter(id: &str) {
    let s = with(|a| fmt_list_full(&compose_repr(a).from_quat(display_quat(a, a.q))));
    set(id, &s);
    render_compose();
}

fn fmt_list_full(v: &[f64]) -> String {
    v.iter().map(|x| fmt_num(*x, DIGITS)).collect::<Vec<_>>().join(", ")
}

// ---------------------------------------------------------------- batch

const BATCH_KINDS: [(&str, &str); 7] = [
    ("wxyz", "r_wxyz"),
    ("xyzw", "r_xyzw"),
    ("euler", "r_euler"),
    ("matrix", "r_matrix"),
    ("aa", "r_aa"),
    ("rotvec", "r_rotvec"),
    ("6d", "r_6d"),
];

fn batch_repr(a: &App, kind: &str) -> Repr {
    match kind {
        "wxyz" => Repr::Quat(QuatOrder::Wxyz),
        "xyzw" => Repr::Quat(QuatOrder::Xyzw),
        "euler" => Repr::Euler { seq: a.seq, frame: a.frame, unit: a.unit },
        "matrix" => Repr::Matrix,
        "aa" => Repr::AxisAngle(a.unit),
        "rotvec" => Repr::RotVec,
        _ => Repr::SixD,
    }
}

fn run_batch() {
    let (input, output, lang) = with(|a| (batch_repr(a, &get("b-from")), batch_repr(a, &get("b-to")), a.lang));
    let lines = convert_batch(&get("b-in"), &input, &output);
    let csv = batch_to_csv(&lines, &output, 9);
    set("b-out", &csv);
    let errs: Vec<_> = lines.iter().filter_map(|l| l.result.as_ref().err().map(|e| (l.line, e.clone()))).collect();
    let ok = lines.len() - errs.len();
    text("b-sum", &trf(lang, "b_sum", &[("ok", &ok.to_string()), ("err", &errs.len().to_string())]));
    let ul = el("b-err");
    ul.set_inner_html("");
    let d = dom::doc();
    for (n, e) in errs.iter().take(200) {
        let li = d.create_element("li").unwrap();
        li.set_text_content(Some(&format!("{} {}: {}", tr(lang, "b_line"), n, e)));
        let _ = ul.append_child(&li);
    }
    let has = !lines.is_empty();
    with(|a| a.batch_csv = if has { csv } else { String::new() });
    let _ = el("b-dl").dyn_into::<web_sys::HtmlButtonElement>().map(|b| b.set_disabled(!has));
}

fn download_csv() {
    let csv = with(|a| a.batch_csv.clone());
    if csv.is_empty() {
        return;
    }
    let parts = js_sys::Array::of1(&JsValue::from_str(&csv));
    let opts = web_sys::BlobPropertyBag::new();
    opts.set_type("text/csv");
    let blob = match web_sys::Blob::new_with_str_sequence_and_options(&parts, &opts) {
        Ok(b) => b,
        Err(_) => return,
    };
    let url = match web_sys::Url::create_object_url_with_blob(&blob) {
        Ok(u) => u,
        Err(_) => return,
    };
    if let Ok(a) = dom::doc().create_element("a").and_then(|e| e.dyn_into::<web_sys::HtmlAnchorElement>().map_err(|e| e.into())) {
        a.set_href(&url);
        a.set_download("rot-lab-batch.csv");
        let _ = dom::doc().body().map(|b| b.append_child(&a));
        a.click();
        a.remove();
    }
    let _ = web_sys::Url::revoke_object_url(&url);
}

fn fill_batch_selects() {
    let lang = with(|a| a.lang);
    for (id, default) in [("b-from", "wxyz"), ("b-to", "euler")] {
        let cur = get(id);
        let sel = el(id);
        sel.set_inner_html("");
        for (v, key) in BATCH_KINDS {
            let o = dom::doc().create_element("option").unwrap();
            let _ = o.set_attribute("value", v);
            o.set_text_content(Some(tr(lang, key)));
            let _ = sel.append_child(&o);
        }
        set(id, if cur.is_empty() { default } else { &cur });
    }
}

// ---------------------------------------------------------------- language

fn apply_lang() {
    let lang = with(|a| a.lang);
    for e in dom::query_all("[data-i18n]") {
        if let Some(k) = e.get_attribute("data-i18n") {
            let s = tr(lang, &k);
            if !s.is_empty() {
                e.set_text_content(Some(s));
            }
        }
    }
    for e in dom::query_all("[data-lang]") {
        let want = if lang == Lang::Ko { "ko" } else { "en" };
        dom::show_el(&e, e.get_attribute("data-lang").as_deref() == Some(want));
    }
    let _ = dom::doc().document_element().map(|h| h.set_attribute("lang", if lang == Lang::Ko { "ko" } else { "en" }));
    text("lang", if lang == Lang::Ko { "English" } else { "한국어" });
    fill_batch_selects();
    render_preset_note();
    with(|a| {
        set_anim_button_inner(a);
        render_all(a, Src::None);
    });
    render_compose();
}

fn set_anim_button_inner(a: &App) {
    text("e-anim", tr(a.lang, if a.anim.is_some() { "stop" } else { "animate" }));
}

fn detect_lang() -> Lang {
    if let Some(s) = dom::storage_get("rot-lab.lang") {
        return if s == "ko" { Lang::Ko } else { Lang::En };
    }
    let nav = dom::win().navigator().language().unwrap_or_default();
    if nav.to_lowercase().starts_with("ko") {
        Lang::Ko
    } else {
        Lang::En
    }
}

// ---------------------------------------------------------------- tabs

fn show_tab(name: &str) {
    for b in dom::query_all("[data-tab]") {
        let on_ = b.get_attribute("data-tab").as_deref() == Some(name);
        dom::set_class(&b, "active", on_);
        let _ = b.set_attribute("aria-selected", if on_ { "true" } else { "false" });
    }
    for id in ["conv", "comp", "batch", "help"] {
        show(&format!("tab-{id}"), id == name);
    }
    if name == "comp" {
        render_compose();
    } else if name == "conv" {
        with(|a| draw_main(a));
    }
}

// ---------------------------------------------------------------- startup

fn load_fragment() {
    let hash = dom::win().location().hash().unwrap_or_default();
    if hash.len() > 1 {
        let s = fragment::decode(&hash);
        with(|a| {
            a.q = s.q;
            a.seq = s.seq;
            a.frame = s.frame;
            a.unit = s.unit;
            a.order = s.order;
        });
    }
}

fn main() {
    console_panic_hook();
    // Euler sequence options
    let sel = el("e-seq");
    for s in EulerSeq::ALL {
        let o = dom::doc().create_element("option").unwrap();
        let name = s.name();
        let _ = o.set_attribute("value", &name);
        let kind = if s.is_proper() { "proper" } else { "Tait-Bryan" };
        o.set_text_content(Some(&format!("{name} ({kind})")));
        let _ = sel.append_child(&o);
    }

    with(|a| a.lang = detect_lang());
    load_fragment();
    with(|a| sync_controls(a));

    // converter inputs
    for (prefix, n, handler) in [
        ("q", 4, on_quat_input as fn()),
        ("e", 3, on_euler_input as fn()),
        ("m", 9, on_matrix_input as fn()),
        ("a", 4, on_aa_input as fn()),
        ("r", 3, on_rv_input as fn()),
        ("s", 6, on_6d_input as fn()),
    ] {
        for i in 0..n {
            on_id(&format!("{prefix}{i}"), "input", move |_| handler());
        }
    }
    // re-format the edited group when leaving a field, if it is valid
    for (prefix, n) in [("q", 4), ("e", 3), ("a", 4), ("r", 3), ("s", 6)] {
        for i in 0..n {
            on_id(&format!("{prefix}{i}"), "change", move |_| {
                if read_fields(prefix, n).is_ok() {
                    with(|a| match prefix {
                        "q" => render_quat(a),
                        "a" => render_aa(a),
                        "r" => render_rv(a),
                        _ => {}
                    });
                }
            });
        }
    }
    for id in ["q-order", "e-seq", "e-frame", "e-unit", "q-canon"] {
        on_id(id, "change", move |_| {
            with(|a| a.preset = None);
            for b in dom::query_all("[data-preset]") {
                dom::set_class(&b, "on", false);
            }
            render_preset_note();
            apply_settings_from_controls();
        });
    }
    on_id("m-fix", "click", |_| on_matrix_fix());
    on_id("e-anim", "click", |_| toggle_anim());
    on_id("viz-reset", "click", |_| {
        with(|a| {
            a.cam = viz::Cam::default();
            draw_main(a);
        })
    });
    for b in dom::query_all("[data-preset]") {
        let id = b.get_attribute("data-preset").unwrap_or_default();
        on(b.as_ref(), "click", move |_| apply_preset(&id));
    }
    for b in dom::query_all("[data-snip]") {
        let k = b.get_attribute("data-snip").unwrap_or_default();
        on(b.as_ref(), "click", move |_| on_snippet(&k));
    }
    for b in dom::query_all("[data-tab]") {
        let k = b.get_attribute("data-tab").unwrap_or_default();
        on(b.as_ref(), "click", move |_| show_tab(&k));
    }
    on_id("lang", "click", |_| {
        let l = with(|a| {
            a.lang = if a.lang == Lang::Ko { Lang::En } else { Lang::Ko };
            a.lang
        });
        dom::storage_set("rot-lab.lang", if l == Lang::Ko { "ko" } else { "en" });
        apply_lang();
    });
    on_id("share", "click", |_| {
        with(|a| update_fragment(a));
        let href = dom::win().location().href().unwrap_or_default();
        dom::copy(&href);
        text("snip-msg", t("share_done"));
        let b = el("share");
        b.set_text_content(Some("✓"));
        let restore = Closure::<dyn FnMut()>::new(move || b.set_text_content(Some(t("share"))));
        let _ = dom::win().set_timeout_with_callback_and_timeout_and_arguments_0(restore.as_ref().unchecked_ref(), 1500);
        restore.forget();
    });
    on(dom::win().as_ref(), "hashchange", |_| {
        let cur = with(|a| {
            format!("#{}", fragment::encode(&fragment::State { q: a.q, seq: a.seq, frame: a.frame, unit: a.unit, order: a.order }))
        });
        if dom::win().location().hash().unwrap_or_default() != cur {
            load_fragment();
            with(|a| {
                sync_controls(a);
                render_all(a, Src::None);
            });
            render_compose();
        }
    });
    on(dom::win().as_ref(), "resize", |_| {
        with(|a| draw_main(a));
        render_compose();
    });
    if let Ok(Some(mq)) = dom::win().match_media("(prefers-color-scheme: dark)") {
        on(mq.as_ref(), "change", |_| {
            with(|a| draw_main(a));
            render_compose();
        });
    }
    attach_orbit("viz", false);
    attach_orbit("c-viz", true);

    // compose
    for id in ["ca", "cb", "c-t"] {
        on_id(id, "input", |_| render_compose());
    }
    on_id("c-fmt", "change", |_| render_compose());
    on_id("ca-from", "click", |_| fill_compose_from_converter("ca"));
    on_id("cb-from", "click", |_| fill_compose_from_converter("cb"));

    // batch
    on_id("b-run", "click", |_| run_batch());
    on_id("b-dl", "click", |_| download_csv());
    set("b-in", "# qw, qx, qy, qz\n1, 0, 0, 0\n0.7071068, 0, 0, 0.7071068\n0.9238795 0.3826834 0 0\n0.5, 0.5, 0.5, 0.5\n");

    // default compose inputs: A = 90° about Z, B = 90° about X (shown in the current format)
    with(|a| {
        let r = compose_repr(a);
        a.compose_fmt = r;
        set("ca", &fmt_list_full(&r.from_quat(Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2))));
        set("cb", &fmt_list_full(&r.from_quat(Quat::from_axis_angle(Vec3::X, std::f64::consts::FRAC_PI_2))));
    });

    show("loading", false);
    apply_lang();
}

/// When the compose format (or the converter convention it uses) changes, re-express A and B
/// in the new format so the same rotations are kept.
fn reexpress_compose_inputs() {
    with(|a| {
        let old = a.compose_fmt;
        let new = compose_repr(a);
        if old == new {
            return;
        }
        for id in ["ca", "cb"] {
            if let Ok(q) = parse_rot(&get(id), &old) {
                set(id, &fmt_list_full(&new.from_quat(q)));
            }
        }
        a.compose_fmt = new;
    });
}

fn console_panic_hook() {
    std::panic::set_hook(Box::new(|info| {
        web_sys::console::error_1(&JsValue::from_str(&info.to_string()));
    }));
}
