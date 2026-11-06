//! Canvas 2D renderer: world axes, rotated body frames and a small box.

use crate::dom;
use rot_core::{Quat, Vec3};
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement};

#[derive(Clone, Copy, Debug)]
pub struct Cam {
    /// Azimuth of the camera around world Z (radians).
    pub az: f64,
    /// Elevation above the XY plane (radians).
    pub el: f64,
}

impl Default for Cam {
    fn default() -> Self {
        Cam { az: 35f64.to_radians(), el: 25f64.to_radians() }
    }
}

pub struct Body {
    pub q: Quat,
    /// 0..1 opacity
    pub alpha: f64,
    pub label: &'static str,
    pub show_box: bool,
}

pub struct Scene<'a> {
    pub bodies: &'a [Body],
    /// Rotation axis to highlight (unit vector in world coordinates) and its label.
    pub highlight: Option<(Vec3, String)>,
}

struct Proj {
    r: Vec3,
    u: Vec3,
    d: Vec3,
    cx: f64,
    cy: f64,
    s: f64,
}

impl Proj {
    fn new(cam: &Cam, w: f64, h: f64) -> Proj {
        let (se, ce) = cam.el.sin_cos();
        let (sa, ca) = cam.az.sin_cos();
        // camera sits along d looking at the origin; (r, u, d) is right-handed, so no mirroring
        let d = Vec3::new(ce * ca, ce * sa, se);
        let f = -d;
        let r = f.cross(Vec3::Z).normalized().unwrap_or(Vec3::Y);
        let u = r.cross(f);
        Proj { r, u, d, cx: w / 2.0, cy: h / 2.0, s: w.min(h) * 0.30 }
    }
    /// Screen x, y (canvas y is down) and depth (larger = closer to the viewer).
    fn p(&self, v: Vec3) -> (f64, f64, f64) {
        let depth = v.dot(self.d);
        let k = 7.0 / (7.0 - depth); // mild perspective
        (self.cx + self.s * k * v.dot(self.r), self.cy - self.s * k * v.dot(self.u), depth)
    }
}

const AXIS_LIGHT: [&str; 3] = ["#e5484d", "#30a46c", "#3e63dd"];
const AXIS_DARK: [&str; 3] = ["#ff6369", "#3dd68c", "#7b9bff"];

pub fn context(canvas: &HtmlCanvasElement) -> Option<CanvasRenderingContext2d> {
    canvas.get_context("2d").ok().flatten().and_then(|c| c.dyn_into::<CanvasRenderingContext2d>().ok())
}

pub fn draw(canvas: &HtmlCanvasElement, cam: &Cam, scene: &Scene) {
    let dpr = dom::win().device_pixel_ratio().max(1.0);
    let cw = canvas.client_width().max(50) as f64;
    let ch = canvas.client_height().max(50) as f64;
    let (pw, ph) = ((cw * dpr).round() as u32, (ch * dpr).round() as u32);
    if canvas.width() != pw || canvas.height() != ph {
        canvas.set_width(pw);
        canvas.set_height(ph);
    }
    let ctx = match context(canvas) {
        Some(c) => c,
        None => return,
    };
    let _ = ctx.set_transform(dpr, 0.0, 0.0, dpr, 0.0, 0.0);
    ctx.clear_rect(0.0, 0.0, cw, ch);
    let dark = dom::dark_mode();
    let colors = if dark { AXIS_DARK } else { AXIS_LIGHT };
    let fg = if dark { "#e8e9eb" } else { "#1c2024" };
    let faint = if dark { "#3a3d44" } else { "#d4d7dd" };
    let pr = Proj::new(cam, cw, ch);
    let o = pr.p(Vec3::default());
    ctx.set_line_cap("round");
    ctx.set_font("12px system-ui, sans-serif");

    // ground circle in the world XY plane
    ctx.set_stroke_style_str(faint);
    ctx.set_line_width(1.0);
    ctx.begin_path();
    for i in 0..=64 {
        let t = i as f64 / 64.0 * std::f64::consts::TAU;
        let (x, y, _) = pr.p(Vec3::new(1.2 * t.cos(), 1.2 * t.sin(), 0.0));
        if i == 0 {
            ctx.move_to(x, y);
        } else {
            ctx.line_to(x, y);
        }
    }
    ctx.stroke();

    // world axes: thin, dashed
    let dash = js_sys::Array::of2(&5.0.into(), &4.0.into());
    let _ = ctx.set_line_dash(&dash);
    for (i, name) in ["X", "Y", "Z"].iter().enumerate() {
        let e = unit(i).scale(1.45);
        let (x, y, _) = pr.p(e);
        ctx.set_global_alpha(0.75);
        ctx.set_stroke_style_str(colors[i]);
        ctx.set_line_width(1.2);
        line(&ctx, o.0, o.1, x, y);
        ctx.set_fill_style_str(colors[i]);
        let _ = ctx.fill_text(name, x + 4.0, y - 4.0);
    }
    let _ = ctx.set_line_dash(&js_sys::Array::new());
    ctx.set_global_alpha(1.0);

    if let Some((axis, label)) = &scene.highlight {
        let amber = if dark { "#ffc53d" } else { "#d98e04" };
        let a = pr.p(axis.scale(-1.6));
        let b = pr.p(axis.scale(1.6));
        let _ = ctx.set_line_dash(&js_sys::Array::of2(&2.0.into(), &5.0.into()));
        ctx.set_stroke_style_str(amber);
        ctx.set_line_width(3.0);
        line(&ctx, a.0, a.1, b.0, b.1);
        let _ = ctx.set_line_dash(&js_sys::Array::new());
        ctx.set_fill_style_str(amber);
        let _ = ctx.fill_text(label, b.0 + 6.0, b.1 + 4.0);
    }

    for body in scene.bodies {
        ctx.set_global_alpha(body.alpha);
        if body.show_box {
            draw_box(&ctx, &pr, body.q, fg, dark);
        }
        // body axes sorted back to front
        let mut axes: Vec<(usize, (f64, f64, f64))> = (0..3).map(|i| (i, pr.p(body.q.rotate(unit(i))))).collect();
        axes.sort_by(|a, b| a.1 .2.partial_cmp(&b.1 .2).unwrap_or(std::cmp::Ordering::Equal));
        for (i, (x, y, _)) in axes {
            ctx.set_stroke_style_str(colors[i]);
            ctx.set_line_width(4.0);
            line(&ctx, o.0, o.1, x, y);
            ctx.set_fill_style_str(colors[i]);
            ctx.begin_path();
            let _ = ctx.arc(x, y, 3.5, 0.0, std::f64::consts::TAU);
            ctx.fill();
            let name = ["x", "y", "z"][i];
            let _ = ctx.fill_text(&format!("{name}{}", body.label), x + 6.0, y + 4.0);
        }
    }
    ctx.set_global_alpha(1.0);
}

fn unit(i: usize) -> Vec3 {
    match i {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

fn line(ctx: &CanvasRenderingContext2d, x0: f64, y0: f64, x1: f64, y1: f64) {
    ctx.begin_path();
    ctx.move_to(x0, y0);
    ctx.line_to(x1, y1);
    ctx.stroke();
}

fn draw_box(ctx: &CanvasRenderingContext2d, pr: &Proj, q: Quat, fg: &str, dark: bool) {
    let h = [0.45, 0.28, 0.14];
    let corner = |i: usize| {
        let s = |b: usize| if i & b != 0 { 1.0 } else { -1.0 };
        q.rotate(Vec3::new(s(1) * h[0], s(2) * h[1], s(4) * h[2]))
    };
    let pts: Vec<Vec3> = (0..8).map(corner).collect();
    // faces as corner index quads
    let faces = [[0, 2, 6, 4], [1, 3, 7, 5], [0, 1, 5, 4], [2, 3, 7, 6], [0, 1, 3, 2], [4, 5, 7, 6]];
    let mut order: Vec<(f64, [usize; 4])> =
        faces.iter().map(|f| (f.iter().map(|&i| pr.p(pts[i]).2).sum::<f64>() / 4.0, *f)).collect();
    order.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let fill = if dark { "rgba(200,205,215,0.10)" } else { "rgba(60,70,90,0.08)" };
    let base_alpha = ctx.global_alpha();
    for (_, f) in order {
        ctx.begin_path();
        for (k, &i) in f.iter().enumerate() {
            let (x, y, _) = pr.p(pts[i]);
            if k == 0 {
                ctx.move_to(x, y);
            } else {
                ctx.line_to(x, y);
            }
        }
        ctx.close_path();
        ctx.set_fill_style_str(fill);
        ctx.fill();
        ctx.set_global_alpha(base_alpha * 0.45);
        ctx.set_stroke_style_str(fg);
        ctx.set_line_width(1.0);
        ctx.stroke();
        ctx.set_global_alpha(base_alpha);
    }
}
