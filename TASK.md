# Task: 3D rotation converter & visualizer for robotics (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code or assets.

## Product
A single-page web app (GitHub Pages, sub-path `/rot-lab/`) for robotics / ML engineers who keep
getting rotation conventions wrong (ROS, URDF, MuJoCo, Isaac, VLA action spaces).

1. **Converter** — edit any one representation and all others update live:
   - Quaternion with an explicit order toggle (wxyz / xyzw), auto-normalize with a warning when the
     input norm is far from 1, canonical sign option (w >= 0).
   - Euler angles: all 12 sequences (XYZ, XZY, ..., ZYZ, ...), intrinsic vs extrinsic toggle, deg/rad.
     Gimbal-lock detection with a visible warning.
   - 3x3 rotation matrix (editable; if not orthonormal show the error and offer the nearest rotation,
     e.g. via iterative polar decomposition), axis-angle, rotation vector (so(3) log),
     and the continuous **6D representation** (first two columns of R, Gram-Schmidt back to R).
   - Presets: "URDF rpy" (fixed-axis roll/pitch/yaw = extrinsic X-Y-Z), "ROS tf quaternion (xyzw)",
     "MuJoCo quat (wxyz)", "aerospace ZYX yaw-pitch-roll". Short Korean/English note per preset.
   - Copy buttons that emit snippets: URDF `<origin rpy="..."/>`, Python list, numpy array,
     ROS `geometry_msgs/Quaternion` YAML.
2. **Visualizer** — canvas 2D (no WebGL needed) drawing of world axes and the rotated body frame
   (RGB = XYZ) plus a small box, draggable to orbit the camera. Optional step-by-step animation of the
   chosen Euler sequence (rotate about axis 1, then 2, then 3) so intrinsic vs extrinsic is visible.
3. **Compose & compare** — enter rotations A and B: show A*B, B*A, A^-1, relative rotation A^-1 B, the
   geodesic angle between A and B (deg), and SLERP at t (slider), visualized.
4. **Batch** — paste lines of numbers (CSV/whitespace), choose input & output representation,
   convert all, download CSV (Blob). Parse errors reported per line.
5. Korean + English UI toggle (default follows browser language). Mobile friendly. Dark/light follows
   system. State shareable via URL fragment (#q=...), never sent to a server.

## Core crate (pure Rust, no web deps, unit tested) — `crates/rot_core`
- Own small math (Quat, Mat3, Vec3); external math crates allowed only if the bundle stays small.
- Conversions for all representations above, all 12 Euler sequences intrinsic/extrinsic,
  composition, inverse, slerp, geodesic angle, 6D <-> matrix, orthonormalization, batch line parsing.
- Tests: round trips (quat→euler→quat for all 12×2 conventions on many pseudo-random rotations, up to
  sign), known values (90° about each axis), intrinsic XYZ == extrinsic ZYX (reversed angles)
  equivalence, gimbal-lock input still reproduces the matrix, slerp endpoints and halfway angle,
  6D round trip, xyzw/wxyz ordering, a URDF rpy known example.
- `crates/rot_web`: UI. Prefer plain web-sys (or a small framework) — keep the bundle small.

## Repo hygiene
- README (Korean first, then English): what it is, conventions explained (intrinsic vs extrinsic,
  quaternion order, Hamilton convention), how to use each panel, local build
  (`cargo test -p rot_core`, `trunk build --release`), MIT license.
- LICENSE: MIT, copyright "rot-lab contributors". `.gitignore` for target/ dist/.
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  Tip: in `Trunk.toml` set `filehash = false` and
  `pattern_script = "<script type=\"module\" src=\"{base}boot.js\"></script>"` with an external
  `web/boot.js` (copied via `<link data-trunk rel="copy-file">`) that imports `./rot_web.js` and calls
  init, so no inline script is needed.
- `Trunk.toml`: `public_url = "/rot-lab/"`.
- `.github/workflows/pages.yml`: on push to main + workflow_dispatch; checkout, rust stable + wasm32
  target, install trunk 0.22.1 binary release, `sh tools/privacy_check.sh`,
  `cargo test -p rot_core --release`, `trunk build --release`, configure-pages@v5,
  upload-pages-artifact@v3 (dist), deploy job with actions/deploy-pages@v4
  (permissions pages: write, id-token: write).
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test -p rot_core` green; `trunk build --release` succeeds; `sh tools/privacy_check.sh` clean.
- Commit everything with clear commits. Do NOT create new GitHub repos or change repo settings.
