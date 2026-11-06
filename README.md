# rot-lab

로보틱스 / ML 엔지니어를 위한 3D 회전 변환기 & 시각화 도구. Rust → WebAssembly로 만든 정적 웹 앱이며,
모든 계산은 브라우저 안에서만 이뤄집니다 (서버, 분석 도구, 외부 CDN, 런타임 네트워크 요청 없음).

[English below](#english)

## 한국어

### 무엇인가요?

ROS, URDF, MuJoCo, Isaac, VLA action space 등은 회전 규약이 제각각이라 실수하기 쉽습니다.
rot-lab은 한 표현을 고치면 나머지 표현이 즉시 갱신되는 변환기입니다.

- **쿼터니언**: wxyz / xyzw 순서 전환, 노름이 1에서 멀면 자동 정규화 + 경고, 부호 정규화 옵션 (w ≥ 0)
- **오일러 각**: 12가지 축 순서 전부 (XYZ, XZY, …, ZYZ, …), intrinsic / extrinsic, deg / rad, 짐벌 락 경고
- **회전 행렬 3×3**: 직접 편집 가능. 정규직교가 아니면 오차를 보여주고 극분해(polar decomposition)로 가장 가까운 회전을 제안
- **축-각**, **회전 벡터** (so(3) log), **6D 표현** (R의 처음 두 열, Gram-Schmidt로 복원)
- **프리셋**: URDF rpy, ROS tf 쿼터니언 (xyzw), MuJoCo quat (wxyz), 항공 ZYX yaw-pitch-roll
- **코드로 복사**: URDF `<origin rpy="..."/>`, Python 리스트, numpy 배열/행렬, ROS `geometry_msgs/Quaternion` YAML

### 규약

| 항목 | rot-lab의 선택 |
|---|---|
| 회전의 의미 | 능동(active) 회전, 열벡터 `v' = R v`, 오른손 좌표계 |
| 쿼터니언 | **Hamilton** (`ij = k`), `q = (w, x, y, z)`, `w = cos(θ/2)`. `q ⊗ p` = p를 먼저, 그다음 q 적용 = `R(q) R(p)` |
| wxyz vs xyzw | MuJoCo, Eigen 생성자, Isaac Sim, scipy `scalar_first=True` → **wxyz**. ROS / tf2 / geometry_msgs, scipy 기본값, PyBullet → **xyzw** |
| Intrinsic A-B-C | 몸체 A축 → *새* 몸체 B축 → C축 순서로 회전. `R = R_A(a) R_B(b) R_C(c)` |
| Extrinsic A-B-C | 고정된 월드 A축 → B축 → C축 순서로 회전. `R = R_C(c) R_B(b) R_A(a)` |
| 동치 관계 | extrinsic `X-Y-Z (r, p, y)` = intrinsic `Z-Y-X (y, p, r)` |
| URDF `rpy` | 라디안 단위 extrinsic X-Y-Z (고정축 roll, pitch, yaw) |
| 짐벌 락 | Tait-Bryan은 가운데 각 ±90°, proper Euler는 0° / 180°. 이때 셋째 각을 0으로 두며 행렬은 정확히 재현됨 |

같은 네 숫자라도 wxyz와 xyzw로 읽으면 완전히 다른 회전입니다. 예: ROS의 단위 쿼터니언 `[0, 0, 0, 1]`을
wxyz로 읽으면 Z축 180° 회전이 됩니다.

### 패널 사용법

1. **변환기**: 아무 표현이나 편집하면 나머지가 갱신됩니다. 순서/규약 드롭다운은 *같은 회전*을 다른 형식으로 다시 보여줍니다.
   행렬이 회전이 아니면 "가장 가까운 회전 행렬 사용" 버튼이 나타납니다.
2. **시각화**: 얇은 점선 = 월드 좌표계, 굵은 선 = 회전된 몸체 좌표계 (RGB = XYZ), 월드 Z가 위쪽. 드래그로 시점 회전.
   "단계별 애니메이션"은 선택한 오일러 순서를 1→2→3 축 순으로 보여주며, intrinsic이면 움직이는 몸체 축,
   extrinsic이면 고정된 월드 축을 강조합니다.
3. **합성 & 비교**: 회전 A, B를 입력하면 A·B, B·A, A⁻¹, A⁻¹B, 두 회전 사이의 측지 각(도), 슬라이더 t에서의 SLERP를 보여줍니다.
4. **일괄 변환**: 한 줄에 회전 하나씩 붙여넣고 (쉼표/공백 구분) 입력/출력 표현을 골라 변환 후 CSV로 다운로드.
   줄마다 파싱 오류를 표시합니다. 오일러/축-각은 변환기의 현재 규약을 사용합니다.

언어는 브라우저 설정을 따르며 (한국어 / English 전환 가능), 다크/라이트 모드는 시스템 설정을 따릅니다.
"공유 링크 복사"는 상태를 URL fragment(`#q=...`)에 담습니다. fragment는 서버로 전송되지 않습니다.

### 로컬 빌드

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk --version 0.22.1 --locked   # 또는 릴리스 바이너리 사용

cargo test -p rot_core          # 핵심 수학 단위 테스트
trunk build --release           # dist/ 에 정적 사이트 생성 (public_url = /rot-lab/)
trunk serve                     # 개발 서버: http://localhost:8080/rot-lab/
sh tools/privacy_check.sh       # 공개 전 개인정보 검사 ("clean" 이어야 함)
```

### 구조

- `crates/rot_core` — 의존성 없는 순수 Rust 회전 수학 (Quat, Mat3, Vec3, 12×2 오일러 규약, 6D, 극분해, SLERP, 배치 파싱, 코드 스니펫). 단위 테스트 포함.
- `crates/rot_web` — `web-sys` 기반 UI와 캔버스 2D 렌더러.
- `index.html`, `web/` — 정적 HTML/CSS, 외부 `boot.js` (CSP에서 inline script 불필요).

### 라이선스

MIT — [LICENSE](LICENSE) 참고. Copyright (c) rot-lab contributors.

---

## English

### What is it?

A single-page converter and visualizer for people who keep getting rotation conventions wrong
(ROS, URDF, MuJoCo, Isaac, VLA action spaces). Edit any one representation and all others update live.
Everything runs locally in your browser: no server, no analytics, no external CDNs, no runtime network requests.

- **Quaternion** with an explicit wxyz / xyzw order toggle, auto-normalization with a warning when the norm is far from 1,
  and an optional canonical sign (w ≥ 0).
- **Euler angles**: all 12 sequences, intrinsic or extrinsic, degrees or radians, with gimbal-lock detection.
- **3×3 rotation matrix** (editable). If it is not orthonormal the error is shown and the nearest rotation
  (iterative polar decomposition) is offered. Reflections (det < 0) are rejected.
- **Axis-angle**, **rotation vector** (so(3) log) and the continuous **6D representation**
  (first two columns of R, Gram-Schmidt back to R).
- **Presets**: URDF rpy, ROS tf quaternion (xyzw), MuJoCo quat (wxyz), aerospace ZYX yaw-pitch-roll, each with a short note.
- **Copy as code**: URDF `<origin rpy="..."/>`, Python list, numpy array / matrix, ROS `geometry_msgs/Quaternion` YAML.

### Conventions

| Topic | What rot-lab uses |
|---|---|
| Meaning of a rotation | Active rotation, column vectors `v' = R v`, right-handed frames |
| Quaternions | **Hamilton** (`ij = k`), `q = (w, x, y, z)`, `w = cos(θ/2)`. `q ⊗ p` applies p first, then q, i.e. `R(q) R(p)` |
| wxyz vs xyzw | **wxyz**: MuJoCo, Eigen constructor, Isaac Sim, scipy `scalar_first=True`. **xyzw**: ROS / tf2 / geometry_msgs, scipy default, PyBullet |
| Intrinsic A-B-C | Rotate about body A, then the *new* body B, then the newest C: `R = R_A(a) R_B(b) R_C(c)` |
| Extrinsic A-B-C | Rotate about fixed world A, then world B, then world C: `R = R_C(c) R_B(b) R_A(a)` |
| Equivalence | extrinsic `X-Y-Z (r, p, y)` == intrinsic `Z-Y-X (y, p, r)` |
| URDF `rpy` | Extrinsic X-Y-Z (fixed-axis roll, pitch, yaw) in radians |
| Gimbal lock | Middle angle ±90° (Tait-Bryan) or 0° / 180° (proper Euler). The third angle is set to 0; the matrix is still reproduced exactly |

The same four numbers read as wxyz or xyzw are completely different rotations: the ROS identity `[0, 0, 0, 1]`
read as wxyz is a 180° turn about Z. The JPL quaternion convention (used in some aerospace / VIO code) multiplies
in the opposite order; rot-lab is Hamilton only.

### Using each panel

1. **Converter** — type into any representation; the others update. The order / sequence / frame / unit selectors
   re-display the *same* rotation. A non-rotation matrix shows its error and a "Use nearest rotation" button.
2. **Visualizer** — thin dashed axes are the world frame, thick axes the rotated body frame (RGB = XYZ), world Z up.
   Drag to orbit. "Animate steps" plays the chosen Euler sequence one axis at a time: the highlighted axis is the moving
   body axis for intrinsic and the fixed world axis for extrinsic, so the difference is visible.
3. **Compose & compare** — enter A and B (quaternion, Euler or rotation vector): shows A·B, B·A, A⁻¹, the relative
   rotation A⁻¹B, the geodesic angle between A and B in degrees, and SLERP at t (slider), visualized.
4. **Batch** — paste one rotation per line (comma / whitespace separated), choose input and output representations,
   convert and download CSV. Parse errors are reported per line. Euler / axis-angle use the converter's current convention.

The UI language follows the browser (toggle Korean / English); dark / light follows the system. "Copy share link"
stores the state in the URL fragment (`#q=...`), which browsers never send to a server.

### Local build

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk --version 0.22.1 --locked   # or use the release binary

cargo test -p rot_core          # core math unit tests
trunk build --release           # static site in dist/ (public_url = /rot-lab/)
trunk serve                     # dev server at http://localhost:8080/rot-lab/
sh tools/privacy_check.sh       # must print "clean" before publishing
```

### Layout

- `crates/rot_core` — dependency-free rotation math (Quat, Mat3, Vec3, 12 × 2 Euler conventions, 6D, polar
  orthonormalization, SLERP, batch parsing, code snippets) with unit tests.
- `crates/rot_web` — UI on plain `web-sys` plus a canvas 2D renderer.
- `index.html`, `web/` — static HTML / CSS and an external `boot.js`, so the CSP needs no inline scripts.
- `.github/workflows/pages.yml` — privacy check, tests, `trunk build --release`, deploy to GitHub Pages.

### License

MIT — see [LICENSE](LICENSE). Copyright (c) rot-lab contributors.
