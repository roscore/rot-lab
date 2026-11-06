//! English / Korean UI strings.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Lang {
    En,
    Ko,
}

/// (key, English, Korean)
const STRINGS: &[(&str, &str, &str)] = &[
    ("tagline", "3D rotation converter & visualizer for robotics", "로보틱스를 위한 3D 회전 변환기 & 시각화 도구"),
    ("share", "Copy share link", "공유 링크 복사"),
    ("share_done", "Link copied (state is in the # fragment, never sent to a server)", "링크 복사됨 (상태는 # fragment에만 있고 서버로 전송되지 않음)"),
    ("tab_conv", "Converter", "변환기"),
    ("tab_comp", "Compose & compare", "합성 & 비교"),
    ("tab_batch", "Batch", "일괄 변환"),
    ("tab_help", "Conventions", "규약 설명"),
    ("loading", "Loading…", "불러오는 중…"),
    ("presets", "Presets", "프리셋"),
    ("preset_aero", "Aerospace ZYX yaw-pitch-roll", "항공 ZYX yaw-pitch-roll"),
    ("quat", "Quaternion", "쿼터니언"),
    ("order", "Order", "순서"),
    ("canon", "w ≥ 0", "w ≥ 0 (부호 정규화)"),
    ("q_hint", "Hamilton convention, scalar part w. q and −q are the same rotation. Changing the order re-displays the same rotation.", "Hamilton 규약, 스칼라 성분 w. q와 −q는 같은 회전입니다. 순서를 바꾸면 같은 회전을 다른 순서로 다시 표시합니다."),
    ("euler", "Euler angles", "오일러 각"),
    ("extrinsic", "extrinsic (fixed axes)", "extrinsic (고정 축)"),
    ("intrinsic", "intrinsic (body axes)", "intrinsic (몸체 축)"),
    ("animate", "▶ Animate steps", "▶ 단계별 애니메이션"),
    ("stop", "■ Stop", "■ 정지"),
    ("matrix", "Rotation matrix (3×3)", "회전 행렬 (3×3)"),
    ("m_fix", "Use nearest rotation", "가장 가까운 회전 행렬 사용"),
    ("axisangle", "Axis-angle", "축-각 (axis-angle)"),
    ("rotvec", "Rotation vector (so(3) log, rad)", "회전 벡터 (so(3) log, rad)"),
    ("sixd", "6D representation (first two columns of R)", "6D 표현 (R의 처음 두 열)"),
    ("sixd_hint", "Continuous representation (Zhou et al. 2019). Back to R by Gram-Schmidt: b1 = a1/|a1|, b2 = normalize(a2 − (b1·a2) b1), b3 = b1 × b2.", "연속 표현 (Zhou et al. 2019). Gram-Schmidt로 R 복원: b1 = a1/|a1|, b2 = normalize(a2 − (b1·a2) b1), b3 = b1 × b2."),
    ("snippets", "Copy as code", "코드로 복사"),
    ("snip_py", "Python list (quat)", "Python 리스트 (쿼터니언)"),
    ("snip_np", "numpy (quat)", "numpy (쿼터니언)"),
    ("snip_npmat", "numpy (matrix)", "numpy (행렬)"),
    ("copied", "Copied to clipboard.", "클립보드에 복사했습니다."),
    ("viz", "Visualizer", "시각화"),
    ("reset_view", "Reset view", "시점 초기화"),
    ("viz_hint", "Drag to orbit. Thin dashed = world frame, thick = rotated body frame (RGB = XYZ). World Z is up.", "드래그해서 시점 회전. 얇은 점선 = 월드 좌표계, 굵은 선 = 회전된 몸체 좌표계 (RGB = XYZ). 월드 Z가 위쪽."),
    ("comp_inputs", "Rotations A and B", "회전 A와 B"),
    ("format", "Format", "형식"),
    ("fmt_quat", "quaternion (converter order)", "쿼터니언 (변환기 순서)"),
    ("fmt_euler", "Euler (converter convention)", "오일러 (변환기 규약)"),
    ("fmt_rotvec", "rotation vector (rad)", "회전 벡터 (rad)"),
    ("from_conv", "← converter", "← 변환기 값"),
    ("results", "Results", "결과"),
    ("comp_hint", "Products use the Hamilton convention with column vectors: (A·B)v = A(B(v)), i.e. apply B first, then A. A⁻¹B is B expressed relative to A.", "곱은 Hamilton 규약 + 열벡터 기준: (A·B)v = A(B(v)), 즉 B를 먼저, 그다음 A를 적용. A⁻¹B는 A 기준으로 본 B의 상대 회전."),
    ("slerp_hint", "Faint frames: A and B. Solid frame: slerp(A, B, t) along the shortest arc.", "흐린 좌표계: A와 B. 진한 좌표계: 최단 경로 slerp(A, B, t)."),
    ("geodesic", "geodesic angle(A, B)", "측지 거리 각(A, B)"),
    ("batch", "Batch convert", "일괄 변환"),
    ("input", "Input", "입력"),
    ("output", "Output", "출력"),
    ("convert", "Convert", "변환"),
    ("download", "Download CSV", "CSV 다운로드"),
    ("b_in", "One rotation per line (comma / space separated). Blank lines and # comments are skipped; a header line is ignored.", "한 줄에 회전 하나 (쉼표/공백 구분). 빈 줄과 # 주석은 건너뛰고, 첫 헤더 줄은 무시합니다."),
    ("b_sum", "{ok} converted, {err} errors.", "{ok}개 변환, 오류 {err}개."),
    ("b_line", "line", "줄"),
    ("foot", "Runs 100% in your browser. No tracking, no network requests.", "100% 브라우저에서 실행. 추적 없음, 네트워크 요청 없음."),
    ("conv_note", "Euler = {conv}; axis-angle angle in {unit}.", "오일러 = {conv}; 축-각의 각도 단위 {unit}."),
    ("r_wxyz", "quaternion wxyz", "쿼터니언 wxyz"),
    ("r_xyzw", "quaternion xyzw", "쿼터니언 xyzw"),
    ("r_euler", "Euler (converter convention)", "오일러 (변환기 규약)"),
    ("r_matrix", "matrix 3×3 (row-major)", "행렬 3×3 (행 우선)"),
    ("r_aa", "axis-angle", "축-각"),
    ("r_rotvec", "rotation vector", "회전 벡터"),
    ("r_6d", "6D", "6D"),
    // dynamic messages
    ("need_n", "Enter {n} numbers.", "숫자 {n}개를 입력하세요."),
    ("q_zero", "Zero quaternion is not a rotation.", "0 쿼터니언은 회전이 아닙니다."),
    ("q_norm", "Norm was {n}: normalized before use.", "노름이 {n}였으므로 정규화해서 사용합니다."),
    ("gimbal", "Gimbal lock: the middle angle is at its singularity, so axes 1 and 3 coincide and only their combination is defined. The third angle is set to 0 (the matrix is still exact).", "짐벌 락: 가운데 각이 특이점에 있어 1번 축과 3번 축이 겹치고, 두 각의 조합만 정의됩니다. 셋째 각을 0으로 두었습니다 (행렬은 그대로 정확)."),
    ("gimbal_near", "Near gimbal lock ({d}° from the singularity): angles 1 and 3 are ill-conditioned.", "짐벌 락 근처 (특이점까지 {d}°): 1번, 3번 각이 수치적으로 불안정합니다."),
    ("m_bad", "Not a rotation: ‖RᵀR − I‖ = {e}, det = {d}.", "회전 행렬이 아님: ‖RᵀR − I‖ = {e}, det = {d}."),
    ("m_offer", "Nearest rotation (polar decomposition) is available below.", "아래 버튼으로 가장 가까운 회전 행렬(극분해)을 사용할 수 있습니다."),
    ("m_reflection", "det < 0: this is a reflection, which no rotation can represent.", "det < 0: 반사(reflection) 행렬이므로 회전으로 나타낼 수 없습니다."),
    ("m_singular", "Matrix is singular (rank deficient).", "특이(singular) 행렬입니다 (rank 부족)."),
    ("m_other", "Could not orthonormalize this matrix.", "이 행렬은 정규직교화할 수 없습니다."),
    ("m_fixed", "Replaced by the nearest rotation.", "가장 가까운 회전 행렬로 바꿨습니다."),
    ("a_zero", "Axis is zero.", "축 벡터가 0입니다."),
    ("a_norm", "Axis normalized (length was {n}).", "축을 정규화했습니다 (길이 {n})."),
    ("s_bad", "Columns are zero or parallel: Gram-Schmidt is undefined.", "열 벡터가 0이거나 평행해서 Gram-Schmidt를 할 수 없습니다."),
    ("s_gs", "Input was not orthonormal: Gram-Schmidt applied.", "입력이 정규직교가 아니라서 Gram-Schmidt를 적용했습니다."),
    ("step", "Step {i}/3: rotate {ang} about {axis}", "{i}/3 단계: {axis} 기준 {ang} 회전"),
    ("step_done", "Done: R = {formula}", "완료: R = {formula}"),
    ("body_axis", "body {a}", "몸체 {a}"),
    ("world_axis", "world {a}", "월드 {a}"),
    ("bad_num", "Not a number: {s}", "숫자가 아님: {s}"),
];

pub fn tr(lang: Lang, key: &str) -> &'static str {
    for (k, en, ko) in STRINGS {
        if *k == key {
            return if lang == Lang::Ko { ko } else { en };
        }
    }
    ""
}

/// Translate and substitute `{name}` placeholders.
pub fn trf(lang: Lang, key: &str, args: &[(&str, &str)]) -> String {
    let mut s = tr(lang, key).to_string();
    for (k, v) in args {
        s = s.replace(&format!("{{{k}}}"), v);
    }
    s
}
