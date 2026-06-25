import numpy as np
import imgui
import mujoco as mj
from scipy.spatial.transform import Rotation as R, Slerp


def draw_tools(editor):
    imgui.set_next_window_position(0.0, 256.0, imgui.ALWAYS)
    imgui.set_next_window_size(240, 0, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
    imgui.push_style_color(imgui.COLOR_WINDOW_BACKGROUND, 0.12, 0.12, 0.14, 0.92)

    expanded, _ = imgui.begin("Tools", False, flags)
    if not expanded:
        imgui.pop_style_color()
        imgui.end()
        return

    m = editor.motion
    f = int(editor.frame_idx)

    changed_gl, editor.ground_lock_enabled = imgui.checkbox(
        "Ground Lock", editor.ground_lock_enabled)
    if changed_gl and editor.ground_lock_enabled:
        # Snap the current frame immediately when turning the lock on.
        editor.commit_frame(f)
    if imgui.is_item_hovered():
        imgui.set_tooltip("Keep the lower foot on the floor (z=0) after edits")

    _, editor.symmetry_enabled = imgui.checkbox("Symmetry", editor.symmetry_enabled)
    imgui.same_line()
    if imgui.button("Mirror L→R", 90, 20):
        _mirror_pose(editor, m, f, left_to_right=True)
    imgui.same_line()
    if imgui.button("Mirror R→L", 90, 20):
        _mirror_pose(editor, m, f, left_to_right=False)

    if imgui.button("Copy Pose", 100, 24):
        editor.copied_pose = m.joint_pos[f].copy()
        editor.copied_root = (m.root_pos[f].copy(), m.root_rot[f].copy())
    imgui.same_line()
    if imgui.button("Paste Pose", 100, 24):
        if editor.copied_pose is not None:
            m.record_history(f"Paste frame {f}")
            m.joint_pos[f] = editor.copied_pose.copy()
            if editor.copied_root:
                m.root_pos[f] = editor.copied_root[0].copy()
                m.root_rot[f] = editor.copied_root[1].copy()
            m.dirty_frames.add(f)
            editor.commit_frame(f)

    if imgui.button("Reset Frame", 100, 24):
        m.record_history(f"Reset frame {f}")
        m.reset_frame(f)
        editor.commit_frame(f)
    imgui.same_line()
    if imgui.button("Reset All", 100, 24):
        m.record_history("Reset all frames")
        m.reset_all()
        editor.commit_frame(f)

    imgui.separator()
    imgui.text("Keyframes")
    is_kf = f in m.keyframes
    if imgui.button("Set Keyframe", 108, 22):
        m.keyframes.add(f)
    imgui.same_line()
    if imgui.button("Clear Keyframe", 108, 22):
        m.keyframes.discard(f)
    if imgui.button("Interpolate Keyframes", 142, 24):
        _interpolate_keyframes(editor, m)
    imgui.same_line()
    if imgui.button("Clear All", 70, 24):
        m.keyframes.clear()
    kfs = sorted(m.keyframes)
    mark = "  [current is keyframe]" if is_kf else ""
    imgui.text(f"{len(kfs)} keyframes{mark}")
    if kfs:
        imgui.text_wrapped(", ".join(str(k) for k in kfs))

    imgui.separator()
    imgui.text("Frame range  (Start–End)")

    _, new_start = imgui.slider_int(
        "##istart", editor.interp_start, 0, editor.interp_end - 1,
        "Start: %d",
    )
    if _:
        editor.interp_start = new_start
    _, new_end = imgui.slider_int(
        "##iend", editor.interp_end, editor.interp_start + 1, m.n_frames - 1,
        "End: %d",
    )
    if _:
        editor.interp_end = new_end
    _, editor.smooth_sigma = imgui.slider_float(
        "##sig", editor.smooth_sigma, 0.5, 10.0, "σ=%.1f",
    )
    imgui.text(f"Range: {editor.interp_end - editor.interp_start + 1} frames")
    if imgui.button("Center on current", 108, 20):
        r = editor.smooth_radius
        editor.interp_start = max(0, f - r)
        editor.interp_end = min(m.n_frames - 1, f + r)

    if imgui.button("Interpolate", 108, 24):
        _interpolate(editor, m, editor.interp_start, editor.interp_end)
    imgui.same_line()
    if imgui.button("Smooth", 108, 24):
        _smooth(editor, m, editor.interp_start, editor.interp_end, editor.smooth_sigma)

    if imgui.button("Ground Lock Range", 108, 24):
        editor.lock_ground_range(editor.interp_start, editor.interp_end)
    imgui.same_line()
    if imgui.button("Lock All", 108, 24):
        editor.lock_ground_range(0, m.n_frames - 1)

    if editor.has_ball:
        imgui.separator()
        imgui.text("Ball")
        changed_ball, editor.ball_enabled = imgui.checkbox("Show", editor.ball_enabled)
        if changed_ball:
            alpha = 1.0 if editor.ball_enabled else 0.0
            editor.model.geom_rgba[editor.ball_geom_id, 3] = alpha
            ct = 1 if editor.ball_enabled else 0
            editor.model.geom_contype[editor.ball_geom_id] = ct
            editor.model.geom_conaffinity[editor.ball_geom_id] = ct
        imgui.same_line()
        if imgui.button("Reset Pos", 90, 20):
            editor.data.qpos[editor.robot_nq:editor.robot_nq + 3] = [0.0, 0.0, editor.ball_radius]
            editor.data.qpos[editor.robot_nq + 3] = 1.0
            editor.data.qpos[editor.robot_nq + 4:editor.robot_nq + 7] = 0.0
            editor.data.qvel[editor.robot_nv:editor.robot_nv + 6] = 0.0

        bx, by, bz = editor.data.qpos[editor.robot_nq:editor.robot_nq + 3]
        cx, bx_new = imgui.slider_float("##bx", float(bx), -2.0, 2.0, "X: %.2f")
        cy, by_new = imgui.slider_float("##by", float(by), -2.0, 2.0, "Y: %.2f")
        cz, bz_new = imgui.slider_float("##bz", float(bz),  0.0, 2.0, "Z: %.2f")
        if cx or cy or cz:
            editor.data.qpos[editor.robot_nq]     = bx_new
            editor.data.qpos[editor.robot_nq + 1] = by_new
            editor.data.qpos[editor.robot_nq + 2] = bz_new
            editor.data.qvel[editor.robot_nv:editor.robot_nv + 6] = 0.0
            mj.mj_forward(editor.model, editor.data)

        changed_r, new_r = imgui.slider_float(
            "##ballr", editor.ball_radius, 0.02, 0.30, "Radius: %.2f m"
        )
        if changed_r:
            editor.ball_radius = new_r
            editor.model.geom_size[editor.ball_geom_id, 0] = new_r

    imgui.separator()
    if imgui.button("Save → output/  (Ctrl+S)", 220, 28):
        path = m.save()
        print(f"Saved → {path}")

    imgui.pop_style_color()
    imgui.end()


def _interpolate(editor, m, start, end):
    if end - start < 2:
        return
    f = int(editor.frame_idx)
    if not (start < f < end):
        print(f"Interpolate: current frame {f} must be inside range {start}–{end}")
        return
    m.record_history(f"Interpolate {start}-{end} (pivot {f})")
    n = m.n_joints
    for j in range(n):
        v_start = m.joint_pos[start, j]
        v_f = m.joint_pos[f, j]
        v_end = m.joint_pos[end, j]
        for fi in range(start + 1, f):
            a = (fi - start) / (f - start)
            m.joint_pos[fi, j] = v_start * (1 - a) + v_f * a
            m.dirty_frames.add(fi)
        for fi in range(f + 1, end):
            a = (fi - f) / (end - f)
            m.joint_pos[fi, j] = v_f * (1 - a) + v_end * a
            m.dirty_frames.add(fi)
    for ax in range(3):
        v_start = m.root_pos[start, ax]
        v_f = m.root_pos[f, ax]
        v_end = m.root_pos[end, ax]
        for fi in range(start + 1, f):
            a = (fi - start) / (f - start)
            m.root_pos[fi, ax] = v_start * (1 - a) + v_f * a
            m.dirty_frames.add(fi)
        for fi in range(f + 1, end):
            a = (fi - f) / (end - f)
            m.root_pos[fi, ax] = v_f * (1 - a) + v_end * a
            m.dirty_frames.add(fi)
    qs = m.root_rot[start]
    qf = m.root_rot[f]
    qe = m.root_rot[end]
    rs = R.from_quat([qs[1], qs[2], qs[3], qs[0]])
    rf = R.from_quat([qf[1], qf[2], qf[3], qf[0]])
    re = R.from_quat([qe[1], qe[2], qe[3], qe[0]])
    if f - start > 1:
        for fi in range(start + 1, f):
            a = (fi - start) / (f - start)
            ri = Slerp([0, 1], R.concatenate([rs, rf]))(a)
            q = ri.as_quat()
            m.root_rot[fi] = [q[3], q[0], q[1], q[2]]
            m.dirty_frames.add(fi)
    if end - f > 1:
        for fi in range(f + 1, end):
            a = (fi - f) / (end - f)
            ri = Slerp([0, 1], R.concatenate([rf, re]))(a)
            q = ri.as_quat()
            m.root_rot[fi] = [q[3], q[0], q[1], q[2]]
            m.dirty_frames.add(fi)
    editor.commit_frame(editor.frame_idx)
    print(f"Interpolated {start}–{end} (pivot: {f})")


def _smooth(editor, m, start, end, sigma):
    if end - start < 2:
        return
    f = int(editor.frame_idx)
    if not (start < f < end):
        print(f"Smooth: current frame {f} must be inside range {start}–{end}")
        return
    m.record_history(f"Smooth {start}-{end} (σ={sigma:.1f}, pivot {f})")
    saved = (m.joint_pos[f].copy(), m.root_pos[f].copy(), m.root_rot[f].copy())
    r = int(3 * sigma)
    kernel = np.exp(-0.5 * np.arange(-r, r + 1) ** 2 / sigma ** 2)
    kernel /= kernel.sum()
    n = m.n_joints
    for j in range(n):
        orig = m.joint_pos[start:end + 1, j].copy()
        m.joint_pos[start:end + 1, j] = np.convolve(orig, kernel, mode='same')
        for fi in range(start, end + 1):
            m.dirty_frames.add(fi)
    for ax in range(3):
        orig = m.root_pos[start:end + 1, ax].copy()
        m.root_pos[start:end + 1, ax] = np.convolve(orig, kernel, mode='same')
        for fi in range(start, end + 1):
            m.dirty_frames.add(fi)
    m.joint_pos[f], m.root_pos[f], m.root_rot[f] = saved
    editor.commit_frame(editor.frame_idx)
    print(f"Smoothed {start}–{end} (pivot: {f})")


def _lerp_segment(m, a, b):
    """Linear-lerp joints + root pos and Slerp root rot for frames strictly
    between keyframes a and b. Endpoints a and b are left untouched."""
    if b - a < 2:
        return
    qa = m.root_rot[a]
    qb = m.root_rot[b]
    ra = R.from_quat([qa[1], qa[2], qa[3], qa[0]])
    rb = R.from_quat([qb[1], qb[2], qb[3], qb[0]])
    slerp = Slerp([0, 1], R.concatenate([ra, rb]))
    for fi in range(a + 1, b):
        t = (fi - a) / (b - a)
        m.joint_pos[fi] = m.joint_pos[a] * (1 - t) + m.joint_pos[b] * t
        m.root_pos[fi] = m.root_pos[a] * (1 - t) + m.root_pos[b] * t
        q = slerp(t).as_quat()
        m.root_rot[fi] = [q[3], q[0], q[1], q[2]]
        m.dirty_frames.add(fi)


def _interpolate_keyframes(editor, m):
    kfs = sorted(int(k) for k in m.keyframes if 0 <= int(k) < m.n_frames)
    if len(kfs) < 2:
        print("Interpolate keyframes: need at least 2 keyframes")
        return
    m.record_history(f"Interpolate keyframes ({len(kfs)})")
    for a, b in zip(kfs, kfs[1:]):
        _lerp_segment(m, a, b)
    editor.commit_frame(editor.frame_idx)
    print(f"Interpolated keyframes: {kfs}")


def move_keyframe(editor, m, old, new):
    """Retime a keyframe: carry its posture from `old` to `new`. In-betweens are
    not rebuilt (manual via Interpolate Keyframes)."""
    new = int(np.clip(new, 0, m.n_frames - 1))
    old = int(old)
    if new == old or old not in m.keyframes or new in m.keyframes:
        return
    m.record_history(f"Move keyframe {old}->{new}")
    m.joint_pos[new] = m.joint_pos[old].copy()
    m.root_pos[new] = m.root_pos[old].copy()
    m.root_rot[new] = m.root_rot[old].copy()
    m.keyframes.discard(old)
    m.keyframes.add(new)
    m.dirty_frames.add(new)
    editor.commit_frame(new)
    print(f"Moved keyframe {old} → {new}")


def _mirror_pose(editor, m, f, left_to_right=True):
    m.record_history(f"Mirror {'L→R' if left_to_right else 'R→L'} frame {f}")
    for left_i, right_i in editor.symmetry_pairs:
        src_i = left_i if left_to_right else right_i
        dst_i = right_i if left_to_right else left_i
        sign = editor.symmetry_links[src_i][1]
        m.joint_pos[f, dst_i] = m.joint_pos[f, src_i] * sign
    m.dirty_frames.add(f)
    editor.commit_frame(f)
