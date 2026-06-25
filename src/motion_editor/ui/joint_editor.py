import imgui
import numpy as np
from scipy.spatial.transform import Rotation as R


def _apply_joint(editor, m, fi, npz_i, new_val, pair_map):
    m.record_history("Edit joints", f"frame {fi}")
    m.joint_pos[fi, npz_i] = float(new_val)
    link = pair_map.get(npz_i)
    if link is not None:
        pair_i, sign = link
        m.joint_pos[fi, pair_i] = float(new_val) * sign
    m.dirty_frames.add(fi)
    editor.commit_frame(fi)


def draw_joint_editor(editor):
    w, h = editor.window_size
    pw = editor.panel_width
    imgui.set_next_window_position(w - pw, 76.0, imgui.ALWAYS)
    imgui.set_next_window_size(pw, h - 76.0 - 30, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
    imgui.push_style_color(imgui.COLOR_WINDOW_BACKGROUND, 0.12, 0.12, 0.14, 0.92)

    expanded, _ = imgui.begin("Joints", False, flags)
    if not expanded:
        imgui.pop_style_color()
        imgui.end()
        return

    m = editor.motion
    ji = editor.joint_info
    ed_to_npz = editor.editor_to_npz
    f = editor.frame_idx

    pair_map = editor.symmetry_links if editor.symmetry_enabled else {}

    avail_h = imgui.get_window_height() - 120
    imgui.begin_child("##jlist", 0, avail_h, False)

    current_group = -1
    for i, info in enumerate(ji):
        g_idx, g_name = info['group']
        if g_idx != current_group:
            current_group = g_idx
            imgui.dummy(0, 2)
            imgui.text_colored(f" > {g_name}", 0.6, 0.8, 1.0, 1.0)
            imgui.separator()

        npz_i = int(ed_to_npz[i])
        if npz_i < 0:
            continue

        fi = int(f)
        val = float(m.joint_pos[fi, npz_i])
        orig = float(m.orig_joint_pos[fi, npz_i])
        lo, hi = info['limit_lo'], info['limit_hi']
        dirty = m.is_dirty(fi, npz_i)

        adjusted = np.clip(val, lo, hi)
        imgui.text(f"  {info['short']}")
        imgui.same_line(160)

        sw = pw - 200 - 64
        imgui.set_next_item_width(sw)
        changed, new_val = imgui.slider_float(f"##j{i}", adjusted, lo, hi, "%.3f")
        if changed:
            _apply_joint(editor, m, fi, npz_i, new_val, pair_map)

        # Click-to-type numeric entry (keeps slider for quick drag).
        imgui.same_line()
        imgui.set_next_item_width(60)
        ch_in, in_val = imgui.input_float(
            f"##ji{i}", val, 0.0, 0.0, "%.3f",
            imgui.INPUT_TEXT_ENTER_RETURNS_TRUE,
        )
        if ch_in:
            _apply_joint(editor, m, fi, npz_i, float(np.clip(in_val, lo, hi)), pair_map)

        if dirty:
            imgui.same_line()
            imgui.text_colored("●", 1.0, 0.7, 0.2, 1.0)

        if imgui.is_item_hovered():
            imgui.set_tooltip(
                f"{info['name']}\nRange: [{lo:.3f}, {hi:.3f}] rad\nOriginal: {orig:.3f}"
            )

        if imgui.begin_popup_context_item(f"ctx{i}", imgui.MOUSE_BUTTON_RIGHT):
            if imgui.selectable(f"Reset to original ({orig:.3f})"):
                m.record_history(f"Reset {info['short']} frame {fi}")
                m.joint_pos[fi, npz_i] = orig
                link = pair_map.get(npz_i)
                if link is not None:
                    pair_i, sign = link
                    m.joint_pos[fi, pair_i] = m.orig_joint_pos[fi, pair_i]
                m.dirty_frames.discard(fi)
                editor.commit_frame(fi)
            imgui.end_popup()

    imgui.end_child()

    imgui.text_colored(" > Trunk Tilt", 0.6, 0.8, 1.0, 1.0)
    imgui.separator()
    _draw_trunk_tilt(editor)

    imgui.pop_style_color()
    imgui.end()


def _apply_trunk_euler(editor, m, f, euler, axis, new_deg, label):
    m.record_history(f"Trunk {label} frame {f}", f"trunk_frame {f}")
    euler[axis] = float(np.radians(new_deg))
    q = R.from_euler('xyz', euler).as_quat()
    m.root_rot[f] = [q[3], q[0], q[1], q[2]]
    m.dirty_frames.add(f)
    editor.commit_frame(f)


def _trunk_axis_row(editor, m, f, euler, axis, label, lo, hi):
    cur_deg = float(np.degrees(euler[axis]))
    imgui.set_next_item_width(140)
    _, new_deg = imgui.slider_float(f"##trunk_{label}", cur_deg, lo, hi, f"{label}: %.1f°")
    if abs(new_deg - cur_deg) > 0.01:
        _apply_trunk_euler(editor, m, f, euler, axis, new_deg, label)
        return
    imgui.same_line()
    imgui.set_next_item_width(60)
    ch_in, in_deg = imgui.input_float(
        f"##trunk_{label}_in", cur_deg, 0.0, 0.0, "%.1f",
        imgui.INPUT_TEXT_ENTER_RETURNS_TRUE,
    )
    if ch_in:
        _apply_trunk_euler(editor, m, f, euler, axis, float(np.clip(in_deg, lo, hi)), label)


def _draw_trunk_tilt(editor):
    m = editor.motion
    f = int(editor.frame_idx)
    quat = m.root_rot[f]
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    euler = r.as_euler('xyz')

    _trunk_axis_row(editor, m, f, euler, 1, "Pitch", -90.0, 90.0)
    _trunk_axis_row(editor, m, f, euler, 0, "Roll", -90.0, 90.0)
    _trunk_axis_row(editor, m, f, euler, 2, "Yaw", -180.0, 180.0)
