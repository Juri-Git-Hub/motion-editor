import imgui
import numpy as np
from scipy.spatial.transform import Rotation as R


def draw_joint_editor(editor):
    w, h = editor.window_size
    pw = editor.panel_width
    imgui.set_next_window_position(w - pw, 56.0, imgui.ALWAYS)
    imgui.set_next_window_size(pw, h - 56.0 - 30, imgui.ALWAYS)

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

        sw = pw - 200
        imgui.set_next_item_width(sw)
        changed, new_val = imgui.slider_float(f"##j{i}", adjusted, lo, hi, "%.3f")

        if changed:
            m.record_history("Edit joints", f"frame {fi}")
            m.joint_pos[fi, npz_i] = float(new_val)
            link = pair_map.get(npz_i)
            if link is not None:
                pair_i, sign = link
                m.joint_pos[fi, pair_i] = float(new_val) * sign
            m.dirty_frames.add(fi)
            editor.renderer.apply_frame(m, fi)

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
                editor.renderer.apply_frame(m, fi)
            imgui.end_popup()

    imgui.end_child()

    imgui.text_colored(" > Trunk Tilt", 0.6, 0.8, 1.0, 1.0)
    imgui.separator()
    _draw_trunk_tilt(editor)

    imgui.pop_style_color()
    imgui.end()


def _draw_trunk_tilt(editor):
    m = editor.motion
    f = int(editor.frame_idx)
    quat = m.root_rot[f]
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    euler = r.as_euler('xyz')

    tilt_deg = float(np.degrees(euler[1]))
    _, new_tilt = imgui.slider_float("##trunk_tilt", tilt_deg, -90.0, 90.0, "Pitch: %.1f°")
    if abs(new_tilt - tilt_deg) > 0.01:
        m.record_history(f"Trunk tilt frame {f}", f"trunk_frame {f}")
        euler[1] = float(np.radians(new_tilt))
        r2 = R.from_euler('xyz', euler)
        q = r2.as_quat()
        m.root_rot[f] = [q[3], q[0], q[1], q[2]]
        m.dirty_frames.add(f)
        editor.renderer.apply_frame(m, f)

    roll_deg = float(np.degrees(euler[0]))
    _, new_roll = imgui.slider_float("##trunk_roll", roll_deg, -90.0, 90.0, "Roll: %.1f°")
    if abs(new_roll - roll_deg) > 0.01:
        m.record_history(f"Trunk roll frame {f}", f"trunk_frame {f}")
        euler[0] = float(np.radians(new_roll))
        r2 = R.from_euler('xyz', euler)
        q = r2.as_quat()
        m.root_rot[f] = [q[3], q[0], q[1], q[2]]
        m.dirty_frames.add(f)
        editor.renderer.apply_frame(m, f)

    yaw_deg = float(np.degrees(euler[2]))
    _, new_yaw = imgui.slider_float("##trunk_yaw", yaw_deg, -180.0, 180.0, "Yaw: %.1f°")
    if abs(new_yaw - yaw_deg) > 0.01:
        m.record_history(f"Trunk yaw frame {f}", f"trunk_frame {f}")
        euler[2] = float(np.radians(new_yaw))
        r2 = R.from_euler('xyz', euler)
        q = r2.as_quat()
        m.root_rot[f] = [q[3], q[0], q[1], q[2]]
        m.dirty_frames.add(f)
        editor.renderer.apply_frame(m, f)
