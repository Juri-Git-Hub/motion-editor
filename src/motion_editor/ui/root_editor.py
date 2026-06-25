import numpy as np
import imgui
from scipy.spatial.transform import Rotation as R


def draw_root_editor(editor):
    imgui.set_next_window_position(0.0, 76.0, imgui.ALWAYS)
    imgui.set_next_window_size(240, 0, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
    imgui.push_style_color(imgui.COLOR_WINDOW_BACKGROUND, 0.12, 0.12, 0.14, 0.92)

    expanded, _ = imgui.begin("Root Pose", False, flags)
    if not expanded:
        imgui.pop_style_color()
        imgui.end()
        return

    m = editor.motion
    f = int(editor.frame_idx)

    def _commit_pos(ax, name, value):
        m.record_history(f"Root Pos {name} frame {f}", f"root_frame {f}")
        m.root_pos[f, ax] = float(value)
        m.dirty_frames.add(f)
        editor.commit_frame(f)

    for ax, name in enumerate(['X', 'Y', 'Z']):
        val = float(m.root_pos[f, ax])
        imgui.set_next_item_width(120)
        changed, new_val = imgui.drag_float(f"##rootpos{name}", val, 0.01, -5.0, 5.0, "%.3f")
        if changed:
            _commit_pos(ax, name, new_val)
        imgui.same_line()
        imgui.set_next_item_width(60)
        ch_in, in_val = imgui.input_float(
            f"##rootpos{name}_in", val, 0.0, 0.0, "%.3f",
            imgui.INPUT_TEXT_ENTER_RETURNS_TRUE,
        )
        if ch_in:
            _commit_pos(ax, name, float(np.clip(in_val, -5.0, 5.0)))
        imgui.same_line()
        imgui.text(f"Pos {name}")

    # Root rotation as euler (editable)
    quat = m.root_rot[f]  # wxyz
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    euler = r.as_euler('xyz')

    def _commit_rot(ax, name, value):
        m.record_history(f"Root Rot {name} frame {f}", f"root_frame {f}")
        e = [euler[0], euler[1], euler[2]]
        e[ax] = float(value)
        q = R.from_euler('xyz', e).as_quat()  # xyzw
        m.root_rot[f] = [q[3], q[0], q[1], q[2]]
        m.dirty_frames.add(f)
        editor.commit_frame(f)

    for ax, name in enumerate(['Roll', 'Pitch', 'Yaw']):
        val = float(euler[ax])
        imgui.set_next_item_width(120)
        changed, new_val = imgui.drag_float(f"##rootrot{name}", val, 0.01, -np.pi, np.pi, "%.3f")
        if changed:
            _commit_rot(ax, name, new_val)
        imgui.same_line()
        imgui.set_next_item_width(60)
        ch_in, in_val = imgui.input_float(
            f"##rootrot{name}_in", val, 0.0, 0.0, "%.3f",
            imgui.INPUT_TEXT_ENTER_RETURNS_TRUE,
        )
        if ch_in:
            _commit_rot(ax, name, float(np.clip(in_val, -np.pi, np.pi)))
        imgui.same_line()
        imgui.text(name)

    imgui.pop_style_color()
    imgui.end()
