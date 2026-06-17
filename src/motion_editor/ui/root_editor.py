import numpy as np
import imgui
from scipy.spatial.transform import Rotation as R


def draw_root_editor(editor):
    imgui.set_next_window_position(0.0, 56.0, imgui.ALWAYS)
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

    for ax, name in enumerate(['X', 'Y', 'Z']):
        val = float(m.root_pos[f, ax])
        changed, new_val = imgui.drag_float(f"Pos {name}", val, 0.01, -5.0, 5.0, "%.3f")
        if changed:
            m.record_history(f"Root Pos {name} frame {f}", f"root_frame {f}")
            m.root_pos[f, ax] = float(new_val)
            m.dirty_frames.add(f)
            editor.renderer.apply_frame(m, f)

    # Root rotation as euler (editable)
    quat = m.root_rot[f]  # wxyz
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    euler = r.as_euler('xyz')
    for ax, name in enumerate(['Roll', 'Pitch', 'Yaw']):
        changed, new_val = imgui.drag_float(name, float(euler[ax]), 0.01, -np.pi, np.pi, "%.3f")
        if changed:
            m.record_history(f"Root Rot {name} frame {f}", f"root_frame {f}")
            # Convert back to quat
            r2 = R.from_euler('xyz', [euler[0], euler[1], euler[2]])
            if name == 'Roll':
                r2 = R.from_euler('xyz', [float(new_val), euler[1], euler[2]])
            elif name == 'Pitch':
                r2 = R.from_euler('xyz', [euler[0], float(new_val), euler[2]])
            else:
                r2 = R.from_euler('xyz', [euler[0], euler[1], float(new_val)])
            q = r2.as_quat()  # xyzw
            m.root_rot[f] = [q[3], q[0], q[1], q[2]]
            m.dirty_frames.add(f)
            editor.renderer.apply_frame(m, f)

    imgui.pop_style_color()
    imgui.end()
