import glfw
import imgui


def draw_status_bar(editor):
    w, h = glfw.get_window_size(editor.window)
    imgui.set_next_window_position(0.0, h - 30.0, imgui.ALWAYS)
    imgui.set_next_window_size(w, 30.0, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE |
             imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_COLLAPSE)

    expanded, _ = imgui.begin("##status", False, flags)
    if not expanded:
        imgui.end()
        return

    m = editor.motion
    n_joints = len(editor.joint_info)
    imgui.text(f"Frames: {m.n_frames}  |  Joints: {n_joints}  |  FPS: {m.fps}")
    imgui.same_line(spacing=w - 500)
    imgui.text_colored(
        "LMB=Orbit  RMB=Pan  MMB=Pan-Z  Scroll=Zoom  Ctrl+Z=Undo  Ctrl+S=Save",
        0.5, 0.5, 0.5, 1.0,
    )
    imgui.end()
