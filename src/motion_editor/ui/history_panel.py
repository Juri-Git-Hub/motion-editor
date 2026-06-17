import imgui


def draw_history(editor):
    w, h = editor.window_size
    hist_y = 586.0
    imgui.set_next_window_position(0.0, hist_y, imgui.ALWAYS)
    imgui.set_next_window_size(240, h - hist_y - 30, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
    imgui.push_style_color(imgui.COLOR_WINDOW_BACKGROUND, 0.12, 0.12, 0.14, 0.92)

    expanded, _ = imgui.begin("History", False, flags)
    if not expanded:
        imgui.pop_style_color()
        imgui.end()
        return

    m = editor.motion
    avail_h = imgui.get_window_height() - 50
    imgui.begin_child("##hlist", 0, max(1, avail_h), False)

    for i, entry in enumerate(reversed(m.history)):
        desc = entry[0]
        idx = len(m.history) - 1 - i
        is_current = idx == m.history_pos

        if is_current:
            imgui.push_style_color(imgui.COLOR_TEXT, 0.31, 0.57, 0.85, 1.0)

        _, clicked = imgui.selectable(f"{desc}##h{idx}", is_current)
        if clicked and idx != m.history_pos:
            m.restore_history(idx)
            editor.renderer.apply_frame(m, editor.frame_idx)

        if is_current:
            imgui.pop_style_color()

    imgui.end_child()
    imgui.pop_style_color()
    imgui.end()
