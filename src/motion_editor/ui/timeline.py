import imgui


def draw_timeline(editor):
    w, h = glfw_window_size(editor)
    imgui.set_next_window_position(0.0, 0.0, imgui.ALWAYS)
    imgui.set_next_window_size(w, 56.0, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE |
             imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_COLLAPSE)

    expanded, _ = imgui.begin("##timeline", False, flags)
    if not expanded:
        imgui.end()
        return

    m = editor.motion
    # Playback buttons
    if imgui.button("|<", 28, 24):
        editor.frame_idx = 0
        editor.playing = False
        editor.renderer.apply_frame(m, 0)

    imgui.same_line()
    if imgui.button("<", 28, 24):
        editor.frame_idx = max(0, editor.frame_idx - 1)
        editor.playing = False
        editor.renderer.apply_frame(m, editor.frame_idx)

    imgui.same_line()
    label = "||##play" if editor.playing else ">##play"
    if imgui.button(label, 52, 24):
        editor.playing = not editor.playing
        if not editor.playing:
            editor.renderer.apply_frame(m, editor.frame_idx)

    imgui.same_line()
    if imgui.button(">##next", 28, 24):
        editor.frame_idx = min(m.n_frames - 1, editor.frame_idx + 1)
        editor.playing = False
        editor.renderer.apply_frame(m, editor.frame_idx)

    imgui.same_line()
    if imgui.button(">|", 28, 24):
        editor.frame_idx = m.n_frames - 1
        editor.playing = False
        editor.renderer.apply_frame(m, editor.frame_idx)

    imgui.same_line()
    _, editor.loop = imgui.checkbox("Loop", editor.loop)

    imgui.same_line(spacing=12)
    imgui.text("Speed:")
    imgui.same_line()
    imgui.set_next_item_width(54)
    _, editor.speed = imgui.slider_float("##spd", editor.speed, 0.1, 5.0, "%.1fx")

    imgui.same_line(spacing=16)
    imgui.text(f"Frame {editor.frame_idx + 1} / {m.n_frames}  ({editor.frame_idx / m.fps:.1f}s)")

    if m.modified:
        imgui.same_line(spacing=16)
        imgui.text_colored("Modified", 1.0, 0.8, 0.2, 1.0)

    # Scrub bar
    imgui.set_next_item_width(w - 20)
    _, frame_f = imgui.slider_float(
        "##scrub", float(editor.frame_idx), 0.0, float(m.n_frames - 1), "",
    )
    if _:
        editor.frame_idx = int(round(frame_f))
        editor.playing = False
        editor.renderer.apply_frame(m, editor.frame_idx)

    # Range markers on scrub bar
    if m.n_frames > 1:
        draw = imgui.get_window_draw_list()
        rect_min = imgui.get_item_rect_min()
        rect_max = imgui.get_item_rect_max()
        bar_x0 = rect_min.x
        bar_x1 = rect_max.x
        n = m.n_frames - 1
        s = editor.interp_start
        e = editor.interp_end
        xs = bar_x0 + (bar_x1 - bar_x0) * s / n
        xe = bar_x0 + (bar_x1 - bar_x0) * e / n
        cy = (rect_min.y + rect_max.y) * 0.5
        col = imgui.get_color_u32_rgba(0.31, 0.57, 0.85, 0.6)
        draw.add_rect_filled(xs, rect_min.y - 4, xe, rect_max.y + 4, col)

    imgui.end()


def glfw_window_size(editor):
    import glfw
    return glfw.get_window_size(editor.window)
