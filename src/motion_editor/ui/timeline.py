import imgui

from .tools import move_keyframe


def _span(editor):
    return max(1e-6, editor.view_hi - editor.view_lo)


def _f2x(editor, f, x0, x1):
    return x0 + (x1 - x0) * (f - editor.view_lo) / _span(editor)


def _x2f(editor, mx, x0, x1):
    return editor.view_lo + (mx - x0) / max(1.0, x1 - x0) * _span(editor)


def _zoom(editor, n, factor, anchor):
    anchor = min(max(anchor, editor.view_lo), editor.view_hi)
    editor.view_lo = anchor - (anchor - editor.view_lo) * factor
    editor.view_hi = anchor + (editor.view_hi - anchor) * factor
    _clamp_view(editor, n)


def _clamp_view(editor, n):
    min_span = 1.0
    if editor.view_hi - editor.view_lo < min_span:
        editor.view_hi = editor.view_lo + min_span
    if editor.view_lo < 0.0:
        editor.view_hi -= editor.view_lo
        editor.view_lo = 0.0
    if editor.view_hi > n:
        editor.view_lo -= (editor.view_hi - n)
        editor.view_hi = float(n)
    editor.view_lo = max(0.0, editor.view_lo)


def draw_timeline(editor):
    w, h = glfw_window_size(editor)
    imgui.set_next_window_position(0.0, 0.0, imgui.ALWAYS)
    imgui.set_next_window_size(w, 76.0, imgui.ALWAYS)

    flags = (imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE |
             imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_COLLAPSE)

    expanded, _ = imgui.begin("##timeline", False, flags)
    if not expanded:
        imgui.end()
        return

    m = editor.motion

    # Yellow glow border around the whole window when parked on a keyframe.
    if int(round(editor.frame_idx)) in m.keyframes:
        fg = imgui.get_foreground_draw_list()
        glow = imgui.get_color_u32_rgba(1.0, 0.85, 0.2, 0.9)
        fg.add_rect(2, 2, w - 2, h - 2, glow, 0.0, 0, 5.0)

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
    if imgui.button("-##zoomout", 24, 24):
        _zoom(editor, m.n_frames - 1, 1.25, editor.frame_idx)
    imgui.same_line()
    if imgui.button("+##zoomin", 24, 24):
        _zoom(editor, m.n_frames - 1, 0.8, editor.frame_idx)
    imgui.same_line()
    if imgui.button("Fit", 34, 24):
        editor.view_lo = 0.0
        editor.view_hi = float(max(1, m.n_frames - 1))

    imgui.same_line()
    _, editor.loop = imgui.checkbox("Loop", editor.loop)

    imgui.same_line(spacing=12)
    imgui.text("Speed:")
    imgui.same_line()
    imgui.set_next_item_width(54)
    _, editor.speed = imgui.slider_float("##spd", editor.speed, 0.1, 5.0, "%.1fx")

    imgui.same_line(spacing=16)
    cur = int(round(editor.frame_idx))
    imgui.text(f"Frame {cur + 1} / {m.n_frames}  ({cur / m.fps:.1f}s)")

    if m.modified:
        imgui.same_line(spacing=16)
        imgui.text_colored("Modified", 1.0, 0.8, 0.2, 1.0)

    # Scrub bar (range = visible zoom window)
    imgui.set_next_item_width(w - 20)
    _, frame_f = imgui.slider_float(
        "##scrub", float(editor.frame_idx), editor.view_lo, editor.view_hi, "",
    )
    if _:
        editor.frame_idx = int(round(frame_f))
        editor.playing = False
        editor.renderer.apply_frame(m, editor.frame_idx)
    # Double-click the scrub bar resets zoom to full view.
    if imgui.is_item_hovered() and imgui.is_mouse_double_clicked(0):
        editor.view_lo = 0.0
        editor.view_hi = float(max(1, m.n_frames - 1))

    # Zoom/pan interactions + overlays on the scrub bar
    if m.n_frames > 1:
        draw = imgui.get_window_draw_list()
        rect_min = imgui.get_item_rect_min()
        rect_max = imgui.get_item_rect_max()
        x0 = rect_min.x
        x1 = rect_max.x
        n = m.n_frames - 1
        io = imgui.get_io()
        hovered_win = imgui.is_window_hovered()

        # Wheel zoom anchored at the cursor.
        if io.mouse_wheel != 0.0:
            print(f"[timeline] wheel={io.mouse_wheel} hovered_win={hovered_win}")
        if hovered_win and io.mouse_wheel != 0.0:
            fa = _x2f(editor, imgui.get_mouse_pos()[0], x0, x1)
            _zoom(editor, n, 0.85 ** io.mouse_wheel, fa)

        # Middle-drag pan.
        if hovered_win and imgui.is_mouse_dragging(2):
            df = -io.mouse_delta[0] / max(1.0, x1 - x0) * _span(editor)
            editor.view_lo += df
            editor.view_hi += df
            _clamp_view(editor, n)

        # Range marker.
        col = imgui.get_color_u32_rgba(0.31, 0.57, 0.85, 0.6)
        xs = max(x0, min(x1, _f2x(editor, editor.interp_start, x0, x1)))
        xe = max(x0, min(x1, _f2x(editor, editor.interp_end, x0, x1)))
        if xe > xs:
            draw.add_rect_filled(xs, rect_min.y - 4, xe, rect_max.y + 4, col)

        # Keyframe ticks.
        kf_col = imgui.get_color_u32_rgba(1.0, 0.85, 0.2, 1.0)
        for kf in m.keyframes:
            if not (0 <= kf <= n):
                continue
            xk = _f2x(editor, kf, x0, x1)
            if x0 <= xk <= x1:
                draw.add_rect_filled(xk - 1.5, rect_min.y - 5, xk + 1.5, rect_max.y + 5, kf_col)

        # Draggable keyframe handles in a lane below the scrub bar.
        hw, hh = 5.0, 11.0
        lane_y = rect_max.y + 3
        hot_col = imgui.get_color_u32_rgba(1.0, 1.0, 0.5, 1.0)
        for kf in sorted(m.keyframes):
            if not (0 <= kf <= n):
                continue
            xk = _f2x(editor, kf, x0, x1)
            if not (x0 <= xk <= x1):
                continue
            imgui.push_id(f"kf{int(kf)}")
            imgui.set_cursor_screen_pos((xk - hw, lane_y))
            imgui.invisible_button("kfh", hw * 2, hh)
            active = imgui.is_item_active()
            hovered = imgui.is_item_hovered()
            tcol = hot_col if (active or hovered) else kf_col
            # Downward triangle handle.
            draw.add_triangle_filled(
                xk - hw, lane_y, xk + hw, lane_y, xk, lane_y + hh, tcol)
            if hovered and not active:
                imgui.set_tooltip(f"Keyframe {kf} — drag to move")
            if active:
                target = int(round(_x2f(editor, imgui.get_mouse_pos()[0], x0, x1)))
                target = max(0, min(n, target))
                editor.kf_drag = (int(kf), target)
                xt = max(x0, min(x1, _f2x(editor, target, x0, x1)))
                draw.add_line(xt, rect_min.y - 5, xt, lane_y + hh,
                              imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9), 1.5)
            if imgui.is_item_deactivated() and editor.kf_drag is not None:
                old_f, target = editor.kf_drag
                move_keyframe(editor, m, old_f, target)
                editor.kf_drag = None
            imgui.pop_id()

    imgui.end()


def glfw_window_size(editor):
    import glfw
    return glfw.get_window_size(editor.window)
