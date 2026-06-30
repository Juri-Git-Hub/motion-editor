import imgui


def draw_collision_panel(editor):
    # HUD overlay at the top-left of the 3D viewport, just right of the
    # Root/Tools column. Auto-resizes to fit the colliding-pair list.
    imgui.set_next_window_position(250.0, 86.0, imgui.ALWAYS)
    flags = (imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE |
             imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_ALWAYS_AUTO_RESIZE)
    imgui.push_style_color(imgui.COLOR_WINDOW_BACKGROUND, 0.10, 0.10, 0.12, 0.85)

    expanded, _ = imgui.begin("Collision", False, flags)
    if not expanded:
        imgui.pop_style_color()
        imgui.end()
        return

    col = editor.renderer.collision
    if not col.colliding:
        imgui.text_colored("No self-collision", 0.4, 0.85, 0.4, 1.0)
        imgui.pop_style_color()
        imgui.end()
        return

    imgui.text_colored(f"⚠ COLLISION — {col.n} contact(s)", 1.0, 0.35, 0.3, 1.0)
    imgui.separator()
    # Worst-first list of colliding link pairs.
    for (a, b), depth in sorted(col.pairs.items(), key=lambda kv: -kv[1]):
        imgui.text_colored(f"{a} ↔ {b}", 1.0, 0.8, 0.8, 1.0)
        imgui.same_line()
        imgui.text(f"{depth * 1000:.1f} mm")
    if col.worst is not None:
        imgui.separator()
        d, a, b = col.worst
        imgui.text(f"worst: {d * 1000:.1f} mm  ({a} ↔ {b})")

    imgui.pop_style_color()
    imgui.end()
