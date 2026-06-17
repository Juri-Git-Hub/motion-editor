import numpy as np
import glfw
import imgui


class CameraController:
    def __init__(self, cam):
        self.cam = cam
        self.mouse_buttons = [False, False, False]
        self.last_mouse_pos = (0, 0)

    def on_mouse_button(self, window, button, action, mods):
        self.mouse_buttons[button] = action == glfw.PRESS
        if action == glfw.PRESS:
            self.last_mouse_pos = glfw.get_cursor_pos(window)

    def on_cursor_pos(self, window, x, y):
        if not any(self.mouse_buttons):
            return
        io = imgui.get_io()
        dx, dy = x - self.last_mouse_pos[0], y - self.last_mouse_pos[1]
        if self.mouse_buttons[glfw.MOUSE_BUTTON_LEFT] and not io.want_capture_mouse:
            self.cam.azimuth += dx * 0.3
            self.cam.elevation = np.clip(self.cam.elevation + dy * 0.3, -90, 90)
        elif self.mouse_buttons[glfw.MOUSE_BUTTON_RIGHT] and not io.want_capture_mouse:
            self.cam.lookat[0] -= dx * 0.005 * self.cam.distance
            self.cam.lookat[1] += dy * 0.005 * self.cam.distance
        elif self.mouse_buttons[glfw.MOUSE_BUTTON_MIDDLE]:
            self.cam.lookat[0] -= dx * 0.005 * self.cam.distance
            self.cam.lookat[2] += dy * 0.005 * self.cam.distance
        self.last_mouse_pos = (x, y)

    def on_scroll(self, window, x_offset, y_offset):
        if imgui.get_io().want_capture_mouse:
            return
        self.cam.distance *= (1 - y_offset * 0.1)
        self.cam.distance = np.clip(self.cam.distance, 0.5, 50.0)
