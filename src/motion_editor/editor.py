import os
import time

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import mujoco as mj
import numpy as np

from .motion_data import MotionData
from .renderer import Renderer
from .camera import CameraController
from .joint_info import (
    build_joint_info,
    build_npz_joint_map,
    build_editor_to_npz_map,
    build_npz_body_map,
    build_symmetry_pairs,
    NPZ_JOINT_NAMES,
)
from .ui.timeline import draw_timeline
from .ui.joint_editor import draw_joint_editor
from .ui.root_editor import draw_root_editor
from .ui.tools import draw_tools
from .ui.status_bar import draw_status_bar
from .ui.history_panel import draw_history


class Editor:
    def __init__(self, xml_path, motion_path):
        # Load model
        self.model = mj.MjModel.from_xml_path(str(xml_path))
        self.data = mj.MjData(self.model)
        mj.mj_forward(self.model, self.data)

        # Build joint mappings
        self.joint_info = build_joint_info(self.model)
        self.npz_to_qpos = build_npz_joint_map(self.model)
        self.editor_to_npz = build_editor_to_npz_map(self.model)
        self.npz_body_map = build_npz_body_map(self.model)

        # Load motion (pass model/maps for PKL remapping + FK on save)
        self.motion = MotionData(
            motion_path,
            model=self.model,
            npz_to_qpos=self.npz_to_qpos,
            npz_body_map=self.npz_body_map,
        )
        # Editor state
        self.frame_idx = 0
        self.playing = False
        self.loop = True
        self.speed = 1.0

        # Copy/paste buffers
        self.copied_pose = None
        self.copied_root = None

        # Tool params / interpolation range
        self.smooth_sigma = 3.0
        self.smooth_radius = 15
        r = self.smooth_radius
        self.interp_start = max(0, self.frame_idx - r)
        self.interp_end = min(self.motion.n_frames - 1, self.frame_idx + r)

        # Symmetry
        self.symmetry_enabled = False
        self.symmetry_pairs = build_symmetry_pairs()
        self.symmetry_links = {}  # npz_i → (paired_npz_i, sign)
        npz_range = {}
        for info in self.joint_info:
            name = info['name']
            if name in NPZ_JOINT_NAMES:
                npz_range[name] = (info['limit_lo'], info['limit_hi'])
        eps = 1e-3
        for li, ri in self.symmetry_pairs:
            lo_l, hi_l = npz_range.get(NPZ_JOINT_NAMES[li], (0, 0))
            lo_r, hi_r = npz_range.get(NPZ_JOINT_NAMES[ri], (0, 0))
            swapped = abs(hi_l + lo_r) < eps and abs(lo_l + hi_r) < eps
            identical = abs(lo_l - lo_r) < eps and abs(hi_l - hi_r) < eps
            sign = -1 if (swapped and not identical) else 1
            self.symmetry_links[li] = (ri, sign)
            self.symmetry_links[ri] = (li, sign)

        # Window — provides OpenGL context needed by MuJoCo renderer
        self.panel_width = 360
        self.window = None
        self.window_size = (1400, 900)
        self._init_window()

        # Renderer — uses NPZ joint order
        self.renderer = Renderer(
            self.model, self.data,
            self.npz_to_qpos, self.npz_body_map,
        )

        # Camera
        self.camera = CameraController(self.renderer.cam)

        # Register callbacks after camera exists
        glfw.set_mouse_button_callback(self.window, self.camera.on_mouse_button)
        glfw.set_cursor_pos_callback(self.window, self.camera.on_cursor_pos)
        glfw.set_scroll_callback(self.window, self.camera.on_scroll)
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_window_size_callback(self.window, self._on_resize)

        # Apply frame 0
        self.renderer.apply_frame(self.motion, 0)

    def _init_window(self):
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")
        glfw.window_hint(glfw.SAMPLES, 4)
        title = f"Motion Editor  |  {os.path.basename(self.motion.path)} → output/"
        self.window = glfw.create_window(1400, 900, title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window. Is DISPLAY set?")

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        style = imgui.get_style()
        style.window_rounding = 4.0
        style.frame_rounding = 3.0
        style.grab_rounding = 3.0
        style.colors[imgui.COLOR_FRAME_BACKGROUND] = (0.15, 0.15, 0.17, 1.0)
        style.colors[imgui.COLOR_FRAME_BACKGROUND_HOVERED] = (0.20, 0.20, 0.22, 1.0)
        style.colors[imgui.COLOR_FRAME_BACKGROUND_ACTIVE] = (0.27, 0.27, 0.30, 1.0)
        style.colors[imgui.COLOR_SLIDER_GRAB] = (0.31, 0.57, 0.85, 1.0)
        style.colors[imgui.COLOR_SLIDER_GRAB_ACTIVE] = (0.40, 0.68, 0.95, 1.0)
        style.colors[imgui.COLOR_BUTTON] = (0.22, 0.22, 0.25, 1.0)
        style.colors[imgui.COLOR_BUTTON_HOVERED] = (0.28, 0.28, 0.32, 1.0)
        style.colors[imgui.COLOR_BUTTON_ACTIVE] = (0.35, 0.35, 0.40, 1.0)
        style.colors[imgui.COLOR_TITLE_BACKGROUND] = (0.18, 0.18, 0.20, 1.0)
        style.colors[imgui.COLOR_TITLE_BACKGROUND_ACTIVE] = (0.22, 0.22, 0.25, 1.0)

    def _on_key(self, window, key, scancode, action, mods):
        if action != glfw.PRESS:
            return
        if mods & glfw.MOD_CONTROL and key == glfw.KEY_S:
            path = self.motion.save()
            print(f"Saved → {path}")
        elif mods & glfw.MOD_CONTROL and key == glfw.KEY_Z:
            if self.motion.history_pos > 0:
                self.motion.restore_history(self.motion.history_pos - 1)
                self.renderer.apply_frame(self.motion, self.frame_idx)
        elif mods & glfw.MOD_CONTROL and key == glfw.KEY_Y:
            if self.motion.history_pos < len(self.motion.history) - 1:
                self.motion.restore_history(self.motion.history_pos + 1)
                self.renderer.apply_frame(self.motion, self.frame_idx)
        elif key == glfw.KEY_SPACE:
            self.playing = not self.playing
        elif key == glfw.KEY_RIGHT:
            self.frame_idx = min(self.motion.n_frames - 1, self.frame_idx + 1)
            self.playing = False
            self.renderer.apply_frame(self.motion, self.frame_idx)
        elif key == glfw.KEY_LEFT:
            self.frame_idx = max(0, self.frame_idx - 1)
            self.playing = False
            self.renderer.apply_frame(self.motion, self.frame_idx)
        elif key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)

    def _on_resize(self, window, w, h):
        self.window_size = (w, h)

    def run(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()

            if self.playing:
                self.frame_idx += self.speed
                if self.frame_idx >= self.motion.n_frames or self.frame_idx < 0:
                    if self.loop:
                        self.frame_idx %= self.motion.n_frames
                    else:
                        self.frame_idx = np.clip(
                            self.frame_idx, 0, self.motion.n_frames - 1
                        )
                        self.playing = False
                idx = int(round(self.frame_idx))
                self.renderer.apply_frame(self.motion, idx)

            imgui.new_frame()

            draw_timeline(self)
            draw_joint_editor(self)
            draw_root_editor(self)
            draw_tools(self)
            draw_history(self)
            draw_status_bar(self)

            fb_w, fb_h = glfw.get_framebuffer_size(self.window)
            viewport = mj.MjrRect(0, 0, fb_w, fb_h)
            self.renderer.render(viewport)

            imgui.render()
            self.impl.render(imgui.get_draw_data())
            glfw.swap_buffers(self.window)

        self.close()

    def close(self):
        self.impl.shutdown()
        glfw.destroy_window(self.window)
        glfw.terminate()
