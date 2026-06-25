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
    def __init__(self, xml_path, motion_path, ball_pos=None):
        # Load model (inject physics ball if requested)
        if ball_pos is not None:
            spec = mj.MjSpec.from_file(str(xml_path))
            ball_body = spec.worldbody.add_body(name='ball', pos=list(ball_pos))
            fj = ball_body.add_freejoint()
            fj.name = 'ball_joint'
            bg = ball_body.add_geom()
            bg.name = 'ball_geom'
            bg.type = mj.mjtGeom.mjGEOM_SPHERE
            bg.size[0] = 0.1
            bg.mass = 0.5
            bg.rgba[:] = [1.0, 0.4, 0.0, 1.0]
            self.model = spec.compile()
            ball_jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'ball_joint')
            self.robot_nq = int(self.model.jnt_qposadr[ball_jid])
            self.robot_nv = int(self.model.jnt_dofadr[ball_jid])
            self.has_ball = True
            self.ball_enabled = True
            self.ball_radius = 0.1
        else:
            self.model = mj.MjModel.from_xml_path(str(xml_path))
            self.robot_nq = self.model.nq
            self.robot_nv = self.model.nv
            self.has_ball = False
            self.ball_enabled = False
            self.ball_radius = 0.1

        self.data = mj.MjData(self.model)
        mj.mj_forward(self.model, self.data)

        if self.has_ball:
            self.ball_geom_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, 'ball_geom')
            self.ball_body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, 'ball')

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

        # Keyframe drag (timeline): (old_frame, target_frame) while dragging.
        self.kf_drag = None

        # Timeline zoom: visible frame window [view_lo, view_hi] (floats).
        self.view_lo = 0.0
        self.view_hi = float(max(1, self.motion.n_frames - 1))

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
            # Mirror across the sagittal plane: Roll (fore-aft axis) and Yaw
            # (vertical axis) joints flip sign, Pitch (lateral axis) keeps it.
            # The joint limits are an unreliable signal — symmetric ranges like
            # [-x, x] (common for Roll/Yaw) made the old range heuristic force
            # sign=+1, so mirroring copied the value without flipping. Derive
            # the sign from the joint name suffix instead.
            name_l = NPZ_JOINT_NAMES[li].lower()
            if name_l.endswith('_roll') or name_l.endswith('_yaw') or name_l.endswith('yaw'):
                sign = -1
            elif name_l.endswith('_pitch') or name_l.endswith('pitch'):
                sign = 1
            else:
                # Fallback: infer from limits (mirrored ranges → flip).
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

        # Ground lock: when enabled, shift root Z after edits so the lower
        # foot's lowest point rests on the floor (z=0). Kinematic playback has
        # no real gravity, so this is what keeps edited feet on the ground.
        self.ground_lock_enabled = False
        self._build_foot_geoms()

        # Camera
        self.camera = CameraController(self.renderer.cam)

        # Register callbacks after camera exists
        glfw.set_mouse_button_callback(self.window, self.camera.on_mouse_button)
        glfw.set_cursor_pos_callback(self.window, self.camera.on_cursor_pos)
        glfw.set_scroll_callback(self.window, self._on_scroll)
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_window_size_callback(self.window, self._on_resize)

        # Apply frame 0
        self.renderer.apply_frame(self.motion, 0)

    def _build_foot_geoms(self):
        """Cache foot geoms and their local corner/vertex points for fast
        lowest-point queries during ground lock."""
        self._foot_geoms = []  # list of (geom_id, local_points Nx3)
        box_corners = np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
            dtype=np.float64,
        )
        foot_bodies = ['left_foot_link', 'right_foot_link']
        foot_bids = set()
        for b in foot_bodies:
            bid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, b)
            if bid >= 0:
                foot_bids.add(bid)
        for g in range(self.model.ngeom):
            if self.model.geom_bodyid[g] not in foot_bids:
                continue
            gtype = int(self.model.geom_type[g])
            if gtype == int(mj.mjtGeom.mjGEOM_MESH):
                mid = self.model.geom_dataid[g]
                a = self.model.mesh_vertadr[mid]
                n = self.model.mesh_vertnum[mid]
                pts = np.asarray(self.model.mesh_vert[a:a + n], dtype=np.float64)
            elif gtype == int(mj.mjtGeom.mjGEOM_BOX):
                pts = box_corners * self.model.geom_size[g]
            else:
                continue
            self._foot_geoms.append((g, pts))

    def _lowest_foot_z(self):
        """Min world z over all foot geom points. Assumes mj_forward ran."""
        zmin = np.inf
        for g, pts in self._foot_geoms:
            R = self.data.geom_xmat[g].reshape(3, 3)
            world = self.data.geom_xpos[g] + pts @ R.T
            zmin = min(zmin, float(world[:, 2].min()))
        return zmin if np.isfinite(zmin) else None

    def lock_ground(self, frame):
        """Shift root Z at `frame` so the lower foot sits on the floor (z=0)."""
        if not self._foot_geoms:
            return
        f = int(frame)
        # FK current pose into self.data, measure, then correct root height.
        self.renderer.apply_frame(self.motion, f)
        z = self._lowest_foot_z()
        if z is not None and abs(z) > 1e-9:
            self.motion.root_pos[f, 2] -= z
            self.motion.dirty_frames.add(f)

    def lock_ground_range(self, start, end):
        """Ground-lock every frame in [start, end] as one undoable action."""
        if not self._foot_geoms:
            return
        lo, hi = sorted((int(start), int(end)))
        lo = max(0, lo)
        hi = min(self.motion.n_frames - 1, hi)
        if hi < lo:
            return
        self.motion.record_history(f"Ground lock {lo}-{hi}")
        for f in range(lo, hi + 1):
            self.lock_ground(f)
        self.renderer.apply_frame(self.motion, self.frame_idx)
        print(f"Ground locked frames {lo}-{hi}")

    def commit_frame(self, frame):
        """Apply ground lock (if enabled) then render the frame. Edit handlers
        call this instead of renderer.apply_frame so edits keep feet grounded."""
        if self.ground_lock_enabled:
            self.lock_ground(frame)
        self.renderer.apply_frame(self.motion, frame)

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
        # This callback replaced imgui's GLFW key callback, so forward events to
        # imgui first — otherwise text inputs never see Enter/Backspace/nav keys.
        self.impl.keyboard_callback(window, key, scancode, action, mods)
        # When a text field (e.g. joint numeric entry) is focused, let imgui own
        # the keyboard and skip editor shortcuts.
        if imgui.get_io().want_capture_keyboard:
            return
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

    def _on_scroll(self, window, x_offset, y_offset):
        # This callback replaced imgui's GLFW scroll callback, so forward the
        # wheel to imgui first (it can't poll wheel events) — otherwise
        # io.mouse_wheel stays 0 and timeline zoom never fires. Camera.on_scroll
        # ignores scroll when imgui wants the mouse, so the two don't fight.
        self.impl.scroll_callback(window, x_offset, y_offset)
        self.camera.on_scroll(window, x_offset, y_offset)

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

                if self.ball_enabled:
                    robot_qpos = self.data.qpos[:self.robot_nq].copy()
                    for _ in range(16):
                        self.data.qpos[:self.robot_nq] = robot_qpos
                        self.data.qvel[:self.robot_nv] = 0.0
                        mj.mj_step(self.model, self.data)
                    self.data.qpos[:self.robot_nq] = robot_qpos
                    mj.mj_forward(self.model, self.data)

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
