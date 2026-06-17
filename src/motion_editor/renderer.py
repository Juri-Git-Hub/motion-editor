import mujoco as mj
from .joint_info import NPZ_JOINT_NAMES


class Renderer:
    def __init__(self, model, data, npz_to_qpos, npz_body_map):
        self.model = model
        self.data = data
        self.npz_to_qpos = npz_to_qpos
        self.npz_body_map = npz_body_map
        self.nz_jidx = {n: i for i, n in enumerate(NPZ_JOINT_NAMES)}

        self.scene = mj.MjvScene(model, maxgeom=1000)
        self.context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
        self.cam = mj.MjvCamera()
        mj.mjv_defaultCamera(self.cam)
        self.cam.distance = 3.0
        self.cam.elevation = -10
        self.cam.azimuth = -90
        self.cam.lookat[:] = [0.0, 0.0, 0.7]
        self.opt = mj.MjvOption()
        mj.mjv_defaultOption(self.opt)
        self.pert = mj.MjvPerturb()
        mj.mjv_defaultPerturb(self.pert)

    def apply_frame(self, motion, frame):
        idx = min(int(frame), motion.n_frames - 1)
        self.data.qpos[:3] = motion.root_pos[idx]
        self.data.qpos[3:7] = motion.root_rot[idx]
        for name in NPZ_JOINT_NAMES:
            qadr = self.npz_to_qpos.get(name)
            if qadr is not None:
                self.data.qpos[qadr] = motion.joint_pos[idx, self.nz_jidx[name]]
        mj.mj_forward(self.model, self.data)

    def render(self, viewport):
        mj.mjv_updateScene(
            self.model, self.data, self.opt, self.pert,
            self.cam, mj.mjtCatBit.mjCAT_ALL, self.scene,
        )
        mj.mjr_render(viewport, self.scene, self.context)
