import mujoco as mj
from .joint_info import NPZ_JOINT_NAMES
from .collision import CollisionDetector, CollisionResult

# Tint applied to colliding geoms in the 3D view (also reveals invisible collision hulls).
COLLISION_RGBA = (0.9, 0.1, 0.1, 0.85)


class Renderer:
    def __init__(self, model, data, npz_to_qpos, npz_body_map):
        self.model = model
        self.data = data
        self.npz_to_qpos = npz_to_qpos
        self.npz_body_map = npz_body_map
        self.nz_jidx = {n: i for i, n in enumerate(NPZ_JOINT_NAMES)}

        self.detector = CollisionDetector(model)
        self.collision = CollisionResult.empty()

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

    def _write_qpos(self, data, motion, frame):
        """Write a motion frame's root + joint pose into `data.qpos` (no FK)."""
        idx = min(int(frame), motion.n_frames - 1)
        data.qpos[:3] = motion.root_pos[idx]
        data.qpos[3:7] = motion.root_rot[idx]
        for name in NPZ_JOINT_NAMES:
            qadr = self.npz_to_qpos.get(name)
            if qadr is not None:
                data.qpos[qadr] = motion.joint_pos[idx, self.nz_jidx[name]]

    def apply_frame(self, motion, frame):
        self._write_qpos(self.data, motion, frame)
        mj.mj_forward(self.model, self.data)
        self.collision = self.detector.detect(self.data)

    def render(self, viewport):
        mj.mjv_updateScene(
            self.model, self.data, self.opt, self.pert,
            self.cam, mj.mjtCatBit.mjCAT_ALL, self.scene,
        )
        ids = self.collision.geom_ids
        if ids:
            geom_obj = int(mj.mjtObj.mjOBJ_GEOM)
            for i in range(self.scene.ngeom):
                g = self.scene.geoms[i]
                if g.objtype == geom_obj and g.objid in ids:
                    g.rgba[:] = COLLISION_RGBA
                    g.emission = 0.4
        mj.mjr_render(viewport, self.scene, self.context)
