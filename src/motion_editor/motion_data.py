import os
import pickle
import shutil
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

from .joint_info import NPZ_JOINT_NAMES, NPZ_BODY_NAMES

# Cap on undo history entries; each entry deep-copies full joint/root arrays.
MAX_HISTORY = 200


class MotionData:
    def __init__(self, path, model=None, npz_to_qpos=None, npz_body_map=None):
        self.path = str(path)
        self.model = model
        self.npz_to_qpos = npz_to_qpos
        self.npz_body_map = npz_body_map
        self._load(path)
        self._store_originals()
        self.dirty_frames = set()
        if not hasattr(self, 'keyframes'):
            self.keyframes = set()
        self.history = []
        self.history_pos = -1
        self.record_history("Loaded")

    def _load(self, path):
        if path.endswith('.npz'):
            self._load_npz(path)
        else:
            self._load_pkl(path)

    def _load_npz(self, path):
        d = np.load(path, allow_pickle=True)
        self.format = 'npz'
        file_joint_names = [str(n) for n in d['joint_names']]
        file_body_names = [str(n) for n in d['body_names']]
        self.fps = int(d['fps'].item()) if d['fps'].ndim == 0 else int(d['fps'][0])
        self.joint_pos = d['joint_pos'].astype(np.float64)
        self.joint_vel = d['joint_vel'].astype(np.float64)
        self.body_pos_w = d['body_pos_w'].astype(np.float64)
        self.body_quat_w = d['body_quat_w'].astype(np.float64)
        self.body_lin_vel_w = d['body_lin_vel_w'].astype(np.float64)
        self.body_ang_vel_w = d['body_ang_vel_w'].astype(np.float64)

        # Some .npz files store joints/bodies in a different order than the
        # canonical NPZ_JOINT_NAMES / NPZ_BODY_NAMES. The renderer, FK and save
        # paths all index by the canonical order, so normalize column order on
        # load. Without this, reordered files render with scrambled joints.
        self.joint_names = self._normalize_columns(
            file_joint_names, NPZ_JOINT_NAMES,
            ['joint_pos', 'joint_vel'],
        )
        self.body_names = self._normalize_columns(
            file_body_names, NPZ_BODY_NAMES,
            ['body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w'],
        )

        self.n_frames = len(self.joint_pos)
        self.n_joints = self.joint_pos.shape[1]

        # Keyframes (anchor frames for interpolation), persisted if present.
        if 'keyframes' in d:
            self.keyframes = set(int(x) for x in d['keyframes'])

        # Root = trunk (first body, index 0)
        self.root_pos = self.body_pos_w[:, 0].copy()
        self.root_rot = self.body_quat_w[:, 0].copy()

        # PKL compat alias
        self.dof_pos = self.joint_pos

    def _normalize_columns(self, file_names, canonical_names, attrs):
        """Reorder axis-1 columns of the given attrs from file order to canonical
        order, matching by name. Returns the canonical name list on success, or
        the original file order (arrays untouched) if any name is missing."""
        if file_names == canonical_names:
            return list(canonical_names)
        missing = [n for n in canonical_names if n not in file_names]
        if missing:
            print(f"[motion_data] WARNING: {self.path} missing expected names "
                  f"{missing}; leaving column order untouched.")
            return list(file_names)
        src_idx = [file_names.index(n) for n in canonical_names]
        for attr in attrs:
            setattr(self, attr, getattr(self, attr)[:, src_idx])
        return list(canonical_names)

    def _load_pkl(self, path):
        # SECURITY: pickle.load executes arbitrary code on a crafted file. Only
        # load .pkl motions from trusted sources (your own server/rsync). For
        # untrusted data prefer the .npz format.
        with open(path, 'rb') as f:
            raw = pickle.load(f)
        self.format = 'pkl'
        self.fps = raw['fps']
        self.joint_names = list(NPZ_JOINT_NAMES)

        root_pos = np.asarray(raw['root_pos'], dtype=np.float64)
        root_rot = np.asarray(raw['root_rot'], dtype=np.float64)
        dof_pos = np.asarray(raw['dof_pos'], dtype=np.float64)
        self.n_frames = len(root_pos)
        self.n_joints = dof_pos.shape[1]

        # root_rot is xyzw → wxyz
        if root_rot.shape[1] == 4:
            root_rot = root_rot[:, [3, 0, 1, 2]]

        # PKL has a different joint order from NPZ.
        # PKL order (XML order) → remap to NPZ order on load.
        # We build the mapping using the model if available, otherwise identity.
        if self.model is not None:
            from .joint_info import build_editor_to_npz_map
            ed_to_npz = build_editor_to_npz_map(self.model)
            n = len(NPZ_JOINT_NAMES)
            remapped = np.zeros((self.n_frames, n), dtype=np.float64)
            for ed_i, npz_i in enumerate(ed_to_npz):
                if npz_i >= 0 and ed_i < dof_pos.shape[1]:
                    remapped[:, npz_i] = dof_pos[:, ed_i]
            self.joint_pos = remapped
        else:
            self.joint_pos = dof_pos

        self.joint_vel = np.zeros_like(self.joint_pos)
        self.root_pos = root_pos
        self.root_rot = root_rot

        # Empty body data — computed on first save
        self.body_pos_w = np.zeros((self.n_frames, len(NPZ_BODY_NAMES), 3), dtype=np.float64)
        self.body_quat_w = np.zeros((self.n_frames, len(NPZ_BODY_NAMES), 4), dtype=np.float64)
        self.body_lin_vel_w = np.zeros_like(self.body_pos_w)
        self.body_ang_vel_w = np.zeros_like(self.body_pos_w)
        self.body_names = list(NPZ_BODY_NAMES)

        # PKL compat alias
        self.dof_pos = self.joint_pos

    @property
    def output_dir(self):
        return os.path.join(os.path.dirname(self.path), 'output')

    @property
    def output_path(self):
        stem = Path(self.path).stem
        d = self.output_dir
        os.makedirs(d, exist_ok=True)
        nums = []
        for p in Path(d).glob(f"{stem}_*.npz"):
            try:
                nums.append(int(p.stem.split('_')[-1]))
            except (ValueError, IndexError):
                pass
        n = (max(nums) + 1) if nums else 1
        return os.path.join(d, f"{stem}_{n:03d}.npz")

    def _store_originals(self):
        self.orig_joint_pos = self.joint_pos.copy()
        self.orig_root_pos = self.root_pos.copy()
        self.orig_root_rot = self.root_rot.copy()

    def is_dirty(self, frame, joint_idx):
        return abs(self.joint_pos[frame, joint_idx] - self.orig_joint_pos[frame, joint_idx]) > 1e-6

    def record_history(self, description, merge_key=None):
        if merge_key is not None and self.history:
            _, last_key, *_ = self.history[-1]
            if last_key == merge_key:
                return
        del self.history[self.history_pos + 1:]
        self.history.append((
            description,
            merge_key,
            self.joint_pos.copy(),
            self.root_pos.copy(),
            self.root_rot.copy(),
        ))
        # Cap memory: drop oldest entries when over the limit.
        if len(self.history) > MAX_HISTORY:
            drop = len(self.history) - MAX_HISTORY
            del self.history[:drop]
        self.history_pos = len(self.history) - 1

    def restore_history(self, pos):
        if 0 <= pos < len(self.history):
            _, _, jp, rp, rr = self.history[pos]
            self.joint_pos[:] = jp
            self.root_pos[:] = rp
            self.root_rot[:] = rr
            self.history_pos = pos
            self.dirty_frames = set(range(self.n_frames))

    def reset_frame(self, frame):
        self.joint_pos[frame] = self.orig_joint_pos[frame]
        self.root_pos[frame] = self.orig_root_pos[frame]
        self.root_rot[frame] = self.orig_root_rot[frame]
        self.dirty_frames.discard(frame)

    def reset_all(self):
        self.joint_pos[:] = self.orig_joint_pos
        self.root_pos[:] = self.orig_root_pos
        self.root_rot[:] = self.orig_root_rot
        self.dirty_frames.clear()

    @property
    def modified(self):
        return len(self.dirty_frames) > 0

    def recompute_fk(self, model, npz_to_qpos, npz_body_map):
        """Recompute body_pos_w and body_quat_w from joint_pos via MuJoCo FK."""
        import mujoco as mj
        data = mj.MjData(model)
        nz_jidx = {n: i for i, n in enumerate(NPZ_JOINT_NAMES)}
        nz_bidx = NPZ_BODY_NAMES

        for f in range(self.n_frames):
            # Set trunk (root) pose
            data.qpos[:3] = self.root_pos[f]
            data.qpos[3:7] = self.root_rot[f]  # wxyz
            # Set joint positions
            for name in NPZ_JOINT_NAMES:
                qadr = npz_to_qpos.get(name)
                if qadr is not None:
                    data.qpos[qadr] = self.joint_pos[f, nz_jidx[name]]
            mj.mj_forward(model, data)

            for b, bname in enumerate(nz_bidx):
                bid = npz_body_map.get(bname)
                if bid is not None:
                    self.body_pos_w[f, b] = data.xpos[bid]
                    self.body_quat_w[f, b] = data.xquat[bid]  # wxyz

    def recompute_velocities(self):
        """Finite-difference velocities from joint_pos and body poses."""
        dt = 1.0 / self.fps
        n = self.n_frames
        if n < 3:
            return

        for f in range(1, n - 1):
            self.joint_vel[f] = (self.joint_pos[f + 1] - self.joint_pos[f - 1]) / (2 * dt)
            self.body_lin_vel_w[f] = (self.body_pos_w[f + 1] - self.body_pos_w[f - 1]) / (2 * dt)
            # Angular velocity: approximate from quaternion difference
            for b in range(len(NPZ_BODY_NAMES)):
                q0 = self.body_quat_w[f - 1, b]  # wxyz
                q1 = self.body_quat_w[f + 1, b]
                r0 = R.from_quat([q0[1], q0[2], q0[3], q0[0]])
                r1 = R.from_quat([q1[1], q1[2], q1[3], q1[0]])
                rel = r0.inv() * r1
                rotvec = rel.as_rotvec()
                self.body_ang_vel_w[f, b] = rotvec / (2 * dt)

        # Fill endpoints
        self.joint_vel[0] = self.joint_vel[1]
        self.joint_vel[-1] = self.joint_vel[-2]
        self.body_lin_vel_w[0] = self.body_lin_vel_w[1]
        self.body_lin_vel_w[-1] = self.body_lin_vel_w[-2]
        self.body_ang_vel_w[0] = self.body_ang_vel_w[1]
        self.body_ang_vel_w[-1] = self.body_ang_vel_w[-2]

    def save(self, path=None):
        if path is None:
            path = self.output_path
        path = str(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)

        # Recompute FK and velocities before saving
        if self.model is not None and self.npz_to_qpos is not None and self.npz_body_map is not None:
            self.recompute_fk(self.model, self.npz_to_qpos, self.npz_body_map)
        self.recompute_velocities()

        # Update trunk from root
        self.body_pos_w[:, 0] = self.root_pos
        self.body_quat_w[:, 0] = self.root_rot

        # Backup the original source file once (output paths are versioned and
        # never overwrite the source, so guarding only `path` left no backup at
        # all). run.sh --sync-back expects "<source>.bak".
        src_bak = self.path + '.bak'
        if os.path.exists(self.path) and not os.path.exists(src_bak):
            shutil.copy2(self.path, src_bak)
        # Also back up the target if we are overwriting an existing file.
        if Path(path).exists():
            shutil.copy2(path, path + '.bak')

        np.savez_compressed(path,
            joint_pos=self.joint_pos.astype(np.float32),
            joint_vel=self.joint_vel.astype(np.float32),
            body_pos_w=self.body_pos_w.astype(np.float32),
            body_quat_w=self.body_quat_w.astype(np.float32),
            body_lin_vel_w=self.body_lin_vel_w.astype(np.float32),
            body_ang_vel_w=self.body_ang_vel_w.astype(np.float32),
            joint_names=np.array(NPZ_JOINT_NAMES, dtype=object),
            body_names=np.array(NPZ_BODY_NAMES, dtype=object),
            fps=np.array([self.fps], dtype=np.int64),
            keyframes=np.array(sorted(self.keyframes), dtype=np.int64),
        )
        self.dirty_frames.clear()
        return path
