"""Round-trip test for ``MotionData.save()``.

Guards the two save() regressions fixed 2026-10-20:

1. ``joint_names``/``body_names`` were written with ``dtype=object``, so every
   consumer that loads without ``allow_pickle``
   (``rl_core/datasets/motion_npz.py``) raises ``ValueError`` on the name
   arrays.
2. ``body_ang_vel_w`` was computed as a BODY-frame rotation vector
   (``r0.inv() * r1``); the key is world-frame everywhere downstream. The fix
   uses the world-frame product ``r1 * r0.inv()``.

The test mirrors the consumers: reload the saved file with plain ``np.load``
(no ``allow_pickle``), then check the name dtypes, the ``ang_vel_frame`` tag,
and the ω frame via the same finite-difference test throwin_motion uses
(``scripts/_motion_fd.py``). The input clip is synthesized in a temp dir so
the test does not depend on the (server-synced) ``motions/`` directory.
"""

import os
import tempfile

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from motion_editor.joint_info import NPZ_BODY_NAMES, NPZ_JOINT_NAMES
from motion_editor.motion_data import MotionData

_AXIS_EPS = 1e-9


def _make_clip(path: str) -> None:
    """Synthetic 40-frame clip: root precesses (z-rate 3.0, y-tilt 0.9 rad),
    so the world-frame and body-frame ω genuinely differ."""
    fps = 50
    t = np.arange(40) / fps
    quats = np.stack([R.from_euler("z", 3.0 * ti).as_quat()  # xyzw
                      for ti in t])
    quats = R.from_quat(quats) * R.from_euler("y", 0.9)
    quats_wxyz = quats.as_quat()[:, [3, 0, 1, 2]]  # wxyz, as body_quat_w

    j, b = len(NPZ_JOINT_NAMES), len(NPZ_BODY_NAMES)
    body_quat_w = np.broadcast_to(quats_wxyz[:, None, :], (40, b, 4)).copy()
    np.savez_compressed(
        path,
        joint_names=np.array(NPZ_JOINT_NAMES),
        body_names=np.array(NPZ_BODY_NAMES),
        fps=np.array([fps], dtype=np.int64),
        joint_pos=np.zeros((40, j), dtype=np.float32),
        joint_vel=np.zeros((40, j), dtype=np.float32),
        body_pos_w=np.zeros((40, b, 3), dtype=np.float32),
        body_quat_w=body_quat_w.astype(np.float32),
        body_lin_vel_w=np.zeros((40, b, 3), dtype=np.float32),
        body_ang_vel_w=np.zeros((40, b, 3), dtype=np.float32),
    )


def _fd(q, fps, world):
    """World/body-frame FD angular velocity, mirroring _motion_fd._fd."""
    q = np.asarray(q, dtype=np.float64)
    out = np.zeros(q.shape[:-1] + (3,), dtype=np.float64)
    q0c, q1c = q[:-2], q[2:]
    dot = np.sum(q0c * q1c, axis=-1, keepdims=True)
    q1c = np.where(dot < 0, -q1c, q1c)
    w0, v0 = q0c[..., :1], q0c[..., 1:]
    w1, v1 = q1c[..., :1], q1c[..., 1:]
    dq = np.empty_like(q0c)
    dq[..., :1] = w1 * w0 + np.sum(v1 * v0, axis=-1, keepdims=True)
    if world:
        dq[..., 1:] = -w1 * v0 + w0 * v1 + np.cross(v0, v1)
    else:
        dq[..., 1:] = -w1 * v0 + w0 * v1 + np.cross(v1, v0)
    angle = 2 * np.arctan2(np.linalg.norm(dq[..., 1:], axis=-1),
                           np.clip(dq[..., 0], -1, 1))
    axis = dq[..., 1:] / (np.linalg.norm(dq[..., 1:], axis=-1,
                                         keepdims=True) + _AXIS_EPS)
    out[1:-1] = axis * angle[..., None] * (fps / 2.0)
    out[0], out[-1] = out[1], out[-2]
    return out


def _residuals(q, w, fps):
    q = np.asarray(q, dtype=np.float64)
    w = np.asarray(w, dtype=np.float64)
    ew = np.linalg.norm(w[1:-1] - _fd(q, fps, True)[1:-1], axis=-1)
    eb = np.linalg.norm(w[1:-1] - _fd(q, fps, False)[1:-1], axis=-1)
    return float(np.median(ew)), float(np.median(eb))


def test_save_roundtrip():
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "src.npz")
        _make_clip(src)
        md = MotionData(src)
        out = os.path.join(tmp, "out.npz")
        md.save(out)

        # Consumer-style load: no allow_pickle. The names must be READ too —
        # numpy raises the object-array ValueError lazily on key access.
        d = np.load(out)
        try:
            joint_names = d["joint_names"]
            body_names = d["body_names"]
        except ValueError as exc:  # pragma: no cover
            pytest.fail(f"consumer np.load rejected the name arrays: {exc}")

        assert joint_names.dtype.kind == "U", joint_names.dtype
        assert body_names.dtype.kind == "U", body_names.dtype
        assert str(np.asarray(d["ang_vel_frame"])) == "world"
        assert d["fps"].dtype == np.int64

        fps = float(np.asarray(d["fps"]).reshape(-1)[0])
        ew, eb = _residuals(d["body_quat_w"], d["body_ang_vel_w"], fps)
        assert ew < 1e-4, f"ω not world-frame: world residual {ew:.3e}"
        assert eb > 1e-2, (f"ω matches both frames? "
                           f"world {ew:.3e}, body {eb:.3e}")
