"""Self-collision detection from MuJoCo contacts.

The K1 model already carries active collision geometry (shank cylinder+box, arm
cylinders, hip cylinders, 12+12 CoACD hand pieces), so every `mj_forward` already
computes self-contacts. This module reads `data.contact`, drops ground/ball contacts
and shallow grazing, and packages what's left for the viewer tint, the side panel,
and the timeline markers.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco as mj

# Only report penetration deeper than this (m). MuJoCo's contact margin is 0, so a
# contact already implies dist < 0; this extra threshold filters grazing noise where
# adjacent collision hulls just kiss in otherwise-valid poses.
PEN_THRESH = 1e-3


@dataclass
class CollisionResult:
    geom_ids: set = field(default_factory=set)          # colliding geom ids (for 3D tint)
    pairs: dict = field(default_factory=dict)           # (bodyA, bodyB) -> worst depth (m, +ve)
    worst: tuple | None = None                          # (depth_m, bodyA, bodyB)
    n: int = 0                                          # qualifying contact count

    @property
    def colliding(self) -> bool:
        return self.n > 0

    @classmethod
    def empty(cls) -> "CollisionResult":
        return cls()


class CollisionDetector:
    def __init__(self, model):
        self.model = model
        self._ignore = set()
        for name in ("ground", "ball_geom"):
            gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if gid >= 0:
                self._ignore.add(gid)

    def body_label(self, gid: int) -> str:
        bid = self.model.geom_bodyid[gid]
        return mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, bid) or f"body{bid}"

    def _qualifying(self, data):
        """Yield (geom1, geom2, dist) for self-contacts past the threshold."""
        for c in range(data.ncon):
            con = data.contact[c]
            g1, g2 = int(con.geom1), int(con.geom2)
            if g1 in self._ignore or g2 in self._ignore:
                continue
            if con.dist > -PEN_THRESH:
                continue
            yield g1, g2, float(con.dist)

    def any_collision(self, data) -> bool:
        for _ in self._qualifying(data):
            return True
        return False

    def detect(self, data) -> CollisionResult:
        res = CollisionResult()
        for g1, g2, dist in self._qualifying(data):
            res.n += 1
            res.geom_ids.add(g1)
            res.geom_ids.add(g2)
            depth = -dist
            b1, b2 = self.body_label(g1), self.body_label(g2)
            pair = (b1, b2) if b1 <= b2 else (b2, b1)
            if depth > res.pairs.get(pair, 0.0):
                res.pairs[pair] = depth
            if res.worst is None or depth > res.worst[0]:
                res.worst = (depth, pair[0], pair[1])
        return res
