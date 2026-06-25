"""
Motion Editor — view and edit retargeted robot motions.

Usage:
    motion-editor --robot booster_k1 --motion /path/to/motion.npz
    motion-editor --motion /path/to/motion.npz
"""

import argparse
import os
import sys


# Local robot assets shipped with the repo (synced by run.sh). Override with
# MOTION_EDITOR_ASSETS_DIR if assets live elsewhere.
ROBOT_ASSETS_DIR = os.environ.get(
    "MOTION_EDITOR_ASSETS_DIR",
    os.path.join(os.path.dirname(__file__), "..", "..", "assets", "robots"),
)


def _resolve_xml(robot):
    robot_assets = {
        "booster_k1": "K1/K1_22dof.xml",
    }
    if robot in robot_assets:
        path = os.path.join(ROBOT_ASSETS_DIR, robot_assets[robot])
        if os.path.exists(path):
            return path

    # GMR fallback. Roots come from env (GMR_ROOT / ~/GMR); no hardcoded paths.
    candidates = []
    gmr_root = os.environ.get("GMR_ROOT")
    if gmr_root:
        candidates.append(gmr_root)
    candidates.append(os.path.expanduser("~/GMR"))

    known = {
        "booster_k1": "assets/booster_k1/K1_serial.xml",
        "booster_t1": "assets/booster_t1/T1_serial.xml",
        "booster_t1_29dof": "assets/booster_t1_29dof/t1_mocap.xml",
        "unitree_g1": "assets/unitree_g1/g1_mocap_29dof.xml",
        "unitree_h1": "assets/unitree_h1/h1.xml",
        "unitree_h1_2": "assets/unitree_h1_2/h1_2_handless.xml",
        "stanford_toddy": "assets/stanford_toddy/toddy_mocap.xml",
        "fourier_n1": "assets/fourier_n1/n1_mocap.xml",
        "engineai_pm01": "assets/engineai_pm01/pm_v2.xml",
        "kuavo_s45": "assets/kuavo_s45/biped_s45_collision.xml",
        "hightorque_hi": "assets/hightorque_hi/hi_25dof.xml",
        "galaxea_r1pro": "assets/galaxea_r1pro/r1_pro.xml",
    }

    if robot not in known:
        return None

    rel = known[robot]
    for root in candidates:
        path = os.path.join(root, rel)
        if os.path.exists(path):
            return path
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Motion Editor — view and edit retargeted robot motions"
    )
    parser.add_argument("--robot", type=str, default="booster_k1",
                        help="Robot type (auto-resolves XML from GMR assets)")
    parser.add_argument("--xml", type=str, default=None,
                        help="MuJoCo XML path (overrides --robot lookup)")
    parser.add_argument("--motion", type=str, required=True,
                        help="Path to the motion file (.npz or .pkl)")
    parser.add_argument("--ball", action="store_true",
                        help="Add a physics ball to test pickup motions")
    parser.add_argument("--ball-pos", type=float, nargs=3, default=[0.0, 0.0, 0.2],
                        metavar=("X", "Y", "Z"),
                        help="Ball center position (default: 0 0 0.2)")

    args = parser.parse_args()

    if args.xml:
        xml_path = args.xml
    else:
        xml_path = _resolve_xml(args.robot)
        if xml_path is None:
            print(f"Could not resolve XML for '{args.robot}'.")
            print("Provide --xml, or run ./run.sh to sync robot assets from server.")
            return 1
        print(f"XML: {xml_path}")

    if not os.path.exists(xml_path):
        print(f"XML not found: {xml_path}")
        return 1
    if not os.path.exists(args.motion):
        print(f"Motion not found: {args.motion}")
        return 1

    print(f"Motion: {args.motion}")
    print("Controls:")
    print("  Space         Play/Pause      Ctrl+S    Save")
    print("  ← →           Step frame      Ctrl+Z    Undo")
    print("  LMB drag      Orbit            Ctrl+Y    Redo")
    print("  MMB drag      Pan XY")
    print("  RMB drag      Pan Z")
    print("  Scroll        Zoom")
    print("  Right-click joint → reset to original")

    from .editor import Editor
    ball_pos = args.ball_pos if args.ball else None
    editor = Editor(xml_path=xml_path, motion_path=args.motion, ball_pos=ball_pos)
    try:
        editor.run()
    except KeyboardInterrupt:
        pass
    finally:
        editor.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
