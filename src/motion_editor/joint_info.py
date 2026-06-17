import numpy as np

NPZ_JOINT_NAMES = [
    'AAHead_yaw', 'ALeft_Shoulder_Pitch', 'ARight_Shoulder_Pitch',
    'Left_Hip_Pitch', 'Right_Hip_Pitch', 'Head_pitch',
    'Left_Shoulder_Roll', 'Right_Shoulder_Roll', 'Left_Hip_Roll', 'Right_Hip_Roll',
    'Left_Elbow_Pitch', 'Right_Elbow_Pitch', 'Left_Hip_Yaw', 'Right_Hip_Yaw',
    'Left_Elbow_Yaw', 'Right_Elbow_Yaw',
    'Left_Knee_Pitch', 'Right_Knee_Pitch',
    'Left_Ankle_Pitch', 'Right_Ankle_Pitch',
    'Left_Ankle_Roll', 'Right_Ankle_Roll',
]

NPZ_BODY_NAMES = [
    'Trunk', 'Head_1', 'Left_Arm_1', 'Right_Arm_1', 'Left_Hip_Pitch', 'Right_Hip_Pitch',
    'Head_2', 'Left_Arm_2', 'Right_Arm_2', 'Left_Hip_Roll', 'Right_Hip_Roll',
    'Left_Arm_3', 'Right_Arm_3', 'Left_Hip_Yaw', 'Right_Hip_Yaw',
    'left_hand_link', 'right_hand_link',
    'Left_Shank', 'Right_Shank', 'Left_Ankle_Cross', 'Right_Ankle_Cross',
    'left_foot_link', 'right_foot_link',
]


def _short_name(name):
    for prefix in ['ALeft_', 'ARight_', 'Left_', 'Right_', 'AA']:
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _group(name):
    if 'Head' in name:
        return 0, 'Head'
    if 'Left_Arm' in name or 'Left_Shoulder' in name or 'Left_Elbow' in name or 'ALeft' in name:
        return 1, 'Left Arm'
    if 'Right_Arm' in name or 'Right_Shoulder' in name or 'Right_Elbow' in name or 'ARight' in name:
        return 2, 'Right Arm'
    if 'Left_Hip' in name or 'Left_Knee' in name or 'Left_Ankle' in name:
        return 3, 'Left Leg'
    if 'Right_Hip' in name or 'Right_Knee' in name or 'Right_Ankle' in name:
        return 4, 'Right Leg'
    return 5, 'Other'


def build_joint_info(model):
    info = []
    skip = {'world_joint', 'ball_joint'}
    for j in range(model.njnt):
        name = model.joint(j).name
        if name in skip:
            continue
        qpos_adr = model.jnt_qposadr[j]
        limited = model.jnt_limited[j] == 1
        if limited:
            lo, hi = float(model.jnt_range[j][0]), float(model.jnt_range[j][1])
        else:
            lo, hi = -np.pi, np.pi
        info.append({
            'name': name,
            'short': _short_name(name),
            'group': _group(name),
            'qpos_adr': qpos_adr,
            'limit_lo': lo,
            'limit_hi': hi,
        })
    return info


def build_npz_joint_map(model):
    """Build mapping from NPZ joint name → qpos address in the MuJoCo model."""
    name_to_jid = {}
    for j in range(model.njnt):
        name_to_jid[model.joint(j).name] = j
    result = {}
    for name in NPZ_JOINT_NAMES:
        if name in name_to_jid:
            jid = name_to_jid[name]
            result[name] = model.jnt_qposadr[jid]
    return result


def build_editor_to_npz_map(model):
    """Return list: for each editor slider (XML order), the index into NPZ joint_pos."""
    npz_lookup = {name: i for i, name in enumerate(NPZ_JOINT_NAMES)}
    mapping = []
    skip = {'world_joint', 'ball_joint'}
    for j in range(model.njnt):
        name = model.joint(j).name
        if name in skip:
            continue
        mapping.append(npz_lookup.get(name, -1))
    return mapping


def build_npz_body_map(model):
    """Map NPZ body names → MuJoCo body IDs."""
    return {
        name: mj_name2id(model, name)
        for name in NPZ_BODY_NAMES
    }


def mj_name2id(model, name):
    """Look up MuJoCo body ID by name."""
    import mujoco as mj
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)


def build_symmetry_pairs():
    """Return [(left_npz_idx, right_npz_idx)] for left/right joint pairs."""
    name_to_idx = {n: i for i, n in enumerate(NPZ_JOINT_NAMES)}
    pairs = []
    for i, name in enumerate(NPZ_JOINT_NAMES):
        if name.startswith('Left_'):
            suffix = name[5:]
            right_name = 'Right_' + suffix
            if right_name in name_to_idx:
                pairs.append((i, name_to_idx[right_name]))
        elif name.startswith('ALeft_'):
            suffix = name[6:]
            right_name = 'ARight_' + suffix
            if right_name in name_to_idx:
                pairs.append((i, name_to_idx[right_name]))
    return pairs
