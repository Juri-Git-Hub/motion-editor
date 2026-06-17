# Motion Editor

GUI editor for retargeted robot motions. View, edit, and refine motion capture data applied to MuJoCo robot models.

## Requirements

- Python >= 3.10
- [MuJoCo](https://github.com/google-deepmind/mujoco) (installed via pip)
- A robot model XML (see [GMR assets](#gmr-assets))

## Installation

```bash
git clone https://github.com/Juri-Git-Hub/motion-editor.git
cd motion-editor

python3 -m venv .venv
source .venv/bin/activate

pip install --editable .
```

## Usage

```bash
./run.sh --robot booster_k1
```

Select a motion from the numbered list when prompted. The script syncs motion
files from a remote server, installs the package automatically into a local
venv, and launches the editor.

| Flag            | Description                           |
|-----------------|---------------------------------------|
| `--server`      | Remote host to sync motions from      |
| `--robot`       | Robot type (default: `booster_k1`)   |
| `--sync-back`   | Sync edited motion back to server     |
| `--no-sync`     | Skip motion file sync                 |

You can also run the CLI directly:

```bash
motion-editor --robot booster_k1 --motion motions/k1_fight_001.npz
```

If `--xml` is provided it takes precedence; otherwise the robot name is used to
look up the XML in `~/GMR/` or the `GMR_ROOT` environment variable.

### Controls

| Key / Action        | Function                      |
|---------------------|-------------------------------|
| `Space`             | Play / Pause                  |
| `←` `→`            | Step frame                    |
| `Ctrl+S`            | Save                          |
| `Ctrl+Z`            | Undo                          |
| `Ctrl+Y`            | Redo                          |
| LMB drag            | Orbit                         |
| MMB drag            | Pan XY                        |
| RMB drag            | Pan Z                         |
| Scroll              | Zoom                          |
| Right-click joint   | Reset joint to original       |

### GMR Assets

Robot XML files are expected under `~/GMR/` structured as:

```
~/GMR/assets/
├── booster_k1/
│   └── K1_serial.xml
├── unitree_g1/
│   └── g1_mocap_29dof.xml
└── ...
```

Set `GMR_ROOT` to point elsewhere if your assets live in a different location.

## Project Structure

```
motion-editor/
├── motions/              # Motion files (.npz, .csv)
├── scripts/              # Utility scripts
├── src/motion_editor/    # Main Python package
│   ├── editor.py         # Core editor loop
│   ├── renderer.py       # MuJoCo rendering
│   ├── motion_data.py    # Motion data handling
│   ├── camera.py         # Camera controls
│   ├── joint_info.py     # Joint metadata
│   └── ui/               # ImGui panels
│       ├── timeline.py
│       ├── joint_editor.py
│       ├── root_editor.py
│       ├── tools.py
│       ├── history_panel.py
│       └── status_bar.py
├── pyproject.toml
└── environment.yaml
```
