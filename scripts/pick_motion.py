#!/usr/bin/env python3
"""
Pick a motion .npz file from a folder.
Tries tkinter GUI first, falls back to terminal numbered list.
"""

import sys
from pathlib import Path


def terminal_picker(folder):
    files = sorted(Path(folder).glob("*.npz"))
    if not files:
        print(f"No .npz files found in {folder}", file=sys.stderr)
        return None
    print("Motion files:")
    for i, f in enumerate(files):
        suffix = ""
        if (f.parent / (f.stem + '_original.npz')).exists():
            suffix = "  (has backup)"
        print(f"  [{i}] {f.name}{suffix}")
    print()
    while True:
        try:
            idx = int(input("Choose file number: ").strip())
            if 0 <= idx < len(files):
                return str(files[idx])
        except (ValueError, IndexError):
            pass
        print(f"Enter 0-{len(files) - 1}")


def gui_picker(folder):
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        path = filedialog.askopenfilename(
            title="Select motion file",
            initialdir=folder,
            filetypes=[("NPZ motion files", "*.npz"), ("All files", "*.*")],
        )
        root.destroy()
        return path if path else None
    except Exception:
        return None


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else str(Path.home() / "motion-editor" / "motions")
    path = gui_picker(folder)
    if not path:
        path = terminal_picker(folder)
    if path:
        print(path)
    else:
        sys.exit(1)
