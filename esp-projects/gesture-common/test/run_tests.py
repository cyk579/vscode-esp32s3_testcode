"""Portable host checks of production code. Requires GCC (CC may override it)."""
from pathlib import Path
import os
import subprocess

root = Path(__file__).resolve().parents[1]
build = root / "build-local" / "host-tests"
build.mkdir(parents=True, exist_ok=True)
exe = build / ("test_control.exe" if os.name == "nt" else "test_control")
sources = [root / "test/test_control.c", root / "gesture_protocol.c", root / "gesture_control.c", root / "third_party/Fusion/FusionAhrs.c"]
subprocess.run([os.environ.get("CC", "gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                "-DFUSION_USE_NORMAL_SQRT", "-I" + str(root / "include"), "-I" + str(root / "third_party/Fusion"),
                *map(str, sources), "-lm", "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)
