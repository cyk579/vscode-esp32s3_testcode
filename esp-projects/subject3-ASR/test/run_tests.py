"""Host tests only; no ESP-IDF installation or hardware access."""
from pathlib import Path
import os
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
build = root / "build-local" / "host-tests"
build.mkdir(parents=True, exist_ok=True)
exe = build / ("test_pcm_level.exe" if os.name == "nt" else "test_pcm_level")
subprocess.run([
    os.environ.get("CC", "gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
    "-I" + str(root / "main"), str(root / "main/pcm_level.c"),
    str(root / "test/test_pcm_level.c"), "-o", str(exe),
], check=True)
subprocess.run([str(exe)], check=True)
media_exe = build / ("test_media.exe" if os.name == "nt" else "test_media")
subprocess.run([
    os.environ.get("CC", "gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
    "-I" + str(root / "main"), str(root / "main/media_protocol.c"),
    str(root / "main/wav_reader.c"), str(root / "test/test_media.c"), "-o", str(media_exe),
], check=True)
subprocess.run([str(media_exe)], check=True)
subprocess.run([sys.executable, str(root / "test/test_prepare_music.py")], check=True)
