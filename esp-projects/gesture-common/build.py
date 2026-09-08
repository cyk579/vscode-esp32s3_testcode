"""Build a content-addressed ASCII snapshot; never alter firmware pin settings.

Run from an ESP-IDF terminal (firmware), or with JAVA_HOME/ANDROID_HOME (Android).
The snapshot and its artifacts are retained for inspection and flashing.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

projects = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("target", choices=["car", "remote", "android"])
parser.add_argument("--cache-root", type=Path, default=Path.home() / ".cache" / "gesture-builds")
parser.add_argument("--car-imu", choices=["0", "1"], default=None, help="Override car IMU for build validation only")
parser.add_argument("--tft", choices=["0", "1"], default=None, help="Override optional display for build validation only")
parser.add_argument("--jobs", type=int, default=4)
args = parser.parse_args()
if args.target != "car" and (args.car_imu is not None or args.tft is not None):
    parser.error("--car-imu/--tft only apply to car")
if args.jobs < 1:
    parser.error("--jobs must be positive")
cache = args.cache_root.resolve()
if not str(cache).isascii() or " " in str(cache):
    parser.error("Use an ASCII cache path without spaces, e.g. C:/gesture-builds")

folders = ["gesture-" + args.target]
if args.target != "android":
    folders.append("gesture-common")
ignored = {"build", "build-local", ".git", ".gradle", ".kotlin", "__pycache__", "managed_components"}
files = {}
for folder in folders:
    for path in sorted((projects / folder).rglob("*")):
        relative = path.relative_to(projects)
        if not path.is_file() or any(part in ignored for part in relative.parts):
            continue
        if path.name in {"sdkconfig", "sdkconfig.old", "local.properties"}:
            continue
        if path.suffix in {".c", ".h", ".txt", ".kt", ".java", ".kts", ".properties", ".xml", ".jar"} or path.name in {"Kconfig", "Kconfig.projbuild", "sdkconfig.defaults", "gradlew", "gradlew.bat"}:
            files[relative.as_posix()] = path.read_bytes()
digest = hashlib.sha256()
for name, data in sorted(files.items()):
    digest.update(name.encode()); digest.update(b"\0"); digest.update(data)
digest.update(str((args.car_imu, args.tft)).encode())
snapshot = cache / digest.hexdigest()[:16]
snapshot.mkdir(parents=True, exist_ok=True)
for name, data in files.items():
    destination = snapshot / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or destination.read_bytes() != data:
        destination.write_bytes(data)
manifest = {name: hashlib.sha256(data).hexdigest() for name, data in files.items()}
(snapshot / "source-manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
project = snapshot / ("gesture-" + args.target)
env = os.environ.copy()
env["PYTHONUTF8"] = "1"
log_path = snapshot / (args.target + "-build.log")
print("Source snapshot:", snapshot, flush=True)
print("Build log:", log_path, flush=True)

if args.target == "android":
    gradle = shutil.which("gradle.bat" if os.name == "nt" else "gradle")
    if gradle is None:
        gradle = str(project / ("gradlew.bat" if os.name == "nt" else "gradlew"))
        if os.name != "nt":
            Path(gradle).chmod(0o755)
    commands = [[gradle, "--no-daemon", "--console=plain", "--max-workers=" + str(args.jobs),
                 ":app:assembleDebug", ":app:testDebugUnitTest", ":app:lintDebug"]]
else:
    idf = env.get("IDF_PATH")
    if not idf or not (Path(idf) / "tools/idf.py").exists():
        parser.error("Open an ESP-IDF terminal first (IDF_PATH is missing)")
    configure = [sys.executable, str(Path(idf) / "tools/idf.py"), "-B", "build-local"]
    defines = []
    if args.car_imu is not None: defines.append("-DENABLE_CAR_IMU=" + args.car_imu)
    if args.tft is not None: defines.append("-DENABLE_TFT=" + args.tft)
    if defines: configure.extend(["-D", "CMAKE_C_FLAGS=" + " ".join(defines)])
    configure.append("reconfigure")
    commands = [configure, ["ninja", "-C", "build-local", "-j", str(args.jobs)]]
with log_path.open("w", encoding="utf-8") as log:
    for command in commands:
        print("Running:", subprocess.list2cmdline(command), flush=True)
        result = subprocess.run(command, cwd=project, env=env, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            log.flush()
            print("\n".join(log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-60:]))
            sys.exit(result.returncode)

variant = "default" if args.car_imu is None and args.tft is None else f"imu-{args.car_imu or 'default'}_tft-{args.tft or 'default'}"
output = projects / ("gesture-" + args.target) / "build-local" / "verified" / variant
output.mkdir(parents=True, exist_ok=True)
if args.target == "android":
    artifact = project / "app/build/outputs/apk/debug/app-debug.apk"
    shutil.copy2(artifact, output / artifact.name)
    for name in ["test-results/testDebugUnitTest", "reports/lint-results-debug.html", "reports/lint-results-debug.txt"]:
        source = project / "app/build" / name
        target = output / name
        if source.is_dir(): shutil.copytree(source, target, dirs_exist_ok=True)
        elif source.exists(): target.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(source, target)
else:
    for name in [f"gesture-{args.target}.bin", f"gesture-{args.target}.elf", f"gesture-{args.target}.map",
                 "bootloader/bootloader.bin", "partition_table/partition-table.bin", "flash_args", "flasher_args.json"]:
        source = project / "build-local" / name
        if source.exists():
            target = output / name; target.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(source, target)
shutil.copy2(snapshot / "source-manifest.json", output / "source-manifest.json")
shutil.copy2(log_path, output / "build.log")
(output / "BUILD_ORIGIN.txt").write_text(f"Verified source: {project}\nArtifacts copied from successful build.\nVerify board silk-screen, wiring, power and motor direction before driving.\n", encoding="utf-8")
print("BUILD AND CHECKS PASSED. Artifacts:", output, flush=True)
