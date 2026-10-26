#!/usr/bin/env python3
"""Stage independent Arduino sketches and compile them without uploading."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"


def prepare_config():
    # Reuse installed tools read-only; keep CLI state and caches in this folder.
    installed = Path.home() / "Library/Arduino15"
    data = BUILD / "arduino-data"
    data.mkdir(parents=True, exist_ok=True)
    for source in installed.glob("*index.json*"):
        shutil.copy2(source, data / source.name)
    packages = data / "packages"
    if not packages.exists():
        packages.symlink_to(installed / "packages", target_is_directory=True)
    user = BUILD / "arduino-user"
    user.mkdir(exist_ok=True)
    config = {
        "directories": {
            "data": str(data),
            "downloads": str(BUILD / "arduino-downloads"),
            "user": str(user),
            "builtin": {"libraries": str(installed / "libraries")},
        },
        "build_cache": {"path": str(BUILD / "arduino-cache")},
    }
    path = BUILD / "arduino-cli.json"
    path.write_text(json.dumps(config, indent=2))
    return path


def stage_lidar(source):
    # Arduino combines every .ino in one directory, so stage one at a time.
    destination = BUILD / "sketches" / source.stem
    destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination / source.name)
    for header in (ROOT / "lidar").glob("*.hpp"):
        shutil.copy2(header, destination / header.name)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fqbn", default="esp32:esp32:esp32")
    parser.add_argument("--cli", default=shutil.which("arduino-cli") or
                        "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli")
    args = parser.parse_args()
    config = prepare_config()
    environment = os.environ.copy()
    temporary = BUILD / "tmp"
    temporary.mkdir(exist_ok=True)
    environment["TMPDIR"] = str(temporary)
    environment["PYTHONPYCACHEPREFIX"] = str(BUILD / "python-cache")
    sources = [ROOT / "lidar/2LiDARs.ino", ROOT / "lidar/direction_sensing.ino",
               ROOT / "lidar/2LiDARs_direction/2LiDARs_direction.ino"]
    sketches = [stage_lidar(source) for source in sources]
    sketches.append(ROOT / "thermal/MLX90640_Thermal_Camera")
    for sketch in sketches:
        print("Compiling", sketch.name, "for", args.fqbn, flush=True)
        command = [args.cli, "compile", "--config-file", str(config), "--fqbn", args.fqbn,
                   "--build-path", str(BUILD / "firmware" / sketch.name), "--warnings", "all",
                   "--library", str(Path.home() / "Documents/Arduino/libraries/SparkFun_VL53L5CX_Arduino_Library"),
                   str(sketch)]
        subprocess.run(command, env=environment, check=True)
    # Compile diagnostic mode too; text output must stay out of binary mode.
    command[command.index("--build-path") + 1] = str(BUILD / "firmware/thermal-diagnostic")
    command.extend(["--build-property", "compiler.cpp.extra_flags=-DTHERMAL_DIAGNOSTIC_MODE=1"])
    subprocess.run(command, env=environment, check=True)
    print("PASS: three LiDAR sketches and both thermal modes compiled.")


if __name__ == "__main__":
    main()
