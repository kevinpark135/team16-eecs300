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


def stage_integrated():
    destination = BUILD / "sketches/integrated"
    destination.mkdir(parents=True, exist_ok=True)
    for source in (ROOT / "integrated").iterdir():
        if source.suffix in {".ino", ".hpp"}:
            shutil.copy2(source, destination / source.name)
    for source in (ROOT / "lidar").glob("*.hpp"):
        shutil.copy2(source, destination / source.name)
    shutil.copy2(ROOT / "thermal/blobDetector.hpp", destination / "blobDetector.hpp")
    for source in (ROOT / "thermal/MLX90640_Thermal_Camera").iterdir():
        if source.suffix in {".h", ".hpp", ".cpp"}:
            shutil.copy2(source, destination / source.name)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fqbn", default="esp32:esp32:esp32")
    parser.add_argument("--cli", default=shutil.which("arduino-cli") or
                        "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli")
    parser.add_argument("--target", choices=["all", "integrated", "lidar", "thermal"], default="all")
    args = parser.parse_args()
    config = prepare_config()
    environment = os.environ.copy()
    temporary = BUILD / "tmp"
    temporary.mkdir(exist_ok=True)
    environment["TMPDIR"] = str(temporary)
    environment["PYTHONPYCACHEPREFIX"] = str(BUILD / "python-cache")
    sketches = []
    if args.target in {"all", "lidar"}:
        sources = [ROOT / "lidar/2LiDARs.ino", ROOT / "lidar/direction_sensing.ino",
                   ROOT / "lidar/2LiDARs_direction/2LiDARs_direction.ino"]
        sketches.extend((stage_lidar(source), source.stem, []) for source in sources)
    if args.target in {"all", "thermal"}:
        source = ROOT / "thermal/MLX90640_Thermal_Camera"
        sketches.append((source, "MLX90640_Thermal_Camera", []))
        sketches.append((source, "thermal-diagnostic", ["--build-property",
                         "compiler.cpp.extra_flags=-DTHERMAL_DIAGNOSTIC_MODE=1"]))
    if args.target in {"all", "integrated"}:
        sketches.append((stage_integrated(), "integrated", []))
    for sketch, name, extra in sketches:
        print("Compiling", name, "for", args.fqbn, flush=True)
        command = [args.cli, "compile", "--config-file", str(config), "--fqbn", args.fqbn,
                   "--build-path", str(BUILD / "firmware" / name), "--warnings", "all",
                   "--library", str(Path.home() / "Documents/Arduino/libraries/SparkFun_VL53L5CX_Arduino_Library"),
                   str(sketch)]
        subprocess.run(command + extra, env=environment, check=True)
    print("PASS:", len(sketches), "firmware builds completed.")


if __name__ == "__main__":
    main()
