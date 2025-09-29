#!/usr/bin/env python3
# Dump RealSense IR intrinsics (und Stereo-Extrinsics) in OpenVINS-kompatible YAMLs.
# Usage:
#   python3 realsense_to_yaml.py --out ./calib --width 640 --height 480 --fps 30 [--serial <D435 serial>]

import argparse, os
from pathlib import Path
import pyrealsense2 as rs
import yaml

def to_yaml_from_intrinsics(intr):
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    D = list(intr.coeffs[:5]) + [0.0]*(5 - len(intr.coeffs[:5]))  # k1,k2,p1,p2,k3
    return {
        "camera_model": "pinhole",
        "distortion_model": "radtan",
        "intrinsics": [float(fx), float(fy), float(cx), float(cy)],
        "distortion_coeffs": [float(x) for x in D],
        "resolution": [int(intr.width), int(intr.height)],
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--serial", default="")
    args = ap.parse_args()

    outdir = Path(args.out); outdir.mkdir(parents=True, exist_ok=True)

    cfg = rs.config()
    if args.serial: cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    cfg.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    try:
        p0 = prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        p1 = prof.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        i0, i1 = p0.get_intrinsics(), p1.get_intrinsics()

        with open(outdir/"cam0.yaml","w") as f: yaml.safe_dump(to_yaml_from_intrinsics(i0), f, sort_keys=False)
        with open(outdir/"cam1.yaml","w") as f: yaml.safe_dump(to_yaml_from_intrinsics(i1), f, sort_keys=False)

        e01 = rs.get_extrinsics(p0, p1)  # cam0 -> cam1
        R = [[float(e01.rotation[r*3+c]) for c in range(3)] for r in range(3)]
        t = [float(e01.translation[0]), float(e01.translation[1]), float(e01.translation[2])]
        baseline = (t[0]**2 + t[1]**2 + t[2]**2) ** 0.5
        stereo = {"T_cam1_cam0": {"R": R, "t": t}, "baseline_m": float(baseline)}
        with open(outdir/"stereo_extrinsics.yaml","w") as f: yaml.safe_dump(stereo, f, sort_keys=False)

        print("OK:", outdir/"cam0.yaml", outdir/"cam1.yaml", outdir/"stereo_extrinsics.yaml")
    finally:
        pipe.stop()

if __name__ == "__main__":
    main()
