#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Creates a EuRoC/Kalibr-like stereo camchain from Intel RealSense (IR1/IR2).
# Outputs:
#   - kalibr_imucam_chain.yaml  (cam0/cam1 with intrinsics; T_imu_cam = identity placeholder)
#   - stereo_extrinsics_ref.yaml (T_c1_c0 reference from device)
#
# Default rostopics: /cam0/image_raw  and  /cam1/image_raw

import argparse, sys, math
from pathlib import Path
import yaml

try:
    import pyrealsense2 as rs
except Exception as e:
    print("pyrealsense2 not available. Install librealsense2/pyrealsense2.", file=sys.stderr)
    raise

def rs_intrinsics_to_kalibr(i):
    fx, fy, cx, cy = float(i.fx), float(i.fy), float(i.ppx), float(i.ppy)
    coeffs = list(i.coeffs[:5]) + [0.0] * (5 - len(i.coeffs[:5]))
    coeffs = [float(x) for x in coeffs]  # [k1,k2,p1,p2,k3]
    return {
        "camera_model": "pinhole",
        "distortion_model": "radtan",
        "intrinsics": [fx, fy, cx, cy],
        # EuRoC examples often use 4 coeffs; Kalibr supports 5. Keep 4 for template-compat.
        "distortion_coeffs": coeffs[:4],
        "resolution": [int(i.width), int(i.height)]
    }

def extrinsics_to_T44(e):
    R = [float(x) for x in e.rotation]
    t = [float(x) for x in e.translation]
    return [
        [R[0], R[1], R[2], t[0]],
        [R[3], R[4], R[5], t[1]],
        [R[6], R[7], R[8], t[2]],
        [0.0,  0.0,  0.0,  1.0],
    ]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=".", help="Output directory")
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--serial", default="", help="Optional RealSense serial to select device")
    ap.add_argument("--topic-left", dest="topic_left", default="/cam0/image_raw")
    ap.add_argument("--topic-right", dest="topic_right", default="/cam1/image_raw")
    args = ap.parse_args()

    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)

    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    cfg.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    try:
        p0 = prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        p1 = prof.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        i0, i1 = p0.get_intrinsics(), p1.get_intrinsics()
        cam0 = rs_intrinsics_to_kalibr(i0)
        cam1 = rs_intrinsics_to_kalibr(i1)
        cam0["rostopic"] = args.topic_left
        cam1["rostopic"] = args.topic_right
        cam0["cam_overlaps"] = [1]
        cam1["cam_overlaps"] = [0]

        # T_imu_cam is unknown here (Pixhawk IMU). Use identity placeholders.
        I44 = [[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]
        data_imucam = {
            "cam0": {
                "T_imu_cam": I44,
                "cam_overlaps": cam0["cam_overlaps"],
                "camera_model": cam0["camera_model"],
                "distortion_coeffs": cam0["distortion_coeffs"],
                "distortion_model": cam0["distortion_model"],
                "intrinsics": cam0["intrinsics"],
                "resolution": cam0["resolution"],
                "rostopic": cam0["rostopic"],
            },
            "cam1": {
                "T_imu_cam": I44,
                "cam_overlaps": cam1["cam_overlaps"],
                "camera_model": cam1["camera_model"],
                "distortion_coeffs": cam1["distortion_coeffs"],
                "distortion_model": cam1["distortion_model"],
                "intrinsics": cam1["intrinsics"],
                "resolution": cam1["resolution"],
                "rostopic": cam1["rostopic"],
            },
        }

        # Stereo extrinsics reference
        try:
            T_c1_c0 = extrinsics_to_T44(p1.get_extrinsics_to(p0))
        except Exception:
            T_c1_c0 = None

        imucam_path = out_dir / "kalibr_imucam_chain.yaml"
        with imucam_path.open("w") as f:
            f.write("%YAML:1.0\n")
            yaml.safe_dump(data_imucam, f, sort_keys=False)
            f.write("\n# NOTE:\n# - T_imu_cam are identity placeholders. Calibrate IMU↔Cam with Kalibr later.\n")

        if T_c1_c0 is not None:
            extr_path = out_dir / "stereo_extrinsics_ref.yaml"
            with extr_path.open("w") as f:
                f.write("%YAML:1.0\n")
                yaml.safe_dump({"T_c1_c0": T_c1_c0}, f, sort_keys=False)
        print("Wrote", imucam_path)
        if T_c1_c0 is not None:
            print("Wrote", extr_path)
    finally:
        try: pipe.stop()
        except Exception: pass

if __name__ == "__main__":
    main()
