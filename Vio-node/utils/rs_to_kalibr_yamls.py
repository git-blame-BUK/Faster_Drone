#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Reads Intel RealSense intrinsics/extrinsics (stereo + IMU) and emits EuRoC/Kalibr-like YAMLs:
# - kalibr_imucam_chain.yaml (cam0/cam1 with T_imu_cam, intrinsics, resolution, rostopic)
# - kalibr_imu_chain.yaml (IMU model + rough noise from motion intrinsics)
#
# Usage (example):
#   python3 rs_to_kalibr_yamls.py \
#       --out-dir . \
#       --width 848 --height 480 --fps 30 \
#       --topic-left /camera/infra1/image_rect_raw \
#       --topic-right /camera/infra2/image_rect_raw \
#       --imu-topic /imu0 \
#       --imu-rate 120 \
#       [--serial <D435 serial>]
#
# If you don't use the RealSense IMU (e.g., Pixhawk), pass --skip-rs-imu.
# In that case, T_imu_cam will still be filled from RealSense gyro->camera if present,
# but IMU noise blocks will be left as TODO placeholders.
import argparse
from pathlib import Path
import math
import sys

try:
    import yaml
except Exception as e:
    print("Please install PyYAML: pip install pyyaml", file=sys.stderr)
    raise

try:
    import pyrealsense2 as rs
except Exception as e:
    print("pyrealsense2 not available. Install librealsense2/pyrealsense2 on the target machine.", file=sys.stderr)
    raise


def extrinsics_to_T44(e):
    # Convert rs.extrinsics (rotation[9], translation[3]) to 4x4 (row-major).
    R = [float(x) for x in e.rotation]
    t = [float(x) for x in e.translation]
    return [
        [R[0], R[1], R[2], t[0]],
        [R[3], R[4], R[5], t[1]],
        [R[6], R[7], R[8], t[2]],
        [0.0,  0.0,  0.0,  1.0],
    ]


def mat4_inv(T):
    # Invert a 4x4 rigid-body transform.
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    t = [T[0][3], T[1][3], T[2][3]]
    Rt = [[R[0][0], R[1][0], R[2][0]],
          [R[0][1], R[1][1], R[2][1]],
          [R[0][2], R[1][2], R[2][2]]]
    ti = [
        -(Rt[0][0]*t[0] + Rt[0][1]*t[1] + Rt[0][2]*t[2]),
        -(Rt[1][0]*t[0] + Rt[1][1]*t[1] + Rt[1][2]*t[2]),
        -(Rt[2][0]*t[0] + Rt[2][1]*t[1] + Rt[2][2]*t[2])
    ]
    return [
        [Rt[0][0], Rt[0][1], Rt[0][2], ti[0]],
        [Rt[1][0], Rt[1][1], Rt[1][2], ti[1]],
        [Rt[2][0], Rt[2][1], Rt[2][2], ti[2]],
        [0.0, 0.0, 0.0, 1.0]
    ]


def rs_intrinsics_to_kalibr(i):
    # Map RealSense intrinsics to Kalibr-style pinhole + radtan.
    # RealSense coeffs order: [k1, k2, p1, p2, k3]
    fx, fy, cx, cy = float(i.fx), float(i.fy), float(i.ppx), float(i.ppy)
    coeffs = list(i.coeffs[:5]) + [0.0] * (5 - len(i.coeffs[:5]))
    coeffs = [float(x) for x in coeffs]  # k1 k2 p1 p2 k3
    return {
        "camera_model": "pinhole",
        "distortion_model": "radtan",
        "intrinsics": [fx, fy, cx, cy],
        # EuRoC examples often show 4 coeffs, Kalibr allows 5. Use 4 for template-compat.
        "distortion_coeffs": coeffs[:4],
        "resolution": [int(i.width), int(i.height)]
    }


def average(vals):
    vals = [float(v) for v in vals]
    return sum(vals) / max(1, len(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=".", help="Output directory for YAML files")
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--serial", default="", help="Optional RealSense serial to select device")
    ap.add_argument("--topic-left", dest="topic_left", default="/camera/infra1/image_rect_raw")
    ap.add_argument("--topic-right", dest="topic_right", default="/camera/infra2/image_rect_raw")
    ap.add_argument("--imu-topic", dest="imu_topic", default="/imu0")
    ap.add_argument("--imu-rate", dest="imu_rate", type=float, default=120.0)
    ap.add_argument("--skip-rs-imu", dest="skip_rs_imu", action="store_true", help="Skip reading RealSense IMU intrinsics (e.g., using Pixhawk)")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Configure pipeline
    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    cfg.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    if not args.skip_rs_imu:
        # enable accel+gyro to query motion intrinsics + extrinsics to cameras
        try:
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        except Exception as e:
            print("Warning: couldn't enable accel/gyro streams:", e, file=sys.stderr)

    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    try:
        # Profiles for cameras
        p_c0 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        p_c1 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        i_c0 = p_c0.get_intrinsics()
        i_c1 = p_c1.get_intrinsics()

        cam0 = rs_intrinsics_to_kalibr(i_c0)
        cam1 = rs_intrinsics_to_kalibr(i_c1)
        cam0["rostopic"] = args.topic_left
        cam1["rostopic"] = args.topic_right
        cam0["cam_overlaps"] = [1]
        cam1["cam_overlaps"] = [0]

        # Stereo extrinsics (for reference; not required in kalibr_imucam_chain.yaml)
        try:
            e_c1_to_c0 = p_c1.get_extrinsics_to(p_c0)
            T_c1_c0 = extrinsics_to_T44(e_c1_to_c0)
        except Exception as e:
            print("Warning: couldn't get c1->c0 extrinsics:", e, file=sys.stderr)
            T_c1_c0 = None

        # Motion profiles + intrinsics
        gyro_intr = None
        accel_intr = None
        T_gyro_c0 = None
        T_gyro_c1 = None
        T_accel_c0 = None
        T_accel_c1 = None

        if not args.skip_rs_imu:
            try:
                p_g = profile.get_stream(rs.stream.gyro).as_motion_stream_profile()
                p_a = profile.get_stream(rs.stream.accel).as_motion_stream_profile()
                gyro_intr = p_g.get_motion_intrinsics()
                accel_intr = p_a.get_motion_intrinsics()

                # Extrinsics from IMU (gyro/accel) to cameras
                T_gyro_c0 = extrinsics_to_T44(p_g.get_extrinsics_to(p_c0))
                T_gyro_c1 = extrinsics_to_T44(p_g.get_extrinsics_to(p_c1))
                T_accel_c0 = extrinsics_to_T44(p_a.get_extrinsics_to(p_c0))
                T_accel_c1 = extrinsics_to_T44(p_a.get_extrinsics_to(p_c1))
            except Exception as e:
                print("Warning: couldn't read RealSense IMU intrinsics/extrinsics:", e, file=sys.stderr)

        # Choose gyro as "IMU" reference if available; else accel; else identity
        def choose_T_imu_to_cam(T_gyro_cam, T_accel_cam):
            if T_gyro_cam is not None:
                return T_gyro_cam
            if T_accel_cam is not None:
                return T_accel_cam
            return [[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]

        # Kalibr expects T_imu_cam (camera->IMU). RealSense gives IMU->cam. Invert to get cam->IMU.
        T_cam0_to_imu = None
        T_cam1_to_imu = None
        if T_gyro_c0 is not None or T_accel_c0 is not None:
            T_cam0_to_imu = mat4_inv(choose_T_imu_to_cam(T_gyro_c0, T_accel_c0))
        if T_gyro_c1 is not None or T_accel_c1 is not None:
            T_cam1_to_imu = mat4_inv(choose_T_imu_to_cam(T_gyro_c1, T_accel_c1))

        # Build kalibr_imucam_chain.yaml
        kalibr_imucam = {
            "cam0": {
                "T_imu_cam": T_cam0_to_imu if T_cam0_to_imu is not None else [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                "cam_overlaps": cam0["cam_overlaps"],
                "camera_model": cam0["camera_model"],
                "distortion_coeffs": cam0["distortion_coeffs"],
                "distortion_model": cam0["distortion_model"],
                "intrinsics": cam0["intrinsics"],
                "resolution": cam0["resolution"],
                "rostopic": cam0["rostopic"],
            },
            "cam1": {
                "T_imu_cam": T_cam1_to_imu if T_cam1_to_imu is not None else [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                "cam_overlaps": cam1["cam_overlaps"],
                "camera_model": cam1["camera_model"],
                "distortion_coeffs": cam1["distortion_coeffs"],
                "distortion_model": cam1["distortion_model"],
                "intrinsics": cam1["intrinsics"],
                "resolution": cam1["resolution"],
                "rostopic": cam1["rostopic"],
            },
        }

        # Build kalibr_imu_chain.yaml
        imu_noise_comment = (
            "NOTE: The noise values below are rough estimates from RealSense motion intrinsics. "
            "For OpenVINS, run Allan variance (allan_variance_ros) to obtain proper densities/walks."
        )

        acc_noise = None
        gyr_noise = None
        acc_bias = None
        gyr_bias = None
        if gyro_intr is not None:
            gyr_noise = [math.sqrt(float(x)) for x in getattr(gyro_intr, "noise_variances", [0.0,0.0,0.0])]
            gyr_bias  = [math.sqrt(float(x)) for x in getattr(gyro_intr, "bias_variances", [0.0,0.0,0.0])]
        if accel_intr is not None:
            acc_noise = [math.sqrt(float(x)) for x in getattr(accel_intr, "noise_variances", [0.0,0.0,0.0])]
            acc_bias  = [math.sqrt(float(x)) for x in getattr(accel_intr, "bias_variances", [0.0,0.0,0.0])]

        kalibr_imu = {
            "imu0": {
                "T_i_b": [[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]],
                "accelerometer_noise_density": float(average(acc_noise) if acc_noise else 2.0e-3),
                "accelerometer_random_walk":   float(average(acc_bias)  if acc_bias  else 3.0e-3),
                "gyroscope_noise_density":     float(average(gyr_noise) if gyr_noise else 1.7e-4),
                "gyroscope_random_walk":       float(average(gyr_bias)  if gyr_bias  else 2.0e-5),
                "rostopic": args.imu_topic,
                "time_offset": 0.0,  # Replace with Kalibr result later
                "update_rate": float(args.imu_rate),
                "model": "kalibr",
                "Tw": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
                "R_IMUtoGYRO": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
                "Ta": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
                "R_IMUtoACC": [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
                "Tg": [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],
                "_comment": imu_noise_comment,
            }
        }

        # Write files
        imucam_path = out_dir / "kalibr_imucam_chain.yaml"
        imu_path = out_dir / "kalibr_imu_chain.yaml"

        with imucam_path.open("w") as f:
            f.write("%YAML:1.0\n")
            yaml.safe_dump(kalibr_imucam, f, sort_keys=False)
            # Add helpful trailing hints
            f.write("\n# HINTS:\n# - T_imu_cam are from RealSense IMU→cam extrinsics (inverted). Verify with Kalibr if possible.\n")
            if 'T_c1_c0' in locals() and T_c1_c0 is not None:
                f.write("# - Stereo T_c1_c0 (for reference):\n")
                yaml.safe_dump({ "T_c1_c0_ref": T_c1_c0 }, f, sort_keys=False)

        with imu_path.open("w") as f:
            f.write("%YAML:1.0\n\n")
            yaml.safe_dump(kalibr_imu, f, sort_keys=False)

        # Also print a compact summary to stdout
        print("Wrote:", imucam_path)
        print("Wrote:", imu_path)
        print("\nSummary (copy this to me):")
        print("==== Cameras ====")
        print("cam0 intrinsics (fx,fy,cx,cy):", cam0["intrinsics"], "dist:", cam0["distortion_coeffs"], "res:", cam0["resolution"])
        print("cam1 intrinsics (fx,fy,cx,cy):", cam1["intrinsics"], "dist:", cam1["distortion_coeffs"], "res:", cam1["resolution"])
        if 'T_c1_c0' in locals() and T_c1_c0 is not None:
            print("T_c1_c0 (row-major 4x4):", T_c1_c0)
        if T_cam0_to_imu is not None:
            print("T_imu_cam0 (camera->IMU):", T_cam0_to_imu)
        if T_cam1_to_imu is not None:
            print("T_imu_cam1 (camera->IMU):", T_cam1_to_imu)
        print("\n==== IMU approx (from RS motion intrinsics) ====")
        if 'acc_noise' in locals() and acc_noise: print("acc_noise_density ~", average(acc_noise), "(m/s^2)/sqrt(Hz)  [approx]")
        if 'acc_bias'  in locals() and acc_bias:  print("acc_random_walk   ~", average(acc_bias),  "(m/s^3)/sqrt(Hz) [approx]")
        if 'gyr_noise' in locals() and gyr_noise: print("gyro_noise_density~", average(gyr_noise), "rad/s/sqrt(Hz)   [approx]")
        if 'gyr_bias'  in locals() and gyr_bias:  print("gyro_random_walk  ~", average(gyr_bias),  "rad/s^2/sqrt(Hz) [approx]")
        print("\nNOTE: Replace noise/random-walk with Allan variance results for OpenVINS stability.")
    finally:
        try:
            pipe.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
