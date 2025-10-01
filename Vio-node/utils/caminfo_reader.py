
#!/usr/bin/env python3
"""
realsense_caminfo_dump.py
Liest die Intrinsics & Distortion-Koeffizienten (RAW) der D435-IR-Kameras (infra1=cam0, infra2=cam1)
und gibt Copy&Paste-Blöcke für kalibr_imucam_chain.yaml aus.

Benutzung (Beispiel):
  python3 realsense_caminfo_dump.py --width 848 --height 480 --fps 30
Optional: --serial <DEVICE_SERIAL>
"""
import argparse
import sys
import pyrealsense2 as rs

def intr_to_dict(intr):
    # intr: rs.intrinsics (fx, fy, ppx, ppy, coeffs[0..4], width, height)
    d = [0.0]*5
    for i in range(min(5, len(intr.coeffs))):
        d[i] = float(intr.coeffs[i])
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "cx": float(intr.ppx),
        "cy": float(intr.ppy),
        "distortion_coeffs": d,  # [k1, k2, p1, p2, k3] (plumb_bob/Brown-Conrady)
    }

def print_yaml_block(name, intr_dict):
    print(f"{name}:")
    print("  camera_model: pinhole")
    print("  distortion_model: radtan  # = plumb_bob/Brown-Conrady")
    print(f"  intrinsics: [{intr_dict['fx']:.6f}, {intr_dict['fy']:.6f}, {intr_dict['cx']:.6f}, {intr_dict['cy']:.6f}]")
    d = intr_dict['distortion_coeffs']
    print(f"  distortion_coeffs: [{d[0]:.8f}, {d[1]:.8f}, {d[2]:.8f}, {d[3]:.8f}, {d[4]:.8f}]")
    print(f"  resolution: [{intr_dict['width']}, {intr_dict['height']}]")
    print("")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--serial", type=str, default="")
    args = ap.parse_args()

    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)

    # D435 IR1/IR2 aktivieren (RAW Y8)
    cfg.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    cfg.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    pipe = rs.pipeline()
    try:
        profile = pipe.start(cfg)  # startet Streams und liefert aktives Profil
        # Streams holen
        vs1 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        vs2 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()

        intr1 = intr_to_dict(vs1.get_intrinsics())
        intr2 = intr_to_dict(vs2.get_intrinsics())

        # Optional: Stereo-Baseline ausgeben (hilfreich für P[3] bei rectified)
        ex12 = vs2.get_extrinsics_to(vs1)  # 2 -> 1
        baseline_m = float(ex12.translation[0])  # Meter (X-Richtung)

        print("# === RealSense D435 IR-RAW Intrinsics & Distortion ===")
        print_yaml_block("cam0", intr1)  # infra1 -> cam0
        print_yaml_block("cam1", intr2)  # infra2 -> cam1
        print(f"# Stereo baseline (cam1 w.r.t cam0) ~ {baseline_m:.6f} m")
        print("# Hinweis: Für RAW-Bilder in OpenVINS diese Werte in kalibr_imucam_chain.yaml eintragen.")
        print("# Für RECT-Bilder (image_rect) D=0 verwenden und K/P aus den rectified CameraInfos übernehmen.")

    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        try:
            pipe.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()
