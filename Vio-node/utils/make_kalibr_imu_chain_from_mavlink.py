#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Creates a Kalibr-style IMU YAML for OpenVINS using your publishing topic (/imu0) and
# Pixhawk/MAVLink parameters. Noise values are placeholders; replace with Allan-variance results.

import argparse, sys
from pathlib import Path
import yaml

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="kalibr_imu_chain.yaml")
    ap.add_argument("--imu-topic", default="/imu0")
    ap.add_argument("--imu-rate", type=float, default=120.0)
    ap.add_argument("--time-offset", type=float, default=0.0, help="seconds; replace with Kalibr result")
    args = ap.parse_args()

    I44 = [[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]
    I33 = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]

    data = {
        "imu0": {
            "T_i_b": I44,
            "accelerometer_noise_density": 2.0e-3,
            "accelerometer_random_walk":   3.0e-3,
            "gyroscope_noise_density":     1.7e-4,
            "gyroscope_random_walk":       2.0e-5,
            "rostopic": args.imu_topic,
            "time_offset": float(args.time_offset),
            "update_rate": float(args.imu_rate),
            "model": "kalibr",
            "Tw": I33,
            "R_IMUtoGYRO": I33,
            "Ta": I33,
            "R_IMUtoACC": I33,
            "Tg": [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],
            "_comment": "Replace noise/random-walk with Allan variance results (allan_variance_ros).",
        }
    }

    out_path = Path(args.out)
    with out_path.open("w") as f:
        f.write("%YAML:1.0\n\n")
        yaml.safe_dump(data, f, sort_keys=False)
    print("Wrote", out_path)

if __name__ == "__main__":
    main()
