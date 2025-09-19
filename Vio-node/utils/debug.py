
#!/usr/bin/env python3
import time
import pyrealsense2 as rs
import numpy as np

def check_color():
    print("→ Starte nur Color-Stream...")
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    pipe = rs.pipeline()
    pipe.start(cfg)
    time.sleep(1)  # Warmup

    for i in range(10):
        frames = pipe.wait_for_frames()
        color = frames.get_color_frame()
        if color:
            print(f"[{i}] Color Frame OK")
        else:
            print(f"[{i}] Kein Color Frame")
    pipe.stop()
    print("✔ Color-Test abgeschlossen.\n")

def check_depth():
    print("→ Starte Depth + Color-Stream...")
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    pipe = rs.pipeline()
    pipe.start(cfg)
    time.sleep(1)  # Warmup

    for i in range(10):
        frames = pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        status = []
        if depth:
            status.append("Depth OK")
        else:
            status.append("Depth ✘")

        if color:
            status.append("Color OK")
        else:
            status.append("Color ✘")

        print(f"[{i}] {' | '.join(status)}")
    pipe.stop()
    print("✔ Depth-Test abgeschlossen.\n")

def check_pointcloud():
    print("→ Starte vollständige PointCloud-Pipeline...")
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    pipe = rs.pipeline()
    pc = rs.pointcloud()
    align = rs.align(rs.stream.color)

    pipe.start(cfg)
    time.sleep(1)

    for i in range(10):
        frames = pipe.wait_for_frames()
        frames = align.process(frames)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth or not color:
            print(f"[{i}] depth or color missing")
            continue

        points = pc.calculate(depth)
        verts = np.asanyarray(points.get_vertices()).reshape(-1, 3)
        print(f"[{i}] PointCloud size: {verts.shape}")
    pipe.stop()
    print("✔ PointCloud-Test abgeschlossen.\n")

if __name__ == "__main__":
    check_color()
    check_depth()
    check_pointcloud()