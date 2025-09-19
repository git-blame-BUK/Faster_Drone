
# realsense_connect_test.py
import pyrealsense2 as rs

try:
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        raise RuntimeError("No RealSense devices found.")
    print(f"Found {len(devices)} RealSense device(s).")
    for dev in devices:
        print(f"- Name: {dev.get_info(rs.camera_info.name)}")
        print(f"  Serial: {dev.get_info(rs.camera_info.serial_number)}")

    # Try to start pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    pipeline.start(config)
    print("Pipeline started successfully.")

    pipeline.stop()
    print("Pipeline stopped.")
except Exception as e:
    print(f"ERROR: {e}")