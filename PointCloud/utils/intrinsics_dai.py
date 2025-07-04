# shows intrinsics for depth cameras from depthai for calcualtion

import depthai as dai
import numpy as np

# Initialize the device
with dai.Device() as device:
    # Read calibration data
    calib_data = device.readCalibration()
    
    # Retrieve intrinsics for the RGB camera
    intrinsics = np.array(calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB))
    
    # Extract focal lengths and optical centers
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx = intrinsics[0, 2]
    cy = intrinsics[1, 2]
    
    print(f"Focal Lengths: fx={fx}, fy={fy}")
    print(f"Optical Centers: cx={cx}, cy={cy}")
