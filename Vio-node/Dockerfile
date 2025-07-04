# Base: minimal L4T runtime – ~220 MB unpacked
FROM nvcr.io/nvidia/l4t-base:r32.7.1

ENV DEBIAN_FRONTEND=noninteractive

# Only what we really need
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-rosdep python3-rospy python3-numpy \
        ros-noetic-sensor-msgs ros-noetic-cv-bridge ros-noetic-message-filters \
        libopencv-core-dev libopencv-imgproc-dev libopencv-video-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Keep wheels small (no full GUI Qt)
RUN pip3 install --no-cache-dir opencv-python-headless==4.10.*

# Copy script
COPY vio_node.py /app/vio_node.py

ENV PYTHONUNBUFFERED=1
WORKDIR /app
ENTRYPOINT ["python3", "vio_node.py"]