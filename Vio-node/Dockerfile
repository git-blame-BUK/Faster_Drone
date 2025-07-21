
# Use minimal L4T base image (Jetson specific)
FROM nvcr.io/nvidia/l4t-base:r32.7.1

ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies for ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release \
    python3 python3-pip python3-rosdep python3-numpy \
    libopencv-core-dev libopencv-imgproc-dev libopencv-video-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Add ROS Noetic repository
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Update apt sources and install ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rospy ros-noetic-cv-bridge ros-noetic-message-filters \
    ros-noetic-sensor-msgs && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install opencv-python-headless to avoid full OpenCV GUI
RUN pip3 install --no-cache-dir opencv-python-headless==4.10.*

# Copy the Python script to the container
COPY vio_node.py /app/vio_node.py

# Set working directory to where the script is
WORKDIR /app

# Set non-interactive mode for Python output
ENV PYTHONUNBUFFERED=1

# Run the script when the container starts
CMD ["python3", "vio_node.py"]
