FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-pip python3-dev python3-opencv \
    ros-noetic-cv-bridge ros-noetic-rospy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy

WORKDIR /app
COPY vio_node.py /app/

ENTRYPOINT ["rosrun", "rospy", "vio.py"]
