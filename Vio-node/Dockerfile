# Minimal ROS Noetic Base (ARM64, ohne GUI)
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Installiere ROS-Abhängigkeiten und Python-Bibliotheken
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-numpy python3-opencv \
    ros-noetic-rospy \
    ros-noetic-cv-bridge \
    ros-noetic-message-filters \
    ros-noetic-sensor-msgs \
    ros-noetic-snapstack-msgs && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Installiere opencv-python ohne GUI
RUN pip3 install --no-cache-dir opencv-python-headless==4.10.*

# Arbeitsverzeichnis
WORKDIR /app

# Kopiere VIO-Skript ins Image
COPY vio_node.py /app/vio_node.py

# Setze Python-Output auf ungepuffert
ENV PYTHONUNBUFFERED=1

# Starte das Skript
CMD ["python3", "vio_node.py"]
