
# ---------- stage 1: tiny image mit deinen libs ----------------------------
FROM scratch AS librealsense
COPY /usr/local /usr/local          # <-- Pfad auf dem Host (build-context)

# ---------- base ------------------------------------------------------------
FROM nvcr.io/nvidia/l4t-ros:r36.4.0-humble-desktop   
# JetPack 6 – includes ROS 2 Humble +
# CUDA + rclpyA

# ---------- librealsense runtime (pre-built RSUSB) --------------------------
# Copy the lib you already built on the host (2.56.1 → /usr/local/lib, /usr/local/include)
# to avoid a long compile inside the container.
COPY --from=local/librealsense:latest /usr/local /usr/local
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# ---------- Python deps -----------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-numpy && \
    rm -rf /var/lib/apt/lists/*

# ---------- pyrealsense2 wheel ---------------------------------------------
# Community wheel for Jetson Orin (2.56.1 RSUSB).  If you built one yourself, copy it.
RUN pip3 install pyrealsense2==2.56.1

# ---------- ROS package -----------------------------------------------------
WORKDIR /root/rs_node
COPY rs_pc_node.py .

# ---------- default entry ---------------------------------------------------
ENTRYPOINT ["python3", "rs_pc_node.py"]