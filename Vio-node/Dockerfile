


########################################################################
# stage 1 – copy your host-built librealsense 2.56.x into a scratch image
########################################################################
FROM scratch AS librealsense
#     Adjust this path if your libs/headers live elsewhere
COPY host_librealsense/ /usr/local/

########################################################################
# stage 2 – ROS 2 Humble runtime with Python node
########################################################################
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

# bring the librealsense libs from stage 1
COPY --from=librealsense /usr/local /usr/local

ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# small Python deps only
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-numpy && \
    pip3 install --no-cache-dir pyrealsense2==2.56.1 && \
    rm -rf /var/lib/apt/lists/*

# copy your script
WORKDIR /root/rs_node
COPY vio.py .

ENTRYPOINT ["python3", "vio.py"]