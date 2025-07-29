

########################################################################
# stage 1 – just your RSUSB libs copied from the host into a tiny stage
########################################################################
FROM scratch AS librealsense
COPY host_librealsense/ /usr/local/        

########################################################################
# stage 2 – ROS + Python node
########################################################################
FROM dustynv/ros:humble-desktop-l4t-r36.4.0   # JetPack 6.0.2  (GPUs okay)

# copy libs from stage 1
COPY --from=librealsense /usr/local /usr/local
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

RUN apt-get update && apt-get install -y --no-install-recommends python3-numpy \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyrealsense2==2.56.1         # wheel, RSUSB backend

WORKDIR /root/rs_node
COPY rs_pc_node.py .

ENTRYPOINT ["python3","rs_pc_node.py"]