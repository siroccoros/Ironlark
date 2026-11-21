# ROS 2 Humble for Ironlark's sensor bridge, rosbag2, and autonomy stack.
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy python3-scipy python3-pip \
    && pip3 install --no-cache-dir pymavlink==2.4.49 \
    && rm -rf /var/lib/apt/lists/*
ENV PYTHONUNBUFFERED=1
WORKDIR /run/ironlark
