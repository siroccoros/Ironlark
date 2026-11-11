# ROS 2 Humble for Ironlark's sensor bridge and rosbag2, apart from Webots' Python.
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends python3-scipy && \
    rm -rf /var/lib/apt/lists/*
ENV PYTHONUNBUFFERED=1
WORKDIR /run/ironlark
