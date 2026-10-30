FROM ubuntu:22.04 AS build

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates git python3 python3-dev python3-pip \
    python3-numpy python3-pexpect python3-serial pkg-config \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir \
    empy==3.3.4 future==1.0.0 lxml==5.4.0 pymavlink==2.4.49 \
    pexpect==4.9.0 dronecan==1.0.27

# Copter-4.7.1. Pin source and its submodules; do not track a moving branch.
ARG ARDUPILOT_COMMIT=dbe792162d06cab66c3475fd5556bf7a120f119e
RUN git init /opt/ardupilot \
    && cd /opt/ardupilot \
    && git remote add origin https://github.com/ArduPilot/ardupilot.git \
    && git fetch --depth 1 origin ${ARDUPILOT_COMMIT} \
    && git checkout --detach FETCH_HEAD \
    && git submodule update --init --depth 1 \
       modules/waf modules/mavlink modules/gtest modules/DroneCAN/DSDL \
       modules/DroneCAN/pydronecan modules/DroneCAN/dronecan_dsdlc \
       modules/DroneCAN/libcanard modules/littlefs
WORKDIR /opt/ardupilot
RUN git submodule update --init --depth 1 modules/lwip
RUN ./waf configure --board sitl --disable-tests \
    && ./waf copter -j2

# Keep the upstream build cached while compiling this small backend correction.
COPY webots-clock.patch /tmp/webots-clock.patch
RUN git apply --check /tmp/webots-clock.patch \
    && git apply /tmp/webots-clock.patch \
    && ./waf copter -j2

# Each simulated IMU's own white noise (SIM_ACCn_NSD, SIM_GYRn_NSD: its datasheet's density) in place of one fixed
# floor for all (config/flight.parm sets the Pixhawk 1's).
COPY sitl-imu-noise.patch /tmp/sitl-imu-noise.patch
RUN git apply --check /tmp/sitl-imu-noise.patch \
    && git apply /tmp/sitl-imu-noise.patch \
    && ./waf copter -j2

# What runs: the SITL binary, its default parameters, the upstream Webots example (setup copies it
# out) and the license, with Python and pymavlink for the flight check. Not the source tree or the
# compilers (2.46 GB -> about 0.2 GB).
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends python3 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /usr/local/lib/python3.10/dist-packages /usr/local/lib/python3.10/dist-packages
COPY --from=build /opt/ardupilot/build/sitl/bin/arducopter /opt/ardupilot/build/sitl/bin/arducopter
COPY --from=build /opt/ardupilot/Tools/autotest/default_params /opt/ardupilot/Tools/autotest/default_params
COPY --from=build /opt/ardupilot/libraries/SITL/examples/Webots_Python \
    /opt/ardupilot/libraries/SITL/examples/Webots_Python
COPY --from=build /opt/ardupilot/COPYING.txt /opt/ardupilot/COPYING.txt

ENV PYTHONUNBUFFERED=1
WORKDIR /run/ironlark
