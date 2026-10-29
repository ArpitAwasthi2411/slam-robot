# Pathik robot software as one container image (ROS 2 Humble + Cartographer + RPLiDAR driver + our package).
#
# Built by GitHub Actions for the Pi (linux/arm64) and laptops (linux/amd64), pushed to
# ghcr.io/arpitawasthi2411/slam-robot. On the Pi: deploy/pi/docker-compose.yml (see docs/deployment.md).
#
#   docker build -t pathik .                       # local build
#   docker run --rm pathik bash -c 'cd /opt/pathik_ws/src/lidar_robot && python3 -m pytest -q'   # tests inside it
#
# Layers are ordered from "rarely changes" to "changes every commit", so a code change only
# rebuilds the last few layers (seconds), not ROS or the LiDAR driver.

ARG ROS_DISTRO=humble

# ---------------------------------------------------------------- base: ROS + system packages
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=${ROS_DISTRO}
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-cartographer-ros \
        ros-${ROS_DISTRO}-tf2-ros-py \
        python3-serial python3-numpy python3-pytest \
        espeak-ng alsa-utils python3-libgpiod python3-opencv python3-yaml \
        git \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- the LiDAR driver (pinned source build)
FROM base AS lidar
ARG SLLIDAR_REF=main
WORKDIR /opt/pathik_ws
RUN git clone --depth 1 --branch ${SLLIDAR_REF} https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2 \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-select sllidar_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log src/sllidar_ros2/.git

# ---------------------------------------------------------------- our package
FROM lidar AS build
COPY ros2_ws/src/lidar_robot /opt/pathik_ws/src/lidar_robot
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . install/setup.sh \
    && colcon build --packages-select lidar_robot \
    && rm -rf build log

# ---------------------------------------------------------------- runtime (default target)
FROM build AS runtime
ARG VERSION=dev
LABEL org.opencontainers.image.title="Pathik robot" \
      org.opencontainers.image.description="ROS 2 robot software for the Pathik campus robot" \
      org.opencontainers.image.source="https://github.com/ArpitAwasthi2411/slam-robot" \
      org.opencontainers.image.version="${VERSION}"
ENV PATHIK_VERSION=${VERSION} \
    ROS_DOMAIN_ID=0
COPY deploy/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /root/maps
# maps, places, rooms, users, tuning and the mission log live here: mount it from the Pi
VOLUME ["/root/maps"]
EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=5s --start-period=60s --retries=3 \
    CMD python3 -c "import urllib.request,sys; urllib.request.urlopen('http://127.0.0.1:8080/api/ping', timeout=4)" || exit 1
ENTRYPOINT ["/entrypoint.sh"]
CMD ["robot"]
