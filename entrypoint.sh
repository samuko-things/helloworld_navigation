#!/bin/bash
set -e

ROS_DISTRO="jazzy"

# 1. Safely inject tools into .bashrc for when humans type interactive commands later
grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
grep -F "source /usr/share/colcon_cd/function/colcon_cd.sh" /root/.bashrc || echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
grep -F "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" /root/.bashrc || echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> /root/.bashrc
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
grep -F "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" /root/.bashrc || echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# 2. Force-source the primary ROS environment for THIS script execution instance
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 3. Handle initial setup/build ONLY if the workspace hasn't been built yet
if [ ! -d "/root/ros_ws/install" ]; then
    echo "First-time setup: building workspace..."
    cd /root/ros_ws
    colcon build --symlink-install
fi

# 4. Source the built custom workspace for immediate use in this terminal session
if [ -f "/root/ros_ws/install/setup.bash" ]; then
    echo "Sourcing my ros workspace..."
    grep -F "source /root/ros_ws/install/setup.bash" /root/.bashrc || echo "source /root/ros_ws/install/setup.bash" >> /root/.bashrc
    source "/root/ros_ws/install/setup.bash"
fi

source "/root/.bashrc"

# Execute the command passed to the docker container (e.g., bash)
exec "$@"