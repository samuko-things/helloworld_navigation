## My Helloworld Navigation (ROS2 Jazzy) with EasyNav

#### Install EasyNav

- **Recommended**: Build EasyNav in full from source - [READ_MORE](https://easynavigation.github.io/build_install/index.html#build-from-source)

```shell
mkdir -p ~/easynav_ws/src && cd ~/easynav_ws/src
```
```shell
git clone -b jazzy https://github.com/EasyNavigation/EasyNavigation.git \
&& git clone -b jazzy https://github.com/EasyNavigation/NavMap.git \
&& git clone -b jazzy https://github.com/EasyNavigation/easynav_plugins.git \
&& git clone -b jazzy https://github.com/fmrico/yaets.git
```
```shell
cd ~/easynav_ws && rosdep install --from-paths src --ignore-src -y -r
```
```shell
cd ~/easynav_ws && colcon build --symlink-install
```

---

#### Build Robot

- clone and build robot repo in your workspace (e.g robot_ws)

---

#### Source Necessary Workspace

- source necessary workspace/packages
```shell
source /opt/ros/jazzy/setup.bash
```
```shell
source ~/easynav_ws/install/setup.bash
```
```shell
source ~/ros_ws/install/setup.bash
```

---

#### Run EasyNav Mapping and Navigation

- run mapping (simple or costmap)
```shell
ros2 launch robot_simulation sim.launch.py #gui:=false
```
```shell
ros2 launch robot_navigation mapping.launch.py #use_costmap:=true
```

- run navigation (simple map)
```shell
ros2 launch robot_simulation sim.launch.py #gui:=false
```
```shell
ros2 launch robot_navigation easynav_simple.launch.py
```

- run navigation (costmap map)
```shell
ros2 launch robot_simulation sim.launch.py #gui:=false
```
```shell
ros2 launch robot_navigation easynav_costmap.launch.py #use_route:=true
```

---

#### Save EasyNav SimpleMap or CostMap

- save simple map with easynav
```shell
ros2 service call /maps_manager_node/simple/savemap std_srvs/srv/Trigger
```
```shell
mv /tmp/default.map ~/ros_ws/src/helloworld_navigation/robot_navigation/maps/<map_name>.map
```

- save costmap map in easynav with slamtoolbox
```shell
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/$USER/ros_ws/src/helloworld_navigation/robot_navigation/maps/room_with_walls'}}"
```

---

#### Create and Save Routes

- create routes (use interractive marker to add and remove routes)
```shell
ros2 launch robot_navigation easynav_create_routes.launch.py 
```

- save route yaml file
```shell
ros2 service call /maps_manager_node/routes/save_routes std_srvs/srv/Trigger {}
```
```shell
mv /tmp/routes.yaml ~/ros_ws/src/helloworld_navigation/robot_navigation/maps/<route_name>.yaml
```
