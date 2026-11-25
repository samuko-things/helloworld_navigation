## My Helloworld Navigation (ROS2 Jazzy) with EasyNav


#### Install and build Build Robot Navigation with EasNav

- clone the robot navigation repo (**helloworld_navigation**) in your workspace
```shell
cd ~/<your_workspace>/src && git clone -b easynav-test https://github.com/samuko-things/helloworld_navigation.git
```

- clone the easynav lazy theta planner repo ([**easynav_lazy_theta_planner**](https://github.com/samuko-things/easynav_lazy_theta_planner)) in your workspace
```shell
cd ~/<your_workspace>/src && git clone -b jazzy https://github.com/samuko-things/easynav_lazy_theta_planner.git
```

- install all necessary dependencies for robot and easynav navigation (ensure you have rosdep installed)
```shell
cd ~/<your_workspace> && rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

---

#### Source Necessary Workspace

- ensure you source necessary workspace/packages
```shell
source /opt/ros/jazzy/setup.bash
```

```shell
source ~/<your_workspace>/install/setup.bash
```

---

#### Run EasyNav Navigation

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

---

#### Run EasyNav Mapping

- run mapping (simple or costmap)
```shell
ros2 launch robot_simulation sim.launch.py #gui:=false
```
```shell
ros2 launch robot_navigation mapping.launch.py #use_costmap:=true
```

---

#### Save EasyNav SimpleMap or CostMap

- save simple map with easynav
```shell
ros2 service call /maps_manager_node/simple/savemap std_srvs/srv/Trigger
```
```shell
mv /tmp/default.map ~/ros_ws/src/helloworld_navigation/robot_navigation/maps/room_with_walls.map
```

- save costmap map in easynav with slamtoolbox
```shell
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/$USER/ros_ws/src/helloworld_navigation/robot_navigation/maps/room_with_walls'}}"
```