## My Helloworld Navigation (ROS2 Jazzy) Test with Nav2 Lazy Theta Planner 


#### Install and build Build Robot Navigation and Nav2 Lazy Theta Planner 

- clone the robot navigation repo (**helloworld_navigation**) in your workspace
```shell
cd ~/<your_workspace>/src && git clone -b nav2-test https://github.com/samuko-things/helloworld_navigation.git
```

- clone the nav2 lazy theta planner repo ([**nav2_lazy_theta_planner**](https://github.com/samuko-things/nav2_lazy_theta_planner)) in your workspace
```shell
cd ~/<your_workspace>/src && git clone -b jazzy https://github.com/samuko-things/nav2_lazy_theta_planner.git
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

#### Run Navigation Test

- run navigation
```shell
ros2 launch robot_navigation navigation.launch.py
```
