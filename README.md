# Car_real

ROS 2 Humble 实车建图与导航工作区。当前推荐流程围绕两件事：

- 建图：`robot_bringup mapping.launch.py` 启动 Cartographer 建图链路。
- 导航：`car_nav2 real_car_nav2.launch.py` 使用 Cartographer localization + Nav2 navigation，不启动 AMCL。

默认 Cartographer 状态地图保存/读取路径：

```text
src/car_nav2/maps/cartographer/latest.pbstream
```

## 1. 环境准备

每次新终端先执行：

```bash
cd /home/mr-cheng/Car_real
source /opt/ros/humble/setup.bash
source install/setup.bash
```

首次使用或代码更新后编译：

```bash
cd /home/mr-cheng/Car_real
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

常用依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-cartographer-ros \
  ros-humble-rviz2
```

## 2. 手动建图

启动实车建图：

```bash
ros2 launch robot_bringup mapping.launch.py
```

建图链路会启动雷达、IMU、底盘控制、RF2O、EKF、Cartographer 和 RViz。建图时可以用键盘遥控小车移动。

建图完成后，用 demo stop 保存 Cartographer `.pbstream`，不要直接 Ctrl-C：

```bash
python3 src/demo/controller_cli.py stop
```

保存结果：

```bash
ls -lh src/car_nav2/maps/cartographer/latest.pbstream
```

如果要保存到自定义路径：

```bash
ROBOT_BRINGUP_PBSTREAM_PATH=src/car_nav2/maps/cartographer/my_map.pbstream \
python3 src/demo/controller_cli.py stop
```

## 3. 单独启动导航

默认读取最新保存的 `latest.pbstream`：

```bash
ros2 launch car_nav2 real_car_nav2.launch.py
```

手动指定 `.pbstream`：

```bash
ros2 launch car_nav2 real_car_nav2.launch.py \
  cartographer_state:=src/car_nav2/maps/cartographer/my_map.pbstream
```

导航链路分工：

```text
Cartographer localization  发布 map -> odom
EKF                         发布 odom -> base_footprint
robot_state_publisher       发布 base_footprint -> base_link -> laser/imu
Nav2 navigation             规划和控制，不启动 AMCL
```

发送一个导航目标：

```bash
python3 src/demo/controller_cli.py goal --x 1.0 --y 0.5 --yaw 0.0
```

停止导航：

```bash
python3 src/demo/controller_cli.py stop
```

## 4. 自动建图

启动自动建图/自动探索：

```bash
python3 src/demo/controller_cli.py mapping
```

自动探索结束或你想停止时，执行：

```bash
python3 src/demo/controller_cli.py stop
```

`stop` 会先调用 Cartographer `/write_state` 保存：

```text
src/car_nav2/maps/cartographer/latest.pbstream
```

然后再停止建图、探索、雷达、底盘、IMU、RF2O、EKF 等进程。

查看当前状态：

```bash
python3 src/demo/controller_cli.py status
```

## 5. 一体化全流程

推荐按下面顺序验证完整流程。

1. 启动自动建图：

```bash
python3 src/demo/controller_cli.py mapping
```

2. 建图结束后保存并停止：

```bash
python3 src/demo/controller_cli.py stop
```

3. 启动导航，默认读取最新保存的 `latest.pbstream`：

```bash
python3 src/demo/controller_cli.py navigation --rviz
```

4. 发送目标点：

```bash
python3 src/demo/controller_cli.py goal --x 1.0 --y 0.5 --yaw 0.0
```

5. 取消当前目标但保留导航进程：

```bash
python3 src/demo/controller_cli.py wait
```

6. 全部停止：

```bash
python3 src/demo/controller_cli.py stop
```

命名点导航使用 `src/demo/named_points.json`，例如：

```bash
python3 src/demo/controller_cli.py a
```

## 6. 快速检查

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

检查速度控制：

```bash
ros2 topic echo /cmd_vel
```

手动发速度排障：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

清理残留进程：

```bash
python3 src/demo/controller_cli.py stop
```
