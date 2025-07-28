# Bessica-D-ROS1
[English](README.md)
## 安装

```bash
mkdir -p ~/bessia/src
cd ~/bessia/src
git clone https://github.com/Xuanya-Robotics/Bessica-D-ROS1.git .
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## 使用方法

```bash
source ~/bessia/devel/setup.bash
roslaunch bessica_d_driver bessica_d_bringup.launch 
```

### 演示

```bash
cd ~/bessia/src/bessica_d_moveit/scripts
python3 demo_move_test.py 
```

按下 `q` 键退出程序。