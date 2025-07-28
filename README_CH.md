# Bessica-D-ROS1

[English](README.md)

## 前置条件

检查机器人串口连接：
```bash
ls /dev/ttyUSB*
# 或
ls /dev/ttyCH*
```

### 串口权限设置

**临时方法：**
```bash
sudo chmod 666 /dev/ttyUSB*
```

**推荐（持久）方法：**  
将当前用户加入 `dialout` 组，然后重新登录：
```bash
sudo usermod -a -G dialout $USER
```
重新登录后生效。

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