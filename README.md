# Bessica-D-ROS1

[中文](README_CH.md)

## Prerequisites

Check the robot wire connection:
```bash
ls /dev/ttyUSB*
# Or
ls /dev/ttyCH*
```

### Serial Port Permission

**Temporary method:**
```bash
sudo chmod 666 /dev/ttyUSB*
```

**Recommended (persistent) method:**  
Add your user to the `dialout` group and re-login:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and log back in for the changes to take effect.

## Installation

```bash
mkdir -p ~/bessia/src
cd ~/bessia/src
git clone https://github.com/Xuanya-Robotics/Bessica-D-ROS1.git .
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Usage

```bash
source ~/bessia/devel/setup.bash
roslaunch bessica_d_driver bessica_d_bringup.launch 
```

### Demo

```bash
cd ~/bessia/src/bessica_d_moveit/scripts
python3 demo_move_test.py 
```

Press `q` to exit the program.