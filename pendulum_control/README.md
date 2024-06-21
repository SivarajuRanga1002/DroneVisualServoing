# LQR_inverted_pendulum_on_cart_controller
The goal of the application is to control the inverted pendulum on moving cart using Linear Quadratic Regulator (LQR). This application is a part of Assignment for SES 598 : Autonomous Exploration Systems at Arizona State University. 

Code uses ROS Noetic on Ubuntu 20.04


# Demonstration

![Studio_Project](https://user-images.githubusercontent.com/97504177/223634066-f439f299-3e42-4caa-8ac9-f319698d40aa.gif)




# Installation

This package relies on a Gazebo simulation of an inverted pendulum.

```
pip install control
```

```
mkdir -p ~/ros_ws
```

```
git clone https://github.com/Mannat-Rana/LQR-Control-For-Inverted_Pendulum.git
```

```
catkin_make
```



# To Run

```
source ~/ros_ws/devel/setup.bash
```

```
roslaunch lqr_invpend_control lqr_invpend_control.launch
```
```
rosrun invpend_control lqr_control.py
```
# Tweak Parameters

Once simulation opens and lqr_control.py is running, right click on pole link and apply torque or force to test the model.
Tweak the Q and R values in the script, to make the model performance more robust.
