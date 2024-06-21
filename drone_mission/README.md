# Rocky-Times-Challenge-SES598-ASU
This code is the part of midterm assignment for SES 598 Autonomous Exploration System Course at Arizona State University (ASU). The project involves moving the quadcopter close to a probe for data mulling, later move towards precariously balanced rock, revolve around it and create PointCloud map using ORBSLAM2 and land safely on Rover.

# Challenge details and objectives
<img width="837" alt="Screenshot 2023-05-03 at 10 33 41 PM" src="https://user-images.githubusercontent.com/97504177/236119781-4f0b61ad-53ec-4eb5-a5ae-0b564a6eda3f.png">
The phase-1 world consist of bishop mars terrain along with a probe, a precariously balanced rock, a rover and a quadcopter.
The task includes below points in the given sequence
1. Set offbaord and go to the probe for data mulling (distanceThreshold = 1 meter)
2. After data mulling is completed, move closer to rock and map the entire rock using OrbSlam2
3. Once mapping is completed, move towards the randomly moving rover and land on it safely.
4. All the above sequence of action must happened autonomoulsy without human intervention in between.

# Demonstration

https://user-images.githubusercontent.com/97504177/236119033-6c1accd3-45f5-4fe9-8ea5-d87159a41130.mp4

# Landing Trails
![Studio_Project (2)](https://user-images.githubusercontent.com/97504177/236709060-351487da-2213-4cca-b190-518aad6e828a.gif)

# Installation

Register on https://cps-vo.org/group/CPSchallenge to get access to pre-configured docker container with 3D graphics, ROS, Gazebo, and PX4 flight stack.
Or you can clone and build using https://github.com/Open-UAV/cps_challenge_2020.git

Install ORBSLAM2 by following the github repository
```
https://github.com/raulmur/ORB_SLAM2.git
```

Follow below instruction to clone the midterm.py in ./scripts/ folder
```
cd cps_challenge/scripts
```
```
git clone https://github.com/skp-1997/Rocky-Times-challenge-SES598-ASU.git
```

# Running the code

Run the phase-1 world of gazebo using the command
```
roslaunch cps_challenge_2020 phase-1.launch
```

Run the ORBSLAM2 using the command
```
rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE /camera/image_raw:='your camera topic'
```

Run the program using the command
```
rosrun cps_challenge_2020 midterm.py or python midterm.py
```

# Pre-requisites

1. Python 2.7
2. Ubuntu 18.04
3. ROS1 MELODIC
4. Gazebo 9.6
