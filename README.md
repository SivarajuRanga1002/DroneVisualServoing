# DroneVisualServoing
A Pixhawk drone equipped with cameras is used to map a precariously placed rock using ORB-SLAM and further landing on a moving rover detecting the aruco marker placed inside it

This code is part of the midterm project for the SES 598 Autonomous Exploration System Course at Arizona State University (ASU). The task includes navigating a quadcopter towards a probe to perform data muling, then proceeding to a precariously balanced rock to orbit it and generate a PointCloud map using ORBSLAM2, before safely landing on a Rover.

The first phase of the world features a Martian terrain setup that includes a bishop, a probe, a precariously balanced rock, a rover, and a quadcopter. The objectives are laid out as follows, in sequential order:

- Activate offboard mode and navigate to the probe for data muling, maintaining a minimum distance of 1 meter.
- Once data muling is complete, approach the rock to comprehensively map it using OrbSlam2.
- After completing the map, proceed to a rover that is moving randomly and execute a safe landing on it.
- All these tasks must be carried out autonomously, without any human intervention throughout the process.

