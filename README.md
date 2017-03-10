# Confidence Aware Occupancy Grid Mapping
## Gym Environment
ROS package implementing 3D mapping and planning using SMAP.

![Replanning](replanning.gif)

Example scenario showing replanning. The robot has only a single pixel (range) sensor (red arrow)
and executes the current trajectory (blue). When the estimated reachability goes below a certain threshold
within the next 5 time steps, the best trajectory is chosen from a set of trajectory candidates (green).
The robot's position trace is highlighted in yellow.

