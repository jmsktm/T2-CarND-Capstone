[//]: # (Image References)
[arch_image]: ./resources/carla_architecture.png
[vehicle]: ./resources/vehicle.jpg

# Udacity Self-Driving Car Engineer Nanodegree

![][vehicle]

## Team: Project Herbie
<table>
 <tr>
    <th>Member</th>
    <th>Email</th>
    <th>&nbsp;</th>
 </tr>
 <tr>
    <td>James Singh</td>
    <td>james.singh@hotmail.com</td>
    <td><img src="resources/james.jpg" width="150px" > </td>
 </tr>
 <tr>
    <td>Emil Ibrahim</td>
    <td>&nbsp;</td>
    <td><img src="resources/emil.jpg" width="150px" > </td>
 </tr>
 <tr>
    <td>Mariam Swetha George</td>
    <td>mariam.george@nexteer.com</td>
    <td><img src="resources/mariam.jpg" width="150px" > </td>
 </tr>
</table>

This is the Capstone Project for Udacity's Self-Driving Car NanoDegree Program. We as a team used ROS (Robotic Operating System) to implement several nodes that built out the core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. This software we developed will be tested on Carla, the Udacity Self Driving Car, which should be able to drive autonomously on a test track.

For setup and development contribution, refer to [CONTRIBUTING.md](./CONTRIBUTING.md)

## ROS Implementation

ROS is an open-source, meta-operating system used for controlling robotics. It provides the services one would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. ROS *processes* are represented as *nodes* in a graph structure, connected by edges called *topics*. ROS nodes can pass messages to one another through topics, make service calls to other nodes, provide a service for other nodes, or set or retrieve shared data from a communal database.

ROS was used as the backbone for this project and the following is a system architecture diagram showing the ROS nodes and topics used in the project:

![][arch_image]

Each of the nodes were coded in Python for the project and all the code can be found in the `ros/src` directory.
### Perception

The perception subsystem is handled by two ROS nodes ``obstacle detection`` and ``traffic light detection``. For the purposes of this project, we focused on the traffic light detection node. This node subscribes to the following topics:
* `/base_waypoints` : provides the complete list of waypoints for the course.
* `/current_pose` : provides the vehicle's current location.
* `/image_color` : provides an image stream from the car's camera which are then used to determine the traffic lights status
* `/vehicle/traffic_lights` : provides the (x, y, z) coordinates of all traffic lights.
This node publishes then the index of the waypoint for nearest upcoming red light's stop line to the topic: `/traffic_waypoint` which is then used by the planning subsystem to decide on a stop or not

### Planning

The planning subsystem comprises of two ROS nodes `waypoint loader` and `waypoint updater`. The former was already provided as a package that loads the static waypoint data and publishes to `/base_waypoints` topic.

The `waypoint updater` node updates the target velocity property of each waypoint based on traffic light and obstacle detection data. This node subscribes to the following topics :
* `/base_waypoints`
* `/current_pose` 
* `/traffic_waypoint`
The node then publishes to the `final_waypoints` topic a list of waypoints ahead of the car with target velocities. This feeds into the control subsystem which ultimately executes maneuvers on the car.

### Control

The control subsystm consists of two ROS nodes `DBW` and `waypoint follower`. Here too the later was already as a package from Autoware which subscribes to the `/final_waypoints` topic and then publishes the target linear and angular velocities of the vehicle in the form of twist commands to the `/twist_cmd` topic.

The `DBW` node controls the drive-by-wire system in Carla which allows for electronic control of the throttle, brake and steering. This node subscribes to:
* `/current_velocity` : provides linear velocity 
* `/twist_cmd` : provides angular velocity
* `/vehicle/dbw_enabled` : identifies if the car is under dbw or driver control
The node utilizes a variety of controllers to then to provide the appropriate throttle, brake, and steering commands. These commands are then published to the appropriate topics: `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and`/vehicle/steering_cmd`


### Code Flow

Each of these nodes are associated with python code that allows us to control and modify their behavior. In order to get the ideal behavior for the car, we modified the code in the following manner:

1. **Waypoint Updater node (Partial)**: Created a node that subscribes to `/base_waypoints` and `/current_pose` and publishes to `/final_waypoints`. The node will eventually  publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles. But for a first pass, to enable movement, we added subscription to the topics and also publish just a fixed number of waypoints currently ahead of the vehicle. This is achieved by modifiying the `ros/src/waypoint_updater/waypoint_updater.py`
2. **DBW node**: Once messages are being published to `/final_waypoints`, the vehicle's waypoint follower will publish twist commands to the `/twist_cmd` topic. The `/twist_cmd` is then used by the `DBW` node which uses various controllers to provide appropriate throttle, brake, and steering commands. This is achieved by modifying the `ros/src/twist_controller/dbw_node.py` which implements the subscription and publishing. The code also imports the `Controller` class from `ros/src/twist_controller/twist_controller.py` which will be used for implementing the necessary controller for the car.
3. **Traffic Light Detection node**: For intial testing purposes the `/vehicle/traffic_lights` topic provides realtime updates for the current location and status of all the traffic lights in the simulation. This is then used to implememt the complete `Waypoint Updater` node (Step 4) to ensure that the car stops appropriately and safely for the different traffic lights. This is achieved by setting the keyword `TEST_MODE_ENABLED` to be *True*.

    Once that is done, the `tl_detector.py` and `tl_classifier.py` codes are further modified for:
    * `tl_detector.py`: To detect the incoming camera images and the traffic light data. This code utilizes the light classifier to get a color prediction, and publishes the location of any upcoming red lights.
    * `tl_classifier.py`: This code implements the traffic light classication which involves both the identification of traffic lights and then classify them with as *red*, *yellow* and *green*. Please refer to the traffic light detection section for more in-depth detail of the our algorithm
4. **Waypoint Updater node (Full)**: Once traffic light detection has been implement, it is incorporated into the `waypoint updater`. Now we can, adjust the target velocities for the waypoints leading up to red traffic lights or other obstacles in order to bring the vehicle to a smooth and full stop. In addition to this a subscriber for the `/traffic_waypoint` topic.
