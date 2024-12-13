# Pool-inator
Authors: An Nguyen, Caroline Terryn, Catherine Maglione, Joseph Blom, Logan Boswell

This ROS package is used to make a Franka Emika Robot (FER) play a game of pool on a tabletop pool set for the final project of ME 495: Embedded Systems in Robotics at Northwestern University.


## Quickstart
1. With the Franka in blue-light mode and MoveIt launched, use `ros2 launch poolinator control.launch.xml` to start the image processing and control nodes.
2. Use `ros2 service call /strike_cue std_srvs/srv/Empty` with red and blue balls on the table to start the game of pool.
3. If the red ball gets knocked into a pocket or off the table, place it back on the table and run `ros2 service call /strike_cue std_srvs/srv/Empty` again.
4. Once all of the blue balls have been knocked into a pocket, then the Franka will knock the red ball into a pocket.

## Nodes
#### /transform:
this node establishes the link between the camera fram and the robot base frame

#### /image_processor_colors:
this node identifies the pool balls using computer vision

#### /control:
this node sends sends commands to the Franka to make it hit balls, move to the home configuration, etc.

## Launchfiles
#### image.laumch.xml:
this file launches nodes necessary for computer vision and an Rviz window that includes the frames of the balls and shows images in the bottom-left corner

#### control.launch.xml:
this file includes image.launch.xml and also starts the control node

## System Overview
The system consists of the Franka arm holding a cue with an apriltag, a realsense camera, and a tabletop pool table with corresponding apriltag. When the command `ros2 launch poolinator control.launch.xml` is run, the /transform node publishes the transform between the camera and the /image_processor_colors node identifies the pool balls. The once the /strike_cue service is called /control node commands the Franka arm to play through a game of pool using MoveIt. The game continues until all of the blue balls are knocked into pockets, and then the red ball is pocketed. If the red ball goes into any of the pockets or accidentally knocked off the table, the Franka will wait until the red ball is replaced and the /strike_cue service is called once again.

## Videos
### Example Game:
[poolinator.mp4](https://github.com/user-attachments/assets/101fc89d-c338-4b47-8d1b-a6687bba9aeb)

### Example Rviz Window:
[poolinator_rviz.webm](https://github.com/user-attachments/assets/7127c4b6-e01d-4338-998d-003e6e6acb7b)

## Testing
Tests can be run with `colcon test`
