# Pool-inator
This ROS package is used to make a Franka Emika Robot (FER) play a game of pool on a tabletop pool set for the final project of ME 495: Embedded Systems in Robotics at Northwestern University.

Authors: An Nguyen, Caroline Terryn, Catherine Maglione, Joseph Blom, Logan Boswell


## Quickstart
1. With the Franka in blue-light mode and MoveIt launched, use `ros2 launch poolinator control.launch.xml` to start the image processing and control nodes.
2. Use `ros2 service call /strike_cue std_srvs/srv/Empty` with red and blue balls on the table to start the game of pool.
3. If the red ball gets knocked into a pocket or off the table, place it back on the table and run `ros2 service call /strike_cue std_srvs/srv/Empty` again.
4. Once all of the blue balls have been knocked into a pocket, then the Franka will knock the red ball into a pocket.

## Videos
### Example Game:
[poolinator.mp4](https://github.com/user-attachments/assets/c430b572-f40c-46c5-9905-023ac7beba72)

### Rviz Window:
[poolinator_rviz.webm](https://github.com/user-attachments/assets/b7c43b19-c84e-4187-915d-9dea66f87472)

## Contributing
Please Please Please Please work on a seperate branch and only merge (or preferably PR) to main with someone else to check it first.

## Testing
Tests can be run with `colcon test`
