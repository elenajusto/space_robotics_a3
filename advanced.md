# Advanced Tasks


Read README.md first and set up the environment

For all advanced tasks you must have gazebo and rviz running with this line:
 * ros2 launch cave_explorer cave_explorer_startup.launch.py




## Advanced 1
Follow this launch sequence:
 * ros2 launch cave_explorer cave_explorer_navigation.launch.py
 In a seperate terminal:
 * ros2 run camera_processor camera_processor
Now in another terminal set the params:
 * ros2 param set /camera_processor effect dust
 * ros2 param set /camera_processor severity 2
 * ros2 param set /camera_processor save_every_n 10

 The params for effect can be dust, blur, low_res, motion_blur, lowlight (defualt is none)
 The params for severity range from 0-3 (if set at 0 nothing will happen)
 Any int can be set for save_every_n (e.g. 120, 400, 5, etc)

Open a new terminal and run:
 * rviz2
 Then add two image topic and add the image_after adn image_before topics. You will now be able to view the changes and original image.

 To see the images detecting the artifacts you must run this line:
  * ros2 run model_runner model_runner

 To then move around use this command in another terminal:
 * ros2 run teleop_twist_keyboard teleop_twist_keyboard








## To Run advanced 4
