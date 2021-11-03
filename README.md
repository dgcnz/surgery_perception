# Surgery Perception

Run
```sh
roslaunch astra_camera embedded_s.launch
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world_frame camera_link
rosrun surgery_perception perception_node
rosrun rviz rviz
```
