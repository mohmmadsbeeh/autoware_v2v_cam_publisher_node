# autoware_v2v_cam_publisher_node
 to create the container and configure it  the Network as Host

 docker run -it --network host autoware_cam_publisher_image

 to run the Node in normal mode :
 ros2 launch  autoware_v2x_cam_publisher cam_publisher_launch.py

 to run the Node in Debuging mode to see which if the the Containers are  populated  and the CAM msg is published :

 ros2 run autoware_v2x_cam_publisher cam_publisher_node --ros-args --params-file ./src/autoware_v2v_cam_publisher_node/params.yaml --log-level DEBUG
 