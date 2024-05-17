# gd_calib_capture

Capture and save camera and 3D LiDAR topics. 

* **Parameters**
  - camera_topic: Image topic name of camera.
  - lidar_topic:  PointCloud2 topic name of 3D LiDAR
  - image_file_prefix: image
  - pcd_file_prefix: pointcloud

* **Install**
  * pip install opencv-python open3d

* **Execution**
  * ros2 launch gd_calib_capture calib_capture_launch.py camera_topic:=/gd/front_color_image lidar_topic:=/gd/lidar_pointcloud
