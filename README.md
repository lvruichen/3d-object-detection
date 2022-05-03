# 3d-object-detection
this is a reproduction of my senior's graduation project

---
## TODO:
- [x] camera intrinsic calibration
- [ ] camera lidar calibration
- [x] yolo object detection
- [ ] cloud identification
- [ ] cloud clustering 

## Environments:
- PyTorch 1.10
- Cuda 11.3
- device NVIDIA RTX 3050ti
## YOLOv5 detection
change the topic in ```yolo_ros/config/config.yaml``` or you can run by default using laptop camera
``` 
roslaunch usb_cam usb_cam-test.launch 
roslaunch yolo_ros demo.launch
```
![](./resource/YOLOV5.png)
## lidar camera fusion
change the lidar and camera topic in ```colored_pointcloud/config/calib_results.yaml```. This package will project the cloud to image and color the cloud.
```
roslaunch colored_pointcloud colored_poincloud_node.launch
```
![](./resource/cam_lidar_fusion.png)
Many thanks to my friend oyjy for his code.
