# 3d-object-detection
this is a reproduction of my senior's graduation project. It main use yolo to detect object in an image, and cluster the lidar points which are projected to the image. Thus, we can get the 3D boundingbox of the object.

---
## TODO:
- [x] camera intrinsic calibration
- [ ] camera lidar calibration
- [x] yolo object detection
- [x] cloud identification
- [x] cloud clustering 
- [x] the scoring method(distance number iou)
- [x] L shape fitter

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
## Iterative lidar ground filter
![](./resource/ground_filter.png)
```
roslaunch cam_lidar_fusion ground_filter.launch 
```
## Project the lidar cloud to yolo img
Since we have detected the car or person using yolo, and then we project the corresponding lidar points of each object. Different object are colored invidually. 
![](./resource/project_img.png)

## Get the object cloud in rviz
We do this supposing we have already known the cameram intrinsis and extrinsic and you can change it in the ```cam_lidar_fusion/config/cloud_cluster.yaml ```
![](./resource/cloud_project.png) 
## L-shape-fitter
We want to get the yaw angle of the cluster of a car, so i naively use a minimum rectangle to encircle the clouds projected to xy plane, thus i get the yaw angle.
![](./resource/l_shape_fitter.png)
## Final results
I just take one frame of kitti dataset for example
![](./resource/final.png)
![](./resource/demo.gif)

## Test in simulation world
I build a simple dynamic world to check my algorithm
![](./resource/world.gif)
the detect results are as followed, we can see that this algorithm distingguish the front view and background validly.
![](./resource/rviz2.gif)
Here is the detection precision in gazebo world, hopefully, the precision is 80% in average.
![](./resource/detect.png)
Many thanks to my friend Ouyang and my senior Caoming for their code.
