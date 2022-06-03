#!/home/eric/miniconda3/envs/torch/bin/python
import rospy
from yolo_bridge_srv import ROS2YOLO

if __name__ == "__main__":
    rospy.init_node('ros_yolo')
    yoloBridge = ROS2YOLO()
    rospy.spin()