#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "gazebo_plugin/bboxPerFrame.h"
#include "yolo_ros/BoundingBox.h"
#include "yolo_ros/BoundingBoxes.h"
#include "yolo_ros/ObjectCount.h"
using namespace std;
class TestDetection
{
public:
    TestDetection(ros::NodeHandle& nh);
    ~TestDetection(){};

private:
    void call_back(
        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr bbox_array);
    ros::NodeHandle nh_;
    ros::Subscriber detection_result_sub_;
    ros::Publisher accuration_pub_;
    ros::ServiceClient client;
};

TestDetection::TestDetection(ros::NodeHandle& nh) : nh_(nh)
{
    detection_result_sub_ = nh_.subscribe("bounding_boxes_lidar", 10,
                                          &TestDetection::call_back, this);
    accuration_pub_ =
        nh_.advertise<std_msgs::Float32>("/detection_accuration", 10);
    client = nh_.serviceClient<gazebo_plugin::bboxPerFrame>("getbbox");
    ros::service::waitForService("getbbox");
}

double getDistance(cv::Point3d pointO, cv::Point3d pointA)

{
    double distance;
    pointO.x -= 0.258;
    pointO.z -= 1.216;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);

    distance = sqrtf(distance);

    return distance;
}

void TestDetection::call_back(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr bbox_array)
{
    int num = bbox_array->boxes.size();
    vector<cv::Point3d> detect_centers;
    for (int i = 0; i < num; i++)
    {
        cv::Point3d tmp_center(bbox_array->boxes[i].pose.position.x,
                               bbox_array->boxes[i].pose.position.y,
                               bbox_array->boxes[i].pose.position.z);
        detect_centers.push_back(tmp_center);
    }
    cout << "detected " << num << " objetcs" << endl;
    gazebo_plugin::bboxPerFrame true_box;
    true_box.request.msg = "bbox";
    bool flag = client.call(true_box);
    vector<cv::Point3d> true_centers;
    if (flag)
    {
        // ROS_INFO("Has received the true box");
        int true_num = true_box.response.box_num;
        for (int i = 0; i < true_num; i++)
        {
            double pos_x = true_box.response.bboxes[i].pos[0] + 0.223;
            double pos_y = true_box.response.bboxes[i].pos[1];
            double pos_z = true_box.response.bboxes[i].pos[2] + 1.212;
            cv::Point3d true_center(pos_x, pos_y, pos_z);
            true_centers.push_back(true_center);
        }
    }
    int success = 0;
    int fail = 0;
    if (!detect_centers.empty() && !true_centers.empty())
    {
        // start compute the accuration

        for (int i = 0; i < detect_centers.size(); i++)
        {
            for (int j = 0; j < true_centers.size(); j++)
            {
                double distance =
                    getDistance(detect_centers[i], true_centers[j]);
                if (distance < 2)
                    success++;
            }
        }
    }
    float accuracy = float(success) / float(detect_centers.size());
    std_msgs::Float32 accu_msg;
    accu_msg.data = accuracy;
    ROS_INFO("the accuracy is: %d, %d", success, detect_centers.size());
    accuration_pub_.publish(accu_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_detection");
    ros::NodeHandle nh("");
    TestDetection test_detect(nh);
    ros::spin();
    return 0;
}