#ifndef _VIS_BBOX_H_
#define _VIS_BBOX_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <iostream>

#include "tf/transform_datatypes.h"
using namespace std;
using namespace Eigen;
namespace vis_utils
{
struct rosTraj
{
    visualization_msgs::Marker line;

    rosTraj(string frame_id, ros::Time stamp)
    {
        std_msgs::Header hd;
        hd.frame_id = frame_id;
        hd.stamp = stamp;

        line.header = hd;
        line.ns = "traj";
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;

        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.scale.x = 0.05;
        line.color.r = 1.0;
        line.color.g = 1.0;
        line.color.b = 0.0;
        line.color.a = 1.0;
    }
};

visualization_msgs::Marker getText(const string& frame_id, const string& msg,
                                   const vector<double>& color);

void make_visualize_traj(int traj_id, ros::Time stamp,
                         const vector<Vector3d>& traj, rosTraj& rT);

visualization_msgs::Marker getBBox(
    int box_id, ros::Time stamp, string frame_id, const vector<double>& color,
    const vector<pcl::PointCloud<pcl::PointXYZ>>& bBoxes);

visualization_msgs::Marker getPoint(string frame_id, int id,
                                    vector<double>& color,
                                    const vector<vector<double>>& trackPoints);

visualization_msgs::Marker getArrow(string frame_id, int id,
                                    vector<double>& color,
                                    const vector<double>& startPoint, double v,
                                    double yaw);

// visualization_msgs::MarkerArray
// getLabels(const object_tracking::resboxes& tbox);

}  // namespace vis_utils

#endif