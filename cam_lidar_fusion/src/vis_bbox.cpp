#include "vis_bbox.h"

namespace vis_utils
{
void make_visualize_traj(int traj_id, ros::Time stamp,
                         const vector<Vector3d>& traj, rosTraj& rT)
{
    rT.line.id = traj_id;
    rT.line.lifetime = ros::Duration(0.5);
    rT.line.header.stamp = stamp;
    rT.line.points.clear();
    for (int i = 0; i < traj.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = traj[i](0);
        p.y = traj[i](1);
        p.z = traj[i](2);

        rT.line.points.emplace_back(p);
    }
}

visualization_msgs::Marker getBBox(
    int box_id, ros::Time stamp, string frame_id, const vector<double>& color,
    const vector<pcl::PointCloud<pcl::PointXYZ>>& bBoxes)
{
    assert(color.size() == 3);
    visualization_msgs::Marker line_list;

    line_list.header.frame_id =
        frame_id;  // 定义frame_id (rviz需要设置世界坐标系为velo_link)
    line_list.header.stamp = stamp;
    line_list.ns = "boxes";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = box_id;
    line_list.type =
        visualization_msgs::Marker::LINE_LIST;  //线条序列  type设置类型

    // LINE_LIST markers use only the x component of scale, for the line width
    // 仅将比例的x分量用于线宽
    line_list.scale.x = 0.1;
    // Points are green
    line_list.color.r = color[0];
    line_list.color.g = color[1];  // 边框绿色
    line_list.color.b = color[2];
    line_list.color.a = 1.0;

    line_list.lifetime = ros::Duration(0.5);

    int id = 0;
    string ids;
    for (int objectI = 0; objectI < bBoxes.size(); objectI++)
    {  // 多少个边界框,循环几次
        for (int pointI = 0; pointI < 4; pointI++)
        {  //内循环4次??
            assert((pointI + 1) % 4 < bBoxes[objectI].size());
            assert((pointI + 4) < bBoxes[objectI].size());
            assert((pointI + 1) % 4 + 4 < bBoxes[objectI].size());

            id++;
            ids = to_string(id);

            geometry_msgs::Point p;  // 定义p
            p.x = bBoxes[objectI][pointI].x;
            p.y = bBoxes[objectI][pointI].y;
            p.z = bBoxes[objectI][pointI].z;
            line_list.points.push_back(p);  // 给line_lists添加点!!!!
            p.x = bBoxes[objectI][(pointI + 1) % 4].x;  // 取余4
            p.y = bBoxes[objectI][(pointI + 1) % 4].y;
            p.z = bBoxes[objectI][(pointI + 1) % 4].z;
            line_list.points.push_back(p);

            p.x = bBoxes[objectI][pointI].x;
            p.y = bBoxes[objectI][pointI].y;
            p.z = bBoxes[objectI][pointI].z;
            line_list.points.push_back(p);
            p.x = bBoxes[objectI][pointI + 4].x;  // 加4?
            p.y = bBoxes[objectI][pointI + 4].y;
            p.z = bBoxes[objectI][pointI + 4].z;
            line_list.points.push_back(p);

            p.x = bBoxes[objectI][pointI + 4].x;
            p.y = bBoxes[objectI][pointI + 4].y;
            p.z = bBoxes[objectI][pointI + 4].z;
            line_list.points.push_back(p);
            p.x = bBoxes[objectI][(pointI + 1) % 4 + 4].x;
            p.y = bBoxes[objectI][(pointI + 1) % 4 + 4].y;
            p.z = bBoxes[objectI][(pointI + 1) % 4 + 4].z;
            line_list.points.push_back(p);
        }
    }
    return line_list;
}

visualization_msgs::Marker getPoint(string frame_id, int id,
                                    vector<double>& color,
                                    const vector<vector<double>>& trackPoints)
{
    visualization_msgs::Marker points;

    points.header.frame_id = frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points";

    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = id;
    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    points.color.r = color[0];
    points.color.g = color[1];  // 边框绿色
    points.color.b = color[2];
    points.color.a = 1.0;

    //  cout << "targetPoints.size() is --=------" << targetPoints.size()
    //  <<endl;

    for (int i = 0; i < trackPoints.size(); i++)
    {
        for (size_t j = 0; j < trackPoints[i].size() / 2; j++)
        {
            geometry_msgs::Point p;
            p.x = trackPoints[i][2 * j];
            p.y = trackPoints[i][2 * j + 1];
            p.z = 2.0;
            points.points.emplace_back(p);
        }
    }

    return points;
}

visualization_msgs::Marker getArrow(string frame_id, int id,
                                    vector<double>& color,
                                    const vector<double>& startPoint, double v,
                                    double yaw)
{
    visualization_msgs::Marker arrowsG;
    arrowsG.lifetime = ros::Duration(0.2);
    arrowsG.header.frame_id = frame_id;

    arrowsG.header.stamp = ros::Time::now();
    arrowsG.ns = "arrows";
    arrowsG.action = visualization_msgs::Marker::ADD;
    arrowsG.type = visualization_msgs::Marker::ARROW;

    // green  设置颜色
    arrowsG.color.g = color[0];
    arrowsG.color.r = color[1];  // 红色
    arrowsG.color.b = color[2];  // 红色
    arrowsG.color.a = 1.0;
    arrowsG.id = id;

    double tv = v;
    double tyaw = yaw;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the
    // frame/time specified in the header
    arrowsG.pose.position.x = startPoint[0];
    arrowsG.pose.position.y = startPoint[1];
    arrowsG.pose.position.z = startPoint[2];

    // convert from 3 angles to quartenion
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(tyaw, 0, 0);  // yaw, pitch, roll
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    arrowsG.pose.orientation.x = q_tf.getX();
    arrowsG.pose.orientation.y = q_tf.getY();
    arrowsG.pose.orientation.z = q_tf.getZ();
    arrowsG.pose.orientation.w = q_tf.getW();

    // Set the scale of the arrowsG -- 1x1x1 here means 1m on a side
    // arrowsG.scale.x = tv;
    arrowsG.scale.x = 3;
    arrowsG.scale.y = 0.3;
    arrowsG.scale.z = 0.3;

    return arrowsG;
}

visualization_msgs::Marker getText(const string& frame_id, const string& msg,
                                   const vector<double>& color)
{
    visualization_msgs::Marker label_marker;
    label_marker.lifetime = ros::Duration(0.5);
    label_marker.header.stamp = ros::Time::now();
    label_marker.header.frame_id = frame_id;
    label_marker.ns = "text";
    label_marker.action = visualization_msgs::Marker::ADD;
    label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label_marker.scale.x = 2.0;
    label_marker.scale.y = 2.0;
    label_marker.scale.z = 2.0;

    label_marker.color.r = color[0];
    label_marker.color.g = color[1];
    label_marker.color.b = color[2];
    label_marker.color.a = 1.0;

    label_marker.id = -1;
    label_marker.text = msg;

    label_marker.pose.position.x = 0.0;
    label_marker.pose.position.y = 0.0;
    label_marker.pose.position.z = 2.0;
    label_marker.scale.z = 2.0;

    return label_marker;
}

// visualization_msgs::MarkerArray getLabels(const object_tracking::resboxes&
// tbox)
// {
//   visualization_msgs::MarkerArray label_markers;
//   for (auto const &e: tbox.rb)
//   {

// 	  visualization_msgs::Marker label_marker;
//       label_marker.lifetime = ros::Duration(0.5);
//       label_marker.header = tbox.header;
//       label_marker.ns = "label_markers";
//       label_marker.action = visualization_msgs::Marker::ADD;
//       label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//       label_marker.scale.x = 1.5;
//       label_marker.scale.y = 1.5;
//       label_marker.scale.z = 1.5;

//       label_marker.color.r = 1.0;
//       label_marker.color.g = 1.0;
//       label_marker.color.b = 1.0;
//       label_marker.color.a = 1.0;

//       label_marker.id = e.id;

// 	  string text_format = "Id: %d\nVel: %.2fm/s\nYaw: %.3f";
// 	  char text[64];
// 	  sprintf(text, text_format.c_str(), e.id, e.vel, e.yaw);
// 	  label_marker.text = string(text);
//       //if(!object.label.empty() && object.label != "unknown")
//         //label_marker.text = object.label + " "; //Object Class if available

//       //std::stringstream distance_stream;
//       //distance_stream << std::fixed << std::setprecision(1)
//                       //<< sqrt((object.pose.position.x *
//                       object.pose.position.x) +
//                                 //(object.pose.position.y *
//                                 object.pose.position.y));
//       //std::string distance_str = distance_stream.str() + " m";
//       //label_marker.text += distance_str;

//       //if (object.velocity_reliable)
//       //{
//         //double velocity = object.velocity.linear.x;
//         //if (velocity < -0.1)
//         //{
//           //velocity *= -1;
//         //}

//         //if (abs(velocity) < object_speed_threshold_)
//         //{
//           //velocity = 0.0;
//         //}

//         //tf::Quaternion q(object.pose.orientation.x,
//         object.pose.orientation.y,
//                          //object.pose.orientation.z,
//                          object.pose.orientation.w);

//         //double roll, pitch, yaw;
//         //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

//         //// convert m/s to km/h
//         //std::stringstream kmh_velocity_stream;
//         //kmh_velocity_stream << std::fixed << std::setprecision(1) <<
//         (velocity * 3.6);
//         //std::string text = "\n<" + std::to_string(object.id) + "> " +
//         kmh_velocity_stream.str() + " km/h";
//         //label_marker.text += text;
//       //}

// 		label_marker.pose.position.x = e.traj_x.at(0);
// 		label_marker.pose.position.y = e.traj_y.at(0);
// 		label_marker.pose.position.z = 1.0;
// 		label_marker.scale.z = 1.0;
// 		if (!label_marker.text.empty())
//       	label_markers.markers.push_back(std::move(label_marker));
//   }  // end in_objects.objects loop
//   return label_markers;
// }//ObjectsToLabels

}  // namespace vis_utils