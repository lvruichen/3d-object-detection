#include "MovingObj.hh"

#include <functional>
#include <ignition/math.hh>

#include "gazebo/physics/physics.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MovingObj)

double PI = 3.1415;
// some global variables
/////////////////////////////////////////////////
MovingObj::MovingObj()
{
}

/////////////////////////////////////////////////
void MovingObj::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->sdf = _sdf;
    this->model = _model;
    this->world = this->model->GetWorld();
    this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
        std::bind(&MovingObj::OnUpdate, this)));
    this->Reset();
}

void MovingObj::Reset()
{
    this->lastUpdate = 0;
    if (this->sdf && this->sdf->HasElement("velocity"))
        this->velocity = this->sdf->Get<double>("velocity");
    else
        this->velocity = 1.0;
    if (this->sdf && this->sdf->HasElement("waypoint"))
    {
        sdf::ElementPtr waypointPtr = sdf->GetElement("waypoint");
        while (waypointPtr)
        {
            this->waypoints_vec.push_back(
                waypointPtr->Get<ignition::math::Vector3d>());
            waypointPtr = waypointPtr->GetNextElement("waypoint");
        }
    }
}

/////////////////////////////////////////////////
void MovingObj::OnUpdate()
{
    // delta time
    double dt = (this->world->SimTime() - this->lastUpdate).Double();
    ignition::math::Pose3d current_pose = this->model->WorldPose();
    ignition::math::Pose3d target_pose;
    ignition::math::Vector3d current_rpy, tmp_pos;
    current_rpy = current_pose.Rot().Euler();
    tmp_pos = this->waypoints_vec[target_idx] - current_pose.Pos();
    double distance = tmp_pos.Length();
    if (distance < 0.1)
        target_idx += 1;
    if (target_idx >= waypoints_vec.size())
        target_idx = 0;
    tmp_pos = tmp_pos.Normalize();
    ignition::math::Angle yaw =
        atan2(tmp_pos.Y(), tmp_pos.X()) - current_rpy.Z();
    while (yaw > PI)
    {
        yaw -= 2 * PI;
    }
    while (yaw < -1 * PI)
    {
        yaw += 2 * PI;
    }
    if (yaw.Radian() > IGN_DTOR(10))
    {
        target_pose.Rot() = ignition::math::Quaterniond(
            0, 0,
            current_rpy.Z() + std::max(yaw.Radian() * 0.001, IGN_DTOR(0.1)));
        target_pose.Pos() = current_pose.Pos();
    }
    else if (yaw.Radian() < IGN_DTOR(-10))
    {
        target_pose.Rot() = ignition::math::Quaterniond(
            0, 0,
            current_rpy.Z() + std::min(yaw.Radian() * 0.001, IGN_DTOR(-0.1)));
        target_pose.Pos() = current_pose.Pos();
    }
    else
    {
        target_pose.Pos() = current_pose.Pos() + tmp_pos * this->velocity * dt;
        target_pose.Rot() =
            ignition::math::Quaterniond(0, 0, current_rpy.Z() + yaw.Radian());
    }
    this->model->SetWorldPose(target_pose, false, true);
    this->lastUpdate = this->world->SimTime();
}