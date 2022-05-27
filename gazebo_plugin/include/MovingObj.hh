#ifndef GAZEBO_PLUGINS_MOVINGObj_HH_
#define GAZEBO_PLUGINS_MOVINGObj_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
class GAZEBO_VISIBLE MovingObj : public ModelPlugin
{
    /// \brief Constructor
public:
    MovingObj();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
public:
    virtual void Reset();

private:
    void OnUpdate();

private:
    physics::ModelPtr model;

private:
    physics::WorldPtr world;

private:
    sdf::ElementPtr sdf;

private:
    ignition::math::Vector3d velocity;

private:
    std::vector<event::ConnectionPtr> connections;

private:
    std::vector<ignition::math::Vector3d> waypoints_vec;

private:
    ignition::math::Vector3d way_point;

private:
    bool forth_flag;

private:
    common::Time lastUpdate;

private:
    double animationFactor = 1.0;

private:
    int target_idx = 0;
};
}  // namespace gazebo
#endif