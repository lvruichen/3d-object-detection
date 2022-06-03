#include "GetBBox.hh"

#include <gazebo/gazebo_config.h>

#include <ignition/math/Box.hh>
#include <ignition/math/Rand.hh>

// for version
#if GAZEBO_MAJOR_VERSION >= 11
using ign_Box = ignition::math::v6::AxisAlignedBox;
#elif GAZEBO_MAJOR_VERSION >= 9
using ign_Box = ignition::math::v4::Box;
#else
#error GAZEBO VERSION SHOULD BE GREATER THAN 11 OR 9
#endif
vector<string> Car = {"pickup", "ambulance", "hatchback", "hatchback_blue",
                      "suv"};
vector<float> Car_yaw_offset = {0, -M_PI_2, 0, 0, 0};
vector<string> Truck = {"bus", "truck"};
vector<float> Truck_yaw_offset = {-M_PI_2, -M_PI_2};
vector<string> Pedestrian = {"Scrubs", "walking", "FemaleVisitor",
                             "MaleVisitorOnPhone"};
vector<float> Ped_yaw_offset = {0, -M_PI, 0, 0};

void toLowerString(string& str)
{
    for (size_t i = 0; i < str.size(); i++)
    {
        if (isalpha(str[i]))
            str[i] = tolower(str[i]);
    }
}

int getClassType(string str)
{
    for (size_t i = 0; i < Car.size(); i++)
    {
        if (str.find(Car[i]) != str.npos)
            return i;
    }
    for (size_t i = 0; i < Truck.size(); i++)
    {
        if (str.find(Truck[i]) != str.npos)
            return Car.size() + i;
    }
    for (size_t i = 0; i < Pedestrian.size(); i++)
    {
        if (str.find(Pedestrian[i]) != str.npos)
            return Car.size() + Truck.size() + i;
    }
    return -1;
}

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GetBBox)
// Constructor
GetBBox::GetBBox()
{
}

// Destructor
GetBBox::~GetBBox()
{
    this->update_connection_.reset();
    // Finalize the controller
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();
    delete this->rosnode_;
}

// Load the controller
void GetBBox::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // save pointers
    this->world_ = _parent->GetWorld();
    this->sdf = _sdf;

    // this->scene_ = gazebo::rendering::get_scene();

    if (!this->scene_)
    {
        ROS_WARN("no camera or other rendering object in this world.\n"
                 "try to add model://camera at origin by default");
        this->world_->InsertModelFile("model://camera");
    }

    // ros callback queue for processing subscription
    this->deferred_load_thread_ =
        boost::thread(boost::bind(&GetBBox::LoadThread, this));
}
// Load the controller
void GetBBox::LoadThread()
{
    // load parameters
    this->robot_namespace_ = "getbbox_in_gazebo";
    if (this->sdf->HasElement("robotNamespace"))
        this->robot_namespace_ =
            this->sdf->Get<std::string>("robotNamespace") + "/";

    if (!this->sdf->HasElement("serviceName"))
    {
        this->service_name_ = "/getbbox";
    }
    else
        this->service_name_ = this->sdf->Get<std::string>("serviceName");

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_ERROR("ros node for gazebo is not initialed!!");
        return;
    }
    ROS_WARN("ros node for gazebo is initialized!!");
    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GetBBox::UpdateChild, this));
}

bool GetBBox::ServiceCallback(gazebo_plugin::bboxPerFrame::Request& req,
                              gazebo_plugin::bboxPerFrame::Response& res)
{
    res.box_num = 0;
    if (req.msg == "bbox")
    {
        for (size_t i = 0; i < this->world_->ModelCount(); i++)
        {
            physics::ModelPtr model = this->world_->ModelByIndex(i);
            string model_name = model->GetName();

            int idx = getClassType(model_name);

            if (idx >= 0)
            {
                // std::cout << "model class is: " << model_name << std::endl;
                gazebo_plugin::bbox bbox;
                // this->scene_ = rendering::get_scene();
                if (!this->scene_ || !this->scene_->Initialized())
                    return false;

                rendering::VisualPtr vis = this->scene_->GetVisual(model_name);
                ign_Box tmp_bbox = vis->BoundingBox();
                float yaw_offset;

                if (idx < Car.size())
                {
                    bbox.id = 1;
                    yaw_offset = Car_yaw_offset[idx];
                }
                else if (idx < Car.size() + Truck.size())
                {
                    bbox.id = 2;
                    idx -= Car.size();
                    yaw_offset = Truck_yaw_offset[idx];
                }
                else
                {
                    bbox.id = 3;
                    idx -= (Car.size() + Truck.size());
                    yaw_offset = Ped_yaw_offset[idx];
                }

                // ROS_INFO("(x, y, z) = (%f, %f, %f)", tmp_bbox.Center().X(),
                // tmp_bbox.Center().Y(), tmp_bbox.Center().Z());
                bbox.xyz.push_back(tmp_bbox.XLength());
                bbox.xyz.push_back(tmp_bbox.YLength());
                bbox.xyz.push_back(tmp_bbox.ZLength());

                double yaw = vis->Pose().Rot().Yaw();

                double cx_in_world = cos(yaw) * tmp_bbox.Center().X() -
                                     sin(yaw) * tmp_bbox.Center().Y();
                double cy_in_world = sin(yaw) * tmp_bbox.Center().X() -
                                     cos(yaw) * tmp_bbox.Center().Y();

                bbox.yaw = model->WorldPose().Rot().Yaw() + yaw_offset;

                bbox.pos.push_back(vis->Pose().Pos().X() + cx_in_world);
                bbox.pos.push_back(vis->Pose().Pos().Y() + cy_in_world);
                bbox.pos.push_back(vis->Pose().Pos().Z() +
                                   tmp_bbox.Center().Z());

                res.box_num++;

                res.bboxes.push_back(bbox);
            }
        }
    }
    return true;
}

// Update the controller
void GetBBox::UpdateChild()
{
    // this->scene_ = gazebo::rendering::get_scene();
    if (this->scene_ == NULL)
    {
        // ROS_WARN("rendering has been created. try to get scene handle once
        // again ...");
        this->scene_ = gazebo::rendering::get_scene();
        if (this->scene_)
        {
            ROS_WARN("get scene handle! begin to search bbox");
            // advertise services on the custom queue
            ros::AdvertiseServiceOptions aso =
                ros::AdvertiseServiceOptions::create<
                    gazebo_plugin::bboxPerFrame>(
                    this->service_name_,
                    boost::bind(&GetBBox::ServiceCallback, this, _1, _2),
                    ros::VoidPtr(), &this->imu_queue_);
            this->srv_ = this->rosnode_->advertiseService(aso);
            // start custom queue for imu
            this->callback_queue_thread_ =
                boost::thread(boost::bind(&GetBBox::QueueThread, this));
        }
        else
        {
            return;
        }
    }
}

// Put laser data to the interface
void GetBBox::QueueThread()
{
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
        this->imu_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

}  // namespace gazebo