#include "gazebo_odometry_plugin.h"

namespace gazebo {                                            

GazeboOdometryPlugin::~GazeboOdometryPlugin() {
    update_connection_->~Connection();
}

void GazeboOdometryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Store the pointer to the model and world.
    model_ = _model;
    world_ = model_->GetWorld();

    namespace_.clear();

    // Get parameters.
    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("linkName"))
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
        gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
    
    // Initialize the node_handle.
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    // Pointer to the link, to be able to apply forces and torques on it.
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboOdometryPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboOdometryPlugin::OnUpdate(const common::UpdateInfo& _info) {
    // Connect to ROS
    if(!pubs_and_subs_created_) {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }

    // Obtain current state
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time  = world_->SimTime();
#else
    common::Time current_time  = world_->GetSimTime();
#endif
    ignition::math::Pose3d w_pose = link_->WorldPose();
    ignition::math::Vector3d w_vel = link_->WorldLinearVel();
    ignition::math::Vector3d w_ang_vel = link_->RelativeAngularVel();


    // Prepare values
    gazebo::msgs::Vector3d* pos = new gazebo::msgs::Vector3d();
    pos->set_x(w_pose.Pos().X());
    pos->set_y(w_pose.Pos().Y());
    pos->set_z(w_pose.Pos().Z());
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    orientation->set_w(w_pose.Rot().W());
    orientation->set_x(w_pose.Rot().X());
    orientation->set_y(w_pose.Rot().Y());
    orientation->set_z(w_pose.Rot().Z());
    gazebo::msgs::Vector3d* lin_vel = new gazebo::msgs::Vector3d();
    lin_vel->set_x(w_vel.X());
    lin_vel->set_y(w_vel.Y());
    lin_vel->set_z(w_vel.Z());
    gazebo::msgs::Vector3d* ang_vel = new gazebo::msgs::Vector3d();
    ang_vel->set_x(w_ang_vel.X());
    ang_vel->set_y(w_ang_vel.Y());
    ang_vel->set_z(w_ang_vel.Z());

    // Publish odometry
    nav_msgs::msgs::Odometry odom;
    odom.set_time_usec(current_time.Double() * 1000000.0);
    odom.set_allocated_position(pos);
    odom.set_allocated_linear_velocity(lin_vel);
    odom.set_allocated_angular_velocity(ang_vel);
    odom.set_allocated_orientation(orientation);
    for(int i = 0 ; i < 6 ; i++)
        odom.add_pose_covariance(1.0);
    for(int i = 0 ; i < 6 ; i++)
        odom.add_velocity_covariance(1.0);

    odometry_pub_->Publish(odom, true);
}

void GazeboOdometryPlugin::CreatePubsAndSubs() {
    // Advertise
    odometry_pub_ = node_handle_->Advertise<nav_msgs::msgs::Odometry>("~/" + model_->GetName() + odom_pub_topic_, 10);
    
    // Create temporary "ConnectGazeboToRosTopic" publisher and message
    gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub = node_handle_->Advertise<std_msgs::msgs::ConnectGazeboToRosTopic>("~/" + kConnectGazeboToRosSubtopic, 1);

    // Connect to ROS
    std_msgs::msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

    connect_gazebo_to_ros_topic_msg.set_ros_topic(odom_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + model_->GetName() + odom_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(std_msgs::msgs::ConnectGazeboToRosTopic::ODOMETRY);
    
    gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboOdometryPlugin)
} // namespace gazebo