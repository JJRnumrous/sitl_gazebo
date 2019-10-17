#ifndef _GAZEBO_FORCES_MOMENTS_ACTUATORS_PLUGIN_H_
#define _GAZEBO_FORCES_MOMENTS_ACTUATORS_PLUGIN_H_

#include <Eigen/Core>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ConnectGazeboToRosTopic.pb.h>

#include <Odometry.pb.h>

#include "common.h"

namespace gazebo {

// Default values
static const std::string defaultNamespace = "";
static const std::string defaultLinkName = "base_link";

static const std::string defaultOdomPubTopic = "/odometry";

class GazeboOdometryPlugin : public ModelPlugin {
    public:
        GazeboOdometryPlugin() : ModelPlugin(),
                                    namespace_(defaultNamespace),
                                    link_name_(defaultLinkName),
                                    odom_pub_topic_(defaultOdomPubTopic),
                                    pubs_and_subs_created_(false) {}
        ~GazeboOdometryPlugin();

    protected:
        // \brief Load the plugin.
        // \param[in] _model Pointer to the model that loaded this plugin.
        // \param[in] _sdf SDF element that describes the plugin.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        // \brief Called when the world is updated.
        // \param[in] _info Update timing information.
        void OnUpdate(const common::UpdateInfo& _info);

        // Publish/Subscribe to ROS
        void CreatePubsAndSubs();

    private:
        // \brief Pointer to the update event connection.
        event::ConnectionPtr update_connection_;
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;

        std::string namespace_;
        std::string link_name_;
        std::string odom_pub_topic_;

        bool pubs_and_subs_created_;

        transport::NodePtr node_handle_;

        gazebo::transport::PublisherPtr odometry_pub_;
};
} // namespace gazebo
#endif // _GAZEBO_FORCES_MOMENTS_ACTUATORS_PLUGIN_H_