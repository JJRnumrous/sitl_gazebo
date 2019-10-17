#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <gazebo/physics/physics.hh>

#include "gazebo_control_plugin.h"


using namespace gazebo;

GazeboControlPlugin::~GazeboControlPlugin() {
    update_connection_->~Connection();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboControlPlugin)

/////////////////////////////////////////////////
void GazeboControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

//   this->connections.push_back(event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboControlPlugin::OnUpdate, this)));
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboControlPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////
void GazeboControlPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Get the desired pose, here giving a random offset
  ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();
  common::Time current_time  = this->model->GetWorld()->GetSimTime();
//   std::string body;
//   this->model->GetBody(std::string& body);
  
  link_ = this->model->GetLink(link_name_);
//   link->AddForce(gazebo::math::Vector3(x, y, z)); //for global force
//   link->AddRelativeForce(math::Vector3(0.0, 0.0, 5.0)); // for relative force depends on actual pose and angular

  ignition::math::Vector3d force;
// link_->AddRelativeForce(force);
    
    // force = ignition::math::Vector3d(0.0f, 0.0f, 11.7f);
    //         link_->AddRelativeForce(force);

  
    if(current_time.sec > 30){
        if(pose.Pos().Z()<2.1){
            // pose += ignition::math::Pose3d(  0.0, 0.0, 0.001, 0.0, 0.0, 0.0 );
            force = ignition::math::Vector3d(0.0f, 0.04f, 11.7f);
            link_->AddRelativeForce(force);
        }else if(pose.Pos().X()<2.1){
            force = ignition::math::Vector3d(0.0f, -1.0f, 11.081458f);
            link_->AddRelativeForce(force);
            // pose += ignition::math::Pose3d(  0.001, 0.0, 0.0, 0.0, 0.0, 0.0 );
        }else{
            force = ignition::math::Vector3d(0.0f, 0.0f, 0.0f);
            link_->AddRelativeForce(force);
        }
    }
  // Don't let it go far under the gound
//   pose.Pos().Z() = pose.Pos().Z() < 0.5 ? 0.5 : pose.Pos().Z();

//   this->model->SetWorldPose(pose);
}
