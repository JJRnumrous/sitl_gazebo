/*
 * Copyright (C) 2017 chapulina
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "common.h"

namespace gazebo {
  static const std::string defaultLinkName = "base_link";

  class GazeboControlPlugin : public ModelPlugin
  {
    public:
        GazeboControlPlugin() : ModelPlugin(),
                               namespace_(""),                                
                               link_name_(defaultLinkName) {}   
        ~GazeboControlPlugin();


    protected:
      // Documentation inherited
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

      /// \brief Main loop to update the pose
      void OnUpdate(const common::UpdateInfo& _info);

    

    
    private:     
      std::vector<event::ConnectionPtr> connections;

      /// \brief Pointer to the model
      physics::ModelPtr model;
      event::ConnectionPtr update_connection_;
      physics::LinkPtr link_;
      std::string namespace_;
      std::string link_name_;

  };

}