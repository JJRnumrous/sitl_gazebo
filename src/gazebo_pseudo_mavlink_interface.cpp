/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo_pseudo_mavlink_interface.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  close();
  sigIntConnection_->~Connection();
  updateConnection_->~Connection();
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();  

  // TODO: Change
  enable_sim_ = true;

  const char *env_alt = std::getenv("PX4_HOME_ALT");
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home = std::stod(env_alt);
  }

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->GetElement("protocol_version")->Get<float>();
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // getSdfParam<std::string>(_sdf, "motorCommandPubTopic", motor_reference_pub_topic_, motor_reference_pub_topic_);
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  // getSdfParam<std::string>(_sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
  // getSdfParam<std::string>(_sdf, "opticalFlowSubTopic", opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  // getSdfParam<std::string>(_sdf, "sonarSubTopic", sonar_sub_topic_, sonar_sub_topic_);
  // getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);  
  getSdfParam<std::string>(_sdf, "imuMsgFile", imu_file_name_, imu_file_name_);  
  getSdfParam<std::string>(_sdf, "gpsMsgFile", gps_file_name_, gps_file_name_);    
  getSdfParam<std::string>(_sdf, "stateMsgFile",  state_file_name_,  state_file_name_);

  

  groundtruth_sub_topic_ = "/groundtruth";

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);
  for (int i = 0; i < n_out_max; ++i)
  {
    input_reference_[i] = 0;
  }

  if(_sdf->HasElement("hil_mode"))
  {
    hil_mode_ = _sdf->GetElement("hil_mode")->Get<bool>();
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->GetElement("hil_state_level")->Get<bool>();
  }

  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled_ = _sdf->GetElement("serialEnabled")->Get<bool>();
  }

  if(_sdf->HasElement("pseudo_mode"))
  {
    pseudo_enable_ = _sdf->GetElement("pseudo_mode")->Get<bool>();
  }

  if(_sdf->HasElement("writeEnabled"))
  {
    write_enable_ = _sdf->GetElement("writeEnabled")->Get<bool>();
  }  
  
    if(_sdf->HasElement("pseudo_gps"))
  {
    gps_enable_ = _sdf->GetElement("pseudo_gps")->Get<bool>();
  }  

  if (!serial_enabled_ && _sdf->HasElement("use_tcp"))
  {
    use_tcp_ = _sdf->GetElement("use_tcp")->Get<bool>();
  }
  gzmsg << "Connecting to PX4 SITL using " << (serial_enabled_ ? "serial" : (use_tcp_ ? "TCP" : "UDP")) << "\n";

  if (!hil_mode_ && _sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->GetElement("enable_lockstep")->Get<bool>();
  }
  gzmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << "\n";




  // When running in lockstep, we can run the simulation slower or faster than
  // realtime. The speed can be set using the env variable PX4_SIM_SPEED_FACTOR.
  if (enable_lockstep_)
  {
    const char *speed_factor_str = std::getenv("PX4_SIM_SPEED_FACTOR");
    if (speed_factor_str)
    {
      speed_factor_ = std::atof(speed_factor_str);
      if (!std::isfinite(speed_factor_) || speed_factor_ <= 0.0)
      {
        gzerr << "Invalid speed factor '" << speed_factor_str << "', aborting\n";
        abort();
      }
    }
    gzmsg << "Speed factor set to: " << speed_factor_ << "\n";

    boost::any param;
#if GAZEBO_MAJOR_VERSION >= 8
    physics::PresetManagerPtr presetManager = world_->PresetMgr();
#else
    physics::PresetManagerPtr presetManager = world_->GetPresetManager();
#endif
    presetManager->CurrentProfile("default_physics");

    // We currently need to have the max_step_size pinned at 4 ms and the
    // real_time_update_rate set to 250 Hz for lockstep.
    // Therefore it makes sense to check these params.

    presetManager->GetCurrentProfileParam("real_time_update_rate", param);
    double real_time_update_rate = boost::any_cast<double>(param);
    const double correct_real_time_update_rate = 250.0;
    if (real_time_update_rate != correct_real_time_update_rate)
    {
      gzerr << "real_time_update_rate is set to " << real_time_update_rate
            << " instead of " << correct_real_time_update_rate << ", aborting.\n";
      abort();
    }

    presetManager->GetCurrentProfileParam("max_step_size", param);
    const double max_step_size = boost::any_cast<double>(param);
    const double correct_max_step_size = 0.004;
    if (max_step_size != correct_max_step_size)
    {
      gzerr << "max_step_size is set to " << max_step_size
            << " instead of " << correct_max_step_size << ", aborting.\n";
      abort();
    }

    // Adapt the real_time_update_rate according to the speed
    // that we ask for in the env variable.
    real_time_update_rate *= speed_factor_;
    presetManager->SetCurrentProfileParam("real_time_update_rate", real_time_update_rate);
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Listen to Ctrl+C / SIGINT.
  sigIntConnection_ = event::Events::ConnectSigInt(
      boost::bind(&GazeboMavlinkInterface::onSigInt, this));

  // Subscribe to messages of other plugins.
  imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
  lidar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + lidar_sub_topic_, &GazeboMavlinkInterface::LidarCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  sonar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + sonar_sub_topic_, &GazeboMavlinkInterface::SonarCallback, this);
  irlock_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + irlock_sub_topic_, &GazeboMavlinkInterface::IRLockCallback, this);
  // vio_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/base_link/color/image", &GazeboMavlinkInterface::vioCallback, this);
  if(!pseudo_enable_ || (write_enable_ && gps_enable_ ) ){    
    gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkInterface::GpsCallback, this);
  }

  groundtruth_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + groundtruth_sub_topic_, &GazeboMavlinkInterface::GroundtruthCallback, this);
  vision_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + vision_sub_topic_, &GazeboMavlinkInterface::VisionCallback, this);   
  
  

  if(enable_sim_) {
    // Publish gazebo's motor_speed message
    motor_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorThrottle>("~/" + model_->GetName() + motor_reference_pub_topic_, 100);

  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
    last_imu_time_ = world_->SimTime();
    gravity_W_ = world_->Gravity();
  #else
    last_time_ = world_->GetSimTime();
    last_imu_time_ = world_->GetSimTime();
    gravity_W_ = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
  #endif

    // This doesn't seem to be used anywhere but we leave it here
    // for potential compatibility
    if (_sdf->HasElement("imu_rate")) {
      imu_update_interval_ = 1 / _sdf->GetElement("imu_rate")->Get<int>();
    }

    mavlink_addr_ = htonl(INADDR_ANY);
    if (_sdf->HasElement("mavlink_addr")) {
      std::string mavlink_addr_str = _sdf->GetElement("mavlink_addr")->Get<std::string>();
      if (mavlink_addr_str != "INADDR_ANY") {
        mavlink_addr_ = inet_addr(mavlink_addr_str.c_str());
        if (mavlink_addr_ == INADDR_NONE) {
          gzerr << "Invalid mavlink_addr: " << mavlink_addr_str << ", aborting\n";
          abort();
        }
      }
    }

  #if GAZEBO_MAJOR_VERSION >= 9
    auto worldName = world_->Name();
  #else
    auto worldName = world_->GetName();
  #endif

    if (_sdf->HasElement("mavlink_udp_port")) {
      mavlink_udp_port_ = _sdf->GetElement("mavlink_udp_port")->Get<int>();
    }
    model_param(worldName, model_->GetName(), "mavlink_udp_port", mavlink_udp_port_);

    if (_sdf->HasElement("mavlink_tcp_port")) {
      mavlink_tcp_port_ = _sdf->GetElement("mavlink_tcp_port")->Get<int>();
    }
    model_param(worldName, model_->GetName(), "mavlink_tcp_port", mavlink_tcp_port_);

    local_qgc_addr_.sin_port = htonl(INADDR_ANY);
    if (_sdf->HasElement("qgc_addr")) {
      std::string qgc_addr = _sdf->GetElement("qgc_addr")->Get<std::string>();
      if (qgc_addr != "INADDR_ANY") {
        local_qgc_addr_.sin_port = inet_addr(qgc_addr.c_str());
        if (local_qgc_addr_.sin_port == INADDR_NONE) {
          gzerr << "Invalid qgc_addr: " << qgc_addr << ", aborting\n";
          abort();
        }
      }
    }
    if (_sdf->HasElement("qgc_udp_port")) {
      qgc_udp_port_ = _sdf->GetElement("qgc_udp_port")->Get<int>();
    }

    local_sdk_addr_.sin_port = htonl(INADDR_ANY);
    if (_sdf->HasElement("sdk_addr")) {
      std::string sdk_addr = _sdf->GetElement("sdk_addr")->Get<std::string>();
      if (sdk_addr != "INADDR_ANY") {
        local_sdk_addr_.sin_port = inet_addr(sdk_addr.c_str());
        if (local_sdk_addr_.sin_port == INADDR_NONE) {
          gzerr << "Invalid sdk_addr: " << sdk_addr << ", aborting\n";
          abort();
        }
      }
    }
    if (_sdf->HasElement("sdk_udp_port")) {
      sdk_udp_port_ = _sdf->GetElement("sdk_udp_port")->Get<int>();
    }

    if (hil_mode_) {          
      local_qgc_addr_.sin_family = AF_INET;
      local_qgc_addr_.sin_port = htons(0);
      local_qgc_addr_len_ = sizeof(local_qgc_addr_);

      remote_qgc_addr_.sin_family = AF_INET;
      remote_qgc_addr_.sin_port = htons(qgc_udp_port_);
      remote_qgc_addr_len_ = sizeof(remote_qgc_addr_);
       
      local_sdk_addr_.sin_family = AF_INET;
      local_sdk_addr_.sin_port = htons(0);
      local_sdk_addr_len_ = sizeof(local_sdk_addr_);

      remote_sdk_addr_.sin_family = AF_INET;
      remote_sdk_addr_.sin_port = htons(sdk_udp_port_);
      remote_sdk_addr_len_ = sizeof(remote_sdk_addr_);

      if ((qgc_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        gzerr << "Creating QGC UDP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(qgc_socket_fd_, (struct sockaddr *)&local_qgc_addr_, local_qgc_addr_len_) < 0) {
        gzerr << "QGC UDP bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if ((sdk_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        gzerr << "Creating SDK UDP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(sdk_socket_fd_, (struct sockaddr *)&local_sdk_addr_, local_sdk_addr_len_) < 0) {
        gzerr << "SDK UDP bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }      

    }

    if (serial_enabled_) {
      // Set up serial interface
      if(_sdf->HasElement("serialDevice"))
      {
        device_ = _sdf->GetElement("serialDevice")->Get<std::string>();
      }

      if (_sdf->HasElement("baudRate")) {
        baudrate_ = _sdf->GetElement("baudRate")->Get<int>();
      }
      io_service.post(std::bind(&GazeboMavlinkInterface::do_read, this));

      // run io_service for async io
      io_thread = std::thread([this] () {
        io_service.run();
      });
      open();

    } else {
      memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
      remote_simulator_addr_.sin_family = AF_INET;
      remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

      memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
      local_simulator_addr_.sin_family = AF_INET;
      local_simulator_addr_len_ = sizeof(local_simulator_addr_);

      if (use_tcp_) {

        local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
        local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
          gzerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting\n";
          abort();
        }

        int yes = 1;
        int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        if (result != 0) {
          gzerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
          abort();
        }

        struct linger nolinger {};
        nolinger.l_onoff = 1;
        nolinger.l_linger = 0;

        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
        if (result != 0) {
          gzerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
          abort();
        }

        if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
          gzerr << "bind failed: " << strerror(errno) << ", aborting\n";
          abort();
        }

        errno = 0;
        if (listen(simulator_socket_fd_, 0) < 0) {
          gzerr << "listen failed: " << strerror(errno) << ", aborting\n";
          abort();
        }

        simulator_tcp_client_fd_ = accept(simulator_socket_fd_, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

      } else {
        remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
        remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);

        local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        local_simulator_addr_.sin_port = htons(0);

        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
          gzerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting\n";
          abort();
        }

        if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
          gzerr << "bind failed: " << strerror(errno) << ", aborting\n";
          abort();
        }
      
      }
    }

    if(_sdf->HasElement("vehicle_is_tailsitter"))
    {
      vehicle_is_tailsitter_ = _sdf->GetElement("vehicle_is_tailsitter")->Get<bool>();
    }

    if(_sdf->HasElement("send_vision_estimation"))
    {
      send_vision_estimation_ = _sdf->GetElement("send_vision_estimation")->Get<bool>();
    }

    if(_sdf->HasElement("send_odometry"))
    {
      send_odometry_ = _sdf->GetElement("send_odometry")->Get<bool>();
    }


    mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

    // set the Mavlink protocol version to use on the link
    if (protocol_version_ == 2.0) {
      chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
      gzmsg << "Using MAVLink protocol v2.0\n";
    }
    else if (protocol_version_ == 1.0) {
      chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
      gzmsg << "Using MAVLink protocol v1.0\n";
    }
    else {
      gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
    }
  }


  if( pseudo_enable_ && !write_enable_ ){
    msg_read_imu();
    msg_read_state();

    if(gps_enable_){
      msg_read_gps();
    }

  }else if( pseudo_enable_ && write_enable_ ){
    std::ofstream outFile;      
    outFile.open( state_file_name_, std::ios::trunc); 
    outFile.close();
    outFile.open(gps_file_name_, std::ios::trunc);    
    outFile.close();
    outFile.open(imu_file_name_, std::ios::trunc);    
    outFile.close();
  }

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "arming");
    ros::NodeHandle nh("~");  
    mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");  
    local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/temp_pose", 1);
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {

  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  // if (previous_imu_seq_ > 0) {
  //   while (previous_imu_seq_ == last_imu_message_.seq() && IsRunning()) {
  //     last_imu_message_cond_.wait_for(lock, std::chrono::microseconds(10));
  //   }
  // }

  previous_imu_seq_ = last_imu_message_.seq();

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

  SendSensorMessages();    
  handle_actuators(dt);

 if (hil_mode_) {
    pollFromQgcAndSdk();
  } else {
    pollForMAVLinkMessages();
  }

  last_time_ = current_time;
}

void GazeboMavlinkInterface::send_mavlink_message(const mavlink_message_t *message)
{
  assert(message != nullptr);

  if (gotSigInt_) {
    return;
  }

  if (serial_enabled_) {

    if (!is_open()) {
      gzerr << "Serial port closed! \n";
      return;
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex);

      if (tx_q.size() >= MAX_TXQ_SIZE) {
          gzwarn << "Tx queue overflow \n";
      }
      tx_q.emplace_back(message);
    }
    io_service.post(std::bind(&GazeboMavlinkInterface::do_write, this, true));
  

  } else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    ssize_t len;
    if (use_tcp_) {
      len = send(simulator_tcp_client_fd_, buffer, packetlen, 0);
    } else {
      len = sendto(simulator_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
    }

    if (len <= 0)
    {
      gzerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
    }
  }
}

void GazeboMavlinkInterface::forward_mavlink_message(const mavlink_message_t *message)
{
    if (gotSigInt_) {
    return;
  }
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len;
  if (qgc_socket_fd_ > 0) {
    len = sendto(qgc_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_qgc_addr_, remote_qgc_addr_len_);

    if (len <= 0) {
      gzerr << "Failed sending mavlink message to QGC: " << strerror(errno) << "\n";
    }
  }

  if (sdk_socket_fd_ > 0) {
    len = sendto(sdk_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_sdk_addr_, remote_sdk_addr_len_);
    if (len <= 0)
    {
      gzerr << "Failed sending mavlink message to SDK: " << strerror(errno) << "\n";
    }
  }
}


void GazeboMavlinkInterface::SendSensorMessages()
{
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

    if((world_->GetSimTime().Double() < 6.0f)&&(world_->GetSimTime().Double() > 2.0f)){
      q_gr = ignition::math::Quaterniond(     
      0.924f,
      0.0f,
      0.0f,
      0.383f);
    }


  ignition::math::Quaterniond q_gb = q_gr*q_br.Inverse();
  ignition::math::Quaterniond q_nb = q_ng*q_gb;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d pos_g = model_->WorldPose().Pos();
#else
  ignition::math::Vector3d pos_g = ignitionFromGazeboMath(model_->GetWorldPose().pos);
#endif
  ignition::math::Vector3d pos_n = q_ng.RotateVector(pos_g);

  // Magnetic declination and inclination (radians)
  float declination_rad = get_mag_declination(groundtruth_lat_rad * 180 / M_PI, groundtruth_lon_rad * 180 / M_PI) * M_PI / 180;
  float inclination_rad = get_mag_inclination(groundtruth_lat_rad * 180 / M_PI, groundtruth_lon_rad * 180 / M_PI) * M_PI / 180;

  // Magnetic strength (10^5xnanoTesla)
  float strength_ga = 0.01f * get_mag_strength(groundtruth_lat_rad * 180 / M_PI, groundtruth_lon_rad * 180 / M_PI);

  // Magnetic filed components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
  float H = strength_ga * cosf(inclination_rad);
  float Z = tanf(inclination_rad) * H;
  float X = H * cosf(declination_rad);
  float Y = H * sinf(declination_rad);

  // Magnetic field data from WMM2018 (10^5xnanoTesla (N, E D) n-frame )
  mag_d_.X() = X;
  mag_d_.Y() = Y;
  mag_d_.Z() = Z;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_b = q_br.RotateVector(model_->RelativeLinearVel());
  ignition::math::Vector3d vel_n = q_ng.RotateVector(model_->WorldLinearVel());
  ignition::math::Vector3d omega_nb_b = q_br.RotateVector(model_->RelativeAngularVel());
#else
  ignition::math::Vector3d vel_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
  ignition::math::Vector3d vel_n = q_ng.RotateVector(ignitionFromGazeboMath(model_->GetWorldLinearVel()));
  ignition::math::Vector3d omega_nb_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeAngularVel()));
#endif

  ignition::math::Vector3d mag_noise_b(
    0.01 * randn_(rand_),
    0.01 * randn_(rand_),
    0.01 * randn_(rand_));

  ignition::math::Vector3d accel_b = q_br.RotateVector(ignition::math::Vector3d(
    last_imu_message_.linear_acceleration().x(),
    last_imu_message_.linear_acceleration().y(),
    last_imu_message_.linear_acceleration().z()));
  ignition::math::Vector3d gyro_b = q_br.RotateVector(ignition::math::Vector3d(
    last_imu_message_.angular_velocity().x(),
    last_imu_message_.angular_velocity().y(),
    last_imu_message_.angular_velocity().z()));
  ignition::math::Vector3d mag_b = q_nb.RotateVectorReverse(mag_d_) + mag_noise_b;

  bool should_send_imu = false;
  if (!enable_lockstep_) {
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time = world_->SimTime();
#else
    common::Time current_time = world_->GetSimTime();
#endif
    double dt = (current_time - last_imu_time_).Double();

    if (imu_update_interval_!=0 && dt >= imu_update_interval_) {
      should_send_imu = true;
      last_imu_time_ = current_time;
    }
  }

  if (enable_lockstep_ || should_send_imu) {
    mavlink_hil_sensor_t sensor_msg;
#if GAZEBO_MAJOR_VERSION >= 9
    sensor_msg.time_usec = world_->SimTime().Double() * 1e6;
#else
    sensor_msg.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
    sensor_msg.xacc = accel_b.X();
    sensor_msg.yacc = accel_b.Y();
    sensor_msg.zacc = accel_b.Z();
    sensor_msg.xgyro = gyro_b.X();
    sensor_msg.ygyro = gyro_b.Y();
    sensor_msg.zgyro = gyro_b.Z();
    sensor_msg.xmag = mag_b.X();
    sensor_msg.ymag = mag_b.Y();
    sensor_msg.zmag = mag_b.Z();       

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    float alt_msl = (float)alt_home - pos_n.Z();
    float temperature_local = temperature_msl - lapse_rate * alt_msl;
    float pressure_ratio = powf((temperature_msl/temperature_local) , 5.256f);
    const float pressure_msl = 101325.0f; // pressure at MSL
    sensor_msg.abs_pressure = pressure_msl / pressure_ratio;

    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    double x1, x2, w, y1;
    if (!baro_rnd_use_last_) {
      do {
        x1 = 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;
        x2 = 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;
        w = x1 * x1 + x2 * x2;
      } while ( w >= 1.0 );
      w = sqrt( (-2.0 * log( w ) ) / w );
      // calculate two values - the second value can be used next time because it is uncorrelated
      y1 = x1 * w;
      baro_rnd_y2_ = x2 * w;
      baro_rnd_use_last_ = true;
    } else {
      // no need to repeat the calculation - use the second value from last update
      y1 = baro_rnd_y2_;
      baro_rnd_use_last_ = false;
    }

    // Apply 1 Pa RMS noise
    float abs_pressure_noise = 1.0f * (float)y1;
    sensor_msg.abs_pressure += abs_pressure_noise;

    // convert to hPa
    sensor_msg.abs_pressure *= 0.01f;

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf((temperature_msl/temperature_local) , 4.256f);
    float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    sensor_msg.pressure_alt = alt_msl; - abs_pressure_noise / (gravity_W_.Length() * rho);

    // calculate differential pressure in hPa
    // if vehicle is a tailsitter the airspeed axis is different (z points from nose to tail)
    if (vehicle_is_tailsitter_) {
      sensor_msg.diff_pressure = 0.005f*rho*vel_b.Z()*vel_b.Z();
    } else {
      sensor_msg.diff_pressure = 0.005f*rho*vel_b.X()*vel_b.X();
    }

    // calculate temperature in Celsius
    sensor_msg.temperature = temperature_local - 273.0f;

    sensor_msg.fields_updated = 4095;

    if( pseudo_enable_){
      if(write_enable_){ // write
          msg_write_imu(sensor_msg);
      }else{        
          msg_read_pseudo_sensors(sensor_msg);          
      }
    }

    // std::cout << sensor_msg.time_usec << " " << sensor_msg.xacc << " " << sensor_msg.yacc<< " " << sensor_msg.zacc<< " " << sensor_msg.xgyro<< " " << sensor_msg.ygyro<< " " << sensor_msg.zgyro<< " " << sensor_msg.xmag << " " << sensor_msg.ymag<< " " << sensor_msg.zmag<< " " << sensor_msg.abs_pressure<< " " << sensor_msg.diff_pressure<< " " << sensor_msg.pressure_alt << " " << sensor_msg.temperature << " " << sensor_msg.fields_updated << std::endl;

    if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
      mavlink_message_t msg;
      mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
      send_mavlink_message(&msg);
    }
  }

  // ground truth
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d accel_true_b = q_br.RotateVector(model_->RelativeLinearAccel());
#else
  ignition::math::Vector3d accel_true_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearAccel()));
#endif

  // send ground truth

  mavlink_hil_state_quaternion_t hil_state_quat;
#if GAZEBO_MAJOR_VERSION >= 9
  hil_state_quat.time_usec = world_->SimTime().Double() * 1e6;
#else
  hil_state_quat.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
  hil_state_quat.attitude_quaternion[0] = q_nb.W();
  hil_state_quat.attitude_quaternion[1] = q_nb.X();
  hil_state_quat.attitude_quaternion[2] = q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = q_nb.Z();

  hil_state_quat.rollspeed = omega_nb_b.X();
  hil_state_quat.pitchspeed = omega_nb_b.Y();
  hil_state_quat.yawspeed = omega_nb_b.Z();

  hil_state_quat.lat = groundtruth_lat_rad * 180 / M_PI * 1e7;
  hil_state_quat.lon = groundtruth_lon_rad * 180 / M_PI * 1e7;
  hil_state_quat.alt = groundtruth_altitude * 1000;

  hil_state_quat.vx = vel_n.X() * 100;
  hil_state_quat.vy = vel_n.Y() * 100;
  hil_state_quat.vz = vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.X();

#if GAZEBO_MAJOR_VERSION >= 9
  hil_state_quat.true_airspeed = model_->WorldLinearVel().Length() * 100;  //no wind simulated
#else
  hil_state_quat.true_airspeed = model_->GetWorldLinearVel().GetLength() * 100;  //no wind simulated
#endif

  hil_state_quat.xacc = accel_true_b.X() * 1000;
  hil_state_quat.yacc = accel_true_b.Y() * 1000;
  hil_state_quat.zacc = accel_true_b.Z() * 1000;

  if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::handle_actuators(double dt)
{
  static int64_t seq = 0;
  mav_msgs::msgs::CommandMotorThrottle motors;

  motors.set_dt(dt);
  motors.set_seq(seq++);

  for (int i = 0; i < input_reference_.size(); i++) {
    motors.add_motor_throttle(constrain((double)input_reference_[i], 0.0, 1.0));
  }
  motor_reference_pub_->Publish(motors);
}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message)
{
  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  const int64_t diff = imu_message->seq() - last_imu_message_.seq();
  if (diff != 1 && imu_message->seq() != 0)
  {
    gzerr << "Skipped " << (diff - 1) << " IMU samples (presumably CPU usage is too high)\n";
  }

  last_imu_message_ = *imu_message;
  lock.unlock();
  last_imu_message_cond_.notify_one();  
}

void GazeboMavlinkInterface::GpsCallback(GpsPtr& gps_msg) {
  // fill HIL GPS Mavlink msg
  mavlink_hil_gps_t hil_gps_msg;
  int current_time = world_->GetSimTime().Double() * 1e6;
  
  hil_gps_msg.time_usec = gps_msg->time_usec();
  hil_gps_msg.fix_type = 3;
  hil_gps_msg.lat = gps_msg->latitude_deg() * 1e7;  
  hil_gps_msg.lon = gps_msg->longitude_deg() * 1e7;
  hil_gps_msg.alt = gps_msg->altitude() * 1000.0;
  hil_gps_msg.eph = gps_msg->eph() * 100.0;
  hil_gps_msg.epv = gps_msg->epv() * 100.0;
  hil_gps_msg.vel = gps_msg->velocity() * 100.0;
  hil_gps_msg.vn = gps_msg->velocity_north() * 100.0;
  hil_gps_msg.ve = gps_msg->velocity_east() * 100.0;
  hil_gps_msg.vd = -gps_msg->velocity_up() * 100.0;
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  ignition::math::Angle cog(atan2(gps_msg->velocity_east(), gps_msg->velocity_north()));
  cog.Normalize();
  hil_gps_msg.cog = static_cast<uint16_t>(GetDegrees360(cog) * 100.0);
  hil_gps_msg.satellites_visible = 10;

  if(pseudo_enable_){
    msg_write_gps(hil_gps_msg);
  }

  // send HIL_GPS Mavlink msg
  if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);	  
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::GroundtruthCallback(GtPtr& groundtruth_msg) {
  // update groundtruth lat_rad, lon_rad and altitude
  groundtruth_lat_rad = groundtruth_msg->latitude_rad();
  groundtruth_lon_rad = groundtruth_msg->longitude_rad();
  groundtruth_altitude = groundtruth_msg->altitude();
  // the rest of the data is obtained directly on this interface and sent to
  // the FCU
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_usec() / 1e3;
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;
  sensor_msg.type = 0;
  sensor_msg.id = 0;
  sensor_msg.orientation = 25;//downward facing
  sensor_msg.covariance = 0;

  //distance needed for optical flow message
  optflow_distance = lidar_message->current_distance();  //[m]

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {
  mavlink_hil_optical_flow_t sensor_msg;
  sensor_msg.time_usec = opticalFlow_message->time_usec();
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();

  bool no_gyro = (ignition::math::isnan(opticalFlow_message->integrated_xgyro())) ||
                 (ignition::math::isnan(opticalFlow_message->integrated_ygyro())) ||
                 (ignition::math::isnan(opticalFlow_message->integrated_zgyro()));
  if (no_gyro) {
    sensor_msg.integrated_xgyro = NAN;
    sensor_msg.integrated_ygyro = NAN;
    sensor_msg.integrated_zgyro = NAN;
  } else {
    sensor_msg.integrated_xgyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_xgyro() : 0.0f;
    sensor_msg.integrated_ygyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_ygyro() : 0.0f;
    sensor_msg.integrated_zgyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_zgyro() : 0.0f;
  }
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance;

  mavlink_message_t msg;
  mavlink_msg_hil_optical_flow_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::SonarCallback(SonarPtr& sonar_message) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = sonar_message->time_usec() / 1e3;
  sensor_msg.min_distance = sonar_message->min_distance() * 100.0;
  sensor_msg.max_distance = sonar_message->max_distance() * 100.0;
  sensor_msg.current_distance = sonar_message->current_distance() * 100.0;
  sensor_msg.type = 1;
  sensor_msg.id = 1;
  sensor_msg.orientation = 0;  // forward facing
  sensor_msg.covariance = 0;

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::IRLockCallback(IRLockPtr& irlock_message) {
  mavlink_landing_target_t sensor_msg;
  sensor_msg.time_usec = irlock_message->time_usec() / 1e3;
  sensor_msg.target_num = irlock_message->signature();
  sensor_msg.angle_x = irlock_message->pos_x();
  sensor_msg.angle_y = irlock_message->pos_y();
  sensor_msg.size_x = irlock_message->size_x();
  sensor_msg.size_y = irlock_message->size_y();
  sensor_msg.position_valid = false;
  sensor_msg.type = LANDING_TARGET_TYPE_LIGHT_BEACON;

  mavlink_message_t msg;
  // way of encoding IMU data for sending over MAVLINK protocol
  mavlink_msg_landing_target_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::VisionCallback(OdomPtr& odom_message) {
  mavlink_message_t msg;

  // transform position from local ENU to local NED frame
  ignition::math::Vector3d position = q_ng.RotateVector(ignition::math::Vector3d(
    odom_message->position().x(),
    odom_message->position().y(),
    odom_message->position().z()));

  // q_gr is the quaternion that represents the orientation of the vehicle
  // the ENU earth/local
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    odom_message->orientation().w(),
    odom_message->orientation().x(),
    odom_message->orientation().y(),
    odom_message->orientation().z());

  // transform the vehicle orientation from the ENU to the NED frame
  // q_nb is the quaternion that represents the orientation of the vehicle
  // the NED earth/local
  ignition::math::Quaterniond q_nb = q_ng * q_gr * q_ng.Inverse();

  // transform linear velocity from local ENU to body FRD frame
  ignition::math::Vector3d linear_velocity = q_ng.RotateVector(
    q_br.RotateVector(ignition::math::Vector3d(
      odom_message->linear_velocity().x(),
      odom_message->linear_velocity().y(),
      odom_message->linear_velocity().z())));

  // transform angular velocity from body FLU to body FRD frame
  ignition::math::Vector3d angular_velocity = q_br.RotateVector(ignition::math::Vector3d(
    odom_message->angular_velocity().x(),
    odom_message->angular_velocity().y(),
    odom_message->angular_velocity().z()));

  // Only sends ODOMETRY msgs if send_odometry is set and the protocol version is 2.0
  if (send_odometry_ && protocol_version_ == 2.0) {
    // send ODOMETRY Mavlink msg
    mavlink_odometry_t odom;

    odom.time_usec = odom_message->time_usec();

    odom.frame_id = MAV_FRAME_VISION_NED;
    odom.child_frame_id = MAV_FRAME_BODY_FRD;

    odom.x = position.X();
    odom.y = position.Y();
    odom.z = position.Z();

    odom.q[0] = q_nb.W();
    odom.q[1] = q_nb.X();
    odom.q[2] = q_nb.Y();
    odom.q[3] = q_nb.Z();

    odom.vx = linear_velocity.X();
    odom.vy = linear_velocity.Y();
    odom.vz = linear_velocity.Z();

    odom.rollspeed= angular_velocity.X();
    odom.pitchspeed = angular_velocity.Y();
    odom.yawspeed = angular_velocity.Z();

    // parse covariance matrices
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrices from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;

        odom.pose_covariance[count++] = odom_message->pose_covariance().data()[index];
        odom.velocity_covariance[count++] = odom_message->velocity_covariance().data()[index];
      }
    }

    mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &odom);
    send_mavlink_message(&msg);
  }
  else if (send_vision_estimation_) {
    // send VISION_POSITION_ESTIMATE Mavlink msg
    mavlink_vision_position_estimate_t vision;

    vision.usec = odom_message->time_usec();

    // transform position from local ENU to local NED frame
    vision.x = position.X();
    vision.y = position.Y();
    vision.z = position.Z();

    // q_nb is the quaternion that represents a rotation from NED earth/local
    // frame to XYZ body FRD frame
    ignition::math::Vector3d euler = q_nb.Euler();

    vision.roll = euler.X();
    vision.pitch = euler.Y();
    vision.yaw = euler.Z();

    // parse covariance matrix
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrix from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;
        vision.covariance[count++] = 0.0f;
        // vision.covariance[count++] = odom_message->pose_covariance().data()[index];
      }
    }

    // ignition::math::Quaterniond q_nb = q_ng * q_gr * q_ng.Inverse();
    ignition::math::Quaterniond q_rot(0.7071, 0, 0, 0.7071);
    
    
    ignition::math::Quaterniond q_orig = ignition::math::Quaterniond(
    odom_message->orientation().w(),
    odom_message->orientation().x(),
    odom_message->orientation().y(),
    odom_message->orientation().z());
    ignition::math::Quaterniond q_db = q_rot * q_orig;// * q_rot.Inverse();


    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "";
    pose.header.stamp.sec = odom_message->time_usec()/1e6;
    pose.header.stamp.nsec = (odom_message->time_usec()-pose.header.stamp.sec*1e6)*1e3;
    pose.pose.position.x = odom_message->position().x();    
    pose.pose.position.y = odom_message->position().y();    
    pose.pose.position.z = odom_message->position().z();    
    // pose.pose.orientation.w = odom_message->orientation().w();
    // pose.pose.orientation.x = odom_message->orientation().x();
    // pose.pose.orientation.y = odom_message->orientation().y();
    // pose.pose.orientation.z = odom_message->orientation().z();
    // pose.pose.position.x = position.X();    
    // pose.pose.position.y = position.Y();    
    // pose.pose.position.z = position.Z();    
    pose.pose.orientation.w = q_db.W();
    pose.pose.orientation.x = q_db.X();
    pose.pose.orientation.y = q_db.Y();
    pose.pose.orientation.z = q_db.Z();

    // gzerr << "Pos: " << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << "\n";
    // gzerr << "Qua: " << pose.pose.orientation.w << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << "\n";

    float signum = 1.0f*((pose.pose.orientation.w > 0.0f) - (pose.pose.orientation.w < 0.0f));
    
    pose.pose.orientation.w = signum*pose.pose.orientation.w;
    pose.pose.orientation.x = signum*pose.pose.orientation.x;
    pose.pose.orientation.y = signum*pose.pose.orientation.y;
    pose.pose.orientation.z = signum*pose.pose.orientation.z;
    // gzerr << pose.pose.orientation.w << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << "\n";
    vision_pub_.publish(pose);   
    ros::spinOnce(); 

    // mavlink_msg_vision_position_estimate_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &vision);
    // send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::pollForMAVLinkMessages()
{
  if (gotSigInt_) {
    return;
  }
  struct pollfd fds[1] = {};

  if (use_tcp_) {
    fds[0].fd = simulator_tcp_client_fd_;
  } else {
    fds[0].fd = simulator_socket_fd_;
  }
  fds[0].events = POLLIN;

  bool received_actuator = false;

  do {
    int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;

    int ret = ::poll(&fds[0], 1, timeout_ms);

    if (ret == 0 && timeout_ms > 0) {
      gzerr << "poll timeout\n";
    }

    if (ret < 0) {
      gzerr << "poll error: " << strerror(errno) << "\n";
    }

    if (fds[0].revents & POLLIN) {

     int len = recvfrom(fds[0].fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
      if (len > 0) {
        mavlink_message_t msg;
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i)
        {
          if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
          {
            if (hil_mode_) {
              send_mavlink_message(&msg);
            }            
            handle_message(&msg, received_actuator);
          }
        }
      }
    }
  } while (received_first_actuator_ && !received_actuator && enable_lockstep_ && IsRunning() && !gotSigInt_);
}

void GazeboMavlinkInterface::pollFromQgcAndSdk()
{
  struct pollfd fds[2] = {};
  fds[0].fd = qgc_socket_fd_;
  fds[0].events = POLLIN;
  fds[1].fd = sdk_socket_fd_;
  fds[1].events = POLLIN;

  const int timeout_ms = 0;

  int ret = ::poll(&fds[0], 2, timeout_ms);

  if (ret < 0) {
    gzerr << "poll error: " << strerror(errno) << "\n";
    return;
  }

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(qgc_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_qgc_addr_, &remote_qgc_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, _buf[i], &msg, &status)) {
          // forward message from QGC to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }

  if (fds[1].revents & POLLIN) {
    int len = recvfrom(sdk_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_sdk_addr_, &remote_sdk_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, _buf[i], &msg, &status)) {
          // forward message from SDK to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }
}


void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg, bool &received_actuator)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(msg, &controls);

    bool armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
    if(pseudo_enable_ && write_enable_){
      StateCallback(armed);
    }

#if GAZEBO_MAJOR_VERSION >= 9
    last_actuator_time_ = world_->SimTime();
#else
    last_actuator_time_ = world_->GetSimTime();
#endif

    for (unsigned i = 0; i < n_out_max; i++) {
      input_index_[i] = i;
    }

    // set rotor speeds, controller targets
    input_reference_.resize(n_out_max);
    for (int i = 0; i < input_reference_.size(); i++) {
      input_reference_[i] = controls.controls[input_index_[i]];
    }

    received_actuator = true;
    received_first_actuator_ = true;
    break;
  }
}

bool GazeboMavlinkInterface::IsRunning()
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world_->Running();
#else
    return world_->GetRunning();
#endif
}

void GazeboMavlinkInterface::open() {
  try{
    serial_dev.open(device_);
    serial_dev.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    gzdbg << "Opened serial device " << device_ << "\n";
  }
  catch (boost::system::system_error &err) {
    gzerr <<"Error opening serial device: " << err.what() << "\n";
  }
}

void GazeboMavlinkInterface::close()
{
  if(serial_enabled_) {
    ::close(qgc_socket_fd_);
    ::close(sdk_socket_fd_);

    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (!is_open())
      return;

    io_service.stop();
    serial_dev.close();

    if (io_thread.joinable())
      io_thread.join();

  } else {

      if (use_tcp_) {
      ::close(simulator_tcp_client_fd_);
    } else {
      ::close(simulator_socket_fd_);
    }
  }
}

void GazeboMavlinkInterface::do_read(void)
{
  serial_dev.async_read_some(boost::asio::buffer(rx_buf), boost::bind(
      &GazeboMavlinkInterface::parse_buffer, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
      )
  );
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void GazeboMavlinkInterface::parse_buffer(const boost::system::error_code& err, std::size_t bytes_t){
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t *buf = this->rx_buf.data();

  assert(rx_buf.size() >= bytes_t);

  for(; bytes_t > 0; bytes_t--)
  {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status);
      m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer.len = 0;
        mavlink_start_checksum(&m_buffer);
      }
    }

    if (msg_received != Framing::incomplete) {
      if (hil_mode_) {
        forward_mavlink_message(&message);
      }
      bool not_used;
      handle_message(&message, not_used);
    }
  }
  do_read();
}

void GazeboMavlinkInterface::do_write(bool check_tx_state){
  if (check_tx_state && tx_in_progress)
    return;

  std::lock_guard<std::recursive_mutex> lock(mutex);
  if (tx_q.empty())
    return;

  tx_in_progress = true;
  auto &buf_ref = tx_q.front();

  serial_dev.async_write_some(
    boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()), [this, &buf_ref] (boost::system::error_code error,   size_t bytes_transferred)
    {
      assert(bytes_transferred <= buf_ref.len);
      if(error) {
        gzerr << "Serial error: " << error.message() << "\n";
      return;
      }

    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (tx_q.empty()) {
      tx_in_progress = false;
      return;
    }

    buf_ref.pos += bytes_transferred;
    if (buf_ref.nbytes() == 0) {
      tx_q.pop_front();
    }

    if (!tx_q.empty()) {
      do_write(false);
    }
    else {
      tx_in_progress = false;
    }
  });
}
void GazeboMavlinkInterface::onSigInt() {
  gotSigInt_ = true;
  close();
}

// writes imu message to file
void GazeboMavlinkInterface::msg_write_imu(mavlink_hil_sensor_t msg){
  std::ofstream outFile;            
  outFile.open(imu_file_name_, std::ios::app);  
  outFile << msg.time_usec << " " << msg.xacc << " " << msg.yacc<< " " << msg.zacc<< " " << msg.xgyro<< " " << msg.ygyro<< " " << msg.zgyro<< " " << msg.xmag << " " << msg.ymag<< " " << msg.zmag<< " " << msg.abs_pressure<< " " << msg.diff_pressure<< " " << msg.pressure_alt << " " << msg.temperature << " " << msg.fields_updated << std::endl;  
  outFile.close();
}

void GazeboMavlinkInterface::msg_write_gps(mavlink_hil_gps_t msg){
  std::ofstream outFile;      
  outFile.open(gps_file_name_, std::ios::app);
  outFile << msg.time_usec << " " << msg.lat << " " << msg.lon << " " << msg.alt << " " << msg.eph << " " << msg.epv << " " << msg.vel << " " << msg.vn << " " << msg.ve << " " << msg.vd << " " << msg.cog << " " << msg.fix_type*1.0 << " " << msg.satellites_visible*1.0 << std::endl;  
  outFile.close();
}

void GazeboMavlinkInterface::StateCallback(bool armed){  
  int current_time = world_->GetSimTime().Double() * 1e6;    
  // TODO test first instance of saving for min time
  std::ofstream outFile;      
  outFile.open( state_file_name_, std::ios::app);      
  outFile << current_time << " " << armed << std::endl;  
  outFile.close();
}

// reads imu message from file and overrite gazebo message
void GazeboMavlinkInterface::msg_read_imu(){

  std::ifstream inFile;    
  inFile.open(imu_file_name_, std::ios::in);
  int tempi[2];
  float tempf[13];  
  mavlink_hil_sensor_t msg;

  while( !inFile.eof() ){

    inFile >> tempi[0] >> tempf[0] >> tempf[1] >> tempf[2] >> tempf[3] >> tempf[4] >> tempf[5] >> tempf[6] >> tempf[7] >> tempf[8] >> tempf[9] >> tempf[10] >> tempf[11] >> tempf[12] >> tempi[1];
    msg.time_usec  = tempi[0]; 
    msg.xacc  = tempf[0];
    msg.yacc  = tempf[1];
    msg.zacc = tempf[2];
    msg.xgyro = tempf[3];
    msg.ygyro = tempf[4];
    msg.zgyro = tempf[5];
    msg.xmag  = tempf[6];
    msg.ymag = tempf[7];
    msg.zmag = tempf[8];
    msg.abs_pressure = tempf[9];
    msg.diff_pressure = tempf[10];
    msg.pressure_alt  = tempf[11];
    msg.temperature  = tempf[12];
    msg.fields_updated = tempi[1];
    imu_array_.push_back(msg);
  }  
  inFile.close();
}

void GazeboMavlinkInterface::msg_read_gps(){
  
  std::ifstream inFile;    
  inFile.open(gps_file_name_, std::ios::in);
  int tempi[13];  
  mavlink_hil_gps_t temp;
  
  while( !inFile.eof() ){    
    inFile >> tempi[0] >> tempi[1] >> tempi[2] >> tempi[3] >> tempi[4] >> tempi[5] >> tempi[6] >> tempi[7] >> tempi[8] >> tempi[9] >> tempi[10] >> tempi[11] >> tempi[12];
    temp.time_usec  = tempi[0];
    temp.lat  = tempi[1];
    temp.lon  = tempi[2];
    temp.alt  = tempi[3];
    temp.eph  = tempi[4];
    temp.epv  = tempi[5];
    temp.vel = tempi[6];
    temp.vn = tempi[7];
    temp.ve = tempi[8];
    temp.vd = tempi[9];
    temp.cog = tempi[10];
    temp.fix_type = tempi[11];
    temp.satellites_visible = tempi[12];
    gps_array_.push_back(temp);
  }  
  inFile.close();
}

void GazeboMavlinkInterface::msg_read_state(){
  std::ifstream inFile;    
  inFile.open(state_file_name_, std::ios::in);
  int tempi;  
  bool tempb;
  mavlink_hil_state_t temp;
  
  while( !inFile.eof() ){    
    inFile >> tempi >> tempb;
    temp.time_usec  = tempi;
    temp.arm  = tempb;    
    state_array_.push_back(temp);
  }  
  inFile.close();
}

// reads in all sensors in the imu function for fastest time steps
void GazeboMavlinkInterface::msg_read_pseudo_sensors(mavlink_hil_sensor_t &sensor_msg){

  double current_time = world_->GetSimTime().Double() * 1e6;

  for(int it = 0; it < imu_array_.size(); it++){
    if(current_time  == imu_array_[it].time_usec ){
        sensor_msg = imu_array_[it];
    }
  }

  for(int it = 0; it < state_array_.size(); it++){
    if(current_time  == state_array_[it].time_usec ){
        if(state_array_[it].arm){
          pseudo_arm();
        }
    }
  } 

  if(gps_enable_){   
    mavlink_hil_gps_t hil_gps_msg;

    for(int it = 0; it < gps_array_.size(); it++){

      if(current_time == gps_array_[it].time_usec ){        
        hil_gps_msg = gps_array_[it];
        
        if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
          mavlink_message_t msg;
          mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);	  
          send_mavlink_message(&msg);
        }

      }

    }

  }
}

void GazeboMavlinkInterface::pseudo_arm(){  

  // std::cout << offboard_ << " " << armed_ <<std::endl;
  if(!offboard_ && armed_){  
    mavros_msgs::SetMode srv_setMode;
    // srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "OFFBOARD";
    if(mode_client_.call(srv_setMode)){        
        offboard_ = true;
    }
  }else{    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "";
    // pose.header.stamp = world_->GetSimTime();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 10.0;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.w = 0.0;
    pose.pose.orientation.w = 0.0;
    pose.pose.orientation.w = 0.0;

    local_pos_pub_.publish(pose);   
    ros::spinOnce(); 
  }
    
  if( !armed_ ){            
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  
    if( arming_client_.call(arm_cmd)){
      armed_ = true;
    }
  }
}

void GazeboMavlinkInterface::vioCallback(GpsPtr& gps_msg) {
  // std::cout << "vio" << std::endl;


}

}
