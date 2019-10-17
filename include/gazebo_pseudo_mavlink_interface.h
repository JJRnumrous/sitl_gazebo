/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
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
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <fstream>
#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <common.h>
#include <CommandMotorThrottle.pb.h>
#include <Imu.pb.h>
#include <OpticalFlow.pb.h>
#include <Range.pb.h>
#include <SITLGps.pb.h>
#include <IRLock.pb.h>
#include <Groundtruth.pb.h>
#include <Odometry.pb.h>

#include <mavlink/v2.0/common/mavlink.h>
#include "msgbuffer.h"

#include <geo_mag_declination.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>

#include <sstream>


static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultMavlinkTcpPort = 4560;
static const uint32_t kDefaultQGCUdpPort = 14550;
static const uint32_t kDefaultSDKUdpPort = 14540;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

namespace gazebo {

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorThrottle> CommandMotorThrottlePtr;
typedef const boost::shared_ptr<const nav_msgs::msgs::Odometry> OdomPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> IRLockPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::OpticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> SonarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> LidarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;

// Default values
static const std::string kDefaultNamespace = "";

static const std::string kDefaultMotorReferencePubTopic = "/command/motor";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultLidarTopic = "/link/lidar";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultSonarTopic = "/sonar_model/link/sonar";
static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
static const std::string kDefaultGPSTopic = "/gps";
static const std::string kDefaultVisionTopic = "/vision_odom";

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
	incomplete = MAVLINK_FRAMING_INCOMPLETE,
	ok = MAVLINK_FRAMING_OK,
	bad_crc = MAVLINK_FRAMING_BAD_CRC,
	bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

class GazeboMavlinkInterface : public ModelPlugin {
public:
  GazeboMavlinkInterface() : ModelPlugin(),
    received_first_actuator_(false),
    namespace_(kDefaultNamespace),
    protocol_version_(2.0),
    motor_reference_pub_topic_(kDefaultMotorReferencePubTopic),
    use_propeller_pid_(false),
    use_elevator_pid_(false),
    use_left_elevon_pid_(false),
    use_right_elevon_pid_(false),
    vehicle_is_tailsitter_(false),
    send_vision_estimation_(false),
    send_odometry_(false),
    imu_sub_topic_(kDefaultImuTopic),
    opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
    lidar_sub_topic_(kDefaultLidarTopic),
    sonar_sub_topic_(kDefaultSonarTopic),
    irlock_sub_topic_(kDefaultIRLockTopic),
    gps_sub_topic_(kDefaultGPSTopic),
    vision_sub_topic_(kDefaultVisionTopic),    
    model_ {},
    world_(nullptr),
    left_elevon_joint_(nullptr),
    right_elevon_joint_(nullptr),
    elevator_joint_(nullptr),
    propeller_joint_(nullptr),
    gimbal_yaw_joint_(nullptr),
    gimbal_pitch_joint_(nullptr),
    gimbal_roll_joint_(nullptr),
    input_index_ {},
    groundtruth_lat_rad(0.0),
    groundtruth_lon_rad(0.0),
    groundtruth_altitude(0.0),
    mavlink_udp_port_(kDefaultMavlinkUdpPort),
    mavlink_tcp_port_(kDefaultMavlinkTcpPort),
    simulator_socket_fd_(0),
    simulator_tcp_client_fd_(0),
    use_tcp_(false),
    qgc_udp_port_(kDefaultQGCUdpPort),
    sdk_udp_port_(kDefaultSDKUdpPort),
    remote_qgc_addr_ {},
    local_qgc_addr_ {},
    remote_sdk_addr_ {},
    local_sdk_addr_ {},
    qgc_socket_fd_(0),
    sdk_socket_fd_(0),
    enable_sim_(true),
    serial_enabled_(false),
    tx_q {},
    rx_buf {},
    m_status {},
    m_buffer {},
    io_service(),
    serial_dev(io_service),
    device_(kDefaultDevice),
    baudrate_(kDefaultBaudRate),
    hil_mode_(false),
    hil_state_level_(false),
    baro_rnd_y2_(0.0),
    baro_rnd_use_last_(false)
    {}

  ~GazeboMavlinkInterface();

  void Publish();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  bool received_first_actuator_;
  Eigen::VectorXd input_reference_;

  float protocol_version_;

  std::string namespace_;
  std::string motor_reference_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_reference_pub_;
  transport::SubscriberPtr mav_control_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr left_elevon_joint_;
  physics::JointPtr right_elevon_joint_;
  physics::JointPtr elevator_joint_;
  physics::JointPtr propeller_joint_;
  physics::JointPtr gimbal_yaw_joint_;
  physics::JointPtr gimbal_pitch_joint_;
  physics::JointPtr gimbal_roll_joint_;
  common::PID propeller_pid_;
  common::PID elevator_pid_;
  common::PID left_elevon_pid_;
  common::PID right_elevon_pid_;
  bool use_propeller_pid_;
  bool use_elevator_pid_;
  bool use_left_elevon_pid_;
  bool use_right_elevon_pid_;

  bool vehicle_is_tailsitter_;

  bool send_vision_estimation_;
  bool send_odometry_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr sigIntConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void ImuCallback(ImuPtr& imu_msg);
  void GpsCallback(GpsPtr& gps_msg);
  void GroundtruthCallback(GtPtr& groundtruth_msg);
  void LidarCallback(LidarPtr& lidar_msg);
  void SonarCallback(SonarPtr& sonar_msg);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void IRLockCallback(IRLockPtr& irlock_msg);
  void VisionCallback(OdomPtr& odom_msg);
  void send_mavlink_message(const mavlink_message_t *message);
  void forward_mavlink_message(const mavlink_message_t *message);
  void handle_message(mavlink_message_t *msg, bool &received_actuator);
  void pollForMAVLinkMessages();
  void pollFromQgcAndSdk();
  void SendSensorMessages();
  void handle_actuators(double dt);
  bool IsRunning();
  void onSigInt();

  // Serial interface
  void open();
  void close();
  void do_read();
  void parse_buffer(const boost::system::error_code& err, std::size_t bytes_t);
  void do_write(bool check_tx_state);
  inline bool is_open(){
    return serial_dev.is_open();
  }

  // Pseudo interfaces
  void msg_write_imu(mavlink_hil_sensor_t msg);
  void msg_read_imu();
  void msg_write_gps(mavlink_hil_gps_t hil_gps_msg);
  void msg_read_gps();
  void msg_read_pseudo_sensors(mavlink_hil_sensor_t &sensor_msg);
  void msg_read_state();
  void pseudo_arm();
  void vioCallback(GpsPtr& gps_msg);

  void StateCallback(bool armed);

  static const unsigned n_out_max = 16;
  double alt_home = 488.0;   // meters

  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  int input_index_[n_out_max];

  transport::SubscriberPtr imu_sub_;
  transport::SubscriberPtr lidar_sub_;
  transport::SubscriberPtr sonar_sub_;
  transport::SubscriberPtr opticalFlow_sub_;
  transport::SubscriberPtr irlock_sub_;
  transport::SubscriberPtr gps_sub_;
  transport::SubscriberPtr groundtruth_sub_;
  transport::SubscriberPtr vision_sub_;  
  transport::SubscriberPtr vio_sub_;

  std::string imu_sub_topic_;
  std::string lidar_sub_topic_;
  std::string opticalFlow_sub_topic_;
  std::string sonar_sub_topic_;
  std::string irlock_sub_topic_;
  std::string gps_sub_topic_;
  std::string groundtruth_sub_topic_;
  std::string vision_sub_topic_;  

  std::mutex last_imu_message_mutex_ {};
  std::condition_variable last_imu_message_cond_ {};
  sensor_msgs::msgs::Imu last_imu_message_;
  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;

  double groundtruth_lat_rad;
  double groundtruth_lon_rad;
  double groundtruth_altitude;

  double imu_update_interval_ = 0.004; ///< Used for non-lockstep

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d mag_d_;

  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;

  
  struct sockaddr_in local_simulator_addr_;
  socklen_t local_simulator_addr_len_;
  struct sockaddr_in remote_simulator_addr_;
  socklen_t remote_simulator_addr_len_;

  int qgc_udp_port_;
  struct sockaddr_in remote_qgc_addr_;
  socklen_t remote_qgc_addr_len_;
  struct sockaddr_in local_qgc_addr_;
  socklen_t local_qgc_addr_len_;

  int sdk_udp_port_;
  struct sockaddr_in remote_sdk_addr_;
  socklen_t remote_sdk_addr_len_;
  struct sockaddr_in local_sdk_addr_;
  socklen_t local_sdk_addr_len_;

  unsigned char _buf[65535];
  bool use_tcp_ = false;

  double optflow_distance;
  double sonar_distance;

  in_addr_t mavlink_addr_;
  int mavlink_udp_port_; // MAVLink refers to the PX4 simulator interface here
  int mavlink_tcp_port_; // MAVLink refers to the PX4 simulator interface here


  int simulator_socket_fd_;
  int simulator_tcp_client_fd_;
 
  int qgc_socket_fd_ {-1};
  int sdk_socket_fd_ {-1};

  bool enable_lockstep_ = false;
  double speed_factor_ = 1.0;
  int64_t previous_imu_seq_ = 0;

  bool enable_sim_;

  // Serial interface
  mavlink_status_t m_status;
  mavlink_message_t m_buffer;
  bool serial_enabled_;
  std::thread io_thread;
  std::string device_;
  std::array<uint8_t, MAX_SIZE> rx_buf;
  std::recursive_mutex mutex;
  unsigned int baudrate_;
  std::atomic<bool> tx_in_progress;
  std::deque<MsgBuffer> tx_q;
  boost::asio::io_service io_service;
  boost::asio::serial_port serial_dev;

  bool hil_mode_;
  bool hil_state_level_;

  std::atomic<bool> gotSigInt_ {false};

  // state variables for baro pressure sensor random noise generator
  double baro_rnd_y2_;
  bool baro_rnd_use_last_;

  struct mavlink_hil_state_t{
    int time_usec;
    bool arm;
  };

  // sudo mavlink variables for writing and reading
  bool pseudo_enable_;    // for using data from gazebo or from stored file
  bool write_enable_;     // 1: write data to file -- 0: for when reading out of file
  bool gps_enable_;       // 1: using gps          -- 0: using vision
  std::string imu_file_name_;  // name of file IMU data is/needs to be stored in  
  std::string gps_file_name_;  // name of file IMU data is/needs to be stored in  
  std::string state_file_name_;
  std::vector<mavlink_hil_sensor_t> imu_array_;
  std::vector<mavlink_hil_gps_t> gps_array_; 
  std::vector<mavlink_hil_state_t> state_array_; 
  
  ros::ServiceClient mode_client_;
  ros::ServiceClient arming_client_; 
  ros::Publisher local_pos_pub_;
  ros::Publisher vision_pub_;

  bool armed_ = false; 
  bool offboard_ = false; 
  };
}
