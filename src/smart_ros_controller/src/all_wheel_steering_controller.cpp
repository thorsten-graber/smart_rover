#include "quaternions.h"

#include <boost/assign.hpp>

#include </home/meteron/smart_rover/src/smart_ros_controller/include/smart_ros_controller/all_wheel_steering_controller.h>

#define NO_OBSTICAL 2.55f

namespace smart_ros_controller{

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

AllWheelSteeringController::AllWheelSteeringController()
  : odom_frame_id_("map")
  , base_frame_id_("base_link")
  , enable_odom_tf_(true)
{
}

bool AllWheelSteeringController::init(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle& root_nh,
                                      ros::NodeHandle &controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Set joint names
  std::vector<std::string> front_wheel_names, middle_wheel_names, rear_wheel_names;
  front_wheel_names.push_back("front_left_wheel_joint");
  front_wheel_names.push_back("front_right_wheel_joint");
  middle_wheel_names.push_back("middle_left_wheel_joint");
  middle_wheel_names.push_back("middle_right_wheel_joint");
  rear_wheel_names.push_back("rear_left_wheel_joint");
  rear_wheel_names.push_back("rear_right_wheel_joint");

  front_wheel_joints_.resize(2);
  middle_wheel_joints_.resize(2);
  rear_wheel_joints_.resize(2);

  // Set steering joint names
  std::vector<std::string> front_steering_names, middle_steering_names, rear_steering_names;
  front_steering_names.push_back("front_left_wheel_steering_joint");
  front_steering_names.push_back("front_right_wheel_steering_joint");
  middle_steering_names.push_back("middle_left_wheel_steering_joint");
  middle_steering_names.push_back("middle_right_wheel_steering_joint");
  rear_steering_names.push_back("rear_left_wheel_steering_joint");
  rear_steering_names.push_back("rear_right_wheel_steering_joint");

  front_steering_joints_.resize(2);
  middle_steering_joints_.resize(2);
  rear_steering_joints_.resize(2);

  // Odometry related:
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Odom frame_id set to " << odom_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is "
                        << (enable_odom_tf_?"enabled":"disabled"));

  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                        << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  // Drive command related:
  controller_nh.param("drive_cmd_timeout", motor_control_parameter_.motor_cmd_timeout, motor_control_parameter_.motor_cmd_timeout);
  ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                        << motor_control_parameter_.motor_cmd_timeout << "s.");

  controller_nh.param("min_velocity", motor_control_parameter_.min_velocity, 0.01);
  ROS_INFO_STREAM_NAMED(name_, "Minimum commanded velocity "
                        << motor_control_parameter_.min_velocity << "s.");

  controller_nh.param("max_velocity", motor_control_parameter_.max_velocity, 0.1);
  ROS_INFO_STREAM_NAMED(name_, "Maximum commanded velocity "
                        << motor_control_parameter_.max_velocity << "s.");

  controller_nh.param("min_object_distance", motor_control_parameter_.min_object_distance, 0.2);
  ROS_INFO_STREAM_NAMED(name_, "Minimum distance for ultrasonic signal to auto stop the base "
                        << motor_control_parameter_.min_object_distance << "s.");

  // Rover model related
  controller_nh.param("wheel_track", motor_control_parameter_.wheel_track, 0.287);
  controller_nh.param("wheel_radius", motor_control_parameter_.wheel_radius, 0.035);
  controller_nh.param("wheel_base", motor_control_parameter_.wheel_base, 0.300);

  setOdomPubFields(controller_nh);

  hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();

  // Get the wheel joint objects to use in the realtime loop
  for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
  {
    ROS_INFO_STREAM_NAMED(name_,
                          "Adding left front wheel with joint name: " << front_wheel_names[i]
                          << " and right middle wheel with joint name: " << middle_wheel_names[i]
                          << " and right front wheel with joint name: " << rear_wheel_names[i]);
    front_wheel_joints_[i]  = vel_joint_hw->getHandle(front_wheel_names[i]);  // throws on failure
    middle_wheel_joints_[i] = vel_joint_hw->getHandle(middle_wheel_names[i]);  // throws on failure
    rear_wheel_joints_[i]   = vel_joint_hw->getHandle(rear_wheel_names[i]);  // throws on failure
  }

  // Get the steering joint objects to use in the realtime loop
  for (size_t i = 0; i < front_steering_joints_.size(); ++i)
  {
    ROS_INFO_STREAM_NAMED(name_,
                          "Adding left steering with joint name: " << front_steering_names[i]
                          << " and right steering with joint name: " << rear_steering_names[i]);
    front_steering_joints_[i]  = pos_joint_hw->getHandle(front_steering_names[i]);  // throws on failure
    middle_steering_joints_[i] = pos_joint_hw->getHandle(middle_steering_names[i]);  // throws on failure
    rear_steering_joints_[i]   = pos_joint_hw->getHandle(rear_steering_names[i]);  // throws on failure
  }

  reset();


  motion_command_subscriber_ = controller_nh.subscribe("command", 10, &AllWheelSteeringController::motionCommandCallback, this);
  ultrasonic_sensors_subscriber_ = root_nh.subscribe("ultrasonic/sensors", 10, &AllWheelSteeringController::ultrasonicSensorsCallback, this);

  odom_publisher_ = controller_nh.advertise<nav_msgs::Odometry>("state", 10, true);

  return true;
}

void AllWheelSteeringController::reset()
{
  ultrasonic_sensors_.front_left.range  = NO_OBSTICAL;
  ultrasonic_sensors_.front_right.range = NO_OBSTICAL;
  ultrasonic_sensors_.rear_left.range   = NO_OBSTICAL;
  ultrasonic_sensors_.rear_right.range  = NO_OBSTICAL;

  // todo: reset odometric pose and tf
  odom_.pose.pose.position.x    = 0;
  odom_.pose.pose.position.y    = 0;
  odom_.pose.pose.orientation.w = 1;
  odom_.pose.pose.orientation.x = 0;
  odom_.pose.pose.orientation.y = 0;
  odom_.pose.pose.orientation.z = 0;

  mode_old_  = "";
  steer_old_ = 0;
}

void AllWheelSteeringController::update(const ros::Time& time, const ros::Duration& period)
{
  updateOdometry(time, period);
  updateCommand(time);
}

void AllWheelSteeringController::starting(const ros::Time& time)
{
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;
}

void AllWheelSteeringController::stopping(const ros::Time& /*time*/)
{
  brake();
}

void AllWheelSteeringController::updateOdometry(const ros::Time& time, const ros::Duration &period)
{
  // vehicle geometry
  const double l = motor_control_parameter_.wheel_base;
  const double r = motor_control_parameter_.wheel_radius;

  const double alpha_l = middle_steering_joints_[0].getVelocity() * period.toSec();
  const double alpha_r = middle_steering_joints_[1].getVelocity() * period.toSec();
  const double phi_l   = front_steering_joints_[0].getPosition();
  const double phi_r   = front_steering_joints_[1].getPosition();

  // Compute angular velocity for ICC which is same as angular velocity of vehicle
  double yaw = (alpha_l * sin(phi_l) + alpha_r * sin(phi_r))* r / l;

  // Compute translation using previous yaw angle using last pose
  double euler[3];
  quaternion2euler(odom_.pose.pose.orientation, euler);
  double s = r * 2 * (alpha_l + alpha_r)/2/3;  // includes gear factor (2/3) // todo: make this parameter configurable
  double x = s * cos(euler[0]);
  double y = s * sin(euler[0]);

  // Publish Odom
  // set header
  odom_.header.stamp = time;

  //Compute velocity
  odom_.twist.twist.linear.x  = x / period.toSec();
  odom_.twist.twist.linear.y  = y / period.toSec();
  odom_.twist.twist.angular.z = yaw / period.toSec();

  // Compute odometric pose
  geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw);
  odom_.pose.pose.position.x  += x;
  odom_.pose.pose.position.y  += y;
  odom_.pose.pose.orientation = odom_.pose.pose.orientation*orientation;

  // Publish odometry message
  if (last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;

    //publish message
    odom_publisher_.publish(odom_);

    // Publish tf /odom frame
    if (enable_odom_tf_)
    {
      // Publish tf
      odom_trans_.header.stamp    = time;
      odom_trans_.transform.translation.x = odom_.pose.pose.position.x;
      odom_trans_.transform.translation.y = odom_.pose.pose.position.y;
      odom_trans_.transform.translation.z = odom_.pose.pose.position.z;
      odom_trans_.transform.rotation      = odom_.pose.pose.orientation;

      odom_broadcaster_.sendTransform(odom_trans_);
    }
  }
}

void AllWheelSteeringController::updateCommand(const ros::Time& time)
{
  // process motion command message
  double steer_l, steer_r, speed_r, speed_l;

  double r = motor_control_parameter_.wheel_radius;


  double cmd_vel = limitVelocity(motion_cmd_.speed);

  // update desired motor velocity/position based on control mode
  if(!strcmp(motion_cmd_.mode.c_str(), "continuous")) {
    // limit max dynamic in steering (max delta per step)
    const double max_delta = 0.0025;
    const double delta = motion_cmd_.steer-steer_old_;
    double steer = (fabs(delta) > max_delta) ? (steer_old_+(double)sgn(delta) * max_delta) : motion_cmd_.steer;
    steer_old_ = steer;

    ComputeLocomotion(cmd_vel, steer, speed_l, speed_r, steer_l, steer_r);

  } else if (!motion_cmd_.mode.compare("point_turn")) {
    steer_l = -M_PI_4;
    steer_r =  M_PI_4;
    speed_l = -cmd_vel;
    speed_r =  cmd_vel;
    steer_old_ = 0;
  } else {
    ROS_INFO_ONCE_NAMED(name_,"No command received, or unknown command mode set - stopping rover!");
    steer_l = 0;
    steer_r = 0;
    speed_l = 0;
    speed_r = 0;
  }

  // only drive, if wheel joint angle error is almost zero
  const double threshold = 3*M_PI/180; // TODO: make this parameter configurable
  const double phi_l = front_steering_joints_[0].getPosition();
  const double phi_r = front_steering_joints_[1].getPosition();

  if ((fabs(steer_l  - phi_l) > threshold || fabs(steer_r  - phi_r) > threshold) && (!motion_cmd_.mode.compare("point_turn") || !mode_old_.compare("point_turn")))
  {
    mode_old_ = "point_turn";
    speed_l = 0.0;
    speed_r = 0.0;
  } else {
    mode_old_ = motion_cmd_.mode.c_str();
  }

//  //omega = speed(wheel speed) * 720(ticks per turn) * 0.02ms(controller frequency) * 3/2(gear factor) / r(wheel radius)
//  motor_control_parameters.control_input.speed_r = speed_r * 720 * 0.02 * 3 / 2 / r; // todo: make parameters configurable
//  motor_control_parameters.control_input.speed_l = speed_l * 720 * 0.02 * 3 / 2 / r; // todo: make parameters configurable

//  ROS_DEBUG_STREAM_THROTTLE(1, "vel_left_rear "<<vel_left_rear<<" front_right_steering "<<front_right_steering);
//  // Set wheels velocities:
//  if(front_wheel_joints_.size() == 2 && rear_wheel_joints_.size() == 2)
//  {
//    front_wheel_joints_[0].setCommand(vel_left_front);
//    front_wheel_joints_[1].setCommand(vel_right_front);
//    rear_wheel_joints_[0].setCommand(vel_left_rear);
//    rear_wheel_joints_[1].setCommand(vel_right_rear);
//  }

//  /// TODO check limits to not apply the same steering on right and left when saturated !
//  if(front_steering_joints_.size() == 2 && rear_steering_joints_.size() == 2)
//  {
//    front_steering_joints_[0].setCommand(front_left_steering);
//    front_steering_joints_[1].setCommand(front_right_steering);
//    rear_steering_joints_[0].setCommand(rear_left_steering);
//    rear_steering_joints_[1].setCommand(rear_right_steering);
//  }
}

double AllWheelSteeringController::limitVelocity(double value) {
  double velocity = value;
  if (value > 0.0) {
    if (value > motor_control_parameter_.max_velocity) velocity = motor_control_parameter_.max_velocity;
    if (value < motor_control_parameter_.min_velocity) velocity = motor_control_parameter_.min_velocity;
    if (ultrasonic_sensors_.front_left.range < motor_control_parameter_.min_object_distance || ultrasonic_sensors_.front_right.range < motor_control_parameter_.min_object_distance) {
      velocity = 0.0f;
      ROS_WARN("OBSTACLE in the FRONT, STOPPING MOTORS distance to obstacle: %f", motor_control_parameter_.min_object_distance);
    }
  } else if (value < 0.0) {
    if (value < -motor_control_parameter_.max_velocity) velocity = -motor_control_parameter_.max_velocity;
    if (value > -motor_control_parameter_.min_velocity) velocity = -motor_control_parameter_.min_velocity;
    if (ultrasonic_sensors_.rear_left.range < motor_control_parameter_.min_object_distance || ultrasonic_sensors_.rear_right.range < motor_control_parameter_.min_object_distance) {
      velocity = 0.0f;
      ROS_WARN("OBSTACLE in the BACK, STOPPING MOTORS distance to obstacle: %f", motor_control_parameter_.min_object_distance);
    }
  }
  return velocity;
}

void AllWheelSteeringController::brake()
{
  const double vel = 0.0;
  for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
  {
    front_wheel_joints_[i].setCommand(vel);
    middle_wheel_joints_[i].setCommand(vel);
    rear_wheel_joints_[i].setCommand(vel);
  }

  const double pos = 0.0;
  for (size_t i = 0; i < front_steering_joints_.size(); ++i)
  {
    front_steering_joints_[i].setCommand(pos);
    middle_steering_joints_[i].setCommand(pos);
    rear_steering_joints_[i].setCommand(pos);
  }
}

void AllWheelSteeringController::motionCommandCallback(const smart_msgs::MotionCommand::ConstPtr& motion_cmd_msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  motion_cmd_ = *motion_cmd_msg;
}

void AllWheelSteeringController::ultrasonicSensorsCallback(const smart_msgs::UltrasonicSensors::ConstPtr& ultrasonic_sensors_msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  ultrasonic_sensors_ = *ultrasonic_sensors_msg;
}

void AllWheelSteeringController::setOdomPubFields(ros::NodeHandle& controller_nh)
{
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_.header.frame_id = odom_frame_id_;
  odom_.child_frame_id = base_frame_id_;
  odom_.pose.pose.position.z = 0.1;
  odom_.pose.covariance = boost::assign::list_of
      (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
      (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
  odom_.twist.twist.linear.y  = 0;
  odom_.twist.twist.linear.z  = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.covariance = boost::assign::list_of
      (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
      (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));

  odom_trans_.transform.translation.z = odom_.pose.pose.position.z;
  odom_trans_.child_frame_id = base_frame_id_;
  odom_trans_.header.frame_id = odom_frame_id_;
}

} // namespace smart_ros_controller
