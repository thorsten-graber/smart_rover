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

  // Odometry related:
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Odom frame_id set to " << odom_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

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

  // Get wheel joint objects
  front_wheel_joints_.resize(2);
  middle_wheel_joints_.resize(2);
  rear_wheel_joints_.resize(2);
  front_wheel_joints_[0]  = vel_joint_hw->getHandle("front_left_wheel_joint");
  front_wheel_joints_[1]  = vel_joint_hw->getHandle("front_right_wheel_joint");
  middle_wheel_joints_[0] = vel_joint_hw->getHandle("middle_left_wheel_joint");
  middle_wheel_joints_[1] = vel_joint_hw->getHandle("middle_right_wheel_joint");
  rear_wheel_joints_[0]   = vel_joint_hw->getHandle("rear_left_wheel_joint");
  rear_wheel_joints_[1]   = vel_joint_hw->getHandle("rear_right_wheel_joint");

  // Get steering joint objects
  front_steering_joints_.resize(2);
  middle_steering_joints_.resize(2);
  rear_steering_joints_.resize(2);
  front_steering_joints_[0]  = pos_joint_hw->getHandle("front_left_wheel_steering_joint");
  front_steering_joints_[1]  = pos_joint_hw->getHandle("front_right_wheel_steering_joint");
  middle_steering_joints_[0] = pos_joint_hw->getHandle("middle_left_wheel_steering_joint");
  middle_steering_joints_[1] = pos_joint_hw->getHandle("middle_right_wheel_steering_joint");
  rear_steering_joints_[0]   = pos_joint_hw->getHandle("rear_left_wheel_steering_joint");
  rear_steering_joints_[1]   = pos_joint_hw->getHandle("rear_right_wheel_steering_joint");

  // Init PID controllers
  pid_controllers_.resize(12);
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/front_left_wheel_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/front_right_wheel_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/middle_left_wheel_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/middle_right_wheel_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/rear_left_wheel_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/rear_right_wheel_joint"));

  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/front_left_wheel_steering_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/front_right_wheel_steering_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/middle_left_wheel_steering_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/middle_right_wheel_steering_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/rear_left_wheel_steering_joint"));
  pid_controllers_.push_back(control_toolbox::Pid().init(controller_nh, "gains/rear_right_wheel_steering_joint"));

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
  updateCommand(period);
}

void AllWheelSteeringController::starting(const ros::Time& time)
{
  for(int i=0;i<12;i++) {
    pid_controllers_[i].reset();
  }

  brake();
}

void AllWheelSteeringController::stopping(const ros::Time& time)
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
  odom_publisher_.publish(odom_);

  // Publish tf /odom frame
  if (enable_odom_tf_) {
    // Publish tf
    odom_trans_.header.stamp    = time;
    odom_trans_.transform.translation.x = odom_.pose.pose.position.x;
    odom_trans_.transform.translation.y = odom_.pose.pose.position.y;
    odom_trans_.transform.translation.z = odom_.pose.pose.position.z;
    odom_trans_.transform.rotation      = odom_.pose.pose.orientation;

    odom_broadcaster_.sendTransform(odom_trans_);
  }
}

void AllWheelSteeringController::updateCommand(const ros::Duration &period)
{
  // process motion command message
  double steer_l, steer_r, speed_r, speed_l;
  double cmd_vel = limitVelocity(motion_cmd_.speed);
  const double r = motor_control_parameter_.wheel_radius;

  // update desired motor velocity/position based on control mode
  if(!strcmp(motion_cmd_.mode.c_str(), "continuous")) {
    // limit max dynamic in steering (max delta per step)
    const double max_delta = 0.0025;
    double delta = motion_cmd_.steer-steer_old_;
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

  const double phi_fl = front_steering_joints_[0].getPosition();
  const double phi_fr = front_steering_joints_[1].getPosition();
  const double phi_ml = middle_steering_joints_[0].getPosition();
  const double phi_mr = middle_steering_joints_[1].getPosition();
  const double phi_rl = rear_steering_joints_[0].getPosition();
  const double phi_rr = rear_steering_joints_[1].getPosition();

  // only drive, if wheel joint angle error is almost zero
  const double threshold = 3*M_PI/180; // TODO: make this parameter configurable
  if ((fabs(steer_l  - phi_fl) > threshold || fabs(steer_r  - phi_fr) > threshold) && (!motion_cmd_.mode.compare("point_turn") || !mode_old_.compare("point_turn")))
  {
    mode_old_ = "point_turn";
    speed_l = 0.0;
    speed_r = 0.0;
  } else {
    mode_old_ = motion_cmd_.mode.c_str();
  }

  //omega = speed(wheel speed) * 720(ticks per turn) * 0.02ms(controller frequency) * 3/2(gear factor) / r(wheel radius)
  speed_l /= r;
  speed_r /= r;

  // Set wheels velocities:
  front_wheel_joints_[0].setCommand(pid_controllers_[0].computeCommand( speed_l / cos(phi_fl) - front_wheel_joints_[0].getVelocity(),  period));
  front_wheel_joints_[1].setCommand(pid_controllers_[1].computeCommand( speed_r / cos(phi_fr) - front_wheel_joints_[1].getVelocity(),  period));
  middle_wheel_joints_[0].setCommand(pid_controllers_[2].computeCommand(speed_l               - middle_wheel_joints_[0].getVelocity(), period));
  middle_wheel_joints_[1].setCommand(pid_controllers_[3].computeCommand(speed_r               - middle_wheel_joints_[1].getVelocity(), period));
  rear_wheel_joints_[0].setCommand(pid_controllers_[4].computeCommand(  speed_l / cos(phi_rl) - rear_wheel_joints_[0].getVelocity(),   period));
  rear_wheel_joints_[1].setCommand(pid_controllers_[5].computeCommand(  speed_r / cos(phi_rr) - rear_wheel_joints_[1].getVelocity(),   period));

  // Set steering
  front_steering_joints_[0].setCommand(pid_controllers_[6].computeCommand( steer_l  - phi_fl, front_steering_joints_[0].getVelocity(),  period));
  front_steering_joints_[1].setCommand(pid_controllers_[7].computeCommand( steer_r  - phi_fr, front_steering_joints_[1].getVelocity(),  period));
  middle_steering_joints_[0].setCommand(pid_controllers_[8].computeCommand(0.0      - phi_ml, middle_steering_joints_[0].getVelocity(), period));
  middle_steering_joints_[1].setCommand(pid_controllers_[9].computeCommand(0.0      - phi_mr, middle_steering_joints_[1].getVelocity(), period));
  rear_steering_joints_[0].setCommand(pid_controllers_[10].computeCommand( -steer_l - phi_fl, rear_steering_joints_[0].getVelocity(),   period));
  rear_steering_joints_[1].setCommand(pid_controllers_[11].computeCommand( -steer_r - phi_fl, rear_steering_joints_[1].getVelocity(),   period));
}

void AllWheelSteeringController::ComputeLocomotion(double speed, double steer, double &speed_l, double &speed_r, double &steer_l, double &steer_r)
{
    const double b = motor_control_parameter_.wheel_track;
    const double l = motor_control_parameter_.wheel_base;

    double tan_steer;
    if(steer > M_PI_2) {
        tan_steer = tan(M_PI_2);
    } else if(steer < -M_PI_2) {
        tan_steer = tan(-M_PI_2);
    } else {
        tan_steer = tan(steer);
    }

    steer_l = atan2(l*tan_steer,l-b*tan_steer);
    steer_r = atan2(l*tan_steer,l+b*tan_steer);

    speed_l = speed*(1-b*tan_steer/l);
    speed_r = speed*(1+b*tan_steer/l);

    // limit wheel speeds for small radius
    if(steer > 0) {
        // turn left -> check right wheel speed
        if(speed_r > motor_control_parameter_.max_velocity) {
            speed_r = motor_control_parameter_.max_velocity;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        } else if(speed_r < -motor_control_parameter_.max_velocity) {
            speed_r = -motor_control_parameter_.max_velocity;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        }
    } else {
        // turn right -> check left wheel speed
        if(speed_l > motor_control_parameter_.max_velocity) {
            speed_l = motor_control_parameter_.max_velocity;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        } else if(speed_l < -motor_control_parameter_.max_velocity) {
            speed_l = -motor_control_parameter_.max_velocity;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        }
    }
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
  for (size_t i = 0; i < front_wheel_joints_.size(); ++i) {
    front_wheel_joints_[i].setCommand(0.0);
    middle_wheel_joints_[i].setCommand(0.0);
    rear_wheel_joints_[i].setCommand(0.0);
  }

  for (size_t i = 0; i < front_steering_joints_.size(); ++i) {
    front_steering_joints_[i].setCommand(0.0);
    middle_steering_joints_[i].setCommand(0.0);
    rear_steering_joints_[i].setCommand(0.0);
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
PLUGINLIB_EXPORT_CLASS(smart_ros_controller::AllWheelSteeringController, controller_interface::ControllerBase)
