#ifndef ROS_CONTROL__SMART_HARDWARE_INTERFACE_H
#define ROS_CONTROL__SMART_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <smart_msgs/MotorCommand.h>
#include <smart_msgs/RawSensors.h>
#include <smart_msgs/UltrasonicSensors.h>

namespace smart_hardware_interface
{
typedef struct {
  double right_wheel_joint;
  double left_wheel_joint;
  double right_suspension_wheel_joint;
  double left_suspension_wheel_joint;
  double camera_pan;
  double camera_tilt;
} Angle;

typedef struct {
  long right_wheel_joint;
  long left_wheel_joint;
  long right_suspension_wheel_joint;
  long left_suspension_wheel_joint;
  long camera_pan;
  long camera_tilt;
} Encoder;

typedef struct {
  ros::Time stamp;
  Encoder overflow;
  Encoder position;
  Angle angle;
} MotorState;


class SmartHardwareInterface: public hardware_interface::RobotHW
{
public:
  SmartHardwareInterface(ros::NodeHandle& nh);
  ~SmartHardwareInterface();
  void init();
  void cleanup();

  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

  double getUpdateLoopFrequency() const;
  double getUpdateLoopRate() const;

private:
  void reset();
  void sensorsCallback(const smart_msgs::RawSensors &sensor_msg);

  ros::NodeHandle nh_;
  ros::Publisher motor_command_publisher_, ultrasonic_publisher_;
  ros::Subscriber read_sensors_subscriber_;

  int                                          num_joints_;
  std::vector<std::string>                     joint_names_;
  std::vector<int>                             joint_types_;
  std::vector<double>                          joint_position_;
  std::vector<double>                          joint_velocity_;
  std::vector<double>                          joint_effort_;
  std::vector<double>                          joint_position_command_;
  std::vector<double>                          joint_velocity_command_;
  std::vector<double>                          joint_lower_limits_;
  std::vector<double>                          joint_upper_limits_;

  hardware_interface::JointStateInterface      joint_state_interface_;
  hardware_interface::PositionJointInterface   position_joint_interface_;
  hardware_interface::VelocityJointInterface   velocity_joint_interface_;

  joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

  hardware_interface::PositionJointInterface positionJointInterface;
  joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;

  boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
  ros::CallbackQueue subscriber_queue_;

  MotorState actual_state;
  MotorState previous_state;

  double loop_hz_;
};

}

#endif
