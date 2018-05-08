/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Thorsten Graber
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/Odometry.h>
#include <smart_msgs/MotionCommand.h>
#include <smart_msgs/UltrasonicSensors.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <control_toolbox/pid.h>

namespace smart_ros_controller{

/**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
class AllWheelSteeringController
    : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
    hardware_interface::PositionJointInterface>
{
public:
  AllWheelSteeringController();

  /**
     * \brief Initialize controller
     * \param robot_hw      Velocity and position joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh);

  /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
     * \brief Starts controller
     * \param time Current time
     */
  void starting(const ros::Time& time);

  /**
     * \brief Stops controller
     * \param time Current time
     */
  void stopping(const ros::Time& /*time*/);

private:
  void reset();
  void ComputeLocomotion(double speed, double steer, double& speed_l, double& speed_r, double& steer_l, double& steer_r);
  double limitVelocity(double speed);

  /**
     * \brief Update and publish odometry
     * \param time   Current time
     */
  void updateOdometry(const ros::Time &time, const ros::Duration& period);
  /**
     * \brief Compute and publish command
     * \param time   Current time
     * \param period Time since the last called to update
     */
  void updateCommand(const ros::Duration& period);

  /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
  void brake();

  /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
  void motionCommandCallback(const smart_msgs::MotionCommand::ConstPtr& motion_cmd_msg);



  void ultrasonicSensorsCallback(const smart_msgs::UltrasonicSensors::ConstPtr& ultrasonic_sensors_msg);

  /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
  void setOdomPubFields(ros::NodeHandle& controller_nh);

  std::string name_;
  boost::mutex mutex_;

  /// ROS related:
  ros::Subscriber motion_command_subscriber_;
  ros::Subscriber ultrasonic_sensors_subscriber_;
  ros::Publisher odom_publisher_;

  /// Hardware handles:
  std::vector<hardware_interface::JointHandle> front_wheel_joints_;
  std::vector<hardware_interface::JointHandle> middle_wheel_joints_;
  std::vector<hardware_interface::JointHandle> rear_wheel_joints_;
  std::vector<hardware_interface::JointHandle> front_steering_joints_;
  std::vector<hardware_interface::JointHandle> middle_steering_joints_;
  std::vector<hardware_interface::JointHandle> rear_steering_joints_;

  /// Command reated:
  struct MotorControlParameter {
    double min_velocity;
    double max_velocity;
    double min_object_distance;
    double wheel_track;
    double wheel_base;
    double wheel_radius;
    double motor_cmd_timeout;
  };
  MotorControlParameter motor_control_parameter_;
  smart_msgs::MotionCommand motion_cmd_;
  smart_msgs::UltrasonicSensors ultrasonic_sensors_;
  double steer_old_;
  std::string mode_old_;

  /// PID controller related:
  std::vector<control_toolbox::Pid> pid_controllers_;

  /// Odometry related:
  nav_msgs::Odometry odom_;
  geometry_msgs::TransformStamped odom_trans_;
  tf::TransformBroadcaster odom_broadcaster_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool enable_odom_tf_;
};
} // namespace smart_ros_controller
