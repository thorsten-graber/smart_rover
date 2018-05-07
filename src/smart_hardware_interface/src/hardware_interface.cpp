#include <smart_hardware_interface/hardware_interface.h>

namespace smart_hardware_interface
{
  static double degToRad(double deg) {
      return (deg * M_PI / 180.0);
  }

  static int16_t posToDeg(long position) {
      return (int16_t) (position / 2l % 360l);
  }

  static long correctPosition(const int16_t actual_position_raw, long &previous_position, long &overflow) {
      long delta_position = previous_position - actual_position_raw + overflow*USHRT_MAX;

      // check if encoder overrun has happend, and correct if so
      if (delta_position > SHRT_MAX) { overflow++; }
      if (delta_position < SHRT_MIN) { overflow--; }

      return (long)actual_position_raw + overflow*USHRT_MAX;
  }

  static double centimeterToMeter(int centimeter) {
    return (double) (centimeter / 100.0f);
  }

  SmartHardwareInterface::SmartHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
	{
		init();
		ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
	}

  SmartHardwareInterface::~SmartHardwareInterface()
	{
	}

  void SmartHardwareInterface::cleanup()
  {
    subscriber_spinner_->stop();
  }

  void SmartHardwareInterface::init()
	{
    // Get update loop frequency
    nh_.param("/smart/hardware_interface/loop_hz", loop_hz_, 0.02);
    ROS_DEBUG_STREAM_NAMED("init","Using loop freqency of " << loop_hz_ << " hz");

		// Get joint names
    nh_.getParam("/smart/hardware_interface/joints", joint_names_);
    num_joints_ = joint_names_.size();
    if (num_joints_ == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}

		// Resize vectors
    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);
    joint_position_command_.resize(num_joints_, 0.0);
    joint_velocity_command_.resize(num_joints_, 0.0);

		// Initialize controller
    for (unsigned int i = 0; i < num_joints_; i++)
		{
		  // Create joint state interface
      hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		  joint_state_interface_.registerHandle(jointStateHandle);

		  // Create position joint interface
      hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
//      joint_limits_interface::JointLimits limits;
//      joint_limits_interface::SoftJointLimits softLimits;
//			if (getJointLimits(joint.name, nh_, limits) == false) {
//				ROS_ERROR_STREAM("Cannot set joint limits for " << joint.name);
//			} else {
//        joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
//				positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
//			}
		  position_joint_interface_.registerHandle(jointPositionHandle);

		  // Create velocity joint interface
      hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
      velocity_joint_interface_.registerHandle(jointVelocityHandle);    
		}

		registerInterface(&joint_state_interface_);
		registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    //registerInterface(&position_joint_limits_interface_);

    read_sensors_subscriber_ = nh_.subscribe("controller/base/raw_sensors", 10, &SmartHardwareInterface::sensorsCallback,this);
    motor_command_publisher_ = nh_.advertise<smart_msgs::MotorCommand>("controller/base/motor_command", 1, true);
    ultrasonic_publisher_    = nh_.advertise<smart_msgs::UltrasonicSensors>("ultrasonic/sensors", 1, true);

    reset();

    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
	}

  void SmartHardwareInterface::reset()
  {
    // reset sensor readings
    actual_state.stamp = ros::Time::now();

    actual_state.position.left_wheel_joint               = 0;
    actual_state.position.right_wheel_joint              = 0;
    actual_state.position.left_suspension_wheel_joint    = 0;
    actual_state.position.right_suspension_wheel_joint   = 0;
    actual_state.position.camera_pan                     = 0;
    actual_state.position.camera_tilt                    = 0;

    actual_state.overflow.left_wheel_joint               = 0;
    actual_state.overflow.right_wheel_joint              = 0;
    actual_state.overflow.left_suspension_wheel_joint    = 0;
    actual_state.overflow.right_suspension_wheel_joint   = 0;
    actual_state.overflow.camera_pan                     = 0;
    actual_state.overflow.camera_tilt                    = 0;

    previous_state.stamp = ros::Time::now();

    previous_state.position.left_wheel_joint             = 0;
    previous_state.position.right_wheel_joint            = 0;
    previous_state.position.left_suspension_wheel_joint  = 0;
    previous_state.position.right_suspension_wheel_joint = 0;
    previous_state.position.camera_pan                   = 0;
    previous_state.position.camera_tilt                  = 0;

    previous_state.overflow.left_wheel_joint             = 0;
    previous_state.overflow.right_wheel_joint            = 0;
    previous_state.overflow.left_suspension_wheel_joint  = 0;
    previous_state.overflow.right_suspension_wheel_joint = 0;
    previous_state.overflow.camera_pan                   = 0;
    previous_state.overflow.camera_tilt                  = 0;
  }

  void SmartHardwareInterface::read(ros::Time time, ros::Duration period)
	{
    joint_velocity_[0]  = (actual_state.angle.right_suspension_wheel_joint - joint_position_[0]) / period.toSec();
    joint_velocity_[1]  = 0;
    joint_velocity_[2]  = (-actual_state.angle.right_suspension_wheel_joint - joint_position_[2]) / period.toSec();

    joint_velocity_[3]  = (actual_state.angle.left_suspension_wheel_joint - joint_position_[3]) / period.toSec();
    joint_velocity_[4]  = 0;
    joint_velocity_[5]  = (-actual_state.angle.left_suspension_wheel_joint - joint_position_[5]) / period.toSec();

    joint_velocity_[6]  = (actual_state.angle.right_wheel_joint - joint_position_[6]) / period.toSec();
    joint_velocity_[7]  = (actual_state.angle.right_wheel_joint - joint_position_[7]) / period.toSec();
    joint_velocity_[8]  = (actual_state.angle.right_wheel_joint - joint_position_[8]) / period.toSec();

    joint_velocity_[9]  = (actual_state.angle.left_wheel_joint - joint_position_[9]) / period.toSec();
    joint_velocity_[0]  = (actual_state.angle.left_wheel_joint - joint_position_[10]) / period.toSec();
    joint_velocity_[11]  = (actual_state.angle.left_wheel_joint - joint_position_[11]) / period.toSec();

    joint_velocity_[12]  = (actual_state.angle.camera_pan - joint_position_[12]) / period.toSec();
    joint_velocity_[13]  = (actual_state.angle.camera_tilt - joint_position_[13]) / period.toSec();


    joint_position_[0]  = actual_state.angle.right_suspension_wheel_joint;
    joint_position_[1]  = 0;
    joint_position_[2]  = -actual_state.angle.right_suspension_wheel_joint;

    joint_position_[3]  = actual_state.angle.left_suspension_wheel_joint;
    joint_position_[4]  = 0;
    joint_position_[5]  = -actual_state.angle.left_suspension_wheel_joint;

    joint_position_[6]  = actual_state.angle.right_wheel_joint / cos(joint_position_[0]);;
    joint_position_[7]  = actual_state.angle.right_wheel_joint;
    joint_position_[8]  = actual_state.angle.right_wheel_joint / cos(joint_position_[0]);;

    joint_position_[9]  = actual_state.angle.left_wheel_joint / cos(joint_position_[5]);;
    joint_position_[10] = actual_state.angle.left_wheel_joint;
    joint_position_[11] = actual_state.angle.left_wheel_joint / cos(joint_position_[5]);;

    joint_position_[12] = actual_state.angle.camera_pan;
    joint_position_[13] = actual_state.angle.camera_tilt;
	}

  void SmartHardwareInterface::write(ros::Time time, ros::Duration period)
  {
    smart_msgs::MotorCommand cmd;
    cmd.steer_r = joint_position_command_[0];
    cmd.steer_l = joint_position_command_[3];
    cmd.speed_r = joint_position_command_[7];
    cmd.speed_l = joint_position_command_[10];
    cmd.cam_yaw = joint_position_command_[12];
    cmd.cam_pit = joint_position_command_[13];

    motor_command_publisher_.publish(cmd);
  }

  void SmartHardwareInterface::sensorsCallback(const smart_msgs::RawSensors &sensor_msg) {

    actual_state.stamp = ros::Time::now();

    actual_state.position.right_wheel_joint         = correctPosition(sensor_msg.wheel_r,previous_state.position.right_wheel_joint,previous_state.overflow.right_wheel_joint);
    actual_state.position.left_wheel_joint          = correctPosition(sensor_msg.wheel_l,previous_state.position.left_wheel_joint,previous_state.overflow.left_wheel_joint);
    actual_state.angle.right_wheel_joint            = degToRad(posToDeg(actual_state.position.right_wheel_joint));
    actual_state.angle.left_wheel_joint             = degToRad(posToDeg(actual_state.position.left_wheel_joint));

    actual_state.angle.right_suspension_wheel_joint = degToRad(sensor_msg.steer_r);
    actual_state.angle.left_suspension_wheel_joint  = degToRad(sensor_msg.steer_l);
    actual_state.angle.camera_pan                   = degToRad(sensor_msg.cam_yaw);
    actual_state.angle.camera_tilt                  = degToRad(sensor_msg.cam_pit);

    smart_msgs::UltrasonicSensors us_sensors;
    us_sensors.header.stamp                         = actual_state.stamp;
    us_sensors.front_left.range                     = centimeterToMeter(sensor_msg.us_fl);
    us_sensors.front_right.range                    = centimeterToMeter(sensor_msg.us_fr);
    us_sensors.rear_left.range                      = centimeterToMeter(sensor_msg.us_rl);
    us_sensors.rear_right.range                     = centimeterToMeter(sensor_msg.us_rr);
    ultrasonic_publisher_.publish(us_sensors);

    // safe sensor readings
    previous_state = actual_state;
  }

  double SmartHardwareInterface::getUpdateLoopFrequency() const
  {
    return loop_hz_;
  }

  double SmartHardwareInterface::getUpdateLoopRate() const
  {
    return 1.0/loop_hz_;
  }
}

int main(int argc, char** argv)
{
  try{
    ROS_INFO("starting");
    ros::init(argc, argv, "smart_hardware_interface");

    ros::NodeHandle nh;
    smart_hardware_interface::SmartHardwareInterface smart_hardware(nh);

    controller_manager::ControllerManager cm(&smart_hardware);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(smart_hardware.getUpdateLoopRate());

    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
      loop_rate.sleep();

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - last_time;
      last_time = current_time;

      smart_hardware.read(current_time, elapsed_time);
      cm.update(current_time, elapsed_time);
      smart_hardware.write(current_time, elapsed_time);
    }

    smart_hardware.cleanup();
  }
  catch(...) {
    ROS_ERROR("Unhandled exception!");
    return -1;
  }

  return 0;
}

