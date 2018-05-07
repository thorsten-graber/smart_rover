#ifndef SMART_DRIVER_H
#define SMART_DRIVER_H

#include <ros/ros.h>
#include <smart_msgs/MotionCommand.h>
#include <smart_msgs/MotorCommand.h>
#include <smart_msgs/RawSensors.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Driver {
public:
    typedef struct {
        double min_velocity;
        double max_velocity;
        double min_object_distance;
        double chassis_width;
        double chassis_length;
        double wheel_radius;
        double wheel_gear;
        double stearing_gear;
        double camera_pan_gear;
        double camera_tilt_gear;
        smart_msgs::MotorCommand control_input;
        sensor_msgs::JointState motor_states;
    } MotorControlParameters;

    typedef struct {
        double range_fl;
        double range_fr;
        double range_rl;
        double range_rr;
    } UltraSonic;

    typedef struct {
        double right_wheel_joint;
        double left_wheel_joint;
        double right_steering_wheel_joint;
        double left_steering_wheel_joint;
        double camera_pan;
        double camera_tilt;
    } Angle;

    typedef struct {
        long right_wheel_joint;
        long left_wheel_joint;
        long right_steering_wheel_joint;
        long left_steering_wheel_joint;
        long camera_pan;
        long camera_tilt;
    } Encoder;

    typedef struct {
        Encoder overflow;
        Encoder position;
        Angle angle;
    } Motor;

    typedef struct {
        ros::Time stamp;
        UltraSonic ultrasonic;
        Motor motor;
    } Sensors;

    Driver(const std::string &ns = std::string());
    ~Driver();

    int main(int, char**);
    void update(const ros::TimerEvent&);

    void reset();
    void stop();

    void ComputeLocomotion(double speed, double steer, double& speed_l, double& speed_r, double& steer_l, double& steer_r);
    double limitVelocity(double speed);

    void motionCommandCallback(const smart_msgs::MotionCommand::ConstPtr& motion_cmd_msg);
    void cameraCommandCallback(const geometry_msgs::QuaternionStamped::ConstPtr& camera_cmd_msg);
    void readSensorsCallback(const smart_msgs::RawSensors &sensor_msg);

    void publishOdometry();
    void publishJointStates();

    ros::NodeHandle nh;
    ros::Timer timer;

    smart_msgs::MotionCommand motion_cmd;
    geometry_msgs::QuaternionStamped camera_cmd;
    MotorControlParameters motor_control_parameters;
    Sensors actual_readings;
    Sensors previous_readings;

    nav_msgs::Odometry odom;

    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher motor_command_publisher;
    ros::Publisher joint_state_publisher;
    ros::Publisher odom_publisher;

    ros::Subscriber motion_command_subscriber;
    ros::Subscriber camera_command_subscriber;
    ros::Subscriber read_sensors_subscriber;

    boost::mutex mutex;
    double steer_old;
    std::string mode_old;
};

#endif // SMART_DRIVER_H
