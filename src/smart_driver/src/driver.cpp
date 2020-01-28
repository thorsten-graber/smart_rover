#include <smart_driver/driver.h>
#include <ros/ros.h>

#include "quaternions.h"

#define NO_OBSTICAL 2.55f

static double roundRadToDeg(double rad) {
    return round (rad * 180.0 / M_PI);
}

static double degToRad(double deg) {
    return (deg * M_PI / 180.0);
}

static int16_t posToDeg(long position) {
    return (int16_t) (position / 2l % 360l);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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

static double maxDeltaFilter(double y, double x, double c) {
    double delta = y-x;
    double test = x+(double)sgn(delta) * c;
    return (fabs(delta) > c) ? test : y;
}

Driver::Driver(const std::string& ns)
    : nh(ns)
{
    world_frame_id = "map";
    base_frame_id = "base_link";

    motor_control_parameters.min_velocity        = 0.01;
    motor_control_parameters.max_velocity        = 0.1;
    motor_control_parameters.min_object_distance = 0.2;

    motor_control_parameters.chassis_width       = 0.295;
    motor_control_parameters.chassis_length      = 0.304;

    motor_control_parameters.wheel_radius        = 0.035;

    motor_control_parameters.motor_states.header.frame_id = "base_link";
    motor_control_parameters.motor_states.name.push_back("front_right_wheel_steering_joint");
    motor_control_parameters.motor_states.name.push_back("middle_right_wheel_steering_joint");
    motor_control_parameters.motor_states.name.push_back("rear_right_wheel_steering_joint");
    motor_control_parameters.motor_states.name.push_back("front_left_wheel_steering_joint");
    motor_control_parameters.motor_states.name.push_back("middle_left_wheel_steering_joint");
    motor_control_parameters.motor_states.name.push_back("rear_left_wheel_steering_joint");
    motor_control_parameters.motor_states.name.push_back("front_right_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("middle_right_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("rear_right_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("front_left_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("middle_left_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("rear_left_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("camera/z");
    motor_control_parameters.motor_states.name.push_back("camera/y");
    motor_control_parameters.motor_states.effort.resize(14);
    motor_control_parameters.motor_states.position.resize(14);
    motor_control_parameters.motor_states.velocity.resize(14);

    ros::NodeHandle params("~");
    params.getParam("min_velocity", motor_control_parameters.min_velocity);
    params.getParam("max_velocity", motor_control_parameters.max_velocity);
    params.getParam("wold_frame_id", world_frame_id);
    params.getParam("base_frame_id", base_frame_id);

    motion_command_subscriber = nh.subscribe("drive", 10, &Driver::motionCommandCallback, this);
    camera_command_subscriber = nh.subscribe("camera/command", 10, &Driver::cameraCommandCallback, this);
    read_sensors_subscriber   = nh.subscribe("sensor_readings", 10, &Driver::readSensorsCallback,this);

    motor_command_publisher = nh.advertise<smart_msgs::MotorCommand>("motor_command", 1, true);
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states",10,true);
    odom_publisher = nh.advertise<nav_msgs::Odometry>("state", 10, true);

    timer = nh.createTimer(ros::Duration(0.02),&Driver::update,this);

    reset();
}

Driver::~Driver()
{
}

void Driver::reset()
{
    // reset motor control data
    motor_control_parameters.control_input.speed_r = 0;
    motor_control_parameters.control_input.steer_r = 0;
    motor_control_parameters.control_input.speed_l = 0;
    motor_control_parameters.control_input.steer_r = 0;
    motor_control_parameters.control_input.cam_yaw = 0;
    motor_control_parameters.control_input.cam_pit = 0;

    steer_old = 0;

    // reset sensor readings
    previous_readings.stamp = ros::Time::now();

    actual_readings.motor.position.left_wheel_joint             = 0;
    actual_readings.motor.position.right_wheel_joint            = 0;
    actual_readings.motor.position.left_steering_wheel_joint    = 0;
    actual_readings.motor.position.right_steering_wheel_joint   = 0;
    actual_readings.motor.position.camera_pan                   = 0;
    actual_readings.motor.position.camera_tilt                  = 0;

    actual_readings.motor.overflow.left_wheel_joint             = 0;
    actual_readings.motor.overflow.right_wheel_joint            = 0;
    actual_readings.motor.overflow.left_steering_wheel_joint    = 0;
    actual_readings.motor.overflow.right_steering_wheel_joint   = 0;
    actual_readings.motor.overflow.camera_pan                   = 0;
    actual_readings.motor.overflow.camera_tilt                  = 0;

    actual_readings.ultrasonic.range_fl = NO_OBSTICAL;
    actual_readings.ultrasonic.range_fr = NO_OBSTICAL;
    actual_readings.ultrasonic.range_rl = NO_OBSTICAL;
    actual_readings.ultrasonic.range_rr = NO_OBSTICAL;

    previous_readings.motor.position.left_wheel_joint           = 0;
    previous_readings.motor.position.right_wheel_joint          = 0;
    previous_readings.motor.position.left_steering_wheel_joint  = 0;
    previous_readings.motor.position.right_steering_wheel_joint = 0;
    previous_readings.motor.position.camera_pan                 = 0;
    previous_readings.motor.position.camera_tilt                = 0;

    previous_readings.motor.overflow.left_wheel_joint           = 0;
    previous_readings.motor.overflow.right_wheel_joint          = 0;
    previous_readings.motor.overflow.left_steering_wheel_joint  = 0;
    previous_readings.motor.overflow.right_steering_wheel_joint = 0;
    previous_readings.motor.overflow.camera_pan                 = 0;
    previous_readings.motor.overflow.camera_tilt                = 0;

    previous_readings.ultrasonic.range_fl = NO_OBSTICAL;
    previous_readings.ultrasonic.range_fr = NO_OBSTICAL;
    previous_readings.ultrasonic.range_rl = NO_OBSTICAL;
    previous_readings.ultrasonic.range_rr = NO_OBSTICAL;

    // reset odometry pose and tf
    odom.pose.pose.position.x    = 0;
    odom.pose.pose.position.y    = 0;
    odom.pose.pose.position.z    = 0;
    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    mode_old = "";
}

void Driver::stop()
{
    motor_control_parameters.control_input.speed_r = 0;
    motor_control_parameters.control_input.steer_r = 0;
    motor_control_parameters.control_input.speed_l = 0;
    motor_control_parameters.control_input.steer_r = 0;
}

void Driver::update(const ros::TimerEvent&)
{
    // process motion command message
    double steer_l, steer_r, speed_r, speed_l;

    double r = motor_control_parameters.wheel_radius;

    double cmd_vel = limitVelocity(motion_cmd.speed);

    // update desired motor velocity/position based on control mode
    if(!strcmp(motion_cmd.mode.c_str(), "continuous")) {
        // limit max dynamic in steering (max delta per step)
        double steer = maxDeltaFilter(motion_cmd.steer, steer_old, 0.0025); // TODO: make this parameter configurable
        steer_old = steer;

        ComputeLocomotion(cmd_vel, steer, speed_l, speed_r, steer_l, steer_r);

    } else if (!motion_cmd.mode.compare("point_turn")) {
        steer_l = -M_PI_4;
        steer_r =  M_PI_4;
        speed_l = -cmd_vel;
        speed_r =  cmd_vel;
        steer_old = 0;
    } else {
        ROS_WARN_ONCE("No command received, or unknown command mode set - stopping rover!");
        steer_l = 0;
        steer_r = 0;
        speed_l = 0;
        speed_r = 0;
    }

    // only drive, if wheel joint angle error is almost zero
    double threshold = 3*M_PI/180; // TODO: make this parameter configurable
    double phi_l = actual_readings.motor.angle.left_steering_wheel_joint;
    double phi_r = actual_readings.motor.angle.right_steering_wheel_joint;

    if ((fabs(steer_l  - phi_l) > threshold || fabs(steer_r  - phi_r) > threshold) && (!motion_cmd.mode.compare("point_turn") || !mode_old.compare("point_turn")))
    {
        mode_old = "point_turn";

        ROS_DEBUG("error: l: [%f] r: [%f]", fabs(steer_r  - phi_r), fabs(steer_l  + phi_l));
        speed_l = 0.0;
        speed_r = 0.0;
    } else {
        mode_old = motion_cmd.mode.c_str();
    }

    motor_control_parameters.control_input.steer_r = roundRadToDeg(steer_r);
    motor_control_parameters.control_input.steer_l = roundRadToDeg(steer_l);

    //omega = speed(wheel speed) * 720(ticks per turn) * 0.02ms(controller frequency) * 3/2(gear factor) / r(wheel radius)
    motor_control_parameters.control_input.speed_r = int16_t (speed_r * 720 * 0.02 * 3 / 2 / r); // todo: make parameters configurable
    motor_control_parameters.control_input.speed_l = int16_t (speed_l * 720 * 0.02 * 3 / 2 / r); // todo: make parameters configurable


    // process camera command message
    double angles[3];
    quaternion2angles(camera_cmd.quaternion, angles);

    motor_control_parameters.control_input.cam_yaw = roundRadToDeg(angles[0]);
    motor_control_parameters.control_input.cam_pit = roundRadToDeg(angles[1]);

    motor_command_publisher.publish(motor_control_parameters.control_input);
}

void Driver::motionCommandCallback(const smart_msgs::MotionCommand::ConstPtr& motion_cmd_msg)
{
    boost::mutex::scoped_lock lock(mutex);
    motion_cmd = *motion_cmd_msg;
}

void Driver::cameraCommandCallback(const geometry_msgs::QuaternionStamped::ConstPtr& camera_cmd_msg)
{
    boost::mutex::scoped_lock lock(mutex);
    camera_cmd = *camera_cmd_msg;
}

void Driver::readSensorsCallback(const smart_msgs::RawSensors &sensor_msg) {

    actual_readings.stamp = ros::Time::now();

    actual_readings.motor.position.right_wheel_joint         = correctPosition(sensor_msg.wheel_r,previous_readings.motor.position.right_wheel_joint,previous_readings.motor.overflow.right_wheel_joint);
    actual_readings.motor.position.left_wheel_joint          = correctPosition(sensor_msg.wheel_l,previous_readings.motor.position.left_wheel_joint,previous_readings.motor.overflow.left_wheel_joint);
    actual_readings.motor.angle.right_wheel_joint            = degToRad(posToDeg(actual_readings.motor.position.right_wheel_joint));
    actual_readings.motor.angle.left_wheel_joint             = degToRad(posToDeg(actual_readings.motor.position.left_wheel_joint));

    actual_readings.motor.angle.right_steering_wheel_joint   = degToRad(sensor_msg.steer_r);
    actual_readings.motor.angle.left_steering_wheel_joint    = degToRad(sensor_msg.steer_l);
    actual_readings.motor.angle.camera_pan                   = degToRad(sensor_msg.cam_yaw);
    actual_readings.motor.angle.camera_tilt                  = degToRad(sensor_msg.cam_pit);

    actual_readings.ultrasonic.range_fl                      = centimeterToMeter(sensor_msg.us_fl);
    actual_readings.ultrasonic.range_fr                      = centimeterToMeter(sensor_msg.us_fr);
    actual_readings.ultrasonic.range_rl                      = centimeterToMeter(sensor_msg.us_rl);
    actual_readings.ultrasonic.range_rr                      = centimeterToMeter(sensor_msg.us_rr);

    // publish joint states
    publishJointStates();

    // publish odometry
    publishOdometry();

    // safe sensor readings
    previous_readings = actual_readings;
}

void Driver::publishOdometry()
{
    double l, r;
    double alpha_l, alpha_r;
    double x, y, yaw;
    double phi_l, phi_r;
    double s;

    // vehicle geometry
    l = motor_control_parameters.chassis_length;
    r = motor_control_parameters.wheel_radius;

    alpha_r = degToRad(posToDeg(actual_readings.motor.position.right_wheel_joint - previous_readings.motor.position.right_wheel_joint));
    alpha_l = degToRad(posToDeg(actual_readings.motor.position.left_wheel_joint - previous_readings.motor.position.left_wheel_joint));
    phi_r   = actual_readings.motor.angle.right_steering_wheel_joint;
    phi_l   = actual_readings.motor.angle.left_steering_wheel_joint;

    // Compute angular velocity for ICC which is same as angular velocity of vehicle
    yaw = (alpha_l * sin(phi_l) + alpha_r * sin(phi_r))* r / l;

    // Compute translation using previous yaw angle using last pose
    double euler[3];
    quaternion2euler(odom.pose.pose.orientation, euler);
    s = r * 2 * (alpha_l + alpha_r)/2/3;  // includes gear factor (2/3) // todo: make this parameter configurable
    x = s * cos(euler[2]);
    y = s * sin(euler[2]);

    // Compute delta t
    double dt = actual_readings.stamp.toSec() - previous_readings.stamp.toSec();

    // Publish Odom
    // set header
    odom.header.stamp    = actual_readings.stamp;
    odom.header.frame_id = world_frame_id.c_str();

    //Compute velocity
    odom.child_frame_id        = base_frame_id.c_str();
    odom.twist.twist.linear.x  = x / dt;
    odom.twist.twist.linear.y  = y / dt;
    odom.twist.twist.angular.z = yaw / dt;

    // Compute odometric pose
    geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(yaw);
    odom.pose.pose.position.x  += x;
    odom.pose.pose.position.y  += y;
    odom.pose.pose.position.z  = 0.1;                // todo: magic number
    odom.pose.pose.orientation = odom.pose.pose.orientation*yaw_quat;

    //publish message
    odom_publisher.publish(odom);

    // Publish tf
    odom_trans.header.stamp    = actual_readings.stamp;
    odom_trans.header.frame_id = world_frame_id.c_str();
    odom_trans.child_frame_id  = base_frame_id.c_str();

    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    odom_trans.transform.rotation      = odom.pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);
}

void Driver::publishJointStates()
{
    // publish motor joints
    motor_control_parameters.motor_states.position[0]  = actual_readings.motor.angle.right_steering_wheel_joint;
    motor_control_parameters.motor_states.position[1]  = 0;
    motor_control_parameters.motor_states.position[2]  = -actual_readings.motor.angle.right_steering_wheel_joint;

    motor_control_parameters.motor_states.position[3]  = actual_readings.motor.angle.left_steering_wheel_joint;
    motor_control_parameters.motor_states.position[4]  = 0;
    motor_control_parameters.motor_states.position[5]  = -actual_readings.motor.angle.left_steering_wheel_joint;

    motor_control_parameters.motor_states.position[6]  = actual_readings.motor.angle.right_wheel_joint / cos(actual_readings.motor.angle.right_steering_wheel_joint);
    motor_control_parameters.motor_states.position[7]  = actual_readings.motor.angle.right_wheel_joint;
    motor_control_parameters.motor_states.position[8]  = actual_readings.motor.angle.right_wheel_joint / cos(actual_readings.motor.angle.right_steering_wheel_joint);

    motor_control_parameters.motor_states.position[9]  = actual_readings.motor.angle.left_wheel_joint / cos(actual_readings.motor.angle.left_steering_wheel_joint);
    motor_control_parameters.motor_states.position[10] = actual_readings.motor.angle.left_wheel_joint;
    motor_control_parameters.motor_states.position[11] = actual_readings.motor.angle.left_wheel_joint / cos(actual_readings.motor.angle.left_steering_wheel_joint);

    motor_control_parameters.motor_states.position[12] = actual_readings.motor.angle.camera_pan;
    motor_control_parameters.motor_states.position[13] = actual_readings.motor.angle.camera_tilt;

    motor_control_parameters.motor_states.header.stamp = actual_readings.stamp;

    joint_state_publisher.publish(motor_control_parameters.motor_states);
}

void Driver::ComputeLocomotion(double speed, double steer, double &speed_l, double &speed_r, double &steer_l, double &steer_r)
{
    double l, b;
    double tan_steer;

    b = motor_control_parameters.chassis_width;
    l = motor_control_parameters.chassis_length;

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
        if(speed_r > motor_control_parameters.max_velocity) {
            speed_r = motor_control_parameters.max_velocity;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        } else if(speed_r < -motor_control_parameters.max_velocity) {
            speed_r = -motor_control_parameters.max_velocity;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        }
    } else {
        // turn right -> check left wheel speed
        if(speed_l > motor_control_parameters.max_velocity) {
            speed_l = motor_control_parameters.max_velocity;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        } else if(speed_l < -motor_control_parameters.max_velocity) {
            speed_l = -motor_control_parameters.max_velocity;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        }
    }
}

double Driver::limitVelocity(double value) { //todo check if cases
    double velocity = value;
    if (value > 0.0) {
        if (value > motor_control_parameters.max_velocity) velocity = motor_control_parameters.max_velocity;
        if (value < motor_control_parameters.min_velocity) velocity = motor_control_parameters.min_velocity;
        if (previous_readings.ultrasonic.range_fl < motor_control_parameters.min_object_distance || previous_readings.ultrasonic.range_fr < motor_control_parameters.min_object_distance) {
            velocity = 0.0f;
            ROS_WARN("OBSTACLE in the FRONT, STOPPING MOTORS distance to obstacle: %f", motor_control_parameters.min_object_distance);
        }
    } else if (value < 0.0) {
        if (value < -motor_control_parameters.max_velocity) velocity = -motor_control_parameters.max_velocity;
        if (value > -motor_control_parameters.min_velocity) velocity = -motor_control_parameters.min_velocity;
        if (previous_readings.ultrasonic.range_rl < motor_control_parameters.min_object_distance || previous_readings.ultrasonic.range_rr < motor_control_parameters.min_object_distance) {
            velocity = 0.0f;
            ROS_WARN("OBSTACLE in the BACK, STOPPING MOTORS distance to obstacle: %f", motor_control_parameters.min_object_distance);
        }
    }
    return velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    Driver d;
    while(ros::ok())
    {
        ros::spin();
    }

    ros::shutdown();
    return 0;
}
