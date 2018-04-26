// Include ros libraries
#include <ros.h>
#include <mocup_msgs/MotorCommand.h>
#include <mocup_msgs/RawSensors.h>

// Include the Bricktronics libraries
#include <BricktronicsMegashield.h>
#include <BricktronicsMotor.h>
#include <BricktronicsUltrasonic.h>

#define BAUD 57600 // 57kbps
#define UPDATE_INTERVAL  20  // 1/update rate in ms -> 20ms = 50Hz

// Select the desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
BricktronicsMotor m1(BricktronicsMegashield::MOTOR_1);
BricktronicsMotor m2(BricktronicsMegashield::MOTOR_2);
BricktronicsMotor m3(BricktronicsMegashield::MOTOR_3);
BricktronicsMotor m4(BricktronicsMegashield::MOTOR_4);
BricktronicsMotor m5(BricktronicsMegashield::MOTOR_5);
BricktronicsMotor m6(BricktronicsMegashield::MOTOR_6);

// Select the sensor port for the sensor (SENSOR_1 through SENSOR_4) below.
// Use the jumpers to connect pins 1-2 and 4-5 for the ultrasonic sensor.
BricktronicsUltrasonic u1(BricktronicsMegashield::SENSOR_1);
BricktronicsUltrasonic u2(BricktronicsMegashield::SENSOR_2);
BricktronicsUltrasonic u3(BricktronicsMegashield::SENSOR_3);
BricktronicsUltrasonic u4(BricktronicsMegashield::SENSOR_4);

// ROS stuff
ros::NodeHandle  nh;
mocup_msgs::RawSensors sensor_readings;

void motorCommandCallback(const mocup_msgs::MotorCommand& cmd_msg)
{   
    m1.setSpeed(-cmd_msg.speed_r);
    m2.goToAngle(cmd_msg.steer_r);
    m3.setSpeed(-cmd_msg.speed_l);
    m4.goToAngle(cmd_msg.steer_l);
    m5.goToAngle(-cmd_msg.cam_yaw);
    m6.goToAngle(-cmd_msg.cam_pit);

    m1.update();
    m2.update();
    m3.update();
    m4.update();
    m5.update();
    m6.update();
}

ros::Publisher sensor_publisher("sensor_readings", &sensor_readings);
ros::Subscriber<mocup_msgs::MotorCommand> motor_command_subscriber("motor_command", motorCommandCallback);

void publishSensorReadings()
{
    // motor encoders
    sensor_readings.wheel_r = -m1.getPosition();
    sensor_readings.steer_r = m2.getAngle();
    sensor_readings.wheel_l = -m3.getPosition();
    sensor_readings.steer_l = m4.getAngle();
    sensor_readings.cam_yaw = -m5.getAngle();
    sensor_readings.cam_pit = -m6.getAngle();

    sensor_publisher.publish(&sensor_readings);
}

void setup()
{
    // Initialize ROS node
    nh.getHardware()->setBaud(BAUD);
    nh.initNode();
    nh.advertise(sensor_publisher);
    nh.subscribe(motor_command_subscriber);

    // Initialize the motor connections
    m1.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
    m1.pidSetTunings(1.1,64.0,0.0);
    m1.begin();
    m2.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
    m2.pidSetTunings(0.6,0.0,0.0);
    m2.setAngleOutputMultiplier(222);
    m2.begin();
    m3.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
    m3.pidSetTunings(1.0,64.0,0.0);
    m3.begin();
    m4.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
    m4.pidSetTunings(0.6,0.0,0.00);
    m4.setAngleOutputMultiplier(222);
    m4.begin();
    m5.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
    m5.pidSetTunings(5.64,0.0,0.12);
    m5.setAngleOutputMultiplier(1);
    m5.begin();
    m6.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
    m6.pidSetTunings(6.64,0.0,0.12);
    m6.setAngleOutputMultiplier(1);
    m6.begin();

    // Initialize the ultrasonic sensor connections
    u1.begin();
    u2.begin();
    u3.begin();
    u4.begin();
}

void loop() 
{
    nh.spinOnce();
    sensor_readings.us_fr = u1.getDistance();
    publishSensorReadings();
    nh.spinOnce();    
    sensor_readings.us_fl = u2.getDistance();
    publishSensorReadings();
    nh.spinOnce();
    sensor_readings.us_rr = u3.getDistance();
    publishSensorReadings();
    nh.spinOnce();
    sensor_readings.us_rl = u4.getDistance();
    publishSensorReadings();
}
