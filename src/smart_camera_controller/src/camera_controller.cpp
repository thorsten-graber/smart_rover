#include <ros/ros.h>
#include <smart_msgs/MotionCommand.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace smart {

  ros::Subscriber cameraCommandInput;
  ros::Subscriber cameraCommandPanInput;
  ros::Subscriber cameraCommandTiltInput;
  ros::Publisher cameraCommandOutput;

  smart_msgs::MotionCommand motionCommand;
  geometry_msgs::QuaternionStamped cameraCommand;


  int button_cameraReset;
  std::string cameraMode;
  double cameraSpeed;
  double cameraMaxPan;
  double cameraMaxTiltDown;
  double cameraMaxTiltUp;

  double cameraPan = 0;
  double cameraTilt = 0;
  double cameraPanSpeed = 0;
  double cameraTiltSpeed = 0;

  void publishCamera();

  void cameraCommandCallback(const geometry_msgs::Twist::ConstPtr &cmd_camera_vel) {

      cameraPanSpeed = cmd_camera_vel->angular.z * cameraSpeed * M_PI/180.0;
      cameraTiltSpeed = -1*cmd_camera_vel->linear.x * cameraSpeed * M_PI/180.0;

      publishCamera();

  }

  void cameraPanCommandCallback(const std_msgs::Float64::ConstPtr &pan) {
    cameraPan = pan->data;

    publishCamera();
  }

  void cameraTiltCommandCallback(const std_msgs::Float64::ConstPtr &tilt) {
    cameraTilt = tilt->data;

    publishCamera();
  }

  void moveCamera(double dt) {
    if (cameraPanSpeed == 0.0 && cameraTiltSpeed == 0.0) return;

    cameraPan += dt * cameraPanSpeed;
    if (cameraPan >  cameraMaxPan*M_PI/180.0) cameraPan =  cameraMaxPan*M_PI/180.0;
    if (cameraPan < -cameraMaxPan*M_PI/180.0) cameraPan = -cameraMaxPan*M_PI/180.0;

    cameraTilt += dt * cameraTiltSpeed;
    if (cameraTilt >  cameraMaxTiltDown*M_PI/180.0) cameraTilt =  cameraMaxTiltDown*M_PI/180.0;
    if (cameraTilt < -cameraMaxTiltUp  *M_PI/180.0) cameraTilt = -cameraMaxTiltUp  *M_PI/180.0;

    publishCamera();
  }

  void publishCamera() {
    cameraCommand.header.stamp = ros::Time::now();
    cameraCommand.header.frame_id = cameraMode;
    cameraCommand.quaternion.w =  cos(cameraPan/2)*cos(cameraTilt/2);
    cameraCommand.quaternion.x = -sin(cameraPan/2)*sin(cameraTilt/2);
    cameraCommand.quaternion.y =  cos(cameraPan/2)*sin(cameraTilt/2);
    cameraCommand.quaternion.z =  sin(cameraPan/2)*cos(cameraTilt/2);

    cameraCommandOutput.publish(cameraCommand);
  }
} // namespace smart

using namespace smart;

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle n;

  cameraCommandInput = n.subscribe<geometry_msgs::Twist>("cmd_camera_vel", 10, cameraCommandCallback);
  cameraCommandPanInput = n.subscribe<std_msgs::Float64>("cmd_camera_pan", 10, cameraPanCommandCallback);
  cameraCommandTiltInput = n.subscribe<std_msgs::Float64>("cmd_camera_tilt", 10, cameraTiltCommandCallback);
  cameraCommandOutput = n.advertise<geometry_msgs::QuaternionStamped>("camera/command", 10, false);

  ros::param::param("~camera_mode", smart::cameraMode, std::string("base_stabilized"));
  ros::param::param("~camera_speed", smart::cameraSpeed, 60.0);
  ros::param::param("~camera_max_pan", smart::cameraMaxPan, 120.0);
  ros::param::param("~camera_max_tilt_down", smart::cameraMaxTiltDown, 30.0);
  ros::param::param("~camera_max_tilt_up", smart::cameraMaxTiltUp, 30.0);

  ros::Rate rate(50.0);
  while(ros::ok()) {
    moveCamera(rate.expectedCycleTime().toSec());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
