/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */

#include <algorithm>
#include <assert.h>
#include <cmath>

#include <smart_gazebo_plugins/all_wheel_steering_plugin.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#if (GAZEBO_MAJOR_VERSION > 1) || (GAZEBO_MINOR_VERSION >= 2)
#define RADIAN Radian
#else
#define RADIAN GetAsRadian
#endif

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin)

enum
{
    FRONT_RIGHT = 0, FRONT_LEFT = 1, MIDDLE_RIGHT = 2, MIDDLE_LEFT = 3, REAR_RIGHT = 4, REAR_LEFT = 5
};

// Constructor
AllWheelSteeringPlugin::AllWheelSteeringPlugin()
{
    rosnode_ = 0;
}

// Destructor
AllWheelSteeringPlugin::~AllWheelSteeringPlugin()
{
    sub_.shutdown();
    delete rosnode_;
}

// Load the controller
void AllWheelSteeringPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get physics
    model = _model;
    world = _model->GetWorld();

    // default parameters
    topicName = "drive";
    odomTopicName = "odom";
    jointStateName = "joint_states";
    proportionalControllerGain = 8.0;
    derivativeControllerGain = 0.0;
    wheelTrack = 0.295;
    wheelBase = 0.304;
    wheelRadius = 0.035;
    jointMaxTorque = 10.0;
    wheelMaxTorque = 10.0;
    maxVelX = 0.1;
    jointMaxVelocity = 0.5;

    // load parameters
    if (_sdf->HasElement("robotNamespace")) robotNamespace = _sdf->Get<std::string>("robotNamespace");
    if (_sdf->HasElement("topicName")) topicName = _sdf->Get<std::string>("topicName");
    if (_sdf->HasElement("odomTopicName")) odomTopicName = _sdf->Get<std::string>("odomTopicName");
    if (_sdf->HasElement("jointStateName")) jointStateName = _sdf->Get<std::string>("jointStateName");
    if (_sdf->HasElement("frontLeftAxle")) wheels[FRONT_LEFT].axleName = _sdf->Get<std::string>("frontLeftAxle");
    if (_sdf->HasElement("frontRightAxle")) wheels[FRONT_RIGHT].axleName = _sdf->Get<std::string>("frontRightAxle");
    if (_sdf->HasElement("middleLeftAxle")) wheels[MIDDLE_LEFT].axleName = _sdf->Get<std::string>("middleLeftAxle");
    if (_sdf->HasElement("middleRightAxle")) wheels[MIDDLE_RIGHT].axleName = _sdf->Get<std::string>("middleRightAxle");
    if (_sdf->HasElement("rearLeftAxle")) wheels[REAR_LEFT].axleName = _sdf->Get<std::string>("rearLeftAxle");
    if (_sdf->HasElement("rearRightAxle")) wheels[REAR_RIGHT].axleName = _sdf->Get<std::string>("rearRightAxle");
    if (_sdf->HasElement("frontLeftJoint")) wheels[FRONT_LEFT].jointName = _sdf->Get<std::string>("frontLeftJoint");
    if (_sdf->HasElement("frontRightJoint")) wheels[FRONT_RIGHT].jointName = _sdf->Get<std::string>("frontRightJoint");
    if (_sdf->HasElement("middleLeftJoint")) wheels[MIDDLE_LEFT].jointName = _sdf->Get<std::string>("middleLeftJoint");
    if (_sdf->HasElement("middleRightJoint")) wheels[MIDDLE_RIGHT].jointName = _sdf->Get<std::string>("middleRightJoint");
    if (_sdf->HasElement("rearLeftJoint")) wheels[REAR_LEFT].jointName = _sdf->Get<std::string>("rearLeftJoint");
    if (_sdf->HasElement("rearRightJoint")) wheels[REAR_RIGHT].jointName = _sdf->Get<std::string>("rearRightJoint");
    if (_sdf->HasElement("proportionalControllerGain")) proportionalControllerGain = _sdf->Get<double>("proportionalControllerGain");
    if (_sdf->HasElement("derivativeControllerGain")) derivativeControllerGain = _sdf->Get<double>("derivativeControllerGain");
    if (_sdf->HasElement("wheelTrack")) wheelTrack = _sdf->Get<double>("wheelTrack");
    if (_sdf->HasElement("wheelTrack")) wheelBase = _sdf->Get<double>("wheelBase");
    if (_sdf->HasElement("wheelRadius")) wheelRadius = _sdf->Get<double>("wheelRadius");
    if (_sdf->HasElement("jointMaxTorque")) jointMaxTorque = _sdf->Get<double>("jointMaxTorque");
    if (_sdf->HasElement("wheelMaxTorque")) wheelMaxTorque = _sdf->Get<double>("wheelMaxTorque");
    if (_sdf->HasElement("maxVelX")) maxVelX = _sdf->Get<double>("maxVelX");
    if (_sdf->HasElement("jointMaxVelocity")) jointMaxVelocity = _sdf->Get<double>("jointMaxVelocity");

    double controlRate = 0.0;
    if (_sdf->HasElement("controlRate")) controlRate = _sdf->Get<double>("controlRate");
    controlPeriod = controlRate > 0.0 ? 1.0/controlRate : 0.0;

    for(int i = 0; i < 6; ++i) {
        wheels[i].axle  = _model->GetJoint(wheels[i].axleName);
        wheels[i].joint = _model->GetJoint(wheels[i].jointName);
    }

    if (!wheels[FRONT_LEFT].axle)
        gzthrow("The controller couldn't get front left axle");

    if (!wheels[FRONT_RIGHT].axle)
        gzthrow("The controller couldn't get front right axle");

    if (!wheels[MIDDLE_LEFT].axle)
        gzthrow("The controller couldn't get middle left axle");

    if (!wheels[MIDDLE_RIGHT].axle)
        gzthrow("The controller couldn't get middle right axle");

    if (!wheels[REAR_LEFT].axle)
        gzthrow("The controller couldn't get rear left axle");

    if (!wheels[REAR_RIGHT].axle)
        gzthrow("The controller couldn't get rear right axle");

    if (!wheels[FRONT_LEFT].joint)
        gzthrow("The controller couldn't get front left hinge joint");

    if (!wheels[FRONT_RIGHT].joint)
        gzthrow("The controller couldn't get front right hinge joint");

    if (!wheels[MIDDLE_LEFT].joint)
        gzthrow("The controller couldn't get middle left hinge joint");

    if (!wheels[MIDDLE_RIGHT].joint)
        gzthrow("The controller couldn't get middle right hinge joint");

    if (!wheels[REAR_LEFT].joint)
        gzthrow("The controller couldn't get rear left hinge joint");

    if (!wheels[REAR_RIGHT].joint)
        gzthrow("The controller couldn't get rear right hinge joint");

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    rosnode_ = new ros::NodeHandle(robotNamespace);

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    if (!topicName.empty()) {
        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<smart_msgs::MotionCommand>(topicName, 1,
                                                                         boost::bind(&AllWheelSteeringPlugin::motionCommandCallback, this, _1),
                                                                         ros::VoidPtr(), &queue_);
        sub_ = rosnode_->subscribe(so);
    }

    if (!odomTopicName.empty()) {
        odomPub_ = rosnode_->advertise<nav_msgs::Odometry>(odomTopicName, 10);
    }

    if (!jointStateName.empty()) {
        jointStatePub_ = rosnode_->advertise<sensor_msgs::JointState>(jointStateName, 10);
    }

    std::string tf_prefix = tf::getPrefixParam(*rosnode_);
    joint_state.header.frame_id = tf::resolve(tf_prefix, model->GetLink()->GetName());
    odom_.header.frame_id = tf::resolve(tf_prefix, "odom");
    odom_.child_frame_id = tf::resolve(tf_prefix, "base_footprint");

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&AllWheelSteeringPlugin::Update, this));
}

// Initialize the controller
void AllWheelSteeringPlugin::Init()
{
    Reset();
}

// Reset
void AllWheelSteeringPlugin::Reset()
{
    // Reset odometric pose
    odomPose[0] = 0.0;
    odomPose[1] = 0.0;
    odomPose[2] = 0.0;

    odomVel[0] = 0.0;
    odomVel[1] = 0.0;
    odomVel[2] = 0.0;

    enableMotors = false;

    wheels[FRONT_RIGHT].jointSpeed = 0;
    wheels[FRONT_LEFT].jointSpeed = 0;
    wheels[MIDDLE_RIGHT].jointSpeed = 0;
    wheels[MIDDLE_LEFT].jointSpeed = 0;
    wheels[REAR_RIGHT].jointSpeed = 0;
    wheels[REAR_LEFT].jointSpeed = 0;

    wheels[FRONT_RIGHT].wheelSpeed = 0;
    wheels[FRONT_LEFT].wheelSpeed = 0;
    wheels[MIDDLE_RIGHT].wheelSpeed = 0;
    wheels[MIDDLE_LEFT].wheelSpeed = 0;
    wheels[REAR_RIGHT].wheelSpeed = 0;
    wheels[REAR_LEFT].wheelSpeed = 0;

    prevUpdateTime = world->SimTime();

    steer_old = 0;
    mode_old = "";
}

// Update the controller
void AllWheelSteeringPlugin::Update()
{
    // TODO: Step should be in a parameter of this function

    double l, r, v;
    double steer_l, steer_r, speed_l, speed_r;
    double omega_fl, omega_fr, omega_ml, omega_mr, omega_rl, omega_rr, omega_phi;
    double phi_fl, phi_fr, phi_ml, phi_mr, phi_rl, phi_rr;
    double vel_phi_fl, vel_phi_fr, vel_phi_ml, vel_phi_mr, vel_phi_rl, vel_phi_rr;

    l = wheelBase;
    r = wheelRadius;

    // handle callbacks
    queue_.callAvailable();

    common::Time stepTime;
    //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
    stepTime = world->SimTime() - prevUpdateTime;

    if (controlPeriod == 0.0 || stepTime > controlPeriod) {
        prevUpdateTime = world->SimTime();

        // get current wheel velocities and wheel joint angles and wheel joint velocities.
        {
            boost::mutex::scoped_lock lock(mutex);

            omega_fl = wheels[FRONT_LEFT].axle->GetVelocity(0);
            omega_fr = wheels[FRONT_RIGHT].axle->GetVelocity(0);
            omega_ml = wheels[MIDDLE_LEFT].axle->GetVelocity(0);
            omega_mr = wheels[MIDDLE_RIGHT].axle->GetVelocity(0);
            omega_rl = wheels[REAR_LEFT].axle->GetVelocity(0);
            omega_rr = wheels[REAR_RIGHT].axle->GetVelocity(0);

            phi_fl = wheels[FRONT_LEFT].joint->Position(0);
            phi_fr = wheels[FRONT_RIGHT].joint->Position(0);
            phi_ml = wheels[MIDDLE_LEFT].joint->Position(0);
            phi_mr = wheels[MIDDLE_RIGHT].joint->Position(0);
            phi_rl = wheels[REAR_LEFT].joint->Position(0);
            phi_rr = wheels[REAR_RIGHT].joint->Position(0);

            vel_phi_fl = wheels[FRONT_LEFT].joint->GetVelocity(0);
            vel_phi_fr = wheels[FRONT_RIGHT].joint->GetVelocity(0);
            vel_phi_ml = wheels[MIDDLE_LEFT].joint->GetVelocity(0);
            vel_phi_mr = wheels[MIDDLE_RIGHT].joint->GetVelocity(0);
            vel_phi_rl = wheels[REAR_LEFT].joint->GetVelocity(0);
            vel_phi_rr = wheels[REAR_RIGHT].joint->GetVelocity(0);
        }

            if (!cmd_.mode.compare("continuous")) {
                // limit max dynamic in steering (max delta per step)
                double steer = maxDeltaFilter(cmd_.steer, steer_old, 0.0025); // TODO: make this parameter configurable
                steer_old = steer;

                ComputeLocomotion(cmd_.speed, steer, speed_l, speed_r, steer_l, steer_r);

            } else if (!cmd_.mode.compare("point_turn")) {
             steer_l = -M_PI_4;
             steer_r = M_PI_4;
             speed_l = -cmd_.speed;
             speed_r = cmd_.speed;
             steer_old = 0;
            } else {
                ROS_WARN_ONCE("No command received, or unknown command mode set - stopping rover!");
                steer_l = 0;
                steer_r = 0;
                speed_l = 0;
                speed_r = 0;
            }

        // only drive, if wheel joint angle error is almost zero
        double threshold = M_PI/180; // TODO: make this parameter configurable
        if ((fabs(steer_l  - phi_fl) > threshold || fabs(steer_r  - phi_fr) > threshold) && (!cmd_.mode.compare("point_turn") || !mode_old.compare("point_turn")))
        {
            mode_old = "point_turn";

            ROS_DEBUG("error: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", fabs(steer_l  - phi_fl), fabs(steer_r  - phi_fr), fabs(steer_l  + phi_rl), fabs(steer_r  + phi_rr));
            speed_l = 0.0;
            speed_r = 0.0;
        } else {
            mode_old = cmd_.mode.c_str();
        }

        ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel commands:\n"
                               << "speed:  "  << cmd_.speed << "\n"
                               << "speed left:  " << speed_l << "\n"
                               << "speed right:  " << speed_r << "\n"
                               << "steer: " << cmd_.steer << "\n"
                               << "steer left: "    << steer_l << "\n"
                               << "steer right: "   << steer_r);

        // calculate wheel joint pid controller
        wheels[FRONT_LEFT].jointSpeed   = ( ((steer_l  - phi_fl) * proportionalControllerGain) - vel_phi_fl * derivativeControllerGain);
        wheels[FRONT_RIGHT].jointSpeed  = ( ((steer_r  - phi_fr) * proportionalControllerGain) - vel_phi_fr * derivativeControllerGain);
        wheels[MIDDLE_LEFT].jointSpeed  = ( ((0        - phi_ml) * proportionalControllerGain) - vel_phi_ml * derivativeControllerGain); // fixed joint
        wheels[MIDDLE_RIGHT].jointSpeed = ( ((0        - phi_mr) * proportionalControllerGain) - vel_phi_mr * derivativeControllerGain); // fixed joint
        wheels[REAR_LEFT].jointSpeed    = ( ((-steer_l - phi_rl) * proportionalControllerGain) - vel_phi_rl * derivativeControllerGain);
        wheels[REAR_RIGHT].jointSpeed   = ( ((-steer_r - phi_rr) * proportionalControllerGain) - vel_phi_rr * derivativeControllerGain);

        // check wheel joint dynamic
        if (jointMaxVelocity > 0.0 && fabs(wheels[FRONT_LEFT].jointSpeed) > jointMaxVelocity) wheels[FRONT_LEFT].jointSpeed = (wheels[FRONT_LEFT].jointSpeed > 0 ? jointMaxVelocity : -jointMaxVelocity);
        if (jointMaxVelocity > 0.0 && fabs(wheels[FRONT_RIGHT].jointSpeed) > jointMaxVelocity) wheels[FRONT_RIGHT].jointSpeed = (wheels[FRONT_RIGHT].jointSpeed > 0 ? jointMaxVelocity : -jointMaxVelocity);
        if (jointMaxVelocity > 0.0 && fabs(wheels[MIDDLE_LEFT].jointSpeed) > jointMaxVelocity) wheels[MIDDLE_LEFT].jointSpeed = (wheels[MIDDLE_LEFT].jointSpeed > 0 ? jointMaxVelocity : -jointMaxVelocity);
        if (jointMaxVelocity > 0.0 && fabs(wheels[MIDDLE_RIGHT].jointSpeed) > jointMaxVelocity) wheels[MIDDLE_RIGHT].jointSpeed = (wheels[MIDDLE_RIGHT].jointSpeed > 0 ? jointMaxVelocity : -jointMaxVelocity);
        if (jointMaxVelocity > 0.0 && fabs(wheels[REAR_LEFT].jointSpeed) > jointMaxVelocity) wheels[REAR_LEFT].jointSpeed = (wheels[REAR_LEFT].jointSpeed > 0 ? jointMaxVelocity : -jointMaxVelocity);
        if (jointMaxVelocity > 0.0 && fabs(wheels[REAR_RIGHT].jointSpeed) > jointMaxVelocity) wheels[REAR_RIGHT].jointSpeed = (wheels[REAR_RIGHT].jointSpeed > 0 ? jointMaxVelocity : -jointMaxVelocity);

//        ROS_DEBUG("i: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", phi_fl, phi_fr, phi_rl, phi_rr);
//        ROS_DEBUG("s: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", vel_phi_fl, vel_phi_fr, vel_phi_rl, vel_phi_rr);
//        ROS_DEBUG("v: fl: [%f] fr: [%f] rl: [%f] rr: [%f]\n", wheels[FRONT_LEFT].jointSpeed, wheels[FRONT_RIGHT].jointSpeed, wheels[REAR_LEFT].jointSpeed, wheels[REAR_RIGHT].jointSpeed);

        wheels[FRONT_LEFT].wheelSpeed   = speed_l / r / cos(phi_fl);
        wheels[FRONT_RIGHT].wheelSpeed  = speed_r / r / cos(phi_fr);
        wheels[MIDDLE_LEFT].wheelSpeed  = speed_l / r;
        wheels[MIDDLE_RIGHT].wheelSpeed = speed_r / r;
        wheels[REAR_LEFT].wheelSpeed    = speed_l / r / cos(phi_rl);
        wheels[REAR_RIGHT].wheelSpeed   = speed_r / r / cos(phi_rr);

        // odometry calculation
        // Compute angular velocity for ICC which is same as angular velocity of vehicle
        //omega_phi = (omega_fl * sin(phi_fl) * r / b); //+ omega_fr * sin(phi_fr) + omega_rl * sin(-phi_rl) + omega_rr * sin(-phi_rr)) * r / (4 * b);
        omega_phi = (omega_fl * sin(phi_fl) + omega_fr * sin(phi_fr))* r / l;

        v = r * (omega_ml + omega_mr)/2;

        // Compute odometric pose
        odomPose[0] += v * stepTime.Double() * cos(odomPose[2]);
        odomPose[1] += v * stepTime.Double() * sin(odomPose[2]);
        odomPose[2] += omega_phi * stepTime.Double() * 0.973;  //TODO MAGIC NUMBER

        // Compute odometric instantaneous velocity
        odomVel[0] = v;
        odomVel[1] = 0.0;
        odomVel[2] = omega_phi;

        publish_odometry();
        publish_joint_states();
    }

    ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel speeds:\n"
                           << "front left:  "  << wheels[FRONT_LEFT].wheelSpeed << "\n"
                           << "front right:  " << wheels[FRONT_RIGHT].wheelSpeed << "\n"
                           << "middle left:  " << wheels[MIDDLE_LEFT].wheelSpeed << "\n"
                           << "middle right: " << wheels[MIDDLE_RIGHT].wheelSpeed << "\n"
                           << "rear left: "    << wheels[REAR_LEFT].wheelSpeed << "\n"
                           << "rear right: "   << wheels[REAR_RIGHT].wheelSpeed);

    ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel poses:\n"
                           << "front left:  "   << wheels[FRONT_LEFT].joint->GetChild()->WorldPose() << "\n"
                           << "front right:  "  << wheels[FRONT_RIGHT].joint->GetChild()->WorldPose() << "\n"
                           << "middle leftt:  " << wheels[MIDDLE_LEFT].joint->GetChild()->WorldPose() << "\n"
                           << "middle right:  " << wheels[MIDDLE_RIGHT].joint->GetChild()->WorldPose() << "\n"
                           << "rear left:  "    << wheels[REAR_LEFT].joint->GetChild()->WorldPose() << "\n"
                           << "rear right:  "   << wheels[REAR_RIGHT].joint->GetChild()->WorldPose());

    wheels[FRONT_LEFT].joint->SetVelocity(0, wheels[FRONT_LEFT].jointSpeed);
    wheels[FRONT_RIGHT].joint->SetVelocity(0, wheels[FRONT_RIGHT].jointSpeed);
    wheels[MIDDLE_LEFT].joint->SetVelocity(0, wheels[MIDDLE_LEFT].jointSpeed);
    wheels[MIDDLE_RIGHT].joint->SetVelocity(0, wheels[MIDDLE_RIGHT].jointSpeed);
    wheels[REAR_LEFT].joint->SetVelocity(0, wheels[REAR_LEFT].jointSpeed);
    wheels[REAR_RIGHT].joint->SetVelocity(0, wheels[REAR_RIGHT].jointSpeed);

//    wheels[FRONT_LEFT].joint->SetMaxForce(0, jointMaxTorque);
//    wheels[FRONT_RIGHT].joint->SetMaxForce(0, jointMaxTorque);
//    wheels[MIDDLE_LEFT].joint->SetMaxForce(0, jointMaxTorque);
//    wheels[MIDDLE_RIGHT].joint->SetMaxForce(0, jointMaxTorque);
//    wheels[REAR_LEFT].joint->SetMaxForce(0, jointMaxTorque);
//    wheels[REAR_RIGHT].joint->SetMaxForce(0, jointMaxTorque);

    wheels[FRONT_LEFT].axle->SetVelocity(0, wheels[FRONT_LEFT].wheelSpeed);
    wheels[FRONT_RIGHT].axle->SetVelocity(0, wheels[FRONT_RIGHT].wheelSpeed);
    wheels[MIDDLE_LEFT].axle->SetVelocity(0, wheels[MIDDLE_LEFT].wheelSpeed);
    wheels[MIDDLE_RIGHT].axle->SetVelocity(0, wheels[MIDDLE_RIGHT].wheelSpeed);
    wheels[REAR_LEFT].axle->SetVelocity(0, wheels[REAR_LEFT].wheelSpeed);
    wheels[REAR_RIGHT].axle->SetVelocity(0, wheels[REAR_RIGHT].wheelSpeed);

//    wheels[FRONT_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
//    wheels[FRONT_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
//    wheels[MIDDLE_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
//    wheels[MIDDLE_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
//    wheels[REAR_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
//    wheels[REAR_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
}

void AllWheelSteeringPlugin::ComputeLocomotion(double speed, double steer, double& speed_l, double& speed_r, double& steer_l, double& steer_r)
{
    double l, b;
    double tan_steer;

    b = wheelTrack;
    l = wheelBase;

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
        if(speed_r > maxVelX) {
            speed_r = maxVelX;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        } else if(speed_r < -maxVelX) {
            speed_r = -maxVelX;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        }
    } else {
        // turn right -> check left wheel speed
        if(speed_l > maxVelX) {
            speed_l = maxVelX;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        } else if(speed_l < -maxVelX) {
            speed_l = -maxVelX;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        }
    }
}

void AllWheelSteeringPlugin::motionCommandCallback(const smart_msgs::MotionCommand::ConstPtr& cmd_msg)
{
    boost::mutex::scoped_lock lock(mutex);
    cmd_ = *cmd_msg;
}

void AllWheelSteeringPlugin::publish_odometry()
{
    if (!odomPub_) return;

    // publish odom topic
    odom_.pose.pose.position.x = odomPose[0];
    odom_.pose.pose.position.y = odomPose[1];

    tf::Quaternion qt = tf::createQuaternionFromRPY(0.0, 0.0, fmod(odomPose[2] + M_PI, 2*M_PI) - M_PI);
    tf::quaternionTFToMsg(qt, odom_.pose.pose.orientation);

    odom_.twist.twist.linear.x = odomVel[0];
    odom_.twist.twist.linear.y = odomVel[1];
    odom_.twist.twist.angular.z = odomVel[2];

    odom_.header.stamp.sec = world->SimTime().sec;
    odom_.header.stamp.nsec = world->SimTime().nsec;

    odomPub_.publish(odom_);
}

void AllWheelSteeringPlugin::publish_joint_states()
{
    if (!jointStatePub_) return;

    joint_state.header.stamp.sec = world->SimTime().sec;
    joint_state.header.stamp.nsec = world->SimTime().nsec;
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    joint_state.effort.resize(12);

    for (unsigned int i = 0; i < 6; i++) {
        joint_state.name[i] = wheels[i].joint->GetName();
        joint_state.position[i] = wheels[i].joint->Position(0);
        joint_state.velocity[i] = wheels[i].joint->GetVelocity(0);
        joint_state.effort[i] = wheels[i].joint->GetForce(0u);
    }

    for (unsigned int i = 0; i < 6; i++) {
        joint_state.name[6+i] = wheels[i].axle->GetName();
        joint_state.position[6+i] = wheels[i].axle->Position(0);
        joint_state.velocity[6+i] = wheels[i].axle->GetVelocity(0);
        joint_state.effort[6+i] = wheels[i].axle->GetForce(0u);
    }

    jointStatePub_.publish(joint_state);
}

float AllWheelSteeringPlugin::maxDeltaFilter(float y, float x, float c) {
    float delta = y-x;
    float test = x+sgn(delta) * c;
    return (fabs(delta) > c) ? test : y;
}

template <typename T> int AllWheelSteeringPlugin::sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

} // namespace gazebo

