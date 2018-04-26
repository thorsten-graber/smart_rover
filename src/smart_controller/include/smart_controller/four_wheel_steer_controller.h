/*
    Copyright (c) 2014, Stefan Kohlbrecher
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FOUR_WHEEL_STEER_CONTROLLER_H
#define FOUR_WHEEL_STEER_CONTROLLER_H

#include <ros/ros.h>
#include "vehicle_control_interface.h"

class FourWheelSteerController: public VehicleControlInterface
{
  public:
    virtual void configure(ros::NodeHandle& params, MotionParameters* mp)
    {
      mp_ = mp;

      ros::NodeHandle nh;
      drivePublisher_ = nh.advertise<smart_msgs::MotionCommand>("drive", 1);

      max_steerAngle = 90.0 * M_PI/180.0;
      params.getParam("max_steerAngle", max_steerAngle);
      wheelBase = 0.304;
      params.getParam("wheelBase", wheelBase);
    }

    virtual void executeTwist(const geometry_msgs::Twist& velocity)
    {
      double backward = (velocity.linear.x < 0) ? -1.0 : 1.0;
      double speed = backward * sqrt(velocity.linear.x*velocity.linear.x + velocity.linear.y*velocity.linear.y);
      mp_->limitSpeed(speed);

//      double kappa = velocity.angular.z * speed;
//      double tan_gamma = tan(velocity.linear.y / velocity.linear.x);
//      setDriveCommand(speed, kappa, tan_gamma);

      double omega = velocity.angular.z;
      double atan_gamma = atan(velocity.linear.y / velocity.linear.x);
      publishDriveCommand(speed, omega, atan_gamma);
    }

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed)
    {
//      double sign = speed < 0.0 ? -1.0 : 1.0;
//      double kappa     = sign * carrot_orientation_error / carrot_distance * 1.5;
//      double tan_gamma = tan(carrot_relative_angle - carrot_orientation_error);

      double sign = speed < 0.0 ? -1.0 : 1.0;
      double kappa     = sign * carrot_orientation_error / carrot_distance * 1.5;
      double tan_gamma = sign * tan(carrot_relative_angle - carrot_orientation_error);

      this->setDriveCommand(speed, kappa ,tan_gamma);
    }

    virtual void stop()
    {
      drive.speed = 0.0;
      drive.mode = "continuous";
      drivePublisher_.publish(drive);
    }

    virtual void reset()
    {
    }

    virtual double getCommandedSpeed() const
    {
      return drive.speed;
    }

    virtual std::string getName()
    {
      return "Four Wheel Steering Controller";
    }

    void setDriveCommand(double speed, double kappa, double tan_gamma) {

      double l = wheelBase / 2.0; // half wheel distance (front - rear)

      drive.speed = speed;
      mp_->limitSpeed(drive.speed);

      if (drive.speed != 0.0) {
        double max_kappa = tan(max_steerAngle) / l;
        if (kappa >= max_kappa) {
          kappa = max_kappa;
          tan_gamma = 0;

        } else if (kappa <= -max_kappa) {
          kappa = -max_kappa;
          tan_gamma = 0;

        } else {
          double max_tan_gamma = tan(max_steerAngle) - fabs(kappa) * l;
          if (tan_gamma >  max_tan_gamma) tan_gamma =  max_tan_gamma;
          if (tan_gamma < -max_tan_gamma) tan_gamma = -max_tan_gamma;
        }

        drive.steer = atan( tan_gamma + kappa * l);
        drive.mode = "continuous";
      }
      drivePublisher_.publish(drive);
    }

    void publishDriveCommand(double speed, double omega, double atan_gamma) {
        double l = wheelBase;

        double sign = (speed < 0) ? -1.0 : 1.0;
        drive.steer = sign*atan_gamma + atan((omega*l)/(2*fabs(speed)));
        if(speed == 0.0) drive.steer = 0;

        drive.speed = speed;
        mp_->limitSpeed(drive.speed);
        drive.mode = "continuous";

        drivePublisher_.publish(drive);
    }


  protected:
    ros::Publisher drivePublisher_;

    smart_msgs::MotionCommand drive;

    MotionParameters* mp_;

    double max_steerAngle, wheelBase;
};

#endif
