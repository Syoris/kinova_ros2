/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_ros_types.h
 *  Desc: Wrappers around Kinova types.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#ifndef KINOVA_DRIVER_KINOVA_TYPES_H
#define KINOVA_DRIVER_KINOVA_TYPES_H

#include <kinova/KinovaTypes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <kinova_msgs/msg/joint_angles.hpp>
#include <kinova_msgs/msg/joint_velocity.hpp>
#include <kinova_msgs/msg/finger_position.hpp>
#include <kinova_msgs/msg/kinova_pose.hpp>
// #include <tf/tf.h>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>


namespace kinova
{

tf2::Quaternion EulerXYZ2Quaternion(float tx, float ty, float tz);

tf2::Matrix3x3 EulerXYZ2Matrix3x3(float tx, float ty, float tz);

void getEulerXYZ(tf2::Matrix3x3 &Rot_matrix, float &txf, float &tyf, float &tzf);

void getEulerXYZ(tf2::Quaternion &q, float &tx, float &ty, float &tz);

bool valid_kinovaRobotType(const std::string &robotType);

class KinovaException : public std::exception {};


class KinovaCommException : public KinovaException
{
 public:
    explicit KinovaCommException(const std::string& message, const int error_code);
    ~KinovaCommException() throw() {}

    const char* what() const throw();
 private:
    std::string desc_;
};


class KinovaPose : public CartesianInfo
{
 public:
    KinovaPose() {}
    explicit KinovaPose(const geometry_msgs::msg::Pose &pose);
    explicit KinovaPose(const CartesianInfo &pose);

    geometry_msgs::msg::Pose   constructPoseMsg();
    kinova_msgs::msg::KinovaPose constructKinovaPoseMsg();
    geometry_msgs::msg::Wrench constructWrenchMsg();
    void getQuaternion(tf2::Quaternion &q);

    bool isCloseToOther(const KinovaPose &, float position_tolerance, float EulerAngle_tolerance) const;
};


class KinovaAngles : public AngularInfo
{
 public:
    KinovaAngles() {}
    explicit KinovaAngles(const kinova_msgs::msg::JointAngles &angles);
    explicit KinovaAngles(const AngularInfo &angles);

    kinova_msgs::msg::JointAngles constructAnglesMsg();
    kinova_msgs::msg::JointVelocity constructVelsMsg();
    bool isCloseToOther(const KinovaAngles &, float tolerance) const;
    void applyShortestAngleDistanceTo(KinovaAngles target_angle);
};


class FingerAngles : public FingersPosition
{
 public:
    FingerAngles() {}
    explicit FingerAngles(const kinova_msgs::msg::FingerPosition &position);
    explicit FingerAngles(const FingersPosition &angle);

    kinova_msgs::msg::FingerPosition constructFingersMsg();
    bool isCloseToOther(const FingerAngles &, float tolerance) const;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_TYPES_H
