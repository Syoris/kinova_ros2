#ifndef KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H
#define KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H

#include <math.h>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// #include <tf/tf.h>
// #include <tf/transform_broadcaster.h>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/******************************************/
/**************DH Parameters***************/
/******************************************/
/* i * alpha(i-1) * a(i-1) * di  * theta1 */
/******************************************/
/* 1 * 0          * 0      * D1  * q1     */
/* 2 * -pi/2      * 0      * 0   * q2     */
/* 3 * 0          * D2     * 0   * q3     */
/* 4 * -pi/2      * 0      * d4b * q4     */
/* 5 * 2*aa       * 0      * d5b * q5     */
/* 6 * 2*aa       * 0      * d6b * q6     */
/******************************************/

namespace kinova
{

class KinovaKinematics
{
 public:
    explicit KinovaKinematics(rclcpp::Node::SharedPtr node);

    void updateForward(float* Q);

    inline float degToRad(float degrees)
    {
        return (degrees * (M_PI / 180));
    }

 private:
    rclcpp::Node::SharedPtr node_;

    // tf::TransformBroadcaster broadcaster_;
    // tf::Transform DHParam2Transform(float d, float theta, float a, float alpha);

    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::Transform DHParam2Transform(float d, float theta, float a, float alpha);

    // Parameters
    std::string tf_prefix_;

    //base frame for robot
    std::string baseFrame;

    /* Robot Length Values (Meters) */
    double D1_;         // base to elbow
    double D2_;         // arm length
    double D3_;         // front arm length
    double D4_;         // frist wrist length
    double D5_;         // second wrist length
    double D6_;         // wrist to center of hand
    double D7_;
    double e2_;         // offset of joint
    double wrist_deg_;  // wrist bend degree

    /* classic DH table parameters */
    int arm_joint_number_;
    std::vector<double> DH_a_;
    std::vector<double> DH_d_;
    std::vector<double> DH_alpha_;
    std::vector<double> DH_theta_sign_;
    std::vector<double> DH_theta_offset_;

};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H
