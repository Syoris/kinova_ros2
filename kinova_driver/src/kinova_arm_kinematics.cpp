
#include <kinova_driver/kinova_arm_kinematics.h>
// #include <kinova_driver/kinova_ros_types.h>
#include <boost/lexical_cast.hpp>


namespace kinova
{

std::string concatTfName(const std::string& prefix, const std::string name)
{
    std::stringstream ss;
    ss << prefix << name;
    return ss.str();
}

std::string concatTfName(const std::string& prefix, const std::string name, const int index)
{
    std::stringstream ss;
    ss << prefix << name << index;
    return ss.str();
}

std::vector<double> array2vector(double* array, int length)
{
    std::vector<double> result;
    for (int i=0; i<length; i++)
    {
        result.push_back(array[i]);
    }
    return result;
}

KinovaKinematics::KinovaKinematics(rclcpp::Node::SharedPtr node): node_(node)
{
    
    tf_prefix_ = "arm_";

    // Initialize the transform broadcaster
    broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // parameters stored in DSP chip
    // node_->declare_parameter("D1", D1_);
    // node_->declare_parameter("D2", D2_);
    // node_->declare_parameter("D3", D3_);
    // node_->declare_parameter("D4", D4_);
    // node_->declare_parameter("D5", D5_);
    // node_->declare_parameter("D6", D6_);
    // node_->declare_parameter("D7", D7_);
    // node_->declare_parameter("e2", e2_);
    node_->get_parameter_or("D1", D1_, 0.2755);
    node_->get_parameter_or("D2", D2_, 0.205);
    node_->get_parameter_or("D3", D3_, 0.205);
    node_->get_parameter_or("D4", D4_, 0.2073);
    node_->get_parameter_or("D5", D5_, 0.1038);
    node_->get_parameter_or("D6", D6_, 0.1038);
    node_->get_parameter_or("D7", D7_, 0.160);
    node_->get_parameter_or("e2", e2_, 0.0098);

    // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
    double DH_a[7] = {0,  0, 0, 0, 0, 0, 0};
    double DH_d[7] = {-D1_, 0, -(D2_+ D3_), -e2_, -(D4_ + D5_), 0, -(D6_ + D7_)};
    double DH_alpha[7] = {M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI};
    // DH_theta = DH_theta_sign*Q + DH_theta_offset
    double DH_theta_sign[7] = {-1, 1, 1, 1, 1, 1, 1};
    double DH_theta_offset[7] = {0, 0, 0, 0, 0, 0, 0};

    // copy local array values to class-scope vector.
    arm_joint_number_ = 7;
    DH_a_ = array2vector(DH_a, arm_joint_number_);
    DH_d_ = array2vector(DH_d, arm_joint_number_);
    DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
    DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
    DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);

    node_->declare_parameter("kinova_robot_name", "ROBOT");

    rclcpp::Parameter baseFrameParam;
    node_->get_parameter_or("base_frame", baseFrameParam, rclcpp::Parameter("base_frame", "world"));
    baseFrame = baseFrameParam.as_string();
}

geometry_msgs::msg::Transform KinovaKinematics::DHParam2Transform(float d, float theta, float a, float alpha)
{
    geometry_msgs::msg::Transform transform;
    tf2::Quaternion rotation_q(0, 0, 0, 1);
    tf2::Matrix3x3 rot_matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    tf2::Vector3 translation_v(0, 0, 0);

    rot_matrix.setValue(cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
                        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                        0,           sin(alpha),             cos(alpha));
    rot_matrix.getRotation(rotation_q);


    tf2::convert(rotation_q.normalize(), transform.rotation);
    // transform.rotation = rotation_q.normalize();

    translation_v.setValue(a*cos(theta), a*sin(theta), d);
    // transform.setOrigin(translation_v);
    tf2::convert(translation_v, transform.translation);

    return transform;
}

/**************************************/
/*                                    */
/*  Computer and publish transform    */
/*                                    */
/**************************************/
// IMPORTANT!!! In the robot DSP chip, the classical D-H parameters are used to define the robot. Therefore, the frame definition is different comparing with the frames in URDF model.
void KinovaKinematics::updateForward(float* Q)
{
    geometry_msgs::msg::Transform transform;
    // the orientation of frame0 is differently defined starting from Jaco 6 spherical robot.
    transform = DHParam2Transform(0, 0, 0, M_PI);

    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.transform = transform;
    stamped_transform.header.stamp = node_->now();
    stamped_transform.header.frame_id = baseFrame;
    stamped_transform.child_frame_id = "link_base";

    broadcaster_->sendTransform(stamped_transform);

    double DH_theta_i;
    for (int i = 0; i<arm_joint_number_; i++)
    {
        DH_theta_i = DH_theta_sign_[i]*Q[i] + DH_theta_offset_[i];
        transform = DHParam2Transform(DH_d_[i], DH_theta_i, DH_a_[i], DH_alpha_[i]);
        if(i==0)
        {
            stamped_transform.transform = transform;
            stamped_transform.header.stamp = node_->now();
            stamped_transform.header.frame_id = "link_base";
            stamped_transform.child_frame_id = "link_1";
            broadcaster_->sendTransform(stamped_transform);

            // broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), concatTfName(tf_prefix_, "link_base"), concatTfName(tf_prefix_, "link_1")));
        }
        else
        {
            stamped_transform.transform = transform;
            stamped_transform.header.stamp = node_->now();
            stamped_transform.header.frame_id = concatTfName("", "link_", i);
            stamped_transform.child_frame_id = concatTfName("", "link_", i+1);
            broadcaster_->sendTransform(stamped_transform);
            
            // broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),                                                            concatTfName(tf_prefix_, "link_", i), concatTfName(tf_prefix_, "link_", i+1)));
        }
    }

}

}  // namespace kinova
