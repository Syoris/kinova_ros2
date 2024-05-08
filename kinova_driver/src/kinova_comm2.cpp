#include "rclcpp/rclcpp.hpp"
#include "kinova_driver/kinova_comm2.h"

#include <string>
#include <vector>
#include <arpa/inet.h>

namespace kinova
{

KinovaComm2::KinovaComm2(rclcpp::Node::SharedPtr node,
                   boost::recursive_mutex &api_mutex,
                   const bool is_movement_on_start,
                   const std::string &kinova_robotType)
    :node_(node), api_mutex_(api_mutex), is_software_stop_(false)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    int result = NO_ERROR_KINOVA;

    //initialize kinova api functions
    std::string api_type = node_->get_parameter("connection_type").as_string();
    
    if (api_type == "USB")
      kinova_api_.initializeKinovaAPIFunctions(USB);
    else{
      RCLCPP_ERROR(node_->get_logger(), "Ethernet not supported");
    //   kinova_api_.initializeKinovaAPIFunctions(ETHERNET);
    }

    // Get the serial number parameter for the arm we wish to connect to
    std::string serial_number = "";
    serial_number = node_->get_parameter("serial_number").as_string();

    // int api_version[API_VERSION_COUNT];
    // result = kinova_api_.getAPIVersion(api_version);
    // if (result != NO_ERROR_KINOVA)
    // {
    //     throw KinovaCommException("Could not get the Kinova API version", result);
    // }

    // ROS_INFO_STREAM("Initializing Kinova "<< api_type.c_str()
    //                 << " API (header version: " << COMMAND_LAYER_VERSION
    //                 << ", library version: " << api_version[0] << "."
    //                                          << api_version[1] << "." << api_version[2] << ")");

    // if (api_type == "USB"){
    //   result = kinova_api_.initAPI();
    // }
    // else{
    //     // result =kinova_api_.initEthernetAPI(ethernet_settings);
    //     RCLCPP_ERROR(node_handle.get_logger(), "Ethernet not supported");
    // }

    // if (result != NO_ERROR_KINOVA)
    // {
    //     throw KinovaCommException("Could not initialize Kinova API", result);
    // }

    // result = kinova_api_.refresDevicesList();

    // result = NO_ERROR_KINOVA;
    // int devices_count = kinova_api_.getDevices(devices_list_, result);
    // if (result != NO_ERROR_KINOVA)
    // {
    //     throw KinovaCommException("Could not get devices list", result);
    // }

    // if (result != NO_ERROR_KINOVA)
    // {
    //     throw KinovaCommException("Could not get devices list count.", result);
    // }

    // bool found_arm = false;
    // for (int device_i = 0; device_i < devices_count; device_i++)
    // {
    //     // If no device is specified, just use the first available device
    //     if (serial_number == "" || serial_number == "not_set" ||
    //         std::strncmp(serial_number.c_str(),
    //                      devices_list_[device_i].SerialNumber,
    //                      std::min(serial_number.length(), 
    //                               strlen(devices_list_[device_i].SerialNumber))) 
    //         == 0)
    //     {
    //         result = kinova_api_.setActiveDevice(devices_list_[device_i]);
    //         if (result != NO_ERROR_KINOVA)
    //         {
    //             throw KinovaCommException("Could not set the active device", result);
    //         }

    //         GeneralInformations general_info;
    //         result = kinova_api_.getGeneralInformations(general_info);
    //         if (result != NO_ERROR_KINOVA)
    //         {
    //             throw KinovaCommException("Could not get general information about the device", result);
    //         }

    //         ClientConfigurations configuration;
    //         getConfig(configuration);

    //         QuickStatus quick_status;
    //         getQuickStatus(quick_status);

    //         robot_type_ = quick_status.RobotType;

    //         ROS_INFO_STREAM("Found " << devices_count << " device(s), using device at index " << device_i
    //                         << " (model: " << configuration.Model
    //                         << ", serial number: " << devices_list_[device_i].SerialNumber
    //                         << ", code version: " << general_info.CodeVersion
    //                         << ", code revision: " << general_info.CodeRevision << ")");

    //         found_arm = true;
    //         break;
    //     }
    // }

    // if (!found_arm)
    // {
    //     ROS_ERROR("Could not find the specified arm (serial: %s) among the %d attached devices",
    //               serial_number.c_str(), devices_count);
    //     throw KinovaCommException("Could not find the specified arm", 0);
    // }

    // //find the number of joints and fingers of the arm using robotType passed from arm node
    // num_joints_ = kinova_robotType[3]-'0';
    // num_fingers_ = kinova_robotType[5]-'0';

    // // On a cold boot the arm may not respond to commands from the API right away.
    // // This kick-starts the Control API so that it's ready to go.
    // startAPI();
    // stopAPI();
    // startAPI();

    // //Set robot to use manual COM parameters
    // bool use_estimated_COM;
    // node_handle.param("torque_parameters/use_estimated_COM_parameters",
    //                       use_estimated_COM,true);
    // if (use_estimated_COM == true)
    //     kinova_api_.setGravityType(OPTIMAL);
    // else
    //     kinova_api_.setGravityType(MANUAL_INPUT);

    // //Set torque safety factor to 1
    // kinova_api_.setTorqueSafetyFactor(1);


    // // Set the angular velocity of each of the joints to zero
    // TrajectoryPoint kinova_velocity;
    // memset(&kinova_velocity, 0, sizeof(kinova_velocity));
    // setCartesianVelocities(kinova_velocity.Position.CartesianPosition);

    // if (is_movement_on_start)
    // {
    //     initFingers();
    // }
    // else
    // {
    //     ROS_WARN("Movement on connection to the arm has been suppressed on initialization. You may "
    //              "have to home the arm (through the home service) before movement is possible");
    // }
}

KinovaComm2::~KinovaComm2()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    kinova_api_.closeAPI();
}

}  // namespace kinova