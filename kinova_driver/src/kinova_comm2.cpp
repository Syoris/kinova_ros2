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
    RCLCPP_INFO(node_->get_logger(), "(kinova_comm2) KinovaComm2 constructor");

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

    // API Version
    int api_version[API_VERSION_COUNT];
    result = kinova_api_.getAPIVersion(api_version);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Kinova API version", result);
    }

    std::string cmd_layer_str = std::to_string(COMMAND_LAYER_VERSION);
    const char* cmd_layer_cstr = cmd_layer_str.c_str();
    std::string version_str = cmd_layer_cstr[0] + std::string(".") + cmd_layer_cstr[1] + cmd_layer_cstr[2] + std::string(".") + cmd_layer_cstr[3] + cmd_layer_cstr[4];
    RCLCPP_INFO(node_->get_logger(), "(kinova_comm2) Initializing Kinova %s API (header version: %s, library version: %d.%d.%d)",
                api_type.c_str(), version_str.c_str(), api_version[0], api_version[1], api_version[2]);

    
    // Initialize the API
    if (api_type == "USB"){
      result = kinova_api_.initAPI();
    }
    else{
        // result =kinova_api_.initEthernetAPI(ethernet_settings);
        RCLCPP_ERROR(node_->get_logger(), "Ethernet not supported");
    }

    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not initialize Kinova API", result);
    }

    // Get the list of devices connected to the computer
    result = kinova_api_.refresDevicesList();

    result = NO_ERROR_KINOVA;
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get devices list", result);
    }

    int devices_count = kinova_api_.getDevices(devices_list_, result);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get devices list count.", result);
    }

    // Find the arm with the specified serial number
    bool found_arm = false;

    result = kinova_api_.setActiveDevice(devices_list_[0]);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the active device", result);
    }

    GeneralInformations general_info;
    result = kinova_api_.getGeneralInformations(general_info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get general information about the device", result);
    }

    ClientConfigurations configuration;
    getConfig(configuration);
    printConfig(configuration);

    QuickStatus quick_status;
    getQuickStatus(quick_status);

    robot_type_ = quick_status.RobotType;

    RCLCPP_INFO_STREAM(node_->get_logger(), "(kinova_comm2) Found " << devices_count << " device(s), using device at index " << 0
                    << " (model: " << configuration.Model
                    << ", serial number: " << devices_list_[0].SerialNumber
                    << ", code version: " << general_info.CodeVersion
                    << ", code revision: " << general_info.CodeRevision << ")");

    found_arm = true;

    if (!found_arm)
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not find the specified arm (serial: %s) among the %d attached devices",
              serial_number.c_str(), devices_count);
        throw KinovaCommException("Could not find the specified arm", 0);
    }

    //find the number of joints and fingers of the arm using robotType passed from arm node
    num_joints_ = kinova_robotType[3]-'0';
    num_fingers_ = kinova_robotType[5]-'0';

    // On a cold boot the arm may not respond to commands from the API right away.
    // This kick-starts the Control API so that it's ready to go.
    RCLCPP_INFO(node_->get_logger(), "(kinova_comm2) Starting the Control API");
    startAPI();
    stopAPI();
    startAPI();

    //Set robot to use manual COM parameters
    bool use_estimated_COM = node_->get_parameter_or("torque_parameters/use_estimated_COM_parameters", true);
    RCLCPP_INFO(node_->get_logger(), "(kinova_comm2) Setting COM params. Using estimated COM: %s", use_estimated_COM ? "true" : "false");
    if (use_estimated_COM == true)
        kinova_api_.setGravityType(OPTIMAL);
    else
        kinova_api_.setGravityType(MANUAL_INPUT);

    //Set torque safety factor to 1
    kinova_api_.setTorqueSafetyFactor(1);


    // Set the angular velocity of each of the joints to zero
    TrajectoryPoint kinova_velocity;
    memset(&kinova_velocity, 0, sizeof(kinova_velocity));
    setCartesianVelocities(kinova_velocity.Position.CartesianPosition);

    if (is_movement_on_start)
    {
        initFingers();
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Movement on connection to the arm has been suppressed on initialization. You may have to home the arm (through the home service) before movement is possible");
    }
}

KinovaComm2::~KinovaComm2()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    kinova_api_.closeAPI();
}


// MARK: General functions
/**
 * @brief This function tells the robotical arm that from now on, the API will control the robotical arm. It must been call before sending trajectories or joystick command.
 */
void KinovaComm2::startAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    if (is_software_stop_)
    {
        is_software_stop_ = false;
        kinova_api_.stopControlAPI();
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    int result = kinova_api_.startControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not start the control API", result);
    }
}


/**
 * @brief This function tells the robotical arm the from now on the API is not controlling the robotical arm.
 */
void KinovaComm2::stopAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    is_software_stop_ = true;

    int result = kinova_api_.stopControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not stop the control API", result);
    }

    result = kinova_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not erase all trajectories", result);
    }
}


/**
 * @brief check if API lost the control of robot.
 * @return true if stopAPI() was called.
 */
bool KinovaComm2::isStopped()
{
    return is_software_stop_;
}


/**
 * @brief get robotType
 * Index for robot type:
 *  JACOV1_ASSISTIVE = 0,
 *  MICO_6DOF_SERVICE = 1,
 *  MICO_4DOF_SERVICE = 2,
 * 	JACOV2_6DOF_SERVICE = 3,
 * 	JACOV2_4DOF_SERVICE = 4,
 * 	MICO_6DOF_ASSISTIVE = 5,
 * 	JACOV2_6DOF_ASSISTIVE = 6,
 * @return index of robot type
 */
int KinovaComm2::robotType() const
{
    return robot_type_;
}


/**
 * @brief This function gets information regarding some status flag of the robotical arm.
 * @param quick_status This structure holds various informations but mostly it is flag status, such as robotype, retractType, forceControlStatus,
 */
void KinovaComm2::getQuickStatus(QuickStatus &quick_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&quick_status, 0, sizeof(quick_status));  // zero structure
    int result = kinova_api_.getQuickStatus(quick_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get quick status", result);
    }
}


/**
 * @brief This function set the client configurations of the robotical arm. The configuration data is stored on the arm itself.
 * @param config config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
 */
void KinovaComm2::setConfig(const ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the client configuration", result);
    }
}


/**
 * @brief obtain the current client configuration.
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
 */
void KinovaComm2::getConfig(ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&config, 0, sizeof(config));  // zero structure

    int result = kinova_api_.getClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get client configuration", result);
    }
}


/**
 * @brief Dumps the client configuration onto the screen.
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
 */
void KinovaComm2::printConfig(const ClientConfigurations &config)
{       
    std::string ClientID(config.ClientID);
    std::string ClientName(config.ClientName);
    std::string Organization(config.Organization);
    std::string Serial(config.Serial);
    std::string Model(config.Model);

    std::string msg = "Arm configuration:\n";
    msg += "\tClientID: " + ClientID + "\n";
    msg += "\tClientName: " + ClientName + "\n";
    msg += "\tOrganization: " + Organization + "\n";
    msg += "\tSerial: " + Serial + "\n";
    msg += "\tModel: " + Model + "\n";
    msg += "\tMaxForce: " + std::to_string(config.MaxForce) + "\n";
    msg += "\tSensibility: " + std::to_string(config.Sensibility) + "\n";
    msg += "\tDrinkingHeight: " + std::to_string(config.DrinkingHeight) + "\n";
    msg += "\tComplexRetractActive: " + std::to_string(config.ComplexRetractActive) + "\n";
    msg += "\tRetractedPositionAngle: " + std::to_string(config.RetractedPositionAngle) + "\n";
    msg += "\tRetractedPositionCount: " + std::to_string(config.RetractedPositionCount) + "\n";
    msg += "\tDrinkingDistance: " + std::to_string(config.DrinkingDistance) + "\n";
    msg += "\tFingers2and3Inverted: " + std::to_string(config.Fingers2and3Inverted) + "\n";
    msg += "\tDrinkingLength: " + std::to_string(config.DrinkingLenght) + "\n";
    msg += "\tDeletePreProgrammedPositionsAtRetract: " + std::to_string(config.DeletePreProgrammedPositionsAtRetract) + "\n";
    msg += "\tEnableFlashErrorLog: " + std::to_string(config.EnableFlashErrorLog) + "\n";
    msg += "\tEnableFlashPositionLog: " + std::to_string(config.EnableFlashPositionLog) + "\n";
    
    RCLCPP_INFO(node_->get_logger(), msg.c_str());

    // RCLCPP_INFO(logger, "Arm configuration:\n" <<
    //                 "\tClientID: " << config.ClientID <<
    //                 "\n\tClientName: " << config.ClientName <<
    //                 "\n\tOrganization: " << config.Organization <<
    //                 "\n\tSerial:" << config.Serial <<
    //                 "\n\tModel: " << config.Model <<
    //                 "\n\tMaxForce: " << config.MaxForce <<
    //                 "\n\tSensibility: " <<  config.Sensibility <<
    //                 "\n\tDrinkingHeight: " << config.DrinkingHeight <<
    //                 "\n\tComplexRetractActive: " << config.ComplexRetractActive <<
    //                 "\n\tRetractedPositionAngle: " << config.RetractedPositionAngle <<
    //                 "\n\tRetractedPositionCount: " << config.RetractedPositionCount <<
    //                 "\n\tDrinkingDistance: " << config.DrinkingDistance <<
    //                 "\n\tFingers2and3Inverted: " << config.Fingers2and3Inverted <<
    //                 "\n\tDrinkingLength: " << config.DrinkingLenght <<
    //                 "\n\tDeletePreProgrammedPositionsAtRetract: " <<
    //                 config.DeletePreProgrammedPositionsAtRetract <<
    //                 "\n\tEnableFlashErrorLog: " << config.EnableFlashErrorLog <<
    //                 "\n\tEnableFlashPositionLog: " << config.EnableFlashPositionLog);
}


/**
 * @brief get current control type
 * The control in ROS is independent to Joystick control type. For example: even set robot in joint control though api in ROS, robot may still controlled in joint level by joystick.
 * @param controlType Cartesian control[0] or joint control[1]
 */
void KinovaComm2::getControlType(int &controlType)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&controlType, 0, sizeof(controlType)); //zero structure
    int result = kinova_api_.getControlType(controlType);
    if (result!=NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get control type", result);
    }
}


/**
 * @brief get almost all information of the robotical arm.
 * @param general_info includes: power statistics, sensor infos, robot position and command, etc.
 */
void KinovaComm2::getGeneralInformations(GeneralInformations &general_info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&general_info, 0, sizeof(general_info));
    int result = kinova_api_.getGeneralInformations(general_info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get general information about the device", result);
    }
}


/**
 * @brief This function returns information about the robotical arm's sensors. (Voltage, Total current, Temperature, acceleration)
 * @param The structure containing the sensor's informations
 */
void KinovaComm2::getSensorsInfo(SensorsInfo &sensor_Info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&sensor_Info, 0, sizeof(sensor_Info));
    int result = kinova_api_.getSensorsInfo(sensor_Info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get sensors information", result);
    }
}


/**
 * @brief This function gets information regarding all forces.
 * @param force_Info A struct containing the information about the forces. Joint torque and end-effector wrench in Nm and N.
 */
void KinovaComm2::getForcesInfo(ForcesInfo &force_Info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&force_Info, 0, sizeof(force_Info));
    int result = kinova_api_.getForcesInfo(force_Info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get force information", result);
    }
}


/**
 * @brief This function gets informations about the robotical arm's gripper. Some information may be missing, it is still in development.
 * @param gripper_status A struct containing the information of the gripper. Most information of each fingers, including model, motion, force, limits, etc.
 */
void KinovaComm2::getGripperStatus(Gripper &gripper_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&gripper_status, 0, sizeof(gripper_status));
    int result = kinova_api_.getGripperStatus(gripper_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the gripper status", result);
    }
}


/**
 * @brief This function move the arm to the "home" position.
 * The code replicates the function of the "home" button on the user controller by "pressing" the home button long enough for the arm to return to the home position.
 * @warning The home position is the default home, rather than user defined home.
 */
void KinovaComm2::homeArm(void)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        RCLCPP_INFO(node_->get_logger(), "Arm is stopped, cannot home");
        return;
    }
    else if (isHomed())
    {

        RCLCPP_INFO(node_->get_logger(), "Arm is already in \"home\" position");
        return;
    }

    stopAPI();
    rclcpp::sleep_for(std::chrono::seconds(1));
    startAPI();

    RCLCPP_INFO(node_->get_logger(), "Homing the arm");
    kinova_api_.moveHome();

    /*JoystickCommand mycommand;
    mycommand.InitStruct();
    // In api mapping(observing with Jacosoft), home button is ButtonValue[2].
    mycommand.ButtonValue[2] = 1;
    for(int i = 0; i<2000; i++)
    {
        kinova_api_.sendJoystickCommand(mycommand);
        usleep(5000);

        // if (myhome.isCloseToOther(KinovaAngles(currentAngles.Actuators), angle_tolerance))
        if(isHomed())
        {
            RCLCPP_INFO(node_->get_logger(), "Arm is in \"home\" position");
            // release home button.
            mycommand.ButtonValue[2] = 0;
            kinova_api_.sendJoystickCommand(mycommand);
            return;
        }
    }

    mycommand.ButtonValue[2] = 0;
    kinova_api_.sendJoystickCommand(mycommand);
    ROS_WARN("Homing arm timer out! If the arm is not in home position yet, please re-run home arm.");*/

}


/**
 * @brief Determines whether the arm has returned to its "Home" state. Checks the current joint angles, then compares them to the known "Home" joint angles.
 * @return true is robot is already in predefined "Home"configuration.
 * @warning The home position is the default home, rather than user defined home.
 */
bool KinovaComm2::isHomed(void)
{
    QuickStatus quick_status;
    getQuickStatus(quick_status);

    if (quick_status.RetractType == RETRACT_TYPE_READY_STANDBY)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief This function Set the end effector offset's parameters. The end effector's offset is a translation offset applied to the end effector of the robotic arm.
 * @param status indicates if the offset is applied or not (0 = not applied, 1 = applied)
 * @param x Unit in meter
 * @param y Unit in meter
 * @param z Unit in meter
 */
void KinovaComm2::setEndEffectorOffset(unsigned int status, float x, float y, float z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    int result = kinova_api_.setEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set end effector offset.", result);
    }
}


/**
 * @brief This function get the end effector offset's parameters. The end effector's offset is a translation offset applied to the end effector of the robotic arm.
 * @param status indicates if the offset is applied or not (0 = not applied, 1 = applied)
 * @param x Unit in meter
 * @param y Unit in meter
 * @param z Unit in meter
 */
void KinovaComm2::getEndEffectorOffset(unsigned int &status, float &x, float &y, float &z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&status, 0, sizeof(status));
    memset(&x, 0, sizeof(x));
    memset(&y, 0, sizeof(y));
    memset(&z, 0, sizeof(z));
    int result = kinova_api_.getEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get current end effector offset.", result);
    }
}


// MARK: Angular Control

/**
 * @brief This function sets the robotical arm in angular control mode.
 * If robot is not in motion, change control model to Angular control
 */
void KinovaComm2::setAngularControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    TrajectoryFIFO trajectory_fifo;
    memset(&trajectory_fifo, 0, sizeof(trajectory_fifo));
    getGlobalTrajectoryInfo(trajectory_fifo);
    if(trajectory_fifo.TrajectoryCount > 0)
    {
        RCLCPP_WARN(node_->get_logger(), "Current tranjectory count is %d, Please wait the trajectory to finish to swich to Angular control.", trajectory_fifo.TrajectoryCount);
        return;
    }
    int result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set angular control", result);
    }
}


/**
 * @brief This function get the angular command of all actuators.
 * @param An AngularPosition struct containing the values. Units are degrees. angular_command {AngularInfo, FingersPosition}
 * @return
 *        - @ref NO_ERROR_KINOVA if operation is a success
 *        - @ref ERROR_API_NOT_INITIALIZED if the API has not been initialized. To initialize it, call the InitAPI() function.
 */
void KinovaComm2::getAngularCommand(AngularPosition &angular_command)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&angular_command, 0, sizeof(angular_command));
    int result = kinova_api_.getAngularCommand(angular_command);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular command", result);
    }
}


/**
 * @brief This function returns the angular position of the robotical arm's end effector.
 * @param angles A structure that contains the position of each actuator. Unit in degrees.
 */
void KinovaComm2::getJointAngles(KinovaAngles &angles)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_angles;
    memset(&kinova_angles, 0, sizeof(kinova_angles));  // zero structure

    int result = kinova_api_.getAngularPosition(kinova_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular position", result);
    }
    angles = KinovaAngles(kinova_angles.Actuators);
}


/**
 * @brief Sends a joint angle command to the Kinova arm.
 * This function sends trajectory point(Angular) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * @param angles target joint angle to set, type float, unit in degree
 * @param timeout default value 0.0, not used.
 * @param push default true, errase all trajectory before request motion..
 */
void KinovaComm2::setJointAngles(const KinovaAngles &angles, double speedJoint123, double speedJoint4567, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        RCLCPP_WARN(node_->get_logger(), "The angles could not be set because the arm is stopped");

        // ROS_WARN_STREAM("In class [" << typeid(*this).name() << "], function ["<< __FUNCTION__ << "]: The angles could not be set because the arm is stopped" << std::endl);
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_joint;
    kinova_joint.InitStruct();
    memset(&kinova_joint, 0, sizeof(kinova_joint));  // zero structure

    if (push)
    {
        result = kinova_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw KinovaCommException("Could not erase trajectories", result);
        }
    }

    //startAPI();

    result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set angular control", result);
    }

    kinova_joint.Position.Delay = 0.0;
    kinova_joint.Position.Type = ANGULAR_POSITION;
    kinova_joint.Position.Actuators = angles;
    kinova_joint.Limitations.speedParameter1 = speedJoint123;
    kinova_joint.Limitations.speedParameter2 = speedJoint4567;
    kinova_joint.LimitationsActive = 1;

    result = kinova_api_.sendAdvanceTrajectory(kinova_joint);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint angle trajectory", result);
    }

}


/**
 * @brief This function get the velocity of each actuator.
 * @param vels A kinovaAngles structure contains joint velocity of each actuator. Unit in degrees/second.
 */
void KinovaComm2::getJointVelocities(KinovaAngles &vels)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_vels;
    memset(&kinova_vels, 0, sizeof(kinova_vels));  // zero structure

    int result = kinova_api_.getAngularVelocity(kinova_vels);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular velocity", result);
    }

    vels = KinovaAngles(kinova_vels.Actuators);

    //velocities reported back by firmware seem to be half of actual value
    vels.Actuator1 = vels.Actuator1*2;
    vels.Actuator2 = vels.Actuator2*2;
    vels.Actuator3 = vels.Actuator3*2;
    vels.Actuator4 = vels.Actuator4*2;
    vels.Actuator5 = vels.Actuator5*2;
    vels.Actuator6 = vels.Actuator6*2;
    vels.Actuator7 = vels.Actuator7*2;
}


/**
 * @brief This function controls robot with joint velocity.
 * This function sends trajectory point(ANGULAR_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * @param joint_vel joint velocity in degree/second
 */
void KinovaComm2::setJointVelocities(const AngularInfo &joint_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        RCLCPP_INFO(node_->get_logger(), "The velocities could not be set because the arm is stopped");
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = ANGULAR_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.Actuators = joint_vel;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint velocity trajectory", result);
    }
}


/**
 * @brief This function get the accelerometer values of each actuator. It does not directly refer to the angular acceleration.
 * @param joint_acc An AngularAcceleration struct containing the values. Units are in G
 */
void KinovaComm2::getJointAccelerations(AngularAcceleration &joint_acc)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&joint_acc, 0, sizeof(joint_acc));
    int result = kinova_api_.getAngularAcceleration(joint_acc);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get joint acceleration.", result);
    }
}


/**
 * @brief This function returns the torque of each actuator.
 * @param tqs A structure that contains the torque of each actuator. Unit is Newton * meter
 */
void KinovaComm2::getJointTorques(KinovaAngles &tqs)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_tqs;
    memset(&kinova_tqs, 0, sizeof(kinova_tqs));  // zero structure

    int result = kinova_api_.getAngularForce(kinova_tqs);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the joint torques", result);
    }

    tqs = KinovaAngles(kinova_tqs.Actuators);
}


void KinovaComm2::setJointTorques(float joint_torque[])
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        RCLCPP_INFO(node_->get_logger(), "The joint torques could not be set because the arm is stopped");
        return;
    }

    //memset(&joint_torque, 0, sizeof(joint_torque));  // zero structure

    //startAPI();
    //RCLCPP_INFO(node_->get_logger(), "Torque %f %f %f %f %f %f %f ", joint_torque[0],joint_torque[1],joint_torque[2],
     //       joint_torque[3],joint_torque[4],joint_torque[5],joint_torque[6]);
    int result = kinova_api_.sendAngularTorqueCommand(joint_torque);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send joint torques", result);
    }
}


/**
 * @brief This function returns the current that each actuator consume on the main supply.
 * @param anguler_current A structure that contains the current of each actuator and finger. Unit in A.
 */
void KinovaComm2::getJointCurrent(AngularPosition &anguler_current)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&anguler_current, 0, sizeof(anguler_current));
    int result = kinova_api_.getAngularCurrent(anguler_current);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the current in joints", result);
    }
}


/**
 * @brief Dumps the current joint angles onto the screen.
 * @param angles A structure contains six joint angles. Unit in degrees
 */
void KinovaComm2::printAngles(const KinovaAngles &angles)
{
    RCLCPP_INFO(node_->get_logger(), "Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f, J7: %f \n",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6,
             angles.Actuator7);

    RCLCPP_INFO(node_->get_logger(), "Joint angles (rad) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f, J7: %f \n",
             angles.Actuator1/180.0*M_PI, angles.Actuator2/180.0*M_PI, angles.Actuator3/180.0*M_PI,
             angles.Actuator4/180.0*M_PI, angles.Actuator5/180.0*M_PI, angles.Actuator6/180.0*M_PI,
             angles.Actuator7/180.0*M_PI);
}


// MAKR: Torque Control
// void KinovaComm2::SetTorqueControlState(int state)
// {
//     int result;
//     if (state)
//     {
//         RCLCPP_INFO(node_->get_logger(), "Switching to torque control");
//         result = kinova_api_.switchTrajectoryTorque(TORQUE);
//     }
//     else
//     {
//         RCLCPP_INFO(node_->get_logger(), "Switching to position control");
//         result = kinova_api_.switchTrajectoryTorque(POSITION);
//     }
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set the torque control state", result);
//     }
// }


// /**
//   *@brief Set zero torque for all joints
//  */
// void KinovaComm2::setZeroTorque()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     int actuator_address[] = {16,17,18,19,20,21,25};
//     int result;
//     for (int i=0;i<num_joints_;i++)
//     {
//         result = kinova_api_.setTorqueZero(actuator_address[i]);
//     }
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set zero torques", result);
//     }
//     ROS_WARN("Torques for all joints set to zero");
// }


void KinovaComm2::getGravityCompensatedTorques(KinovaAngles &tqs)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_tqs;
    memset(&kinova_tqs, 0, sizeof(kinova_tqs));  // zero structure

    int result = kinova_api_.getAngularForceGravityFree(kinova_tqs);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the joint torques", result);
    }

    tqs = KinovaAngles(kinova_tqs.Actuators);
}


//Set torque parameters

// /** @brief Sets COM and COMxyz for all links
//   * @arg command[42] - {m1,m2..m7,x1,x2,..x7,y1,y2,...,y7,z1,z2,...z7}
// //! */
// void KinovaComm2::setRobotCOMParam(GRAVITY_TYPE type,std::vector<float> params)
// {
//     float com_parameters[GRAVITY_PARAM_SIZE];
//     memset(&com_parameters, 0, sizeof(com_parameters));
//     std::ostringstream com_params;
//     com_params<<"Setting COM parameters to ";
//     for (int i=0; i<params.size(); i++)
//     {
//         com_parameters[i] = params[i];
//         com_params<<params[i]<<", ";
//     }
//     ROS_INFO_STREAM(com_params.str());
//     int result;
//     if (type == MANUAL_INPUT)
//         result = kinova_api_.setGravityManualInputParam(com_parameters);
//     else
//         result = kinova_api_.setGravityOptimalZParam(com_parameters);
//     if (result != NO_ERROR_KINOVA && result!=2005)
//     {
//         throw KinovaCommException("Could not set the COM parameters", result);
//     }
// }


// /**
//  * @brief This function set the angular torque's maximum and minimum values.
//  * @param min A struct that contains all angular minimum values. (Unit: N * m)
//  * @param max 6 A struct that contains all angular max values.     (Unit: N * m)
//  */
// void KinovaComm2::setJointTorqueMinMax(AngularInfo &min, AngularInfo &max)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     RCLCPP_INFO(node_->get_logger(), "Setting min torues - %f %f %f %f %f %f %f", min.Actuator1,
//               min.Actuator2,min.Actuator3,min.Actuator4,min.Actuator5,
//               min.Actuator6,min.Actuator7);
//     RCLCPP_INFO(node_->get_logger(), "Setting max torues - %f %f %f %f %f %f %f", max.Actuator1,
//               max.Actuator2,max.Actuator3,max.Actuator4,max.Actuator5,
//               max.Actuator6,max.Actuator7);

//     int result = kinova_api_.setAngularTorqueMinMax(min, max);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set the limits for joint torques", result);
//     }
// }


// /**
//  * @brief setPayload
//  * @param payload Array - Mass, COMx, COMy, COMz
//  */
// void KinovaComm2::setPayload(std::vector<float> payload)
// {
//     float payload_[4];
//     std::copy(payload.begin(), payload.end(), payload_);
//     RCLCPP_INFO(node_->get_logger(), "Payload set to - %f %f %f %f", payload_[0],payload_[1],
//             payload_[2],payload_[3]);
//     int result = kinova_api_.setGravityPayload(payload_);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set the gravity payload", result);
//     }
// }


// /**
//  * @brief Safety factor defines a velocity threshold at which torque control switches to position control
//  * @param factor between 0 and 1
//  */
// void KinovaComm2::setTorquesControlSafetyFactor(float factor)
// {
//     RCLCPP_INFO(node_->get_logger(), "Setting torque safety factor to %f", factor);
//     int result = kinova_api_.setTorqueSafetyFactor(factor);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set torque safety factor", result);
//     }
// }


// /**
// * @brief This function is used to run a sequence to estimate the optimal gravity parameters when the robot is
// * standing (Z).

// The arm must be in Trajectory-Position mode before to launch the procedure.

// Before using this procedure, you should make sure that the torque sensors are well calibrated. This procedure is
// explained in the user guide and in the Advanced Specification Guide.

// When the program is launched, the robot will execute a trajectory. The user must remain alert and turn off the
// robot if something wrong occurs (for example if the robot collides with an object). When the program ends, it will
// output the parameters in the console and in a text file named “ParametersOptimal_Z.txt” in the program folder.
// These parameters can then be sent as input to the function SetOptimalZParam().
// *
// * @param type The robot type
// * @param OptimalzParam The result of the sequence
// */
// int KinovaComm2::runCOMParameterEstimation(ROBOT_TYPE type)
// {
//     float COMparams[GRAVITY_PARAM_SIZE];
//     memset(&COMparams[0],0,sizeof(COMparams));
//     int result;
//     if(type == SPHERICAL_7DOF_SERVICE)
//     {
//         RCLCPP_INFO(node_->get_logger(), "Running 7 dof robot COM estimation sequence");
//         result = kinova_api_.runGravityZEstimationSequence7DOF(type,COMparams);
//     }
//     else
//     {
//         double params[OPTIMAL_Z_PARAM_SIZE];
//         RCLCPP_INFO(node_->get_logger(), "Running COM estimation sequence");
//         result = kinova_api_.runGravityZEstimationSequence(type,params);
//         for (int i=0;i<OPTIMAL_Z_PARAM_SIZE;i++)
//             COMparams[i] = (float)params[i];
//     }
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not launch COM parameter estimation sequence", result);
//     }
//     result = kinova_api_.setGravityOptimalZParam(COMparams);
//     if (result != NO_ERROR_KINOVA && result!=2005)
//     {
//         throw KinovaCommException("Could not set COM Parameters", result);
//     }

//     return 1;
// }



// int KinovaComm2::sendCartesianForceCommand(float force_cmd[COMMAND_SIZE])
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     if (isStopped())
//     {
//         RCLCPP_INFO(node_->get_logger(), "The force cmd could not be set because the arm is stopped");
//         return 0;
//     }

//     //memset(&joint_torque, 0, sizeof(joint_torque));  // zero structure

//     //startAPI();
//     //RCLCPP_INFO(node_->get_logger(), "Force %f %f %f %f %f %f", force_cmd[0],force_cmd[1],force_cmd[2],
//      //       force_cmd[3],force_cmd[4],force_cmd[5]);
//     int result = kinova_api_.sendCartesianForceCommand(force_cmd);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not send force cmd", result);
//     }
//     return result;
// }


// MARK: Cartesian Control

// /**
//  * @brief This function sets the robotical arm in cartesian control mode if this is possible.
//  * If robot is not in motion, change control model to Cartesian control
//  */
// void KinovaComm2::setCartesianControl()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     TrajectoryFIFO trajectory_fifo;
//     memset(&trajectory_fifo, 0, sizeof(trajectory_fifo));
//     getGlobalTrajectoryInfo(trajectory_fifo);
//     if(trajectory_fifo.TrajectoryCount > 0)
//     {
//         ROS_WARN("Current tranjectory count is %d, Please wait the trajectory to finish to swich to Cartesian control.", trajectory_fifo.TrajectoryCount);
//         return;
//     }
//     int result = kinova_api_.setCartesianControl();
//     ROS_WARN("%d", result);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set Cartesian control", result);
//     }
// }


// /**
//  * @brief This function get the cartesian command of the end effector. The Cartesian orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
//  * @param cartesian_command An CartesianPosition struct containing the values of end-effector and fingers.
//  *
//  * @htmlonly
//  *
//  * <table border="0" cellspacing="10">
//  * <tr>
//  * <th>Member</th>
//  * <th>Unit</th>
//  * </tr>
//  * <tr><td width="50">X</td><td>meter</td></tr>
//  * <tr><td>Y</td><td>meter</td></tsr>
//  * <tr><td>Z</td><td>meter</td></tr>
//  * <tr><td>Theta X</td><td>RAD</td></tr>
//  * <tr><td>Theta Y</td><td>RAD</td></tr>
//  * <tr><td>Theta Z</td><td>RAD</td></tr>
//  * <tr><td>Finger 1</td><td>No unit</td></tr>
//  * <tr><td>Finger 2</td><td>No unit</td></tr>
//  * <tr><td>Finger 3</td><td>No unit</td></tr>
//  * </table>
//  *
//  * @endhtmlonly
//  */
// void KinovaComm2::getCartesianCommand(CartesianPosition &cartesian_command)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     memset(&cartesian_command, 0, sizeof(cartesian_command));
//     int result = kinova_api_.getCartesianCommand(cartesian_command);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not get the Cartesian command", result);
//     }
// }


// /**
//  * @brief This function returns the cartesian position of the robotical arm's end effector.
//  * In KinovaPose, orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
//  * @param position pose in [X,Y,Z,ThetaX,ThetaY,ThetaZ] form, Units in meters and radians.
//  */
// void KinovaComm2::getCartesianPosition(KinovaPose &position)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     CartesianPosition kinova_cartesian_position;
//     memset(&kinova_cartesian_position, 0, sizeof(kinova_cartesian_position));  // zero structure

//     int result = kinova_api_.getCartesianPosition(kinova_cartesian_position);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not get the Cartesian position", result);
//     }

// //    ROS_INFO_STREAM_ONCE("Cartesian pose in [X,Y,Z, ThetaX, ThetaY, ThetaZ] is : " << kinova_cartesian_position.Coordinates.X << ", "
// //                    << kinova_cartesian_position.Coordinates.Y << ", "
// //                    << kinova_cartesian_position.Coordinates.Z << ", "
// //                    << kinova_cartesian_position.Coordinates.ThetaX << ", "
// //                    << kinova_cartesian_position.Coordinates.ThetaY << ", "
// //                    << kinova_cartesian_position.Coordinates.ThetaZ << std::endl);

//     position = KinovaPose(kinova_cartesian_position.Coordinates);
// }


// /**
//  * @brief Sends a cartesian coordinate trajectory to the Kinova arm.
//  * This function sends trajectory point(Cartesian) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendBasicTrajectory() is called in api to complete the motion.
//  * In KinovaPose, orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
//  * @param pose target pose of robot [X,Y,Z, ThetaX, ThetaY, ThetaZ], unit in meter and radians.
//  * @param timeout default 0.0, not used.
//  * @param push default false, does not erase previous trajectory point before new motion. If you want to erase all trajectory before request motion, set to true..
//  */
// void KinovaComm2::setCartesianPosition(const KinovaPose &pose, int timeout, bool push)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);

//     if (isStopped())
//     {
//         ROS_WARN_STREAM("In class [" << typeid(*this).name() << "], function ["<< __FUNCTION__ << "]: The pose could not be set because the arm is stopped" << std::endl);
//         return;
//     }

//     int result = NO_ERROR_KINOVA;
//     TrajectoryPoint kinova_pose;
//     kinova_pose.InitStruct();
//     memset(&kinova_pose, 0, sizeof(kinova_pose));  // zero structure

//     if (push)
//     {
//         result = kinova_api_.eraseAllTrajectories();
//         if (result != NO_ERROR_KINOVA)
//         {
//             throw KinovaCommException("Could not erase trajectories", result);
//         }
//     }

//     //startAPI();

//     result = kinova_api_.setCartesianControl();
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set Cartesian control", result);
//     }

//     kinova_pose.Position.Delay = 0.0;
//     kinova_pose.Position.Type = CARTESIAN_POSITION;
// //    kinova_pose.Position.HandMode = HAND_NOMOVEMENT;
//     kinova_pose.Position.CartesianPosition = pose;

//     result = kinova_api_.sendBasicTrajectory(kinova_pose);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not send basic trajectory", result);
//     }
// }


/**
 * @brief Linear and angular velocity control in Cartesian space
 * This function sends trajectory point(CARTESIAN_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
 * @param velocities unit are meter/second for linear velocity and radians/second for "Omega".
 */
void KinovaComm2::setCartesianVelocities(const CartesianInfo &velocities)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        RCLCPP_INFO(node_->get_logger(), "The cartesian velocities could not be set because the arm is stopped");
        kinova_api_.eraseAllTrajectories();
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.CartesianPosition = velocities;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced Cartesian velocity trajectory", result);
    }
}


// /**
//  * @brief Linear and angular velocity control in Cartesian space
//  * This function sends trajectory point(CARTESIAN_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
//  * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
//  * @param velocities unit are meter/second for linear velocity and radians/second for "Omega".
//  * @param fingers finger positions to reach at the same time as moving, in closure percentage
//  */
// void KinovaComm2::setCartesianVelocitiesWithFingers(const CartesianInfo &velocities, const FingerAngles& fingers)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);

//     if (isStopped())
//     {
//         RCLCPP_INFO(node_->get_logger(), "The cartesian velocities could not be set because the arm is stopped");
//         kinova_api_.eraseAllTrajectories();
//         return;
//     }

//     TrajectoryPoint kinova_velocity;
//     kinova_velocity.InitStruct();

//     memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

//     //startAPI();
//     kinova_velocity.Position.Type = CARTESIAN_VELOCITY;

//     // confusingly, velocity is passed in the position struct
//     kinova_velocity.Position.CartesianPosition = velocities;

//     // Fill fingers
//     kinova_velocity.Position.Fingers = fingers;
//     kinova_velocity.Position.HandMode = POSITION_MODE;

//     int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not send advanced Cartesian velocity trajectory", result);
//     }
// }


// /**
//  * @brief Linear and angular velocity control in Cartesian space
//  * This function sends trajectory point(CARTESIAN_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
//  * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
//  * @param velocities unit are meter/second for linear velocity and radians/second for "Omega".
//  * @param fingers finger velocities, unit should be steps/second (tested it, seems to be slower than that)
//  */
// void KinovaComm2::setCartesianVelocitiesWithFingerVelocity(const CartesianInfo &velocities, const FingerAngles& fingers)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);

//     if (isStopped())
//     {
//         RCLCPP_INFO(node_->get_logger(), "The cartesian velocities could not be set because the arm is stopped");
//         kinova_api_.eraseAllTrajectories();
//         return;
//     }

//     TrajectoryPoint kinova_velocity;
//     kinova_velocity.InitStruct();

//     memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

//     //startAPI();
//     kinova_velocity.Position.Type = CARTESIAN_VELOCITY;

//     // confusingly, velocity is passed in the position struct
//     kinova_velocity.Position.CartesianPosition = velocities;

//     // Fill fingers
//     kinova_velocity.Position.Fingers = fingers;
//     kinova_velocity.Position.HandMode = VELOCITY_MODE;

//     int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not send advanced Cartesian velocity trajectory", result);
//     }
// }


// /**
//  * @brief This function returns the max translation(X, Y and Z) velocity of the robot's end effector in ClientConfigurations
//  * @return MaxTranslationVelocity Unit in meter per second
//  */
// float KinovaComm2::getMaxTranslationVelocity()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     ClientConfigurations configuration;
//     getConfig(configuration);
//     return configuration.MaxTranslationVelocity;
// }


// /**
//  * @brief This function set the max translation(X, Y and Z) velocity of the robot's end effector in ClientConfigurations
//  * @param max_trans_vel Unit in meter per second
//  */
// void KinovaComm2::setMaxTranslationVelocity(const float &max_trans_vel)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     ClientConfigurations configuration;
//     getConfig(configuration);
//     usleep(100000);
//     configuration.MaxTranslationVelocity = max_trans_vel;
//     setConfig(configuration);
// }


// /**
//  * @brief This function get max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
//  * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
//  * @return Unit in rad/second
//  */
// float KinovaComm2::getMaxOrientationVelocity()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     ClientConfigurations configuration;
//     getConfig(configuration);
//     return configuration.MaxOrientationVelocity;
// }


// /**
//  * @brief This function set max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
//  * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
//  * @param max_orient_vel Unit in rad/second
//  */
// void KinovaComm2::setMaxOrientationVelocity(const float &max_orient_vel)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     ClientConfigurations configuration;
//     getConfig(configuration);
//     usleep(100000);
//     configuration.MaxOrientationVelocity = max_orient_vel;
//     setConfig(configuration);
// }


// /**
//  * @brief This function returns the cartesian wrench at the robotical arm's end effector.
//  * @param cart_force A structure that contains the wrench vector at the end effector. Unit in N and N * m.
//  */
// void KinovaComm2::getCartesianForce(KinovaPose &cart_force)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     CartesianPosition kinova_cartesian_force;
//     memset(&kinova_cartesian_force, 0, sizeof(kinova_cartesian_force));  // zero structure

//     int result = kinova_api_.getCartesianForce(kinova_cartesian_force);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not get the Cartesian force", result);
//     }

//     cart_force = KinovaPose(kinova_cartesian_force.Coordinates);
// }


// /**
//  * @brief This function set the Cartesian force's maximum and minimum values.
//  * @param min A struct that contains all Cartesian minimum values. (Translation unit: N     Orientation unit: N * m)
//  * @param max A struct that contains all Cartesian maximum values. (Translation unit: N     Orientation unit: N * m)
//  */
// void KinovaComm2::setCartesianForceMinMax(const CartesianInfo &min, const CartesianInfo& max)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     int result = kinova_api_.setCartesianForceMinMax(min, max);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set cartesian min/max force.", result);
//     }
// }


// /**
//  * @brief This function set the Cartesian inertia and damping value.
//  * @param inertia A struct that contains all Cartesian inertia values. (Translation unit: Kg,  Orientation unit: Kg * m^2)
//  * @param damping A struct that contains all Cartesian damping values. (Translation unit: (N * s) / m,   Orientation unit: (N * s) / RAD)
//  */
// void KinovaComm2::setCartesianInertiaDamping(const CartesianInfo &inertia, const CartesianInfo& damping)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     int result = kinova_api_.setCartesianInertiaDamping(inertia, damping);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set cartesian inertia and damping", result);
//     }
// }


/**
 * @brief Dumps the current cartesian pose onto the screen.
 * In KinovaPose, orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
 * @param position in [X,Y,Z,ThetaX,ThetaY,ThetaZ], where orientation is using Euler-ZYX convention.
 */
void KinovaComm2::printPosition(const KinovaPose &position)
{
    RCLCPP_INFO(node_->get_logger(), "Arm position\n"
             "\tposition (m) -- x: %f, y: %f z: %f\n"
             "\trotation (rad) -- theta_x: %f, theta_y: %f, theta_z: %f",
             position.X, position.Y, position.Z,
             position.ThetaX, position.ThetaY, position.ThetaZ);
}


// /**
//  * @brief This function extract the UserPosition from trajectory.
//  * @param user_position contains POSITION_TYPE(Angular/Cartesian position/velocity, etc), CartesianInfo and AngularInfo, finger positions etc.
//  */
// void KinovaComm2::getUserCommand(UserPosition &user_position)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     memset(&user_position, 0, sizeof(user_position));
//     TrajectoryPoint trajecory_point;
//     int result = kinova_api_.getActualTrajectoryInfo(trajecory_point);
//     if(result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not get trajecory information", result);
//     }
//     user_position = trajecory_point.Position;
// }


/**
 * @brief This function returns informations about the trajectories FIFO stored inside the robotical arm. Detail of trajectory point is not stored in trajectoryFIFO.
 * @param trajectoryFIFO The structure containing the FIFO's informations: {TrajectoryCount; UsedPercentage; MaxSize}.
 */
void KinovaComm2::getGlobalTrajectoryInfo(TrajectoryFIFO &trajectoryFIFO)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&trajectoryFIFO, 0, sizeof(trajectoryFIFO));
    int result = kinova_api_.getGlobalTrajectoryInfo(trajectoryFIFO);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get trjectoryFIFO.", result);
    }
}


// /**
//  * @brief This function erases all the trajectories inside the robotical arm's FIFO. All trajectory will be cleared including angular, cartesian and fingers.
//  */
// void KinovaComm2::eraseAllTrajectories()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     int result = kinova_api_.eraseAllTrajectories();
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not errase all trajectories.", result);
//     }
// }


// MARK: Fingers
// /**
//  * @brief This function get number of fingers. number of fingers determined by robotType. 3 fingers for robotType(0,3,4,6) and 2 fingers for robotType(1,2,5)
//  * @return returns number of fingers.
//  */
// int KinovaComm2::numFingers() const
// {
//     return num_fingers_;
// }


/**
 * @brief This function obtain the joint position of fingers.
 * @param fingers in degrees, range from 0 to 6800
 */
void KinovaComm2::getFingerPositions(FingerAngles &fingers)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_position;
    memset(&kinova_cartesian_position, 0, sizeof(kinova_cartesian_position));  // zero structure

    int result = kinova_api_.getCartesianPosition(kinova_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get Cartesian finger position", result);
    }

    if (num_fingers_ == 2)
    {
        kinova_cartesian_position.Fingers.Finger3 = 0.0;
    }

    fingers = FingerAngles(kinova_cartesian_position.Fingers);
}


// /**
//  * @brief This function sets the finger positions
//  * The new finger position, combined with current joint values are constructed as a trajectory point. sendAdvancedTrajectory() is called in api to complete the motion.
//  * @param fingers in degrees from 0 to about 6800
//  * @param timeout timeout default 0.0, not used.
//  * @param push default true, errase all trajectory before request motion.
//  */
// void KinovaComm2::setFingerPositions(const FingerAngles &fingers, int timeout, bool push)
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);

//     if (isStopped())
//     {
//         RCLCPP_INFO(node_->get_logger(), "The fingers could not be set because the arm is stopped");
//         return;
//     }

//     int result = NO_ERROR_KINOVA;
//     int control_type;
//     result=kinova_api_.getControlType(control_type); // are we currently in angular or Cartesian mode? Response	0 = Cartesian control type, 1 = Angular control type.


//     //initialize the trajectory point. same initialization for an angular or Cartesian point
//     TrajectoryPoint kinova_point;
//     kinova_point.InitStruct();
//     memset(&kinova_point, 0, sizeof(kinova_point));  // zero structure

//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not get the current control type", result);
//     }
//     else
//     {
// 	if (push)
//     	{
//         	result = kinova_api_.eraseAllTrajectories();
//         	if (result != NO_ERROR_KINOVA)
//         	{
//            		throw KinovaCommException("Could not erase trajectories", result);
//         	}
//     	}
// 	// Initialize Cartesian control of the fingers
// 	kinova_point.Position.HandMode = POSITION_MODE;
// 	kinova_point.Position.Fingers = fingers;
// 	kinova_point.Position.Delay = 0.0;
// 	kinova_point.LimitationsActive = 0;
// 	if(control_type==0) //Cartesian
// 	{
// 		kinova_point.Position.Type = CARTESIAN_POSITION;
// 		CartesianPosition pose;
//                 memset(&pose, 0, sizeof(pose));  // zero structure
// 		result = kinova_api_.getCartesianCommand(pose);
//     		if (result != NO_ERROR_KINOVA)
//     		{
//         		throw KinovaCommException("Could not get the Cartesian position", result);
//     		}
// 		kinova_point.Position.CartesianPosition=pose.Coordinates;
// 	}
//         else if(control_type==1) //angular
// 	{
// 		kinova_point.Position.Type = ANGULAR_POSITION;
// 		AngularPosition joint_angles;
//     		memset(&joint_angles, 0, sizeof(joint_angles));  // zero structure
// 		result = kinova_api_.getAngularCommand(joint_angles);
//     		if (result != NO_ERROR_KINOVA)
//     		{
//         		throw KinovaCommException("Could not get the angular position", result);
//     		}
// 		kinova_point.Position.Actuators = joint_angles.Actuators;
// 	}
// 	else
// 	{
// 		throw KinovaCommException("Wrong control type", result);
// 	}
//     }


//     // getAngularPosition will cause arm drop
//     // result = kinova_api_.getAngularPosition(joint_angles);

//     result = kinova_api_.sendBasicTrajectory(kinova_point);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not send advanced finger trajectory", result);
//     }
// }


// /**
//  * @brief Dumps the current finger agnles onto the screen.
//  * @param fingers Unit in degrees 0 to 6800
//  */
// void KinovaComm2::printFingers(const FingersPosition &fingers)
// {
//     RCLCPP_INFO(node_->get_logger(), "Finger joint value -- F1: %f, F2: %f, F3: %f",
//              fingers.Finger1, fingers.Finger2, fingers.Finger3);
// }


/**
 * @brief This function initializes the fingers of the robotical arm. After the initialization, the robotical arm is in angular control mode. If you want to use the cartesian control mode, use the function setCartesianControl().
 * Move fingers to the full-open position to initialize them for use.
 * @warning This routine requires firmware version 5.05.x (or higher).
 */
void KinovaComm2::initFingers(void)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing fingers...this will take a few seconds and the fingers should open completely");
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.initFingers();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not init fingers", result);
    }
    return;
}




// MARK: Cartesian Admittance Control

// /**
//  * @brief This function activates the reactive force control for admittance control. Admittance control may be applied to joint or Cartesian depending to the control mode.
//  * @warning You can only use this function if your robotic device has torque sensors on it. Also, the robotic device must be in a standard vertical position.
//  */
// void KinovaComm2::startForceControl()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     int result = kinova_api_.startForceControl();
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not start force control.", result);
//     }
// }


// /**
//  * @brief This function stops the admittance control. Admittance control may be applied to joint or Cartesian depending to the control mode.
//  */
// void KinovaComm2::stopForceControl()
// {
//     boost::recursive_mutex::scoped_lock lock(api_mutex_);
//     int result = kinova_api_.stopForceControl();
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not stop force control.", result);
//     }
// }

// int KinovaComm2::SelfCollisionAvoidanceInCartesianMode(int state)
// {
//     int result = kinova_api_.ActivateCollisionAutomaticAvoidance(state);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set the self collision avoidance in cartesian mode", result);
//     }

//     return 1;
// }


// int KinovaComm2::SingularityAvoidanceInCartesianMode(int state)
// {
//     int result = kinova_api_.ActivateSingularityAutomaticAvoidance(state);
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set the singularity avoidance in cartesian mode", result);
//     }

//     return 1;
// }

// MARK: 7 DoF
// int KinovaComm2::SetRedundantJointNullSpaceMotion(int state)
// {
//     RCLCPP_INFO(node_->get_logger(), "Setting null space mode to %d",state);
//     int result;
//     if (state)
//         result = kinova_api_.StartRedundantJointNullSpaceMotion();
//     else
//         result = kinova_api_.StopRedundantJointNullSpaceMotion();
//     if (result != NO_ERROR_KINOVA)
//     {
//         throw KinovaCommException("Could not set redundant joint null space mode", result);
//     }

//     return 1;
// }


// int KinovaComm2::SetRedundancyResolutionToleastSquares(int state)
// {
//     //Not Available in API
//     return 1;
// }


}  // namespace kinova