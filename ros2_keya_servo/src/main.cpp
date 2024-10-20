/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include <math.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "can_msgs/msg/frame.hpp"

#include "ros2_keya_servo_msgs/msg/servo_command.hpp"
#include "ros2_keya_servo_msgs/msg/servo_status.hpp"
#include "ros2_keya_servo_msgs/srv/position.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Keya Motor CAN Commands
//
// The General format for CAN instructions are:
// Send Data: 0x0600000 + Controller ID (Hex)
// Received Data: 0x0580000 + Controller ID (Hex)
// Heartbeat: 0x0700000  + Controller ID (Hex)

// TODO: Commands must be sent every 1,000 ms or watchdog disables motor?

// Heartbeat format:
// 0x0700000 + ID, then 8 Data Bytes:
// [0][1] - Angle:  0-1,000 Degrees?
// [2][3] - Motor Speed: +/- rads/s? Degree/s?
// [4][5] - Rated Speed: 0-1,000
// [6][7] - Fault Code

// TODO: Conflicting data on bytes 4/5 & 2/3. One section says that 4/5 is the actual speed, one says it is the rated
// speed.

// Command Format:
// 0x0600000 + ID
// [0][1][2][3][4][5][6][7] Write Address
// [4][5][6][7][8][9][A][B] Command Specific

// The following Commands are supported:
//
// Motor Enable
// Send: 0x0600000 0x230D2001 0x00000000
// Return: 0x0580000 0x600D2000 0x00000000

// Motor Disable
// Send: 0x0600000 0x230C2001 0x00000000
// Return: 0x0580000 0x600C2000 0x00000000

// Set Motor Speed
// Send: 0x0600000 0x23002001 0xssssssss
// Return: 0x0580000 0x60002000 0x00000000
// Notes:
// Speed: 32bit signed integer, -1,000 to 1,000
// the 1,000 value is interpreted as percent of max set speed, where 1,000 == 100% of max speed#include <thread>
// 0x3E8 = 1000
// 0xFFFFFC18 = -1000
// So:
// 0x23002001 0x01F40000 is 500
// 0x23002001 0xC1C0FFFF is -500

// Position Control
// Send: 0x0600000 0x23022001 0xpppppppp

// Notes:
// Position: a 32bit signed integer
// A value of 10,000 represents 360 degrees
// Use the Formula: (Degrees * 10,000/360)
// For example:
// 76 degrees * (10,000/360) = 2,111 = 0x0000083F
// So:
// 0x0600000 0x23022001 0x083F0000

// Order is important, so the following commands need to be sent to enter position control mode:
// 0x0600000 0x230C2001 0x00000000  // Disable Motor
// 0x0600000 0x230D2001 0x00000000  // Enable Motor
// 0x0600000 0x23022001 0xpppppppp  // Set Position

constexpr inline float DegToRad(float deg)
{
  return (deg * M_PI) / 180.0f;
}

/**
 * @class KeyaServo
 * @brief A ROS 2 node for controlling a Keya servo motor.
 *
 * The KeyaServo class provides functionality to control a Keya servo motor using ROS 2.
 * It includes methods for reading parameters, initializing the node, handling various
 * callbacks, and sending CAN messages to the servo motor.
 *
 * @details
 * The KeyaServo class inherits from rclcpp::Node and provides the following functionalities:
 * - Reading and declaring parameters for calibration.
 * - Initializing the node with timers, subscribers, publishers, and services.
 * - Handling callbacks for position, torque, and servo commands.
 * - Sending various CAN messages to control the servo motor.
 *
 * The class also includes methods for running calibration, setting the position in degrees,
 * and sending heartbeat and status messages.
 *
 * @note The parameters `can_id` and `heartbeat_interval` are commented out and not currently being used.
 *
 * @param calibrate_step_size_degress The step size in degrees for calibration, default is 1.
 * @param calibrate_current_limit_amps The current limit in amps for calibration, default is 3.
 * @param calibrate_dwell_time_ms The dwell time in milliseconds for calibration, default is 100ms.
 * @param can_id_ The CAN ID for the servo motor, default is 1.
 * @param heartbeat_interval_ The interval for sending heartbeat messages, default is 500ms.
 * @param status_interval_ The interval for sending status messages, default is 1000ms.
 * @param current_position_ The current position of the servo motor.
 * @param degrees_to_encoder The conversion factor from degrees to encoder units.
 * @param calibration_current_set_angle The current set angle during calibration.
 * @param command_mode_ The current command mode of the servo motor.
 * @param servo_status_ The current status of the servo motor.
 * @param speed_or_position_ A flag indicating whether the servo is in speed or position mode.
 * @param command_subscriber_ The subscriber for servo command messages.
 * @param position_subscriber_ The subscriber for position messages.
 * @param torque_subscriber_ The subscriber for torque messages.
 * @param position_service_ The service for exposing the position of the servo motor.
 * @param status_publisher_ The publisher for servo status messages.
 * @param can_subscriber_ The subscriber for CAN messages.
 * @param can_publisher_ The publisher for CAN messages.
 * @param heartbeat_timer_ The timer for sending heartbeat messages.
 * @param status_timer_ The timer for sending status messages.
 */
class KeyaServo : public rclcpp::Node
{
public:
  KeyaServo() : Node("keya_servo")
  {
  }

  /**
   * @brief Reads and declares parameters for calibration. 
   *
   * This function declares and initializes the following parameters:
   * - `calibrate_step_size_degress` (int32_t): The step size in degrees for calibration, default is 1.
   * - `calibrate_current_limit_amps` (int32_t): The current limit in amps for calibration, default is 3.
   * - `calibrate_dwell_time_ms` (std::chrono::milliseconds): The dwell time in milliseconds for calibration, default is 100ms.
   *
   * Note: The parameters `can_id` and `heartbeat_interval` are commented out and not currently being used.
   */
  void read_params()
  {
    calibrate_step_size_degress = declare_parameter<std::int32_t>("calibrate_step_size_degress", 1);
    calibrate_current_limit_amps = declare_parameter<std::int32_t>("calibrate_current_limit_amps", 3);
    //calibrate_dwell_time_ms = declare_parameter<std::chrono::milliseconds>("calibrate_dwell_time_ms", 100ms);
    calibrate_dwell_time_ms = 100ms;

    // this->get_parameter_or("can_id", can_id_, 0x01);
    // this->get_parameter_or("heartbeat_interval", heartbeat_interval_, 1000ms);
  }

  /**
   * @brief Initializes the KeyaServo node.
   *
   * This function sets up various timers, subscribers, publishers, and services required for the KeyaServo node.
   * It performs the following actions:
   * - Creates a heartbeat timer to periodically send heartbeat messages.
   * - Creates a status timer to periodically send status messages.
   * - Subscribes to the CAN bus messages from the topic "/from_can_bus".
   * - Subscribes to the servo command messages from the topic "/servo/in/servo_command".
   * - Subscribes to the position messages from the topic "/servo/in/position".
   * - Subscribes to the torque messages from the topic "/servo/in/torque".
   * - Creates a publisher to send CAN bus messages to the topic "/to_can_bus".
   * - Creates a publisher to send servo status messages to the topic "/servo_status".
   * - Creates a service to expose an integer value for the position of the servo.
   * - Sends a disable command to the servo.
   * - Waits for 100 milliseconds.
   * - Sends a command to set the servo to position control mode.
   */
  void initialize()
  {
    heartbeat_timer_ = this->create_wall_timer(heartbeat_interval_, std::bind(&KeyaServo::sendHeartbeat, this));

    status_timer_ = this->create_wall_timer(status_interval_, std::bind(&KeyaServo::sendStatus, this));

    can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>("/from_can_bus", 10,
                                                                      std::bind(&KeyaServo::can_callback, this, _1));

    command_subscriber_ = this->create_subscription<ros2_keya_servo_msgs::msg::ServoCommand>(
        "/servo/in/servo_command", 1, std::bind(&KeyaServo::command_callback, this, _1));

    position_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/servo/in/position", 1, std::bind(&KeyaServo::position_callback, this, _1));

    torque_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/servo/in/torque", 1, std::bind(&KeyaServo::torque_callback, this, _1));

    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 500);

    status_publisher_ = this->create_publisher<ros2_keya_servo_msgs::msg::ServoStatus>("/servo_status", 10);

    // Create a ROS 2 service to expose an integer value for the position of the servo
    position_service_ = this->create_service<ros2_keya_servo_msgs::srv::Position>(
        "/position_service", std::bind(&KeyaServo::position_service_callback, this, _1, _2, _3));

    sendDisable();
    std::this_thread::sleep_for(100ms);
    sendPositionControlMode();
    // sendPositionReset();
  }

  /**
   * @brief Callback function for the position service.
   *
   * This function is called when a request is made to the position service. It sets the response
   * position to the current position.
   *
   * @param request_header A shared pointer to the request header.
   * @param request A shared pointer to the request message.
   * @param response A shared pointer to the response message.
   */
  void position_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<ros2_keya_servo_msgs::srv::Position::Request> request,
                                 const std::shared_ptr<ros2_keya_servo_msgs::srv::Position::Response> response)
  {
    (void)request_header;
    (void)request;
    response->position = current_position_;
  }

  /**
   * @brief Callback function for handling CAN messages.
   *
   * This function processes incoming CAN messages and updates the servo status
   * based on the message content. It handles two types of messages:
   * 1. Messages intended for this motor, identified by a specific CAN ID.
   * 2. Responses to queries, identified by another specific CAN ID.
   *
   * @param msg Shared pointer to the received CAN message.
   *
   * The function performs the following actions:
   * - If the message is for this motor, it extracts and updates the servo's
   *   position, speed, current draw, and fault code from the message data.
   * - If the message is a response to a query for current, it extracts and
   *   prints the motor current.
   */
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    // Check if the message is for this motor
    if (msg->id == (can_id_ + 0x07000000))
    {
      // Read position value from heartbeat and set it as current_position_
      servo_status_.angle = (msg->data[0] << 8) | msg->data[1];
      current_position_ = servo_status_.angle;

      servo_status_.speed = (msg->data[2] << 8) | msg->data[3];
      servo_status_.amp_draw = (msg->data[4] << 8) | msg->data[5];
      servo_status_.fault_code = (msg->data[6] << 8) | msg->data[7];

      // RCLCPP_INFO(this->get_logger(), "Position: %d, Speed: %d, Rated Speed: %d, Fault Code: %d", position, speed,
      // amp_draw, fault_code);
    }

    // Check if the message is a response to a query
    if (msg->id == (can_id_ + 0x05800000))
    {
      // Check if the message is a response to a query for current
      if (msg->data[1] == 00)
      {
        auto current = msg->data[4];
        if(verbose_)
          printf("motor current %u\n", current);
      }
    }
  }

  /**
   * @brief Callback function to handle servo commands.
   *
   * This function is called whenever a new servo command message is received. It processes
   * the command and performs the corresponding action based on the command type.
   *
   * @param msg Shared pointer to the received ServoCommand message.
   *
   * The function handles the following commands:
   * - ENABLE: Enables the servo and prints a message.
   * - DISABLE: Disables the servo, prints a message, and calls the sendDisable function.
   * - SET_CURRENT_AS_CENTER: Prints a message and starts a new thread to run calibration.
   * - RUN_CENTER_CALIBRATION: Prints a message indicating the start of center calibration.
   */
  void command_callback(const ros2_keya_servo_msgs::msg::ServoCommand::SharedPtr msg)
  {
    command_mode_ = (command_mode_enum_t)msg->command;

    switch (msg->command)
    { 
      case ros2_keya_servo_msgs::msg::ServoCommand::ENABLE:
        printf("command_callback(): ENABLE\n");
        break;
      case ros2_keya_servo_msgs::msg::ServoCommand::DISABLE:
        printf("command_callback(): DISABLE\n");
        void sendDisable();
        break;
      case ros2_keya_servo_msgs::msg::ServoCommand::SET_CURRENT_AS_CENTER:
        printf("command_callback(): SET_CURRENT_AS_CENTER\n");
      break;      
      case ros2_keya_servo_msgs::msg::ServoCommand::RUN_CENTER_CALIBRATION:
        printf("command_callback(): RUN_CENTER_CALIBRATION\n");
        // todo, wait for receipt of the current position before starting calibration
        std::thread thread1(std::bind(&KeyaServo::run_calibration, this, 1));
        thread1.detach();        
      break;      

    }
  }

  /**
   * @brief Sets the position in degrees.
   * 
   * This function converts the given degrees to an encoder position and sends the 
   * position to the hardware. It first enables the hardware, waits for a short 
   * duration, and then sends the position.
   * 
   * @param degrees The desired position in degrees.
   */
  void set_degrees(int32_t degrees)
  {
    sendEnable();
    std::this_thread::sleep_for(100ms);

    // int32_t position = (int32_t)((DegToRad(degrees) * 10000.0f) / 360.0f);
    current_position_ = degrees * degrees_to_encoder;

    sendSetPosition(current_position_);
  }

  /**
   * @brief Callback function to handle position messages.
   *
   * This function is called whenever a new position message is received. It sets
   * the degrees based on the received message data. The function currently 
   * returns immediately after setting the degrees.
   *
   * @param msg Shared pointer to the received Int32 message containing the position data.
   */
  void position_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    set_degrees(msg->data);
    return;

    sendEnable();
    std::this_thread::sleep_for(100ms);

    // int32_t position = (int32_t)((DegToRad(degrees) * 10000.0f) / 360.0f);
    current_position_ = msg->data * degrees_to_encoder;

    sendSetPosition(current_position_);
  }

  /**
   * @brief Callback function to handle torque messages.
   *
   * This function is called whenever a new torque message is received. It first
   * enables the system by calling sendEnable(), waits for 100 milliseconds, and
   * then sets the torque using the value from the received message.
   *
   * @param msg A shared pointer to the received torque message of type std_msgs::msg::Int32.
   */
  void torque_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    sendEnable();
    std::this_thread::sleep_for(100ms);

    int32_t torque = msg->data;

    sendSetTorque(torque);
  }

  /**
   * @brief Sends a CAN message to set the position control mode.
   * 
   * This function constructs a CAN message with a predefined data payload
   * and sends it using the `send_can_message` function. The CAN ID used
   * is offset by 0x06000000 from the base `can_id_`.
   */
  void sendPositionControlMode()
  {
    uint8_t data[8] = { 0x03, 0x0D, 0x20, 0x31, 0x00, 0x00, 0x00, 0x00 };
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message to reset the position.
   *
   * This function constructs a CAN message with a specific data payload
   * and sends it using the `send_can_message` function. The CAN message
   * is intended to reset the position of a device.
   *
   * The data payload is an array of 8 bytes with the following values:
   * - 0x23
   * - 0x0C
   * - 0x20
   * - 0x09
   * - 0x00
   * - 0x00
   * - 0x00
   * - 0x00
   *
   * The CAN ID used for the message is calculated by adding 0x06000000
   * to the `can_id_` member variable.
   */
  void sendPositionReset()
  {
    uint8_t data[8] = { 0x23, 0x0C, 0x20, 0x09, 0x00, 0x00, 0x00, 0x0 };
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a heartbeat message if the command mode is not disabled.
   * 
   * This function checks the current command mode and sends a CAN message
   * with a predefined data payload if the command mode is not set to DISABLE.
   * The CAN message ID is calculated by adding an offset to the base CAN ID.
   * 
   * @note If the command mode is DISABLE, the function returns immediately
   *       without sending any message.
   */
  void sendHeartbeat()
  {
    if (command_mode_ == DISABLE)
    {
      return;
    }

    uint8_t data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    send_can_message(can_id_ + 0x06000000, data);

    // sendCurrentQuery();
  }

  /**
   * @brief Publishes the current servo status.
   *
   * This function sends the current servo status by publishing a 
   * `ServoStatus` message using the `status_publisher_`. The message 
   * contains information about the servo's current position and other 
   * relevant status details.
   */
  void sendStatus()
  {
    // servo_status_.angle = current_position_;

    // auto status_p = std::make_unique<ros2_keya_servo_msgs::msg::ServoStatus>(servo_status_);

    // publish a servostatus message
    // status_publisher_->publish(std::move(status_p));

    status_publisher_->publish(servo_status_);
  }

  /**
   * @brief Sends an enable command via CAN bus.
   *
   * This function constructs a CAN message with a specific data payload
   * and sends it using the `send_can_message` function. The CAN message
   * ID is derived from the `can_id_` member variable with an offset of
   * 0x06000000.
   */
  void sendEnable()
  {
    uint8_t data[8] = { 0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message to disable a device.
   *
   * This function constructs a CAN message with a specific data payload
   * and sends it to the device using the CAN protocol. The message is
   * intended to disable the device.
   */
  void sendDisable()
  {
    uint8_t data[8] = { 0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message to query the current.
   * 
   * This function constructs a CAN message with a specific data payload and sends it
   * using the `send_can_message` function. The CAN ID used is `can_id_` offset by 
   * `0x06000000`.
   * 
   * The data payload is an array of 8 bytes initialized as follows:
   * - 0x40
   * - 0x00
   * - 0x21
   * - 0x01
   * - 0x00
   * - 0x00
   * - 0x00
   * - 0x00
   */
  void sendCurrentQuery()
  {
    uint8_t data[8] = { 0x40, 0x00, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00 };
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message to set the speed of a device.
   *
   * This function constructs a CAN message with the specified speed and sends it
   * to the device. The speed is encoded into the data array and sent using the
   * `send_can_message` function.
   *
   * @param speed The speed value to set, represented as a 32-bit integer.
   */
  void sendSetSpeed(int32_t speed)
  {
    uint8_t data[8];
    data[0] = 0x23;
    data[1] = 0x00;
    data[2] = 0x20;
    data[3] = 0x01;
    data[4] = (speed >> 8) & 0xFF;
    data[5] = speed & 0xFF;
    data[6] = (speed >> 24) & 0xFF;
    data[7] = (speed >> 16) & 0xFF;
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message to set the torque value.
   *
   * This function constructs a CAN message with the specified torque value and sends it.
   * The torque value is split into bytes and placed into the data array in a specific format.
   *
   * @param torque The torque value to be set, represented as a 32-bit integer.
   */
  void sendSetTorque(int32_t torque)
  {
    uint8_t data[8];
    data[0] = 0x23;
    data[1] = 0x01;
    data[2] = 0x20;
    data[3] = 0x01;
    data[4] = (torque >> 8) & 0xFF;
    data[5] = torque & 0xFF;
    data[6] = (torque >> 24) & 0xFF;
    data[7] = (torque >> 16) & 0xFF;
    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message to set the position.
   *
   * This function constructs a CAN message with the specified position and sends it.
   * The position is encoded into the data array and sent using the `send_can_message` function.
   *
   * @param position The desired position to set, represented as a 32-bit integer.
   */
  void sendSetPosition(int32_t position)
  {
    uint8_t data[8];
    data[0] = 0x23;
    data[1] = 0x02;
    data[2] = 0x20;
    data[3] = 0x01;
    data[4] = (position >> 8) & 0xFF;
    data[5] = position & 0xFF;
    data[6] = (position >> 24) & 0xFF;
    data[7] = (position >> 16) & 0xFF;

    send_can_message(can_id_ + 0x06000000, data);
  }

  /**
   * @brief Sends a CAN message with the specified ID and data.
   *
   * This function creates a CAN frame with the provided ID and data, and then publishes it.
   * The frame is configured with an extended ID, no remote transmission request, and no error flag.
   * The data length code (DLC) is set to 8, and the data array is populated with the provided data.
   *
   * @param id The identifier for the CAN frame.
   * @param data A pointer to an array of 8 bytes containing the data to be sent in the CAN frame.
   */
  void send_can_message(uint32_t id, uint8_t* data)
  {
    // Create a can frame with a heartbeat message
    can_msgs::msg::Frame frame;
    frame.id = id;
    frame.is_extended = true;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.dlc = 8;
    frame.data[0] = data[0];
    frame.data[1] = data[1];
    frame.data[2] = data[2];
    frame.data[3] = data[3];
    frame.data[4] = data[4];
    frame.data[5] = data[5];
    frame.data[6] = data[6];
    frame.data[7] = data[7];

    // Publish the frame
    can_publisher_->publish(frame);
  }

  /**
   * @brief Incrementally steps the servo motor until a current threshold is exceeded.
   *
   * This function continuously increments the servo motor angle by a specified
   * number of degrees and checks the current draw after each step. If the current
   * draw exceeds a predefined threshold, the function exits.
   *
   * @param incrementDegrees The number of degrees to increment the servo motor angle
   *                         in each step.
   */
  void step_to_stop(int incrementDegrees)
  {
    // find out current angle
    calibration_current_set_angle = servo_status_.angle;

    while (true)
    {
      calibration_current_set_angle += incrementDegrees;

      // step n degrees
      set_degrees(calibration_current_set_angle);

      if(verbose_)
        printf("step_to_stop(): set_degrees(%d)\n", calibration_current_set_angle);

      // wait for motor to stop
      std::this_thread::sleep_for(calibrate_dwell_time_ms);

      // request current
      sendCurrentQuery();

      // wait for current response
      std::this_thread::sleep_for(calibrate_dwell_time_ms);

      int32_t amps = servo_status_.amp_draw;

      // check current threshold

      if (abs(amps) > calibrate_current_limit_amps)
        return;
    }
  }

  /**
   * @brief Runs the calibration process for the motor.
   *
   * This function performs a calibration by moving the system in steps.
   * It first waits for a specified duration, then moves the system by a 
   * given step size in one direction, and then moves it back by the same 
   * step size in the opposite direction.
   *
   * @param stepSize The size of the step to move during calibration.
   */
  void run_calibration(int stepSize)
  {
    std::this_thread::sleep_for(1000ms);

    printf("------ run_calibration(): start ------\n");

    step_to_stop(calibrate_step_size_degress);

    printf("------ run_calibration(): stop1 ------\n");

    step_to_stop(-calibrate_step_size_degress);

    printf("------ run_calibration(): end ------\n");
  }

private:
  bool verbose_ = false;
  float degrees_to_encoder = 10000.0 / 360.0;

  int32_t calibrate_step_size_degress = 1;
  int32_t calibrate_current_limit_amps = 3;
  std::chrono::milliseconds calibrate_dwell_time_ms = 100ms;

  int32_t calibration_current_set_angle = 0;

  typedef enum command_mode_enum
  {
    ENABLE = 0,
    DISABLE = 1,
    SET_CURRENT_AS_CENTER = 2,
    RUN_CENTER_CALIBRATION = 3
  } command_mode_enum_t;

  command_mode_enum_t command_mode_ = ENABLE;

  uint32_t can_id_ = 1;
  std::chrono::milliseconds heartbeat_interval_ = 500ms;
  std::chrono::milliseconds status_interval_ = 1000ms;
  int32_t current_position_ = 0;

  ros2_keya_servo_msgs::msg::ServoStatus servo_status_;

  bool speed_or_position_ = true;  // True = speed, False = position

  rclcpp::Subscription<ros2_keya_servo_msgs::msg::ServoCommand>::SharedPtr command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr torque_subscriber_;
  rclcpp::Service<ros2_keya_servo_msgs::srv::Position>::SharedPtr position_service_;
  rclcpp::Publisher<ros2_keya_servo_msgs::msg::ServoStatus>::SharedPtr status_publisher_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<KeyaServo>();

  node->initialize();

  // todo, wait for receipt of the current position before starting calibration
  // std::thread thread1(std::bind(&KeyaServo::run_calibration, node, 1));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}