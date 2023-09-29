#ifndef ROBOTARM_INTERFACE_POSITION_HPP
#define ROBOTARM_INTERFACE_POSITION_HPP

#include "LowLevelDriver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <string>

class Position
{
public:
	/**
	 * @brief Construct a new Position object
	 * 
	 * @param servo_base_angle angle for base
	 * @param servo_shoulder_angle angle for shoulder
	 * @param servo_elbow_angle angle for elbow
	 * @param servo_wrist_angle angle for wrist
	 * @param servo_gripper_angle angle for gripper
	 */
	Position(uint16_t servo_base_angle, uint16_t servo_shoulder_angle, uint16_t servo_elbow_angle, uint16_t servo_wrist_angle, uint16_t servo_gripper_angle);

	/**
	 * @brief writes all positions to serial
	 * 
	 * @param lld low level driver object
	 * @param time time it takes to move
	 */
	void commands_to_serial(LowLevelDriver &lld, uint16_t time);
private:
	/**
	 * @brief vector with angles
	 * 
	 */
	std::vector<uint16_t> servo_angles;
};

#endif // ROBOTARM_INTERFACE_POSITION_HPP