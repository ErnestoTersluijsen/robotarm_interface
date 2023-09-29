#ifndef ROBOTARM_INTERFACE_ROSINTERFACE_HPP
#define ROBOTARM_INTERFACE_ROSINTERFACE_HPP

#include "robotarm_interface/action/position.hpp"
#include "robotarm_interface/action/servo.hpp"

#include "Position.hpp"
#include "LowLevelDriver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <thread>

enum Positions
{
	PARK = 0,
	READY = 1,
	STRAIGHT = 2,
	EMERGENCY_STOP = 3
};

class RosInterface : public rclcpp::Node
{
public:
	/**
	 * @brief Construct a new Ros Interface object
	 * 
	 */
	RosInterface();

private:
	/**
	 * @brief server for postions
	 * 
	 */
	rclcpp_action::Server<robotarm_interface::action::Position>::SharedPtr position_server;

	/**
	 * @brief server for specific servo angles
	 * 
	 */
	rclcpp_action::Server<robotarm_interface::action::Servo>::SharedPtr servo_server;

	/**
	 * @brief vector with positions
	 * 
	 */
	std::vector<Position> positions=
	{
		Position(0, 60, 108, 45,0),
        Position(0, 48, 74, 10, 0),
        Position(0, 30, 14, 0, 0)
	};

	/**
	 * @brief low level driver object
	 * 
	 */
	LowLevelDriver lld;

	/**
	 * @brief action server handle goal
	 * 
	 */
	rclcpp_action::GoalResponse position_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robotarm_interface::action::Position::Goal> goal);

	/**
	 * @brief action server handle cancel
	 * 
	 */
	rclcpp_action::CancelResponse position_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle);

	/**
	 * @brief action server handle accepted
	 * 
	 */
	void position_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle);

	/**
	 * @brief action server handle execute
	 * 
	 */
	void position_execute(Position pos, const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle);

	/**
	 * @brief action server handle goal
	 * 
	 */
	rclcpp_action::GoalResponse servo_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robotarm_interface::action::Servo::Goal> goal);

	/**
	 * @brief action server handle cancel
	 * 
	 */
	rclcpp_action::CancelResponse servo_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Servo>> goal_handle);

	/**
	 * @brief action server handle accepted
	 * 
	 */
	void servo_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Servo>> goal_handle);

	/**
	 * @brief action server handle execute
	 * 
	 */
	void servo_execute(uint16_t servo_id, int16_t angle, uint16_t time, const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Servo>> goal_handle);

	/**
	 * @brief action server emergency stop execute
	 * 
	 */
	void emergency_stop(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle);

	/**
	 * @brief prints state info
	 * 
	 * @param state state
	 */
	void state_info(std::string state);

	/**
	 * @brief prints event info
	 * 
	 * @param event 
	 */
	void event_info(std::string event);
};

#endif // ROBOTARM_INTERFACE_ROSINTERFACE_HPP