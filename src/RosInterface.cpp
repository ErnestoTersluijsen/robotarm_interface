#include "RosInterface.hpp"

#include "LowLevelDriver.hpp"

RosInterface::RosInterface() : Node("interface"), lld("/dev/pts/4")
{
    event_info("CREATE_ROSINTERFACE");
    position_server = rclcpp_action::create_server<robotarm_interface::action::Position>(this, "position", std::bind(&RosInterface::position_handle_goal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RosInterface::position_handle_cancel, this, std::placeholders::_1), std::bind(&RosInterface::position_handle_accepted, this, std::placeholders::_1));
    servo_server = rclcpp_action::create_server<robotarm_interface::action::Servo>(this, "servo", std::bind(&RosInterface::servo_handle_goal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RosInterface::servo_handle_cancel, this, std::placeholders::_1), std::bind(&RosInterface::servo_handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse RosInterface::position_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robotarm_interface::action::Position::Goal> goal)
{
    (void)uuid;

    if (goal->position == PARK || goal->position == READY || goal->position == STRAIGHT || goal->position == EMERGENCY_STOP)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse RosInterface::position_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RosInterface::position_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle)
{
    event_info("POSITION_HANDLE_ACCEPTED_EVENT");
    switch (goal_handle->get_goal()->position)
    {
        case PARK:
        {
            state_info("PARK");
            std::thread{std::bind(&RosInterface::position_execute, this, positions.at(PARK), goal_handle)}.detach();
            break;
        }
        case READY:
        {
            state_info("READY");
            std::thread{std::bind(&RosInterface::position_execute, this, positions.at(READY), goal_handle)}.detach();
            break;
        }
        case STRAIGHT:
        {
            state_info("STRAIGHT");
            std::thread{std::bind(&RosInterface::position_execute, this, positions.at(STRAIGHT), goal_handle)}.detach();
            break;
        }
        case EMERGENCY_STOP:
        {
            state_info("EMERGENCY_STOP");
            std::thread{std::bind(&RosInterface::emergency_stop, this, goal_handle)}.detach();
            break;
        }
        default:
        {
            break;
        }
    }
}

void RosInterface::position_execute(Position pos, const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle)
{
    event_info("POSITION_EXECUTE_EVENT");
    std::shared_ptr<robotarm_interface::action::Position_Feedback> feedback = std::make_shared<robotarm_interface::action::Position::Feedback>();
    std::shared_ptr<robotarm_interface::action::Position_Result> result = std::make_shared<robotarm_interface::action::Position::Result>();

    pos.commands_to_serial(lld, 3000);

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {
        if(feedback->progress < 100)
        {
            feedback->progress += 10;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
        else
        {
            result->response = "done";
            goal_handle->succeed(result);
            break;
        }
    }
}

rclcpp_action::GoalResponse RosInterface::servo_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robotarm_interface::action::Servo::Goal> goal)
{
    (void)uuid;
    (void)goal;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosInterface::servo_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Servo>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RosInterface::servo_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Servo>> goal_handle)
{
    state_info("DEGREES_OF_FREEDOM");
    event_info("SERVO_HANDLE_ACCEPTED");
    std::thread{std::bind(&RosInterface::servo_execute, this, goal_handle->get_goal()->servoid, goal_handle->get_goal()->angle, goal_handle->get_goal()->time, goal_handle)}.detach();
}

void RosInterface::servo_execute(uint16_t servo_id, int16_t angle, uint16_t time, const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Servo>> goal_handle)
{
    event_info("SERVO_EXECUTE");
    std::shared_ptr<robotarm_interface::action::Servo_Feedback> feedback = std::make_shared<robotarm_interface::action::Servo::Feedback>();
    std::shared_ptr<robotarm_interface::action::Servo_Result> result = std::make_shared<robotarm_interface::action::Servo::Result>();

    lld.write_to_serial(lld.input_to_command(servo_id, angle, time));

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {
        if (feedback->progress < 100)
        {
            feedback->progress += 10;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
        else
        {
            result->response = "done";
            goal_handle->succeed(result);
            break;
        }
    }
}

void RosInterface::emergency_stop(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_interface::action::Position>> goal_handle)
{
    event_info("EMERGENCY_STOP_EVENT");
    std::shared_ptr<robotarm_interface::action::Position_Feedback> feedback = std::make_shared<robotarm_interface::action::Position::Feedback>();
    std::shared_ptr<robotarm_interface::action::Position_Result> result = std::make_shared<robotarm_interface::action::Position::Result>();

    lld.emergency_stop(6);

    if (rclcpp::ok())
    {
        feedback->progress = 100;
        goal_handle->publish_feedback(feedback);
        result->response = "done";
        goal_handle->succeed(result);
    }
}

void RosInterface::state_info(std::string state)
{
    std::ostringstream oss;
    oss << "STATE:{" << state << "}";
    RCLCPP_DEBUG(this->get_logger(), oss.str().c_str());
}

void RosInterface::event_info(std::string event)
{
    std::ostringstream oss;
    oss << "EVENT:{" << event << "}";
    RCLCPP_DEBUG(this->get_logger(), oss.str().c_str());
}