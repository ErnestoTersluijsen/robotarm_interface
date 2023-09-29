#ifndef ROBOTARM_INTERFACE_LOWLEVELDRIVER_HPP
#define ROBOTARM_INTERFACE_LOWLEVELDRIVER_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <string>

class LowLevelDriver
{
public:
	/**
	 * @brief Construct a new Low Level Driver object
	 * 
	 * @param a_port_name the name of the port
	 */
	LowLevelDriver(std::string a_port_name);

	/**
	 * @brief converts angle to pwn value
	 * 
	 * @param channel servo id
	 * @param angle angle of servo
	 * @return uint16_t pwm value
	 */
	uint16_t map_values(uint16_t channel, int16_t angle);

	/**
	 * @brief converts variables to command to be sent over serial
	 * 
	 * @param servo_id servo id
	 * @param angle angle of servo
	 * @param time time it takes to travel to the new location
	 * @return std::string string to be sent to serial
	 */
	std::string input_to_command(uint16_t servo_id, int16_t angle, uint16_t time);

	/**
	 * @brief sets the servo up
	 * 
	 */
	void setup_servo();

	/**
	 * @brief writes a string to serial
	 * 
	 * @param command string that has to be sent to serial
	 */
	void write_to_serial(std::string command);

	/**
	 * @brief stops all servos
	 * 
	 * @param amount_of_servos amount of servos that need to be stopped
	 */
	void emergency_stop(uint16_t amount_of_servos);

private:
	boost::asio::io_service ioservice;

	/**
	 * @brief contains the limits for servos
	 * 
	 */
	std::vector<std::pair<int16_t, int16_t>> servo_limits =
		{
			{-90, 90},
			{-30, 90},
			{0, 135},
			{90, -90},
			{-90, 90}
		};

	std::string port_name;
	boost::asio::serial_port serial;
	boost::asio::streambuf string_stream_buffer;
	std::ostream os;
};

#endif // ROBOTARM_INTERFACE_LOWLEVELDRIVER_HPP