#include "LowLevelDriver.hpp"

LowLevelDriver::LowLevelDriver(std::string a_port_name) : port_name(a_port_name), serial(ioservice, a_port_name), os(&string_stream_buffer)
{
	setup_servo();
}

uint16_t LowLevelDriver::map_values(uint16_t channel, int16_t gradient)
{
	if(channel != 3)
	{
		if (gradient < servo_limits.at(channel).first)
		{
			gradient = servo_limits.at(channel).first;
		}
		else if (gradient > servo_limits.at(channel).second)
		{
			gradient = servo_limits.at(channel).second;
		}
	} else {
		if(gradient > servo_limits.at(channel).first)
		{
			gradient = servo_limits.at(channel).first;
		} else if(gradient < servo_limits.at(channel).second) {
			gradient = servo_limits.at(channel).second;
		}
	}
	uint16_t pwm = 0;
	int16_t oldMinRange = servo_limits.at(channel).first;
	int16_t oldMaxRange = servo_limits.at(channel).second;
	int16_t oldRange = oldMaxRange - oldMinRange;
	int16_t newMin = 500;
	int16_t newMax = 2500;
	int16_t newRange = newMax - newMin;
	pwm = static_cast<uint16_t>((((gradient - oldMinRange) * newRange) / oldRange) + newMin);
	return pwm;
}

std::string LowLevelDriver::input_to_command(uint16_t servo_id, int16_t angle, uint16_t time)
{
	std::stringstream ss;
	ss << '#' << servo_id << 'P' << map_values(servo_id, angle) << 'T' << time << '\r';
	return ss.str();
}

void LowLevelDriver::write_to_serial(std::string command)
{
	os << command;
	boost::asio::write(serial, string_stream_buffer);
}

void LowLevelDriver::setup_servo()
{
	serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
	serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
}

void LowLevelDriver::emergency_stop(uint16_t amount_of_servos)
{
	for (size_t i = 0; i < amount_of_servos; i++)
	{
		std::stringstream ss;
		ss << "STOP " << i << "\r";
		write_to_serial(ss.str());
	}
}
