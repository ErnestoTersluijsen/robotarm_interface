#include "Position.hpp"
#include "LowLevelDriver.hpp"

Position::Position(uint16_t servo_0_angle, uint16_t servo_1_angle, uint16_t servo_2_angle, uint16_t servo_3_angle, uint16_t servo_4_angle)
{
	servo_angles = {servo_0_angle, servo_1_angle, servo_2_angle, servo_3_angle, servo_4_angle};
}

void Position::commands_to_serial(LowLevelDriver &lld, uint16_t time)
{
	for (uint16_t i = 1; i < servo_angles.size(); ++i)
	{
		lld.write_to_serial(lld.input_to_command(i, servo_angles.at(i), time));
	}
}