// ZeroErr_control.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include "ECanVci.h"

#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>


using namespace std;



#define	SEND_DATA_LENGTH		6
#define	SEND_CMD_LENGTH			2
#define	GET_CMD_LENGTH			4
#define ENCODER_COUNT			524288


int init_USBCAN(int* m_connect, int m_devtype);

enum CTRL_MODE
{
	TOURQE = 1,
	SPEED = 2,
	POS = 3,
	LOADER = 4,
};

enum MOTION_MODE
{
	START = 0x83,
	STOP = 0x84,
};

enum MOTOR_TYPE
{
	DIU = 0x83,
	DLLM = 0x84,
};

class joint_ctrl
{
private:
	CAN_OBJ		frameinfo;
	CAN_OBJ		revframeinfo;

	// parameters for motor setting
	uint32_t	acceleration;
	uint32_t	deceleration;
	int32_t		speed;

	string		joint_name;
	int			joint_id;
	uint8_t		motor_enabled;

	// input value
	int32_t		analog_speed;		//speed mode
	int32_t		target_pos;			//position mode
	int32_t		relative_pos;		//

	int send_command(uint8_t buffer[], int len);

	
public:
//	read value
	int32_t		cur_position;
	int32_t		cur_speed;
	int32_t		cur_current;
	int32_t		cur_torque;
	int32_t		cur_arr[0] ={};
	int32_t		encoder_cnt;


	joint_ctrl(std::string name, int id);
	~joint_ctrl() {};
	int set_speed_mode(int acc, int dec);
	int set_angle_mode(int acc, int dec, int speed);
	int set_torque_mode();

	int set_acceleration(int acc);
	int set_deceleration(int dec);
	int set_speed(int spd);

	int set_analog_speed(int input);			//analog can be speed (cnt/s) or torque (mA)
	int set_target_pos(int pos);
	int set_relative_pos(int pos);			
	int motor_enable(uint8_t enabled);

	int set_motion(int motion);

	int get_recent_position();
	int get_recent_speed();
	int get_recent_current();
	int get_recent_torque();
	

};

// TODO: Reference additional headers your program requires here.
