// ZeroErr_control.cpp : Defines the entry point for the application.
//

#include "ZeroErr_control.h"



//	constructor
joint_ctrl::joint_ctrl(std::string name, int id)
{
	this->joint_name = name;
	this->joint_id = id;
	this->frameinfo.ID = 0x640 + this->joint_id;
	this->frameinfo.ExternFlag = 0;
	this->frameinfo.RemoteFlag = 0;
	this->frameinfo.DataLen = 6;
	this->frameinfo.SendType = 0;

	// motor configuration
	this->acceleration = 10000;
	this->deceleration = 10000;
	this->speed = 1000;

	this->encoder_cnt = 0;

	this->motor_enabled = 0;

}


int joint_ctrl::send_command(uint8_t buffer[], int len)
{	
	this->frameinfo.DataLen = len;
	for (int i = 0; i < this->frameinfo.DataLen; i++)
	{
		this->frameinfo.Data[i] = buffer[i];
	}

	if (Transmit(USBCAN1, 0, 0, &frameinfo, 1) == 1)
	{
		int len = 1;
		int tlen = 0;
		len = Receive(USBCAN1, 0, 0, &(this->revframeinfo), 50, 50);
		tlen = this->revframeinfo.DataLen - 1;
		
		if ((len > 0) && (this->revframeinfo.Data[tlen] == 0x3e))
		{
			cout << "write success!!" << endl;
			return STATUS_OK;
		}

		else
			return STATUS_ERR;
	}
	else
	{
		cout << "write failure!!" << endl;
		return STATUS_ERR;
	}
	return STATUS_OK;
}



int joint_ctrl::set_motion(int motion)
{
	uint8_t send_data[2] = {0x00, 0x00};
	send_data[1] = (uint8_t)motion;
	return send_command(send_data, SEND_CMD_LENGTH);

}



int joint_ctrl::motor_enable(uint8_t enabled)
{
	this->motor_enabled = enabled;
	uint8_t send_data[6] = { 0x01, 0x00, 0x00, 0x00, 0x00 , 0x00 };		//enable motor
	send_data[5] = (uint8_t)enabled;
	return send_command(send_data, SEND_DATA_LENGTH);
}


int joint_ctrl::get_recent_position()
{

	int tlen = 0;
	uint8_t send_data[2] = {0x00, 0x02};
	if (send_command(send_data,2) != STATUS_OK)
		return STATUS_ERR;
	// return xx xx xx xx 3E
	tlen = Receive(USBCAN1, 0, 0, &(this->revframeinfo), 50, 50);
	tlen = this->revframeinfo.DataLen;

	if (!((tlen == 5) && (this->revframeinfo.Data[tlen - 1] == 0x3e)))
		return STATUS_ERR;
	
	this->cur_position = 0;
	this->cur_position |= (uint32_t)(this->revframeinfo.Data[0] << 24);
	this->cur_position |= (uint32_t)(this->revframeinfo.Data[1] << 16);
	this->cur_position |= (uint32_t)(this->revframeinfo.Data[2] << 8);
	this->cur_position |= (uint32_t)(this->revframeinfo.Data[3]);
	
	return STATUS_OK;

}



int joint_ctrl::get_recent_speed()
{
	int tlen = 0;
	uint8_t send_data[] = { 0x00, 0x05, 0x00, 0x01 };
	if (send_command(send_data,4) != STATUS_OK)
		return STATUS_ERR;
	// return xx xx xx xx 3E
	tlen = Receive(USBCAN1, 0, 0, &(this->revframeinfo), 50, 50);
	tlen = this->revframeinfo.DataLen;

	if (!((tlen == 5) && (this->revframeinfo.Data[tlen - 1] == 0x3e)))
		return STATUS_ERR;

	this->cur_speed = 0;
	this->cur_speed |= (uint32_t)(this->revframeinfo.Data[0] << 24);
	this->cur_speed |= (uint32_t)(this->revframeinfo.Data[1] << 16);
	this->cur_speed |= (uint32_t)(this->revframeinfo.Data[2] << 8);
	this->cur_speed |= (uint32_t)(this->revframeinfo.Data[3]);

	return STATUS_OK;
}

int joint_ctrl::get_recent_current()
{
	int tlen = 0;
	uint8_t send_data[2] = { 0x00, 0x08 };
	
	if (send_command(send_data,SEND_CMD_LENGTH) != STATUS_OK)
		return STATUS_ERR;
	// return xx xx xx xx 3E
	tlen = Receive(USBCAN1, 0, 0, &(this->revframeinfo), 50, 50);
	tlen = this->revframeinfo.DataLen;

	if (!((tlen == 5) && (this->revframeinfo.Data[tlen - 1] == 0x3e)))
		return STATUS_ERR;

	this->cur_current = 0;
	this->cur_current |= (uint32_t)(this->revframeinfo.Data[0] << 24);
	this->cur_current |= (uint32_t)(this->revframeinfo.Data[1] << 16);
	this->cur_current |= (uint32_t)(this->revframeinfo.Data[2] << 8);
	this->cur_current |= (uint32_t)(this->revframeinfo.Data[3]);


	return STATUS_OK;
}

int joint_ctrl::get_recent_torque()
{
	int tlen = 0;

	uint8_t send_data[2] = { 0x02, 0x0d };

	if (send_command(send_data, SEND_CMD_LENGTH) != STATUS_OK)
		return STATUS_ERR;
	// return xx xx xx xx 3E
	tlen = Receive(USBCAN1, 0, 0, &(this->revframeinfo), 50, 50);
	tlen = this->revframeinfo.DataLen;

	if (!((tlen == 5) && (this->revframeinfo.Data[tlen - 1] == 0x3e)))
		return STATUS_ERR;

	this->cur_torque = 0;
	this->cur_torque |= (uint32_t)(this->revframeinfo.Data[0] << 24);
	this->cur_torque |= (uint32_t)(this->revframeinfo.Data[1] << 16);
	this->cur_torque |= (uint32_t)(this->revframeinfo.Data[2] << 8);
	this->cur_torque |= (uint32_t)(this->revframeinfo.Data[3]);


	return STATUS_OK;
}


int joint_ctrl::set_relative_pos(int pos)
{
	this->relative_pos = pos;
	uint8_t send_data[] = { 0x00, 0x87, 0x00, 0x00, 0x00 , 0x00 };			//set input
	send_data[2] = (uint8_t)((this->relative_pos >> 24) & 0x000000ff);		// high byte
	send_data[3] = (uint8_t)((this->relative_pos >> 16) & 0x000000ff);		// high byte
	send_data[4] = (uint8_t)((this->relative_pos >> 8) & 0x000000ff);		// high byte
	send_data[5] = (uint8_t)((this->relative_pos & 0x000000ff));			// low byte

	return send_command(send_data, SEND_DATA_LENGTH);
}


int joint_ctrl::set_target_pos(int pos)
{
	this->target_pos = pos;
	uint8_t send_data[] = { 0x00, 0x86, 0x00, 0x00, 0x00 , 0x00 };		//set input
	send_data[2] = (uint8_t)((this->target_pos >> 24) & 0x000000ff);		// high byte
	send_data[3] = (uint8_t)((this->target_pos >> 16) & 0x000000ff);		// high byte
	send_data[4] = (uint8_t)((this->target_pos >> 8) & 0x000000ff);		// high byte
	send_data[5] = (uint8_t)((this->target_pos & 0x000000ff));   // low byte

	if (send_command(send_data, SEND_DATA_LENGTH) != STATUS_OK)
		return STATUS_ERR;

	return this->set_motion(MOTION_MODE::START);
	
}


int joint_ctrl::set_analog_speed(int input)
{
	this->analog_speed = int32_t(input);
	uint8_t send_data[] = { 0x01, 0xFE, 0x00, 0x00, 0x00 , 0x00 };		//set input

	send_data[2] = (uint8_t)((this->analog_speed >> 24) & 0x000000ff );		// high byte
	send_data[3] = (uint8_t)((this->analog_speed >> 16) & 0x000000ff);		// high byte
	send_data[4] = (uint8_t)((this->analog_speed >> 8) & 0x000000ff) ;		// high byte
	send_data[5] = (uint8_t)((this->analog_speed & 0x000000ff));   // low byte

	return send_command(send_data, SEND_DATA_LENGTH);
}

int joint_ctrl::set_acceleration(int acc)
{
	
	this->acceleration = (uint32_t)acc;
	uint8_t send_data[] = { 0x00, 0x88, 0x00, 0x00, 0x00 , 0x00 };		//set acceleration

	send_data[2] = (uint8_t)((this->acceleration >> 24) & 0x000000ff);		// high byte
	send_data[3] = (uint8_t)((this->acceleration >> 16) & 0x000000ff);		// high byte
	send_data[4] = (uint8_t)((this->acceleration >> 8) & 0x000000ff);		// high byte
	send_data[5] = (uint8_t)((this->acceleration & 0x000000ff));			// low byte

	
	return send_command(send_data, SEND_DATA_LENGTH);
}


int joint_ctrl::set_deceleration(int dec)
{
	this->deceleration = (uint32_t)dec;
	uint8_t send_data[] = { 0x00, 0x89, 0x00, 0x00, 0x00 , 0x00 };		//set acceleration


	send_data[2] = (uint8_t)((this->deceleration >> 24) & 0x000000ff);		// high byte
	send_data[3] = (uint8_t)((this->deceleration >> 16) & 0x000000ff);		// high byte
	send_data[4] = (uint8_t)((this->deceleration >> 8) & 0x000000ff);		// high byte
	send_data[5] = (uint8_t)((this->deceleration & 0x000000ff));			// low byte

	return send_command(send_data, SEND_DATA_LENGTH);
}

int joint_ctrl::set_speed(int spd)
{
	this->speed = (int32_t)spd;
	uint8_t send_data[] = { 0x00, 0x8A, 0x00, 0x00, 0x00 , 0x00 };		//set acceleration

	send_data[2] = (uint8_t)((this->speed >> 24) & 0x000000ff);		// high byte
	send_data[3] = (uint8_t)((this->speed >> 16) & 0x000000ff);		// high byte
	send_data[4] = (uint8_t)((this->speed >> 8) & 0x000000ff);		// high byte
	send_data[5] = (uint8_t)((this->speed & 0x000000ff));			// low byte

	return send_command(send_data, SEND_DATA_LENGTH);
}



int joint_ctrl::set_speed_mode(int acc, int dec)
{
	uint8_t send_data[6] = { 0x00, 0x4e, 0x00, 0x00, 0x00 , CTRL_MODE::SPEED };		// set speed mode
	
	if (send_command(send_data, SEND_DATA_LENGTH) != STATUS_OK)
		return STATUS_ERR;

	uint8_t control_source[6] = { 0x01, 0x12, 0x00, 0x00, 0x00 , 0x00 };		// set control source

	if (send_command(control_source, SEND_DATA_LENGTH) != STATUS_OK)
		return STATUS_ERR;

	uint8_t use_anagle[6] = { 0x01, 0xFD, 0x00, 0x00, 0x00 , 0x00 };		// set input analog use
	if (send_command(control_source, SEND_DATA_LENGTH) != STATUS_OK)
		return STATUS_ERR;

	if (set_acceleration(acc) != STATUS_OK)
		return STATUS_ERR;
	
	if (set_deceleration(dec) != STATUS_OK)
		return STATUS_ERR;

	if (set_analog_speed(0) != STATUS_OK)
		return STATUS_ERR;
	
	return motor_enable(1);
	
}




int joint_ctrl::set_angle_mode(int acc, int dec, int speed)
{
	uint8_t send_data[6] = { 0x00, 0x4e, 0x00, 0x00, 0x00 , CTRL_MODE::POS };		// set position mode
	if (send_command(send_data,SEND_DATA_LENGTH) != STATUS_OK)
		return STATUS_ERR;

	uint8_t motion_mode[6] = { 0x00, 0x8d, 0x00, 0x00, 0x00 , 0x01 };				// set speed mode
	if (send_command(motion_mode, SEND_DATA_LENGTH) != STATUS_OK)
		return STATUS_ERR;

	if (set_acceleration(acc) != STATUS_OK)
		return STATUS_ERR;

	if (set_deceleration(dec) != STATUS_OK)
		return STATUS_ERR;
	
	if (set_speed(speed) != STATUS_OK)
		return STATUS_ERR;

	if (motor_enable(1) != STATUS_OK)
		return STATUS_ERR;

	// set relative position (angle)
	return set_relative_pos(0);
}

int joint_ctrl::set_torque_mode()
{
	uint8_t send_data[6] = { 0x00, 0x4e, 0x00, 0x00, 0x00 , CTRL_MODE::TOURQE };		// set speed mode
	return send_command(send_data,SEND_DATA_LENGTH);
}


int init_USBCAN(int* m_connect, int m_devtype)
{
	INIT_CONFIG init_config;

	init_config.AccCode = 0;
	init_config.AccMask = 0xffffff;
	init_config.Filter = 0;

	// 1M baudrate
	init_config.Timing0 = 0;
	init_config.Timing1 = 0x14;

	init_config.Mode = 0;

	int ret= 0;
	
	while (ret == 0 ) {

		ret= OpenDevice(m_devtype, 0, 0);
		if (ret != STATUS_OK)
		{
			cerr << "Open device fault!" << endl;
			//return -1;
		}
		// Open USBCAN devie Success

		}
	cout << "Open device Success" << endl;
	ret=0;


	while (ret == 0) {

		ret = InitCAN(m_devtype, 0, 0, &init_config);

		if (ret != STATUS_OK)
		{
			cerr << "Init can fault!" << endl;
			CloseDevice(m_devtype, 0);
			//return -1;
		}
	}

	cout << "Init Success" << endl;
	*m_connect = 1;
	ret=0;

	// start CAN 
	while (ret == 0) {

		//cout << "Init Success" << endl;
		ret = StartCAN(m_devtype, 0, 0);
		

		if (*m_connect == 0)
		{
			cerr << "Not open device!" << endl;
			//return -1;
		}
		
		if (ret == 1)
		{
			cout << "Start Success" << endl;
		}
		
		else
		{
			cerr << "Start Failure!" << endl;
			//return -1;
		}
		}
}

/*
int main(int argc, char* argv[])
{
	if (argc > 1)
    {
        cout << "passing arguments" << endl;
    }

	
	// motor function testing 
	

	int m_connect;
	int m_devtype;

	// setup USBCAN
	m_connect = 0;			// can bus is closed
	m_devtype = USBCAN1;
	init_USBCAN(&m_connect, m_devtype);
	

	joint_ctrl	motor0 = joint_ctrl("L1", 1);
	joint_ctrl	motor1 = joint_ctrl("L1", 2);
	joint_ctrl	motor2 = joint_ctrl("L1", 3);
	joint_ctrl	motor3 = joint_ctrl("L1", 4);
	joint_ctrl	motor4 = joint_ctrl("L1", 5);
	joint_ctrl	motor5 = joint_ctrl("L1", 6);

	motor0.motor_enable(1);
	motor0.set_angle_mode(524288, 524288, 524288);
    motor1.motor_enable(1);
	motor1.set_angle_mode(524288, 524288, 524288);
    motor2.motor_enable(1);
	motor2.set_angle_mode(524288, 524288, 524288);
    motor3.motor_enable(1);
	motor3.set_angle_mode(524288, 524288, 524288);
    motor4.motor_enable(1);
	motor4.set_angle_mode(524288, 524288, 524288);
    motor5.motor_enable(1);
	motor5.set_angle_mode(524288, 524288, 524288);
	
	
	cout << "motor curr pos" << motor1.cur_position<< "motor curr speed "<<motor1.cur_speed<< "motor curr torque" <<motor1.cur_torque<< endl;
	int vel = 0 ;
	int num0 = 0 ;
	int num1 = 0 ;
	int num2 = 0 ;
	int num3 = 0 ;
	int num4 = 0 ;
	int num5 = 0 ;
	while(1){
		
		motor0.get_recent_position();
		motor1.get_recent_position();
		motor2.get_recent_position();
		motor3.get_recent_position();
		motor4.get_recent_position();
		motor5.get_recent_position();



		cout << "Enter a vel: ";

		// take integer input
		cin >> vel;

		cout << "Enter a pos0: ";

		// take integer input
		cin >> num0;
		cout << "Enter a pos1: ";

		// take integer input
		cin >> num1;
		cout << "Enter a pos2: ";

		// take integer input
		cin >> num2;
		cout << "Enter a pos3: ";

		// take integer input
		cin >> num3;
		cout << "Enter a pos4: ";

		// take integer input
		cin >> num4;
		cout << "Enter a pos5: ";

		// take integer input
		cin >> num5;

		motor0.set_speed(vel);
		motor1.set_speed(vel);
		motor2.set_speed(vel);
		motor3.set_speed(vel);
		motor4.set_speed(vel);
		motor5.set_speed(vel);


		motor0.set_target_pos(num0*524288/360);
		motor1.set_target_pos(num1*524288/360);
		motor2.set_target_pos(num2*524288/360);
		motor3.set_target_pos(num3*524288/360);
		motor4.set_target_pos(num4*524288/360);
		motor5.set_target_pos(num5*524288/360);


		cout << "motor curr pos0" << motor0.cur_position
		<< "motor curr pos1" << motor1.cur_position 
		<< "motor curr pos2" << motor2.cur_position 
		<< "motor curr pos3" << motor3.cur_position 
		<< "motor curr pos4" << motor4.cur_position 
		<< "motor curr pos5" << motor5.cur_position <<endl;

	}
	
	return 0;
}
*/
