/*
 * ak70_9.c
 *
 *  Created on: Feb 4, 2026
 *      Author: Juan Reyes
 *
 *  Description: The following document contains the functions for controlling an ak70-9 KV60 CubeMars Motor using an STM32F446Re
 *
 */

/* This first section relates to the Servo Mode Control Modes.
 * There are 6 control modes in the servo mode, each corresponding to a specific ID number.
 * Duty Cycle Mode: A specified duty cycle voltage is given to the motor, similar to square wave drive form;
 * Current Loop Mode: A specified Iq current is given to the motor, and since the motor output torque = iq*KT, it can be used as a torque loop (KT data can be referenced on the official website);
 * Current Brake Mode: A specified braking current is given to the motor to hold it in the current position (pay attention to motor temperature when using);
 * Velocity Mode: A specified operating speed is given to the motor (acceleration is default to the maximum value);
 * Position Mode: A specified position is given to the motor, and the motor will move to the specified position (speed and acceleration are default to the maximum value);
 * Position-Velocity Loop Mode: A specified position, speed, and acceleration are given to the motor
 *
 *Servo mode driver board data transmission definition
 *		Identifier:[28]-[8] Control Mode ID + [7]-[0] Driver ID
 *		Frame Type: Extended Frame
 *		Frame Format: DATA
 *		DLC: 8 bytes
 */

// Control mode has {0,1,2,3,4,5,6} 7 characteristic values corresponding to 7 control modes.
typedef enum {
	CAN_PACKET_SET_DUTY = 0, // Duty Cycle Mode
	CAN_PACKET_SET_CURRENT, // Current Loop Mode
	CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
	CAN_PACKET_SET_RPM, // Speed Mode
	CAN_PACKET_SET_POS, // Position Mode
	CAN_PACKET_SET_ORIGIN_HERE, // Set origin position mode (zero mode)
	CAN_PACKET_SET_POS_SPD, // Position-Velocity Loop Mode
} CAN_PACKET_ID;

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	uint8_t i=0;
	if (len > 8) {
		len = 8;
	}
	CanTxMsg TxMessage;
	TxMessage.StdId = 0;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.ExtId = id;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len;
	for(i=0;i<len;i++) TxMessage.Data[i]=data[i];
	CAN_Transmit(CHASSIS_CAN, &TxMessage); //CAN port trasmissionTxMessage Data Canport sends TxMesage Data
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// Duty Cycle Mode
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// Current Loop Mode
// The current value is of int32 type, and the values -60000 to 60000 represent -60 to 60 A.
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

// Current Brake Mode
// The brake current value is of int32 type, and the values 0 to 60000 represent 0 to 60 A.
void comm_can_set_cb(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

// Velocity Loop Mode
// The speed value is of int32 type, and the range -100000 to 100000 represents -100000 to 100000electrical RPM.
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// Position Loop Mode
// The position value is of int32 type, and the range -360000000 to 360000000 represents positions from -36000° to 36000°.
void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

// Setting Origin Mode
// The set command is of uint8_t type, 0 represents setting a temporary origin (erased upon power loss), and 1 represents setting a permanent origin (parameters are automatically saved).
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
	int32_t send_index = 1;
	uint8_t buffer;
	buffer=set_origin_mode;
	comm_can_transmit_eid(controller_id | ((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index);
}

// Position-Velocity Loop Mode
// The position value is of int32 type, and the range -360000000 to 360000000 corresponds to positions from -36000° to 36000
// The speed value is of int16 type, and the range -32768 to 32767 corresponds to -327680 to327680 ERPM;
// The acceleration value is of int16 type, and the range 0 to 32767 corresponds to 0 to 327670, with 1 unit equal to 10 ERPM/s².
void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA ) {
	int32_t send_index = 0;
	Int16_t send_index1 = 4;
	uint8_t buffer[8];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	buffer_append_int16(buffer,spd/10.0, & send_index1);
	buffer_append_int16(buffer,RPA/10.0, & send_index1);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}

/* This second section relates to the Force Control Mode (MIT).
 * The MIT Control Mode has three control modes, all sharing a single Control ID number, with the specific control mode determined by the transmitted data.
 *
 * Position Loop Mode: Operates the motor with a specified position and Kp, Kd values, the motor is going to rotate to the specified position.
 * Velocity Loop Mode: Operates the motor with a specified speed and Kd value, the motor is going to rotate at the specified speed.
 * Current Loop Mode: Operates the motor with a specified torque value, the motor is going to rotate at the specified torque value.
 *
 *The Control Mode ID for the Force Control Mode has a characteristic value of 8
 *
 *Force Control Mode driver board data transmission definition
 *		Identifier:[28]-[8] Control Mode ID + [7]-[0] Driver ID
 *		Frame Type: Extended Frame
 *		Frame Format: DATA
 *		DLC: 8 bytes
 */

u8 SERVO_Can_Send_Msg(uint32_t ExtId,u8* msg,u8 len)
{
	u8 mbox;
	u16 i=0;

	// TxMessage.StdId=stdId; // standard identifier
	TxMessage.ExtId=ExtId; // Setting the Extended Marker
	TxMessage.IDE=CAN_Id_Extended; // standard frame
	TxMessage.RTR=CAN_RTR_Data; // data frame
	TxMessage.DLC=len; // Length of data to be sent
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];
	mbox= CAN_Transmit(CANx, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;
	//Waiting for the end of sending
	if(i>=0XFFF)return 1;
	return 0;
	}
float fmaxf(float x, float y){
/// Returns maximum of x, y ///
return (((x)>(y))?(x):(y));
}
float fminf(float x, float y){
/// Returns minimum of x, y ///
return (((x)<(y))?(x):(y));
}
float fmaxf3(float x, float y, float z){
/// Returns maximum of x, y, z ///
return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}
float fminf3(float x, float y, float z){
	/// Returns minimum of x, y, z ///
	return (x < y ? (x < z ? x : z) : (y < z ? y : z));
	}

float P_MIN =-12.56f;
float P_MAX =12.56f;
float V_MIN =-30.0f;
float V_MAX =30.0f;
float T_MIN =-32.0f;
float T_MAX =32.0f;
float Kp_MIN =0;
float Kp_MAX =500.0f;
float Kd_MIN =0;
float Kd_MAX =5.0f;
float Test_Pos=0.0f;

int p_int ;
int v_int ;
int kp_int ;
int kd_int ;
int t_int ;

void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff){
	uint8_t buffer[8];
	p_des = fminf(fmaxf(P_MIN, 0), P_MAX);
	v_des = fminf(fmaxf(V_MIN, 0), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

	/// convert floats to unsigned ints ///
	p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
	v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
	kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
	kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
	t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

	/// pack ints into the can buffer ///
	buffer[0] = kp_int>>4; //KP high 8 bits
	buffer[1] = ((kp_int&0xF)<<4)|( kd_int>>8); //KP Low 4 bits, Kd High 4 bits
	buffer[2] = kd_int&0xFF; //Kd low 8 bits
	buffer[3] = p_int>>8; //position high 8 bits
	buffer[4] = p_int&0xFF; //position low 8 bits
	buffer[5] = v_int>>4; //speed high 8 bits
	buffer[6] = ((v_int&0xF)<<4)|(t_int>>8); //speed low 4 bits torque high 4 bits
	buffer[7] = t_int&0xff; //torque low 8 bits
	SERVO_Can_Send_Msg(controller_id |((uint32_t) CAN_PACKET_SET_mit << 8), buffer, 8);
}

// When sending data, all numbers must be converted to integers through the following function before being sent to the motor.
int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	if(x < x_min) x = x_min;
	else if(x > x_max) x = x_max;
	return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

/* This third section relates to CAN Upload Message Protocol.
 * The motor CAN message uses a timed upload mode, with an upload frequency that can be set from 1 to 500 Hz, and the upload byte is 8 bytes.
 *
 * The position is of int16 type, with a range of -32000 to 32000 representing a position of -3200°to 3200°;
 * The speed is of int16 type, with a range of -32000 to 32000 representing an electrical speed of -320000 to 320000 rpm;
 * The current is of int16 type, with values -6000 to 6000 representing -60 to 60 A;
 * The temperature is of int8 type, with a range of -20 to 127 representing the driver board temperature of -20°C to 127°C;
 * The error code is of uint8 type, with 0 indicating no fault, 1 indicating motor over-temperaturefault, 2 indicating over-current fault, 3 indicating over-voltage fault, 4 indicating under-voltagefault, 5 indicating encoder fault, 6 indicating MOSFET over-temperature fault, and 7 indicating motor lock-up.
 *
 */

// This function gets all the monitoring values from the motor and updates their global variables
void motor_receive(float* motor_pos,float* motor_spd,float* cur,int_8* temp,int_8* error,rx_message) {
	int16_t pos_int = (rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
	int16_t spd_int = (rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
	int16_t cur_int = (rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
	&motor_pos= (float)( pos_int * 0.1f); //Motor position
	&motor_spd= (float)( spd_int * 10.0f);//Motor speed
	&motor_cur= (float) ( cur_int * 0.01f);//Motor current
	&motor_temp= (rx_message)->Data[6] ;//Motor temperature
	&motor_error= (rx_message)->Data[7] ;//Motor error code
}
