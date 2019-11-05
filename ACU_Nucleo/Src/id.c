#include "id.h"

void ID_init(ID* _ID){
	//ACU ID
	_ID->ACU = 0x100;
	//IMU
	_ID->imu_angular_rate = 0x4EC;
	_ID->imu_acceleration = 0x4ED;
	//--- Infrared Temperature Sensor ---//
	_ID->ITS_1_0 = 0x5B0;
	_ID->ITS_1_1 = 0x5B1;
	_ID->ITS_1_2 = 0x5B2;
	_ID->ITS_1_3 = 0x5B3;
	_ID->ITS_2_0 = 0x5B4;
	_ID->ITS_2_1 = 0x5B5;
	_ID->ITS_2_2 = 0x5B6;
	_ID->ITS_2_3 = 0x5B7;
	_ID->ITS_3_0 = 0x5B8;
	_ID->ITS_3_1 = 0x5B9;
	_ID->ITS_3_2 = 0x5BA;
	_ID->ITS_3_3 = 0x5BB;
	_ID->ITS_4_0 = 0x5BC;
	_ID->ITS_4_1 = 0x5BD;
	_ID->ITS_4_2 = 0x5BE;
	_ID->ITS_4_3 = 0x5BF;
	//from inverter

	_ID->REQ_INV_SX = 0x181;
	_ID->REQ_INV_DX = 0x182;
	_ID->ASK_INV_SX = 0x201;
	_ID->ASK_INV_DX = 0x202;
	_ID->ASK_STATE = 0x10;


	//from steer
	_ID->STEERING_WEEL_1 = 0xA0;
	_ID->STEERING_WEEL_2 = 0xAF;
	//from BMS
	_ID->BMS_HV = 0xAA;
	_ID->BMS_LV = 0xFF;


}
