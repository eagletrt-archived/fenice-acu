#include "id.h"

void ID_init(ID* _ID){
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

}
