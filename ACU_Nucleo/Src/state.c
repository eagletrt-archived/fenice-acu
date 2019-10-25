#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "stm32f7xx_hal.h"


// Default state
// Init variables
void init(){
	if(fifoRxDataCAN_pop(&can1)){
		if(can1.id == id.imu_acceleration){

		}else if(can1.id == id.imu_angular_rate){

		}
	}
	if(fifoRxDataCAN_pop(&can3)){

	}
}
void idle(){

}
void calib(){

}
void setup(){

}
void run(){

}
