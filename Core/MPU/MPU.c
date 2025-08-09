/*
 * MPU.c
 *
 *  Created on: Aug 6, 2025
 *      Author: User
 */
#include "stm32f4xx_hal.h" // Change it for your requirements.
#include "MPU.h"


I2C_HandleTypeDef MPU_I2C_Handler;

#define DEFAULT_MPU_ADDRESS 0x68
uint8_t Gyro_XOut_H_Reg = 0x43;
int8_t Gyro_XOut[2];
uint8_t Gyro_XOut_L;
int16_t angular_rate;
double angular_rate_1;
double Angle;
double dreif = 0;
double dreif_1 = 0;
double W_robot;

void Get_MPU(int16_t dT){ // - запускать как можно чаще в цикле
	static long T_last;
	if(HAL_GetTick() - T_last >= dT){
		HAL_I2C_Master_Transmit(&MPU_I2C_Handler, (DEFAULT_MPU_ADDRESS << 1), &Gyro_XOut_H_Reg, 1, 10);
		HAL_I2C_Master_Receive(&MPU_I2C_Handler, (DEFAULT_MPU_ADDRESS << 1), Gyro_XOut, 2, 10);
		Gyro_XOut_L = Gyro_XOut[1];
		angular_rate = (Gyro_XOut[0] << 8) + Gyro_XOut_L;
		angular_rate_1 = angular_rate - dreif_1;
		Angle = Angle + angular_rate_1 * dT / 131000; // град
		W_robot = angular_rate_1 * 0.0076; // град/сек
		T_last = HAL_GetTick();
	}
}

void Calibration_MPU(I2C_HandleTypeDef *handler, int N){ // - Калибровка(&hi2c1, 100)
	memcpy(&MPU_I2C_Handler, handler, sizeof(*handler));
	int i = 0;
	while(i < N){
		Get_MPU(10);
		dreif = dreif + angular_rate_1;
		i++;
		HAL_Delay(10);
	}
	dreif_1 = dreif / N;
	Angle = 0.0;
	return 1;
}

double Return_MPU_Angle(){ // - возвращает угол
	return Angle;
}

double Return_MPU_Rotational_Speed(){ // - возвращает угловую скорость
	return W_robot;
}
