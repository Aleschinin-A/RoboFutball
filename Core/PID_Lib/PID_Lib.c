/*
 * PID_Lib.c
 *
 *  Created on: Aug 8, 2025
 *      Author: User
 */
#include "stm32f4xx_hal.h" // Change it for your requirements.
#include "PID_Lib.h"

int oo_vel_A;
int oo_vel_B;
int oo_vel_C;

int PID_Speed_A(int y_vel, int alf, int dT){  // - Регулятор скорости мотора (заданная скорость в тик/dT мс, положение в тик, dT в мс)
	#define Max_I 100
	#define Min_I -100
	#define Max_Out 255
	#define Min_Out -255

	#define Kp 20
	#define Ki 0.05
	#define Kd 10


	static int Sum;
	static int Last_Err;

	static long T_last;
	static int Out;
	static int last_alf;

	if(HAL_GetTick() - T_last >= dT){
		oo_vel_A = alf - last_alf;
		int err = y_vel - oo_vel_A;
		// PID
		if((err < 2 && err > -2)) Sum = 0;
		if(Sum > Max_I) Sum = Max_I;
		if(Sum < Min_I) Sum = Min_I;
		Sum = Sum + err;
		Out = (Kp * err) + (Ki * Sum * dT) + (Kd * (err - Last_Err) / dT);
		Last_Err = err;

		if(Out > Max_Out) Out = Max_Out;
		if(Out < Min_Out) Out = Min_Out;
		// PID
		last_alf = alf;
		T_last = HAL_GetTick();
	}
	return Out;
}

int PID_Speed_B(int y_vel, int alf, int dT){  // - Регулятор скорости мотора (заданная скорость в тик/dT мс, положение в тик, dT в мс)
	#define Max_I 100
	#define Min_I -100
	#define Max_Out 255
	#define Min_Out -255

	#define Kp 20
	#define Ki 0.05
	#define Kd 10

	static int Sum;
	static int Last_Err;

	static long T_last;
	static int Out;
	static int last_alf;

	if(HAL_GetTick() - T_last >= dT){
		oo_vel_B = alf - last_alf;
		int err = y_vel - oo_vel_B;
		// PID
		if((err < 2 && err > -2)) Sum = 0;
		if(Sum > Max_I) Sum = Max_I;
		if(Sum < Min_I) Sum = Min_I;
		Sum = Sum + err;
		Out = (Kp * err) + (Ki * Sum * dT) + (Kd * (err - Last_Err) / dT);
		Last_Err = err;

		if(Out > Max_Out) Out = Max_Out;
		if(Out < Min_Out) Out = Min_Out;
		// PID
		last_alf = alf;
		T_last = HAL_GetTick();
	}
	return Out;
}

int PID_Speed_C(int y_vel, int alf, int dT){  // - Регулятор скорости мотора (заданная скорость в тик/dT мс, положение в тик, dT в мс)
	#define Max_I 100
	#define Min_I -100
	#define Max_Out 255
	#define Min_Out -255

	#define Kp 20
	#define Ki 0.05
	#define Kd 10

	static int Sum;
	static int Last_Err;

	static long T_last;
	static int Out;
	static int last_alf;

	if(HAL_GetTick() - T_last >= dT){
		oo_vel_C = alf - last_alf;
		int err = y_vel - oo_vel_C;
		// PID
		if((err < 2 && err > -2)) Sum = 0;
		if(Sum > Max_I) Sum = Max_I;
		if(Sum < Min_I) Sum = Min_I;
		Sum = Sum + err;
		Out = (Kp * err) + (Ki * Sum * dT) + (Kd * (err - Last_Err) / dT);
		Last_Err = err;

		if(Out > Max_Out) Out = Max_Out;
		if(Out < Min_Out) Out = Min_Out;
		// PID
		last_alf = alf;
		T_last = HAL_GetTick();
	}
	return Out;
}

int Return_OO_Vel_A(){
	return oo_vel_A;
}

int Return_OO_Vel_B(){
	return oo_vel_B;
}

int Return_OO_Vel_C(){
	return oo_vel_C;
}
