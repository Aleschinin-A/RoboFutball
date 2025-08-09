/*
 * PID_Lib.h
 *
 *  Created on: Aug 8, 2025
 *      Author: User
 */
#ifndef PID_Lib_h
#define PID_Lib_h

int PID_Speed_A(int y_vel, int alf, int dT);
int PID_Speed_B(int y_vel, int alf, int dT);
int PID_Speed_C(int y_vel, int alf, int dT);
int Return_OO_Vel_A();
int Return_OO_Vel_B();
int Return_OO_Vel_C();

#endif
