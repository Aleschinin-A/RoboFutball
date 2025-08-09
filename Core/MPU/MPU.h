#ifndef MPU_h
#define MPU_h

void Get_MPU(int16_t dT);
void Calibration_MPU(I2C_HandleTypeDef *handler, int N);
double Return_MPU_Angle();
double Return_MPU_Rotational_Speed();

#endif
