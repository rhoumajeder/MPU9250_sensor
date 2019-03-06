/*
// register map : file:///C:/Users/bouma/Downloads/MPU-9250-Register-Map.pdf
// datasheet : file:///C:/Users/bouma/Downloads/PS-MPU-9250A-01-v1.1.pdf

 
#include "stm32f4xx_hal.h"
#include "main.h"
#include "math.h"
#include "stdlib.h"

#define MPU_9250_Address	                               0xD0
#define MPU_Register_Addresse_Accel                      0x28
#define MPU_Accel_Scale_4g                               0x08                 
#define MPU_Register_Address_Accel_XOUT_H							 	 0x3B
#define MPU_Sensibilite_4g															 8192.0






//void MPU_9250_Init(void);




*/


















////////////////////////////
#include "stm32f4xx_hal.h"
#include "stdlib.h"















#define SQR(x) ((x)*(x))

#define mpu9265Address	0xD0
#define MPU6050_KOEF_COMPL  0.1
#define R2DEG 57.29577951308232087679815481410517033f
#define pi 3.14159265359

#include "MadgwickAHRS.h"


 
typedef struct  {
    float Roll; 
   
    float Yaw; 
	  float accx;
		float accy;
		float accz;
		float T;
} datastruc;





void MPU_9250_Init(void);

void mpudata(float *data);

void calculdata(void);

//void calibrate(void);