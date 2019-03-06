#include "mpu.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;
datastruc datastruct;


int16_t ax,ay,az,Gx,Gy,Gz,T;
float Xaccel,Yaccel,Zaccel,j ,tempe ,cp, cr;
float fGX_Cal=0.0f,fGY_Cal=0.0f,fGZ_Cal=0.0f,fax_cal=0.0f,fay_cal=0.0f,faz_cal=0.0f;
float gyro_x_rad;
float gyro_y_rad;
float gyro_z_rad;
float pitch,ko;
float axx,ayy,azz,Gxx,Gyy,Gzz;


float Ro =0.0f;
float t=0.0f;
float pp,rr;
uint8_t i2cBuf[14];
 

void MPU_9250_Init(void){
	
	i2cBuf[0] = 25   ;	 //    exit sleep 
	i2cBuf[1] = 0x6D;
	HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);

i2cBuf[0] = 28   ;			//Register address: Accelerometer config 1
	i2cBuf[1] = 0x08 ;		//Data to write, 4g range
	HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);
	
	
	i2cBuf[0] = 27   ;			 //Register 27 – Gyroscope Configuration
	i2cBuf[1] = 0x18 ;		   // gyro full scale = +/- 2000dps
	HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);

	
	
	
	
	
	i2cBuf[0] = 25   ;	 //    Sample Rate Divider , sample rate = 1kHz / 10 = 100Hz   *************
	i2cBuf[1] = 0x09;
	HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);

	
	
	 // Register 26 – Configuration   DLPF_CFG
	 i2cBuf[0] = 26   ;
	 i2cBuf[1] = 0x01   ; // 1 POUR AVOIR 1kh frequence 
	 HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);
	
	 
	 
	 //Register 27 – Gyroscope Configuration
	 i2cBuf[0] = 27  ;
	 i2cBuf[1] = 0x03   ;
	 HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);
	 
	 
	 //Register 29 – Accelerometer Configuration 2
	  i2cBuf[0] = 29 ;
	 i2cBuf[1] = 0x08 ;
	 HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);
	 
	 
	// Register 29 – Accelerometer Configuration 2
	 i2cBuf[0] = 29 ;
	 i2cBuf[1] = 0x01 ;
	 HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 2, 10);


	 
	 
	 
	
	
	
	
	
	
	
	
	
	
}
		 
void mpudata(float *data){
	
	
i2cBuf[0] = 0x3B;			 
		HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 1, 10);
		 
		i2cBuf[1] = 0x00;
		HAL_I2C_Master_Receive(&hi2c1, mpu9265Address|0x01, &i2cBuf[1], 14, 10);
		
		ax = -(i2cBuf[1]<<8 | i2cBuf[2]);
		ay = -(i2cBuf[3]<<8 | i2cBuf[4]);
		az = 	(i2cBuf[5]<<8 | i2cBuf[6]);
	  T = -(i2cBuf[7]<<8 | i2cBuf[8]);
	  Gx = -(i2cBuf[9]<<8 | i2cBuf[10]);
  	Gy = -(i2cBuf[11]<<8 | i2cBuf[12]);
	  Gz =(i2cBuf[13]<<8 | i2cBuf[14]);
	   
	
	 
	 
			  Xaccel= ax/8192.0;
			  Yaccel= ay/8192.0;
			  Zaccel= az/8192.0;
				
			  tempe= (T/340)+36.53;
				
				gyro_x_rad = Gx /939.650784f;
				gyro_y_rad = Gy /939.650784f;
				gyro_z_rad = Gz /939.650784f;
				
				data[0]=Xaccel;
				data[1]=Yaccel;
				data[2]=Zaccel;
				data[3]=tempe;
				data[4]=gyro_x_rad ;
				data[5]=gyro_y_rad;
				data[6]=gyro_z_rad;
				
	
	
	
	
	
	
	   
   
		HAL_Delay(50);
		
}


void calculdata(void){
	 
 
	 float data[7];
	   
	 mpudata(data);
	
	
	
	axx=data[0];
	ayy=data[1];
	azz=data[2];
	
	
	Gxx=data[4];
	Gyy=data[5];
	Gzz=data[6];
	
	 MadgwickAHRSupdateIMU(Gxx, Gyy, Gzz, axx,ayy, azz);
	
	  
	 pitch =  atan2 (2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	 ko = asin(2*q0*q2 - 2*q1*q3); 
	 
	 cp = (pitch *180)/pi;
	 cr = (ko *180)/pi;
	 	 
}



//void calibrate(void){

//int16_t data[7];
//    uint16_t iNumCM = 50;
//    for (int i = 0; i < iNumCM; i++) {
//        mpudata(data);
//        fGX_Cal += data[4];
//        fGY_Cal += data[5];
//        fGZ_Cal += data[6];
//			  fax_cal += data[0];
//			  fay_cal += data[1];
//			  faz_cal += data[2];
//        HAL_Delay(10);  
//    }
//		
//    fGX_Cal /= iNumCM;
//    fGY_Cal /= iNumCM;
//    fGZ_Cal /= iNumCM;
//		
//		faz_cal /= iNumCM;
//		fay_cal /= iNumCM;
//		fax_cal /= iNumCM;
//		
//		if(fax_cal <0 ){ fax_cal =  -fax_cal; }
//			if(fay_cal <0 ){ fay_cal =  -fay_cal; }
//			
//				if(faz_cal > 8192 ){ faz_cal =  faz_cal-8192; }
//				if(faz_cal < 8192 ){ faz_cal =  (faz_cal-8192); }
//				
//		
//		
//		
//		
//		
//		
//		uiMPU6050_TicksCNT = HAL_GetTick();


//}