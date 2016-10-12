/*
 * This  code calcultes linear accleration form the acclerometer
 * Refer to below link for the reference http://www.kircherelectronics.com/blog/index.php/11-android/sensors/10-low-pass-filter-linear-acceleration
 */


#include "MegunoLink.h"
#include <Wire.h>
float x = 0;
TimePlot XPlot("X Plot"),YPlot("Y Plot"),ZPlot("Z Plot");
#define rad2degree 57.3 
float gForceX, gForceY, gForceZ; 
long accelX, accelY, accelZ;
float accel_x_cal,accel_y_cal,accel_z_cal;
float angle_x,angle_y,angle_z; 
float angleX_gyro,angleY_gyro,angleZ_gyro;                       //Variables for angles calculated from gyro
int gyro_x, gyro_y, gyro_z;                                       //Variables for RAW gyro values
int temperature;
long loop_timer;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;   
float gravity [3],linearAcceleration[3];

void setup() 
{
  Wire.begin();
  Serial.begin(9600); 
  setup_mpu_6050_registers();                                      //Setup MPU Registors
  callibrate_acc();                                                //Callibrate Accelerometer
  for (int cal_int = 0; cal_int < 2048 ; cal_int ++){              //Callibrate Gyroscope
  read_mpu_6050_data();  
  gyro_x_cal += gyro_x; 
  gyro_y_cal += gyro_y;  
  gyro_z_cal += gyro_z; 
  delay(1);  
  }
  gyro_x_cal >>= 11; 
  gyro_y_cal >>= 11;   
  gyro_z_cal >>= 11; 
  loop_timer = micros();
}

void loop() 
{
  read_mpu_6050_data();
  processAccelData();
  gravity[0] = 0.9 * gravity[0] + (1 - 0.9) * gForceX;                  //Calculate the gravity component of the acclerometer
  gravity[1] = 0.9 * gravity[1] + (1 - 0.9) * gForceY;
  gravity[2] = 0.9 * gravity[2] + (1 - 0.9) * gForceZ;
 
  linearAcceleration[0] = gForceX - gravity[0];                         //Substract the gravity component from the Raw readings to get linear acceleration
  linearAcceleration[1] = gForceY - gravity[1];
  linearAcceleration[2] = gForceZ - gravity[2];
  
  XPlot.SendData("X Plot", linearAcceleration[0]);
  YPlot.SendData("Y Plot", linearAcceleration[1]);
  ZPlot.SendData("Z Plot", linearAcceleration[2]);
  while(micros() - loop_timer < 10000);  
  loop_timer = micros(); 
}

void callibrate_acc()
{
  int i;
  for (i=1;i<=1000;i++){
    read_mpu_6050_data();
    accel_x_cal+=(accelX);
    accel_y_cal+=(accelY);
    accel_z_cal+=(accelZ);
   // Serial.print(".");
    delay(1);
  }
  //Serial.println(".");
  accel_x_cal/=1000;
  accel_y_cal/=1000;
  accel_z_cal/=1000;
} 

void read_mpu_6050_data(){                                            
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                           
  while(Wire.available() < 14);                                        
  accelX = Wire.read()<<8|Wire.read();                                 
  accelY = Wire.read()<<8|Wire.read();                                  
  accelZ = Wire.read()<<8|Wire.read();                                  
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                
}

void processAccelData(){
  gForceX = (accelX-accel_x_cal) / 16384.0;             //Refer 0G offset callibration Application note by Freescale calculating the offsets
  gForceY = (accelY-accel_y_cal)/ 16384.0; 
  gForceZ = (accelZ-(accel_z_cal-16384)) / 16384.0;

//  angles calculated from the accelerometer
  
 }

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                  
  Wire.endTransmission();                                              
  //Configure the accelerometer (+/-2g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                   
  Wire.write(0x00);                                                  
  Wire.endTransmission();                                            
  //Configure the gyro (250dps full scale)
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x1B);                                                   
  Wire.write(0x00);                                                    
  Wire.endTransmission();                                             
}

