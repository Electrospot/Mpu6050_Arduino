/********************************************************************************
 MPU 6050 Based Complimentary filter to calculate the tilt around x y & z axis
 ******************************************************************************** */



#include <Wire.h>
#define rad2degree 57.3 
float gForceX, gForceY, gForceZ;                                 //Variables to store Accelerometer values converted to G values
long accelX, accelY, accelZ;
float accel_x_cal,accel_y_cal,accel_z_cal;                       //Varialbes for Offsets of the Acclerometer Readings
float angle_x,angle_y,angle_z;                                   //Variables for tilt angles measured from the accleometer
float angleX_gyro,angleY_gyro,angleZ_gyro;                       //Variables for angles calculated from gyro
int gyro_x, gyro_y, gyro_z;                                       //Variables for RAW gyro values
int temperature;
long loop_timer;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;                        //Variables for callibrate gyroscope values
float angle_pitch, angle_roll;                                  //Variables for Pitch and Roll
float pitch,roll;
#define M_PII    3.14159265359
void setup() 
{
 Wire.begin();
 Serial.begin(115200);
 setup_mpu_6050_registers();                                      //Setup MPU Registors
 callibrate_acc();                                                //Callibrate Accelerometer
for (int cal_int = 0; cal_int < 2048 ; cal_int ++){               //Callibrate Gyroscope
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
 
 //Gyro callibrated Values 
  gyro_x -= gyro_x_cal;         
  gyro_y -= gyro_y_cal; 
  gyro_z -= gyro_z_cal;   
  
  angleX_gyro = (gyro_x * 0.01/131.0)+angle_roll;  
  angleY_gyro = (gyro_y *0.01/131.0)+angle_pitch; 
 
  //Complimentary Filter
  angle_roll=0.95*angleX_gyro+(1.0-0.95)*angle_x; 
  angle_pitch=0.95*angleY_gyro+(1.0-0.95)*angle_y;

  pitch = 180 *(atan(gForceX/(gForceY*gForceY + gForceZ * gForceZ)))/M_PII;
  roll = 180 *(atan(gForceY/(gForceX * gForceX + gForceZ * gForceZ)))/M_PII; 
 /* Serial.print(angle_pitch);
  Serial.print(" ");
  Serial.println(angle_roll);*/

  Serial.print(F("DEL:"));              //Delta T
  Serial.print(10, DEC);
  Serial.print(F("#ACC:"));              //Accelerometer angle
  Serial.print(pitch, 2);
  Serial.print(F(","));
  Serial.print(roll, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(angleX_gyro, 2);        //Gyroscope angle
  Serial.print(F(","));
  Serial.print(angleY_gyro, 2);
  Serial.print(F(","));
  Serial.print(angleZ_gyro, 2);
  Serial.print(F("#FIL:"));             //Filtered angle
  Serial.print(angle_pitch, 2);
  Serial.print(F(","));
  Serial.print(angle_roll, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.println(F(""));
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
  angle_y = atan(1*gForceX/sqrt((gForceY*gForceY)+(gForceZ*gForceZ)))*(float)rad2degree;
  angle_x = atan(gForceY/sqrt((gForceX*gForceX)+(gForceZ*gForceZ)))*(float)rad2degree;
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
