#include <Wire.h>
#define rad2degree 57.3 
float gForceX, gForceY, gForceZ;                //Variables to store Accelerometer values converted to G values
long accelX, accelY, accelZ;
float accel_x_cal,accel_y_cal,accel_z_cal;     //Varialbes for Offsets of the Acclerometer Readings
float angle_x,angle_y,angle_z;                //Variables for tilt angles measured from the accleometer
      
void setup()
{
  Wire.begin();
  Serial.begin(115200);
  setup_mpu_6050_registers();  
  callibrate_mpu();
}

void loop() 
{
  recordAccelRegisters();
  processAccelData();
  printData();
  delay(10);                            //100Hz Sampling Rate
}

void callibrate_mpu()
{
  int i;
  for (i=1;i<=2000;i++){
    recordAccelRegisters();
    accel_x_cal+=(accelX);
    accel_y_cal+=(accelY);
    accel_z_cal+=(accelZ);
   // Serial.print(".");
    delay(1);
  }
  //Serial.println(".");
  accel_x_cal/=2000;
  accel_y_cal/=2000;
  accel_z_cal/=2000;
  Serial.print(accel_x_cal);
  Serial.print(" ");
  Serial.print(accel_y_cal);
  Serial.print(" ");
  Serial.print(accel_z_cal);
} 

void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read(); 
  //processAccelData();
}

void processAccelData(){
  gForceX = (accelX-accel_x_cal) / 16384.0;             //Refer 0G offset callibration Application note by Freescale calculating the offsets
  gForceY = (accelY-accel_y_cal)/ 16384.0; 
  gForceZ = (accelZ-(accel_z_cal-16384)) / 16384.0;
  angle_x = atan2(gForceX,sqrt((gForceY*gForceY)+(gForceZ*gForceZ)))*(float)rad2degree;
  angle_y = atan2(gForceY,(sqrt((gForceX*gForceX)+(gForceZ*gForceZ))))*(float)rad2degree;
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
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();                                              
}

void printData() {
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" ");
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" ");
  Serial.print(gForceZ);
  Serial.println(" ");
/* Serial.print(angle_x);
 Serial.print(" angleY=");
 Serial.print(angle_y);
 Serial.print(" angleZ=");
 Serial.println(angle_z);*/
  //Serial.print("AngleX=");
//  Serial.print(angleX_gyro);
 // Serial.print(" angleY=");
//  Serial.print(angleY_gyro);
 // Serial.print(" angleZ=");
 // Serial.println(angleZ_gyro);
  
}



