#include <Wire.h>

int gyro_x, gyro_y, gyro_z;
int temperature;
long loop_timer;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_pitch, angle_roll,angle_yaw;


void setup()
{
  Wire.begin();
  Serial.begin(115200);
  setup_mpu_6050_registers();
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){ 
  read_mpu_6050_data();  
  gyro_x_cal += gyro_x; 
  gyro_y_cal += gyro_y;  
  gyro_z_cal += gyro_z; 
  delay(3);  
  }
  gyro_x_cal /= 2000; 
  gyro_y_cal /= 2000;   
  gyro_z_cal /= 2000; 
  loop_timer = micros();
}

void loop()
{
   read_mpu_6050_data();
   gyro_x -= gyro_x_cal;   
   gyro_y -= gyro_y_cal; 
   gyro_z -= gyro_z_cal;   
   angle_pitch += (gyro_x * 0.01/131.0);  
   angle_roll  += (gyro_y *0.01/131.0); 
   angle_yaw   += (gyro_z*0.01/131.0);
   Serial.print(angle_pitch);  
   Serial.print(" ");
   Serial.print(angle_roll);
   Serial.print(" ");
   Serial.println(angle_yaw);
   while(micros() - loop_timer < 10000);  
   loop_timer = micros();       
}

void read_mpu_6050_data(){                                            
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                           
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                                 
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                  
  Wire.endTransmission();                                              
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                   
  Wire.write(0x00);                                                  
  Wire.endTransmission();                                            
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x1B);                                                   
  Wire.write(0x00);                                                    
  Wire.endTransmission();                                             
}

