#include "Wire.h"       
#include "I2Cdev.h"     
#include "MPU6050.h"    

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

void CrackByte( byte b, int variable[9] )
{
  byte i;
  
  for ( i=0; i < 9; ++i )
  {
    variable[i] = b & 1;
    b = b >> 1;
  }
}

MyData data;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  int axis_X[9];
  int axis_Y[9];

  // pinMode(17, OUTPUT);
  // pinMode(16, OUTPUT);
  // pinMode(15, OUTPUT);
  // pinMode(14, OUTPUT);
  // pinMode(13, OUTPUT);
  // pinMode(12, OUTPUT);
  // pinMode(11, OUTPUT);
  // pinMode(10, OUTPUT);
  // pinMode(9, OUTPUT);

  // pinMode(8, OUTPUT);
  // pinMode(7, OUTPUT);
  // pinMode(6, OUTPUT);
  // pinMode(5, OUTPUT);
  // pinMode(4, OUTPUT);
  // pinMode(3, OUTPUT);
  // pinMode(2, OUTPUT);
  // pinMode(1, OUTPUT);
  // pinMode(0, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.X = map(ax, -17000, 17000, 0, 480 ); // X axis data
  data.Y = map(ay, -17000, 17000, 0, 360); 
  data.Z = map(az, -17000, 17000, 0, 255);  // Y axis data
  delay(500);
  Serial.print("Axis X = ");
  Serial.print(data.X);
  Serial.print("  ");
  Serial.print("Axis Y = ");
  Serial.print(data.Y);
  Serial.print("  ");
  Serial.print("Axis Z  = ");
  Serial.println(data.Z);
  // CrackByte(data.X,axis_X);
  // CrackByte(data.Y,axis_Y);

  // analogWrite(17, axis_y[8]);
  // analogWrite(16, axis_y[7]);
  // analogWrite(15, axis_y[6]);
  // analogWrite(14, axis_y[5]);
  // analogWrite(13, axis_y[4]);
  // analogWrite(12, axis_y[3]);
  // analogWrite(11, axis_y[2]);
  // analogWrite(10, axis_y[1]);
  // analogWrite(9, axis_y[0]);
  
  // analogWrite(8, axis_x[8]);
  // analogWrite(7, axis_x[7]);
  // analogWrite(6, axis_x[6]);
  // analogWrite(5, axis_x[5]);
  // analogWrite(4, axis_x[4]);
  // analogWrite(3, axis_x[3]);
  // analogWrite(2, axis_x[2]);
  // analogWrite(1, axis_x[1]);
  // analogWrite(0, axis_x[0]);
}
