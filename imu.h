#include <Wire.h>
#include "motors.h"

void circle(float&x)
{
  int xi=int(x);
  x=xi%360+(x-xi);
}

class imu{
  private:
  float prev_time=0,elapsed;
  float aX,aY,aZ,gex,gey,gez;
  int16_t accX,accY,accZ,gyrX,gyrY,gyrZ,temp;

  public:

  float angleX=0,angleY=0,angleZ=0,acc;
  float gyrVelX,gyrVelY,gyrVelZ,angAccX,angAccY,angAccZ,FangAccX=0,FangAccY=0,FangAccZ=0;

  imu(){
    Wire.begin();
  }
  
  imu(int a){
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
  }

  void update(){
    elapsed=(float)(millis()-prev_time)/1000.0;
    prev_time=millis();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,14,true);

    accX=Wire.read()<<8|Wire.read();
    accY=Wire.read()<<8|Wire.read();
    accZ=Wire.read()<<8|Wire.read();
    temp=Wire.read()<<8|Wire.read();
    gyrX=Wire.read()<<8|Wire.read();
    gyrY=Wire.read()<<8|Wire.read();
    gyrZ=Wire.read()<<8|Wire.read();

    aX=float(accX)/4096.0;
    aY=float(accX)/4096.0;
    aZ=float(accX)/4096.0;

    acc=sqrt(aX*aX+aY*aY+aZ*aZ);

    angAccX=-atan2(accY,accZ)*RAD_TO_DEG+180;
    angAccY=-atan2(accX,accZ)*RAD_TO_DEG+180;
    angAccZ=-atan2(accX,accY)*RAD_TO_DEG;
    if(angAccX>180)angAccX-=360;
    if(angAccY>180)angAccY-=360;
    if(angAccZ>180)angAccZ-=360;
    if(acc<1.5){
      FangAccX=0.92*FangAccX+0.08*angAccX;
      FangAccY=0.92*FangAccY+0.08*angAccY;
      FangAccZ=0.92*FangAccZ+0.08*angAccZ;
    }

    gyrVelX=-gyrX/65.5+gex;
    gyrVelY=gyrY/65.5-gey;
    gyrVelZ=gyrZ/65.5-gez;

    circle(angleX);
    circle(angleY);
    circle(angleZ);

    angleX=0.9996*(angleX+gyrVelX*elapsed)+0.0004*angAccX;
    angleY=0.9996*(angleY+gyrVelY*elapsed)+0.0004*angAccY;
    angleZ=0.9996*(angleZ+gyrVelZ*elapsed)+0.0004*angAccZ;
  }

  void reset(){
    angleX=angAccX;
    angleY=angAccY;
    angleZ=angAccZ;
  }

  void gyrCalib(int a=1000){
	  gex=gey=gez=0;
    for(int i=0;i<a;++i){
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);
      gex+=float(Wire.read()<<8|Wire.read())/65.5;
      gey+=float(Wire.read()<<8|Wire.read())/65.5;
      gez+=float(Wire.read()<<8|Wire.read())/65.5;
      m.setMotor();
      delay(2);
    }
    gex/=float(a);
    gey/=float(a);
    gez/=float(a);
  }
};
