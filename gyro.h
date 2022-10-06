#include <Wire.h>
#include <math.h>
#include "motors.h"

class gyro
{
  private:
    float gx,gy,gz;
    float geX=0,geY=0,geZ=0;
    float ptime=0,ctime=0,elapsed=0;

    void circle(float&x)
    {
      int xi=int(x);
      x=xi%360+(x-xi);
    }
    
  public:
    float accx=0,accy=0,accz=0,acc=0;
    float Accx=0,Accy=0,Accz=0;
    float aX=0,aY=0,aZ=0,afX=0,afY=0;
    float gX=0,gY=0,gZ=0,gXf=0,gYf=0,gZf=0;
    float magX=0,magY=0,magZ=0,refY=0;
    float angleX=0,angleY=0,angleZ=0,anglegX=0,anglegY=0;
    float maxmag=-9999999,minmag=9999999;
    float magneticZ=0;
    float accelXangle,accelYangle;
    
    gyro()
    {
      Wire.begin();
    }

    gyro(int a)
    {
      Wire.beginTransmission(0x68); // Start communication with MPU
      Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
      Wire.write(0x00);                    // Apply the desired configuration to the register
      Wire.endTransmission();              // End the transmission
  
      // Configure the gyro's sensitivity
      Wire.beginTransmission(0x68); // Start communication with MPU
      Wire.write(0x1B);                    // Request the GYRO_CONFIG register
      Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
      Wire.endTransmission();              // End the transmission
  
      // Configure the acceleromter's sensitivity
      Wire.beginTransmission(0x68); // Start communication with MPU
      Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
      Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
      Wire.endTransmission();              // End the transmission
  
      // Configure low pass filter
      Wire.beginTransmission(0x68); // Start communication with MPU
      Wire.write(0x1A);                    // Request the CONFIG register
      Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
      Wire.endTransmission();    
      Wire.beginTransmission(0x0C);
      Wire.write(0x0A);
      Wire.write(0x16);
      Wire.endTransmission();
    }

    void update(int m=0)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);
      accx=(Wire.read()<<8|Wire.read())/16384.0;
      accy=(Wire.read()<<8|Wire.read())/16384.0;
      accz=(Wire.read()<<8|Wire.read())/16384.0;
      Accx=0.99*Accx+0.01*accx;
      Accy=0.99*Accy+0.01*accy;
      Accz=0.99*Accz+0.01*accz;
      acc=sqrt(Accx*Accx+Accy*Accy+Accz*Accz);
      aX=-atan2(accy,accz)*RAD_TO_DEG-180;
      aY=-atan2(accx,accz)*RAD_TO_DEG-180;
      aZ=atan2(accy,accx)*RAD_TO_DEG;
      aX=((aX>180)?(aX-360):aX);
      aX=((aX<-180)?(aX+360):aX);
      aY=((aY>180)?(aY-360):aY);
      aY=((aY<-180)?(aY+360):aY);
      afX=0.9*afX+0.1*aX;
      afY=0.9*afY+0.1*aY;

      accelXangle = (atan2(accy, accz)) * 180 / PI-180;                                   //calculate roll
      accelYangle = (atan2(accx, sqrt(pow(accy, 2) + pow(accz, 2)))) * 180 / PI;   //calculate pitc

      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);
      gx=(Wire.read()<<8|Wire.read())/131;
      gy=(Wire.read()<<8|Wire.read())/131;
      gz=(Wire.read()<<8|Wire.read())/131;
      gX=-gx+geX;
      gY=gy-geY;
      gZ=gz-geZ;
      gXf=gXf*0.7+gX*0.3;
      gYf=gYf*0.7+gY*0.3;
      gZf=gZf*0.7+gZ*0.3;
      ptime=ctime;
      ctime=millis();
      elapsed=(ctime-ptime)/1000.0;
      if(m)
      {
        anglegX=angleX=aX;
        anglegY=angleY=aY;
        afX=aX;
        afY=aY;
      }
      else
      {
        angleX=0.9996*(angleX+gX*elapsed)+0.0004*aX;
        angleY=0.9996*(angleY+gY*elapsed)+0.0004*aY;
        //works ok -slight offset Right Forward
        anglegX+=gX*elapsed;
        anglegY+=gY*elapsed;
      }
      angleZ-=gZ*elapsed;
      
      Wire.beginTransmission(0x0C);
      Wire.write(0x03);
      Wire.endTransmission();
      Wire.requestFrom(0x0C,7);
      magX=Wire.read()|Wire.read()<<8;
      magY=Wire.read()|Wire.read()<<8;
      magZ=Wire.read()|Wire.read()<<8;
      if(magX>maxmag)
      {
        maxmag=magX;
        refY=magY;
      }
      if(magX<minmag)minmag=magX;
      magneticZ=(float)map(magX,minmag,maxmag,0,180);
      if(magY>refY)magneticZ+=180;
      else magneticZ-=180;
      magneticZ=abs(magneticZ);
      //if(abs(accx)<0.05&&abs(accy)<0.05)angleZ=magneticZ;
      circle(angleX);
      circle(angleY);
      circle(angleZ);
      if(angleX>180)angleX-=360;
      if(angleY>180)angleY-=360;
      if(angleZ>180)angleZ-=360;
    }

    void gyrocal()
    {
      angleX=0;
      angleY=0;
      angleZ=0;
      geX=0;
      geY=0;
      geZ=0;
      for(int i=0;i<2000;++i)
      {
        Wire.beginTransmission(0x68);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68,6,true);
        gx=(Wire.read()<<8|Wire.read())/131;
        gy=(Wire.read()<<8|Wire.read())/131;
        gz=(Wire.read()<<8|Wire.read())/131;
        geX+=gx;
        geY+=gy;
        geZ+=gz;
        m.setMotor();
        delay(2);
      }
      geX/=2000.0;
      geY/=2000.0;
      geZ/=2000.0;
    }
};
