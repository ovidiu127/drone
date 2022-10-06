#include <SPI.h>
#include <math.h>
#include "imu.h"

#include "pid.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "joy.h"
#include "SparkFun_VL53L1X.h"
#include "ComponentObject.h"
#include "RangeSensor.h"
#include "vl53l1x_class.h"
#include "vl53l1_error_codes.h"

#define ROLL 0
#define PITCH 1
#define YAW 2

#define X 0
#define Y 1
#define Z 2

SFEVL53L1X distanceSensor;
bool flightAuth=0;
int tim;
unsigned long long timer;
long long t1;

PID rollPID(1.1,0.013,1.75,400),pitchPID(1.1,0.013,1.75,400),yawPID(6,0.03,5,300);

int rc;
float elapsed;

float pid_out[3];
float yaw;

imu g;
int power[5],v,voltage,distance;

RF24 radio(7,8);
const byte add1[6]="00001";
const byte add2[6]="00002";

struct nod
{
  int x0,y0,k0;
  int x1,y1,k1;
}t;

void setup() {
  //Serial.begin(115200);
  DDRD|=B00000011;
  PORTD&=B11111100;
  radio.begin();
  radio.openWritingPipe(add2);
  radio.openReadingPipe(1,add1);
  radio.setPALevel(RF24_PA_MIN);
  radio.setRetries(0,0);
  radio.startListening(); 
  m.setMotor();
  int it=600;
  while(it-->0)
  {
    m.setMotor(2000,2000,2000,2000);
    delay(1);
  }
  DDRB|=B00100000;
  PORTB|=B00100000;
  PORTB&=B11011111;
  TWBR=12;
  g=imu(0);
  g.gyrCalib();
  distanceSensor.begin();
}

void loop() {
  elapsed=float(t1-millis())/1000;
  t1=millis();
  distanceSensor.startRanging();
  distance=distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  
  g.update();
  voltage=analogRead(A1);
  ++rc;
  if(radio.available())
  {
    radio.read(&t,sizeof(t));
    t.x0=map(t.x0,0,1023,1000,2000);
    t.x1=map(t.x1,0,1023,1000,2000);
    t.y0=map(t.y0,0,1023,1000,2000);
    t.y1=map(t.y1,0,1023,1000,2000);
    rc=0;
  }
  if(rc>200||t.y0<1051)reset();
  else fly();
}

void reset()
{
  m.setMotor();
  rollPID.reset();
  pitchPID.reset();
  yawPID.reset();
  g.reset();
}

void fly()
{
  yaw+=float(t.x0-1500)/50*elapsed;
  rollPID.getSetpoint(t.x1);
  pitchPID.getSetpoint(t.y1);
  yawPID.getSetpointStatic(yaw);
  pid_out[ROLL]=rollPID.getPID(g.angleX);
  pid_out[PITCH]=pitchPID.getPID(g.angleY);
  pid_out[YAW]=0;//yawPID.getPIDStatic(g.angleZ);
  calculateMotor();
  m.setMotor(power[1],power[2],power[3],power[4]);
}

void sendData()
{
  timer=millis();
  Serial.print(String("v")+String(voltage));
  Serial.print(String("d")+String(distance));
  Serial.print(String("x")+String(int(g.angleX)));
  Serial.print(String("y")+String(int(g.angleY)));
  Serial.print(String("z")+String(int(g.angleZ)));
  //Serial.println(String("m")+String(int(g.magneticZ)));
}

void readData()
{
  if(Serial.available()>5){Serial.end();Serial.begin(115200);return;}
  if(Serial.available())
  {
    char c=Serial.read();
    if(c=='a')c=Serial.read();else return;
    if(c=='0')flightAuth=0;
    else if(c=='1')flightAuth=1;
  }
}


void calculateMotor()
{
  int throttle=min(1800,t.y0);
  power[1]=throttle+pid_out[PITCH]+pid_out[ROLL]+pid_out[YAW];
  power[2]=throttle-pid_out[PITCH]+pid_out[ROLL]-pid_out[YAW];
  power[3]=throttle-pid_out[PITCH]-pid_out[ROLL]+pid_out[YAW];
  power[4]=throttle+pid_out[PITCH]-pid_out[ROLL]-pid_out[YAW];
  if(voltage>900&&voltage<1240)
  {
    power[1]+=power[1]*float(1240-voltage)/float(3500);
    power[2]+=power[2]*float(1240-voltage)/float(3500);
    power[3]+=power[3]*float(1240-voltage)/float(3500);
    power[4]+=power[4]*float(1240-voltage)/float(3500);
  }
  power[1]=max(1100,power[1]);power[1]=min(2000,power[1]);
  power[2]=max(1100,power[2]);power[2]=min(2000,power[2]);
  power[3]=max(1100,power[3]);power[3]=min(2000,power[3]);
  power[4]=max(1100,power[4]);power[4]=min(2000,power[4]);
}
