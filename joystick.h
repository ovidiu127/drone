#ifndef JOYSTICK
#define JOYSTICK

class joystick
{
  private:
  
  int xpin,ypin,kpin;
  
  public:

  joystick(){}
  
  joystick(int xpin,int ypin,int kpin)
  {
    this->xpin=xpin;
    this->ypin=ypin;
    this->kpin=kpin;
    pinMode(this->kpin,INPUT);
    digitalWrite(this->kpin,HIGH);
  }

  int getX()
  {
    return analogRead(xpin);
  }

  int getY()
  {
    return analogRead(ypin);
  }

  bool getK()
  {
    return !digitalRead(kpin);
  }
};

#endif
