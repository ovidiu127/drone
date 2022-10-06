//PID controller

const float DEG2SET=13.42857;

class PID{
  private:
  float kp,ki,kd,maxPIDi,setpoint,PIDp,PIDi,PIDd,last_error;
  float period,last_time=0;
  public:

  PID(float _kp,float _ki,float _kd,float _maxPIDi){
    kp=_kp;
    ki=_ki;
    kd=_kd;
    maxPIDi=_maxPIDi;
    PIDp=PIDi=PIDd=last_error=0;
  }

  void reset(){
    PIDi=0;
  }

  void getSetpoint(int signal){
    if(signal>1530)setpoint=signal-1530;
    else if(signal<1470)setpoint=signal-1470;
    else setpoint=0;
  }

  float getPID(float angle){
    period=(millis()-last_time)/1000;
    last_time=millis();
    float error=angle-setpoint/15;
    PIDp=kp*error;
    if(error>-3&&error<3)
    {
      PIDi+=ki*error;
      if(PIDi>maxPIDi)PIDi=maxPIDi;
      if(PIDi<-maxPIDi)PIDi=-maxPIDi;
    }
    PIDd=kd*((error-last_error)/period);
    last_error=error;
    return max(-maxPIDi,min(maxPIDi,PIDp+PIDi+PIDd));
  }

  void getSetpointStatic(float _setpoint){
    setpoint=_setpoint;
  }

  float getPIDStatic(float angle){
    period=(millis()-last_time)/1000;
    last_time=millis();
    float error=angle-setpoint;
    PIDp=kp*error;
    if(error>-3&&error<3)
    {
      PIDi+=ki*error;
      if(PIDi>maxPIDi)PIDi=maxPIDi;
      if(PIDi<-maxPIDi)PIDi=-maxPIDi;
    }
    PIDd=kd*((error-last_error)/period);
    last_error=error;
    return max(-maxPIDi,min(maxPIDi,PIDp+PIDi+PIDd));
  }
};
