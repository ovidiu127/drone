class motors
{
  public:
    motors()
    {
      DDRD|=B01101000;
      DDRB|=B00000010;
    }

    void setMotor(int f1=1000,int f2=1000,int f3=1000,int f4=1000)
    {
      noInterrupts();
      PORTB|=B00000010;
      delayMicroseconds(f1);
      PORTB&=B11111101;
      PORTD|=B01000000;
      delayMicroseconds(f2);
      PORTD&=B10111111;
      PORTD|=B00100000;
      delayMicroseconds(f3);
      PORTD&=B11011111;
      PORTD|=B00001000;
      delayMicroseconds(f4);
      PORTD&=B11110111;
      interrupts();
    }
}m;
