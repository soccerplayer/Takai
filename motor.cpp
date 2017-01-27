#include "motor.h"


void motor_control2(int motorL, int motorR)
{
  if (motorL == 0 && motorR == 0)
  {
    digitalWrite(LDIR, LOW);
    analogWrite(LPWM, motorL);
    analogWrite(RPWM, motorR);
    digitalWrite(RDIR, LOW);
  }

  else if (motorL <= 0 && motorR <= 0)
  {
    digitalWrite(LDIR, HIGH);
    analogWrite(LPWM, abs(motorL));
    analogWrite(RPWM, abs(motorR));
    digitalWrite(RDIR, HIGH);
  }
  else if (motorL <= 0 && motorR >= 0)
  {
    digitalWrite(LDIR, HIGH);
    analogWrite(LPWM, abs(motorL));
    analogWrite(RPWM, motorR);
    digitalWrite(RDIR, LOW);
  }
  else if (motorL >= 0 && motorR <= 0)
  {
    digitalWrite(LDIR, LOW);
    analogWrite(LPWM, motorL);
    analogWrite(RPWM, abs(motorR));
    digitalWrite(RDIR, HIGH);
  }
  else if (motorL >= 0 && motorR >= 0)
  {
    digitalWrite(LDIR, LOW);
    analogWrite(LPWM, motorL);
    analogWrite(RPWM, motorR);
    digitalWrite(RDIR, LOW);
  }
}

void motor_init()
{
  pinMode(LDIR, OUTPUT);
  pinMode(LPWM, OUTPUT); //pwm
  pinMode(RPWM, OUTPUT);
  pinMode(RDIR, OUTPUT); //pwm
}

