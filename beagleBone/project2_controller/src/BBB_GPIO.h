#ifndef BBB_GPIO
#define BBB_GPIO

// Digital IO
int digitalEnable(int pinNumber);
int digitalWrite(int pinNumber, int value);
int digitalRead(int pinNumber);

// PWM output
int enablePWM(int pwmNumber, int period);
int writePWM(int pwmNumber, int duty_cycle);
int disablePWM(int pwmNumber);

// eQEP (encoder)
long int readEQEP(int eQEPnumber);

#endif
