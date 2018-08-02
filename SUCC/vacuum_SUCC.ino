
 
int SENSOR_PIN = 0; // center pin of the potentiometer YALL CAN CHANGE THIS TO WHATEVER YALL NEED
 
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to RPWM pin on the motor driver
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to LPWM pin on the motor driver
 
void setup()
{
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  Serial.begin(9600);
}
 
void loop()
{
  int sensorValue = analogRead(SENSOR_PIN);
  Serial.println(sensorValue);
  // sensor value is in the range 0 to 1023
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (sensorValue <300)
  {
    // reverse rotation
    int reversePWM = -(sensorValue - 300) / 2;
    digitalWrite(LPWM_Output, 0);
    digitalWrite(RPWM_Output, 1); //MAKES THE VACUUM SUCK 
  }
  else
  {
    // forward rotation
    int forwardPWM = (sensorValue - 300) / 2;
    digitalWrite(LPWM_Output, 0);
    digitalWrite(RPWM_Output, 0); //MAKES IT STOP SUCKING
  }
}
