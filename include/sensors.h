//Ultrasonic Sensor Variables (Set as if we are using two Ultrasonic sensors)

int Ultrasonic_TRIG_PIN1 = 9;
int Ultrasonic_ECHO_PIN1 = 11;
duration1 = pulseIn(Ultrasonic_ECHO_PIN1, HIGH);
distance1 = duration1 * 0.034/2;

int Ultrasonic_TRIG_PIN2 = 10;
int Ultrasonic_ECHO_PIN2 = 12;
duration2 = pulseIn(Ultrasonic_ECHO_PIN2, HIGH);
distance2 = duration2 * 0.034/2;

//Active IR Sensor variables

const int ActiveIR_OUTPUT1 = A0;
int sensorValue1 = analogRead(ActiveIR_OUTPUT1); 
const int ActiveIR_Output2 = A1;
int sensorValue2 = analogRead(ActiveIR_Output2);


//Flame IR Sensor Variables
const int FlameIR_OUTPUT1 = A2;
int sensorValue3 = analogRead(FlameIR_OUTPUT1);
const int FlameIR_OUTPUT2 = A3;
int sensorValue4 = analogRead(FlameIR_OUTPUT2);
const int FlameIR_OUTPUT3 = A4;
int sensorValue5 = analogRead(FlameIR_OUTPUT3);
const int FlameIR_OUTPUT4  = A5;
int sensorValue6 = analogRead(FlameIR_OUTPUT4);

//KW12-3 Switch Variables
const int KW_pin1 = 0;
const int KW_pin2 = 1;
int state = limitSwitch.getState()
