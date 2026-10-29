long duration1;
long duration2;
float distance1;
float distance2;

void setup() {
    //Pin modes set to Output

    pinMode(Ultrasonic_TRIG_PIN1, OUTPUT);
    pinMode(Ultrasonic_TRIG_PIN2, OUTPUT);
    pinMode(ActiveIR_OUTPUT1, OUTPUT);
    pinMode(ActiveIR_Output2, OUTPUT);
    pinMode(FlameIR_OUTPUT1, OUTPUT);
    pinMode(FlameIR_OUTPUT2, OUTPUT);
    pinMode(FlameIR_OUTPUT3, OUTPUT);
    pinMode(FlameIR_OUTPUT4, OUTPUT);

    //Pin modes set to Input

    pinMode(Ultrasonic_ECHO_PIN1, INPUT);
    pinMode(Ultrasonic_ECHO_PIN2, INPUT);
}

void loop() {
    //Ultrasonic Sensor Code

    digitalWrite(Ultrasonic_TRIG_PIN1, LOW);
    digitalWrite(Ultrasonic_TRIG_PIN2, LOW);
    delayMicroseconds(2);

    digitalWrite(Ultrasonic_TRIG_PIN1, HIGH);
    digitalWrite(Ultrasonic_TRIG_PIN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(Ultrasonic_TRIG_PIN1, LOW);
    digitalWrite(Ultrasonic_TRIG_PIN2, LOW);


    print("Distance 1: " + distance1 + "cm")
    print("Distance 2: " + distance2 + "cm")

    delay(300)


    //Active IR Distance Sensor Code 

    digitalWrite()

}