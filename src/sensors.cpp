
    
void getFrontDistance() {

    long duration1;
    long duration2;
    float distance1;
    float distance2;

        //Pin modes set to Output

    pinMode(Ultrasonic_TRIG_PIN1, OUTPUT);
    pinMode(Ultrasonic_TRIG_PIN2, OUTPUT);
        

        //Pin modes set to Input

    pinMode(Ultrasonic_ECHO_PIN1, INPUT);
    pinMode(Ultrasonic_ECHO_PIN2, INPUT);
    



    digitalWrite(Ultrasonic_TRIG_PIN1, LOW);
    digitalWrite(Ultrasonic_TRIG_PIN2, LOW);
    delayMicroseconds(2);

    digitalWrite(Ultrasonic_TRIG_PIN1, HIGH);
    digitalWrite(Ultrasonic_TRIG_PIN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(Ultrasonic_TRIG_PIN1, LOW);
    digitalWrite(Ultrasonic_TRIG_PIN2, LOW);


    print("Distance 1: " + distance1 + "cm");
    print("Distance 2: " + distance2 + "cm");

    delay(300);

        

    }


void detectObstacleTest() {
    Serial.begin (9600);

    Serial.print("Analog Value: ");
    Serial.println(sensorValue1);
    delay(200);
    
    //To test the other sensor, switch the value to print to sensorValue2
}


void detectFlameTest() {
    Serial.begin(9600);

    Serial.print("Analog Value: ");
    Serial.println(sensorValue3);
    delay(200)

}


void detectObstacleInteractionTest(){

    //ezButton installation via ArduinoIDE required to run this code 
    #include <ezButton.h>

    ezButton limitSwitch(0);

    Serial.begin(9600);
    limitSwitch.setDebounceTime(50)

    limitSwitch.loop();

    if(limitSwitch.isPressed())
        Serial.println("Untouched to Touched");

    if(limitSwitch.isRelease())
        Serial.println("Touched to Untouched");

    if(state == HIGH)
        Serial.println("Untouched")

    if(state == LOW)
        Serial.println("Touched")


}