// Pins
// Motor
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6

#define SERVO_MOTOR_PIN 6

// Ultrasonic
#define ULTRASONIC_TRIGGER_PIN 8
#define ULTRASONIC_ECHO_PIN 7

// Temperature
#define TEMPERATURE_SENSOR_PIN A0

// Humidity
#define DHT_SENSOR_PIN A1


/////////////////////////////////////////
// Other Constants
#define MOTOR_DELAY 1000
#define HAZARD_MIN_TEMP 20
#define HAZARD_MAX_TEMP 27
#define HAZARD_MIN_HUMIDITY 40
#define HAZARD_MAX_HUMIDITY 50
#define ULTRASONIC_MAX_DISTANCE 50

/////////////////////////////////////////

//Global Variables

bool isDoorClosed = true;


/////////////////////////////////////////

#include <Servo.h>

Servo servoMotor;

void servo_setup(){
    servoMotor.attach(SERVO_MOTOR_PIN);
}

void setup(){
    servo_setup();
    Serial.begin(9600);
}

void loop(){
    Serial.print("servo open");
    servo_open();
    delay(1000);
    Serial.print("servo close");
    servo_close();
    delay(1000);
}

void servo_open(){
    servoMotor.write(90);
}

void servo_close(){
    servoMotor.write(0);
}