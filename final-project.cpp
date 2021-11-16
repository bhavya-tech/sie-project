// Pins
// Motor
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6

// Ultrasonic
#define ULTRASONIC_TRIGGER_PIN 8
#define ULTRASONIC_ECHO_PIN 7

// Temperature
// #define TEMPERATURE_SENSOR_PIN A0

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

/////////////////////////////////////////

#define USING_SERVO_MOTOR true

//Global Variables

bool isDoorClosed = true;


/////////////////////////////////////////


#include "DHT.h"

DHT dht(DHT_SENSOR_PIN, DHT11);

#include <Servo.h>

Servo servoMotor;

void setup(){

    init_motor();
    init_ultrasonic();
    //init_temperature();
    init_dht();

    Serial.begin(9600);
}


void loop(){
    // Check conditions of peripherals

    // Check ultrasonic
    long ultrasonic_distance = get_ultrasonic_distance();

    // Check temperature
    double temperature = getTemperature();

    // Check humidity
    double humidity = getHumidity();


    Serial.print("Ultrasonic Distance: ");
    Serial.println(ultrasonic_distance);
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    
    // Check conditions

    // If the conditions inside the lab are not hazardous
    if(!is_hazardous(temperature, humidity)){
        
        // If someone is in close range of door
        if(is_in_range(ultrasonic_distance)){
            door_actions(true);
        }

        // If no one in range, close the door
        else{
            door_actions(false);
        }
    }

    // If conditions in lab are hazardous
    else{
        // Keep the door closed
        door_actions(false);
    }

    delay(500);        
    
}

bool is_hazardous(double temperature, double humidity){
    // Safe condition is when temperature is between HAZARD_MIN_TEMP and HAZARD_MAX_TEMP.
    return !(
        temperature > HAZARD_MIN_TEMP && temperature < HAZARD_MAX_TEMP |
        humidity > HAZARD_MIN_HUMIDITY && humidity < HAZARD_MAX_HUMIDITY
        );
}

bool is_in_range(double ultrasonic_distance){
    // If the distance is less than ULTRASONIC_MAX_DISTANCE, return true
    return (ultrasonic_distance < ULTRASONIC_MAX_DISTANCE);
}

// Action funcitons
void door_actions(bool doorOpen){
    if(isDoorClosed){
        // If door is closed, open it
        if(doorOpen){
            openDoor();
            isDoorClosed = false;
        }
        else{
            // If door is already closed state and it is requested to close, do nothing
        }
    }
    
    else{
        if(!doorOpen){
            closeDoor();
            isDoorClosed = true;
        }
        else{
            // If door is already open state and it is requested to open, do nothing
        }
    }
}

void openDoor()
{   
    if(USING_SERVO_MOTOR)
    {
        servo_open();
    }
    else{
        // Open the door
        Serial.println("Opening door");
        digitalWrite(MOTOR_PIN1, HIGH);
        digitalWrite(MOTOR_PIN2, LOW);
        delay(MOTOR_DELAY);
        digitalWrite(MOTOR_PIN1, LOW);
        Serial.println("Door opened");

    }
}

void closeDoor()
{
    if(USING_SERVO_MOTOR)
    {
        servo_close();
    }
    else{
        // Close the door

        Serial.println("Closing door");
        digitalWrite(MOTOR_PIN2, HIGH);
        digitalWrite(MOTOR_PIN1, LOW);
        delay(MOTOR_DELAY);
        digitalWrite(MOTOR_PIN2, LOW);
        Serial.println("Door closed");
    }
}

void servo_open(){
    servoMotor.write(90);
}

void servo_close(){
    servoMotor.write(0);
}



// Sensor funcitons

// Ultrasonic
long get_ultrasonic_distance()
{
    long duration, distance;

    digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);

    duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

    distance = microsecondsToCentimeters(duration);

    return distance;
}

// Temperature
float getTemperature(){
    return dht.readTemperature();
}

float getHumidity(){
    return dht.readHumidity();
}



// Misc functions
long microsecondsToCentimeters(long microseconds)
{
    return microseconds / 29 / 2;
}

// Init funcitons
// void init_temperature(){
//     pinMode(TEMPERATURE_SENSOR_PIN, INPUT);
// }
void init_motor()
{
	pinMode(MOTOR_PIN1, OUTPUT);
	pinMode(MOTOR_PIN2, OUTPUT);
}
void init_ultrasonic()
{
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}
void init_dht(){
    dht.begin();
}