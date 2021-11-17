// Pins

// Servo motor
#define SERVO_MOTOR_PIN 6

// Ultrasonic
#define ULTRASONIC_TRIGGER_PIN 8
#define ULTRASONIC_ECHO_PIN 7

// Temperature and Humidity
#define DHT_SENSOR_PIN A1


/////////////////////////////////////////

// Other Constants
#define MOTOR_DELAY 1000
#define HAZARD_MIN_TEMP 20
#define HAZARD_MAX_TEMP 30
#define HAZARD_MIN_HUMIDITY 30
#define HAZARD_MAX_HUMIDITY 50
#define ULTRASONIC_MAX_DISTANCE 20

#define LOOP_DELAY_MS 1000

/////////////////////////////////////////


//Global Variables
bool isDoorClosed = true;


/////////////////////////////////////////


#include "DHT.h"

DHT dht(DHT_SENSOR_PIN, DHT11);

#include <Servo.h>

Servo servoMotor;

void setup(){

    // Initialize the servo motor
    init_servo();

    // Initialize the ultrasonic sensor
    init_ultrasonic();

    // Initialize the temperature and humidity sensor
    init_dht();

    // Begin serial communication
    Serial.begin(9600);
}


void loop(){
    
    /* 
        Check conditions of peripherals
    */

    // Check ultrasonic
    long ultrasonic_distance = get_ultrasonic_distance();

    // Check temperature
    double temperature = getTemperature();

    // Check humidity
    double humidity = getHumidity();


    // Print readings from peripherals
    Serial.print("Ultrasonic Distance: ");
    Serial.println(ultrasonic_distance);
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    
    /* 
        Check conditions
    */

    // If the conditions inside the lab are not hazardous
    if(!is_hazardous(temperature, humidity)){

        Serial.println("Non-hazardous");
        
        // If someone is in close range of door
        if(is_in_range(ultrasonic_distance))
            door_actions(true);
        
        // If no one in range, close the door
        else
            door_actions(false);
        
    }

    // If conditions in lab are hazardous
    else{
        // Keep the door closed
        door_actions(false);
    }

    delay(LOOP_DELAY_MS);        
    
}

bool is_hazardous(double temperature, double humidity){
    /* 
        Safe condition is when temperature is between HAZARD_MIN_TEMP and HAZARD_MAX_TEMP, and
        the humidity is between HAZARD_MIN_HUMIDITY and HAZARD_MAX_HUMIDITY
        
    */
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

    // If door is closed
    if(isDoorClosed){

        // Open door
        if(doorOpen){
            openDoor();
            isDoorClosed = false;
        }
        
        /* 
            If door is already closed state and it is requested to close, 
            do nothing.
        */
    }
    
    else{

        // Close door
        if(!doorOpen){
            closeDoor();
            isDoorClosed = true;
        }
        
        /*
            If door is already open state and it is requested to open, 
            do nothing.
        */
    }
}

void openDoor()
{  
    Serial.println("Opening door");
    servo_open();   
}

void closeDoor()
{
    Serial.println("Closing door");
    servo_close(); 
}

void servo_open(){
    servoMotor.write(90);
    delay(1000);
}

void servo_close(){
    servoMotor.write(0);
    delay(1000);
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
void init_ultrasonic()
{
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}
void init_dht(){
    dht.begin();
}
void init_servo(){
    servoMotor.attach(SERVO_MOTOR_PIN);
}