// Pins
// Motor
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6

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
#define ULTRASONIC_MAX_DISTANCE 50

#include "DHT.h"

DHT dht(DHT_SENSOR_PIN, DHT11);

void init_dht(){
    dht.begin();
}

void setup(){
    Serial.begin(9600);
    init_dht();
}

float getTemperature(){
    return dht.readTemperature();
}

float getHumidity(){
    return dht.readHumidity();
}

void loop(){
    Serial.print("Temperature: ");
    Serial.print(getTemperature());
    Serial.print("\tHumidity: ");
    Serial.print(getHumidity());
    Serial.println();
    delay(1000);
}