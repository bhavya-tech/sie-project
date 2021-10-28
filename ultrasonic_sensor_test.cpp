// Pins
// Motor
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6

// Ultrasonic
#define ULTRASONIC_TRIGGER_PIN 8
#define ULTRASONIC_ECHO_PIN 7

// Temperature
#define TEMPERATURE_SENSOR_PIN A0



/////////////////////////////////////////
// Other Constants
#define MOTOR_DELAY 1000

/////////////////////////////////////////



long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void setup() {

    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

    Serial.begin(9600); // Starting Serial Terminal
}

long getDistance() {
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

void loop(){

    long cm = getDistance();
    Serial.println(cm);
    delay(100);
}


