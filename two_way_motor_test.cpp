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

void init_motor()
{
	pinMode(MOTOR_PIN1, OUTPUT);
	pinMode(MOTOR_PIN2, OUTPUT);
}

void setup()
{
	init_motor();
	Serial.begin(9600);
}

void loop()
{
	Serial.println("Rotate left");
	openDoor();
	Serial.println("Rotate right");
	closeDoor();
}

void openDoor()
{
	digitalWrite(MOTOR_PIN1, HIGH);
	digitalWrite(MOTOR_PIN2, LOW);
	delay(MOTOR_DELAY);
	digitalWrite(MOTOR_PIN1, LOW);
}

void closeDoor()
{
	digitalWrite(MOTOR_PIN2, HIGH);
	digitalWrite(MOTOR_PIN1, LOW);
	delay(MOTOR_DELAY);
	digitalWrite(MOTOR_PIN2, LOW);
}