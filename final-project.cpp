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

//Global Variables

bool isDoorClosed = true;


/////////////////////////////////////////

void setup(){

    init_motor();
    init_ultrasonic();
    init_temperature();

    Serial.begin(9600);
    
}


void loop(){
    // Check confitions of peripherals
    // Check ultrasonic
    long ultrasonic_distance = get_ultrasonic_distance();

    // Check temperature
    double temperature = get_temperature();

    bool openDoor = false;

    // Check conditions

    // If the conditions inside the lab are not hazardous
    if(!is_hazardous(temperature)){
        
        // If someone is in close range of door
        if(is_in_range(ultrasonic_distance)){
            door_actions(true);
        }
        else{
            door_actions(false);
        }
    }

    // If conditions in lab are hazardous
    else{

    }
        
    
}

bool is_hazardous(double temperature){
    return (temperature > 20 && temperature < 30);
}

bool is_in_range(double ultrasonic_distance){
    return (ultrasonic_distance < 50);
}

// Action funcitons
void door_actions(bool doorOpen){
    if(isDoorClosed){
        if(doorOpen){
            openDoor();
            isDoorClosed = false;
        }
    }
    
    else{
        if(!doorOpen){
            closeDoor();
            isDoorClosed = true;
        }
    }
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
double get_temperature(){
    int sensor_input = analogRead(TEMPERATURE_SENSOR_PIN);
    double temperature;
    temperature = (double)sensor_input / 1024;       //find percentage of input reading
    temperature = temperature * 5;                 //multiply by 5V to get voltage
    temperature = temperature - 0.5;               //Subtract the offset 
    temperature = temperature * 100;
    
    return temperature;
}



// Misc functions
long microsecondsToCentimeters(long microseconds)
{
    return microseconds / 29 / 2;
}

// Init funcitons
void init_temperature(){
    pinMode(TEMPERATURE_SENSOR_PIN, INPUT);
}
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