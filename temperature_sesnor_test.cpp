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


double get_temperature(){
    int sensor_input = analogRead(TEMPERATURE_SENSOR_PIN);
    double temperature;
    temperature = (double)sensor_input / 1024;       //find percentage of input reading
    temperature = temperature * 5;                 //multiply by 5V to get voltage
    temperature = temperature - 0.5;               //Subtract the offset 
    temperature = temperature * 100;
    
    return temperature;
}

void init_temperature(){
    pinMode(TEMPERATURE_SENSOR_PIN, INPUT);
}

void setup(){
    init_temperature();
    Serial.begin(9600);
}

void loop(){
    double temperature = get_temperature();
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    delay(1000);
}