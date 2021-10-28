//2-Way motor control

#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define MOTOR_DELAY 1000


void setup()   {                
  
  pinMode(MOTOR_PIN1, OUTPUT); 
  pinMode(MOTOR_PIN2, OUTPUT);  
  Serial.begin(9600);
}


void loop()                     
{
	Serial.println("Rotate left");
	rotateLeftFull();
	Serial.println("Rotate right");
	rotateRightFull();
}


void rotateLeftFull(){
  digitalWrite(MOTOR_PIN1, HIGH); 
  digitalWrite(MOTOR_PIN2, LOW);    
  delay(MOTOR_DELAY); 
  digitalWrite(MOTOR_PIN1, LOW);    
}

void rotateRightFull(){
  digitalWrite(MOTOR_PIN2, HIGH); 
  digitalWrite(MOTOR_PIN1, LOW);    
  delay(MOTOR_DELAY); 
  digitalWrite(MOTOR_PIN2, LOW);  
}