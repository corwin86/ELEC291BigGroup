#define FORWARDS        LOW   //motor direction forward
#define BACKWARDS       HIGH  //motor direction backward
#define LEFT_MOTOR      4     //left motor pin
#define RIGHT_MOTOR     7     //right motor pin
#define LEFT_SPEED_PIN  5     //left motor speed setting pin
#define RIGHT_SPEED_PIN 6     //right motor speed setting pin
#define BUMPER 13

void setup(){
pinMode(RIGHT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(BUMPER, INPUT_PULLUP);  
}

void loop(){
  while(digitalRead(BUMPER) == HIGH){
  int speed =255;
  digitalWrite(LEFT_MOTOR, FORWARDS);
  analogWrite(LEFT_SPEED_PIN, speed);
  digitalWrite(RIGHT_MOTOR, FORWARDS);
  analogWrite(RIGHT_SPEED_PIN, speed);
  }
  
  digitalWrite(LEFT_SPEED_PIN, 0);
  digitalWrite(RIGHT_SPEED_PIN, 0);
  
}
