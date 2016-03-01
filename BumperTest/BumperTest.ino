#define FORWARDS        LOW   //motor direction forward
#define BACKWARDS       HIGH  //motor direction backward
#define LEFT_MOTOR      4     //left motor pin
#define RIGHT_MOTOR     7     //right motor pin
#define LEFT_SPEED_PIN  5     //left motor speed setting pin
#define RIGHT_SPEED_PIN 6     //right motor speed setting pin
#define BUMPER 13

void setup() {
  pinMode(RIGHT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(BUMPER, INPUT_PULLUP);
}

void loop()   {
  if (digitalRead(BUMPER) == LOW) {
    delay(50);
    if (digitalRead(BUMPER) == LOW) {
    digitalWrite(LEFT_SPEED_PIN, 0);
      digitalWrite(RIGHT_SPEED_PIN, 0);
      delay(500);
      digitalWrite(LEFT_MOTOR, BACKWARDS);
      digitalWrite(RIGHT_MOTOR, BACKWARDS);
      digitalWrite(LEFT_SPEED_PIN, 150);
      digitalWrite(RIGHT_SPEED_PIN, 150);
      delay(500);
    }
  } else {
    digitalWrite(LEFT_MOTOR, FORWARDS);
    digitalWrite(RIGHT_MOTOR, FORWARDS);
    digitalWrite(LEFT_SPEED_PIN, 200);
    digitalWrite(RIGHT_SPEED_PIN, 200);
  }
}
