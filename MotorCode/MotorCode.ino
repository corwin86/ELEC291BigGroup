#define MOTOR_A 5
#define MOTOR_B 6
#define BRAKE_A 
#define BRAKE_B 
#define SPEED_A 4
#define SPEED_B 7



void setup() {
  //set up channels A & B with respecetive pins
  pinMode(MOTOR_A, OUTPUT);  //Motor A pin
  //pinMode(BRAKE_A, OUTPUT);   //Brake A pin

  pinMode(MOTOR_B, OUTPUT);  //Motor B pin
  //pinMode(BRAKE_B, OUTPUT);   //Brake B pin

}

void loop() {
  digitalWrite(MOTOR_A, HIGH);
  //digitalWrite(BRAKE_A, LOW);
  analogWrite(SPEED_A, 255);

  digitalWrite(MOTOR_B, HIGH);
 // digitalWrite(BRAKE_B, LOW);
  analogWrite(SPEED_B, 255);

  delay(3000);

 // digitalWrite(BRAKE_A, HIGH);
 // digitalWrite(BRAKE_B, HIGH);

  delay(2000);

}


