<<<<<<< HEAD
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
=======
void setup() {
  //set up channels A & B with respecetive pins
  pinMode(12, OUTPUT);  //Motor A pin
  pinMode(9, OUTPUT);   //Brake A pin

  pinMode(13, OUTPUT);  //Motor B pin
  pinMode(8, OUTPUT);   //Brake B pin
>>>>>>> 0bf2fe1e06be0880c3c1dc77b819b743fb5da3e1

}

void loop() {
<<<<<<< HEAD
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


=======
  // 

}
>>>>>>> 0bf2fe1e06be0880c3c1dc77b819b743fb5da3e1
