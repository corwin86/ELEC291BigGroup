#define HALL_EFFECT_LEFT 8
#define HALL_EFFECT_RIGHT 9



void setup() {
  pinMode(HALL_EFFECT_LEFT, INPUT);
  pinMode(HALL_EFFECT_RIGHT, INPUT);
  Serial.begin(9600);

}

void loop() {
  digitalWrite(MOTOR_A, HIGH);
  analogWrite(SPEED_A, 255);

  digitalWrite(MOTOR_B, HIGH);
  analogWrite(SPEED_B, 255);

  delay(3000);

}

void calibrate(int speed_left, int speed_right){
  //find time difference for sensor1
  analogWrite(MOTOR_A, speed_right);
  analogWrite(MOTOR_B, speed_left);
  int sensor1 = analogRead(HALL_EFFECT_LEFT);
  int sensor2 = analogRead(HALL_EFFECT_RIGHT);

  int countLeft = 0;
  int countRight = 0;
  while(sensor1 < HALL_HIGH){
    analogRead(HALL_EFFECT_LEFT);
    countLeft++;
  }

  while(sensor2 < HALL_HIGH){
    analogRead(HALL_EFFECT_RIGHT);
    countRight++;
  }

  
  if(countLeft < countRight){
    while(countLeft < countRight){
      analogWrite(MOTOR_B, --speed_left);
    }
  }
  else{
    while(countRight < countLeft){
      analogWrite(MOTOR_A, --speed_right);
    }
  }
}

