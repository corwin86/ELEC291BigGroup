
#define FORWARDS        LOW  //motor direction forward
#define BACKWARDS       HIGH //motor direction backward
#define LEFT_MOTOR      4    //right motor pin
#define RIGHT_MOTOR     7    //left motor pin
#define LEFT_SPEED_PIN  5    //right motor speed setting pin
#define RIGHT_SPEED_PIN 6    //left motor speed setting pin
#define MAX_SPEED       255

#define HALL_EFFECT_LEFT  2
#define HALL_EFFECT_RIGHT 3

#define HALL_GAIN 15
#define HALL_DEBOUNCE 5

int  t_left       = 0,
     t_right      = 0,
     last_t_left  = 0,
     last_t_right = 0,
     leftspeed    = 0,
     rightspeed   = 0;

void setup() {
  Serial.begin(9600);
  pinMode(HALL_EFFECT_LEFT,  INPUT);
  pinMode(HALL_EFFECT_RIGHT, INPUT);
}

void loop() {
  int left  = digitalRead(HALL_EFFECT_LEFT);
  int right = digitalRead(HALL_EFFECT_RIGHT);
  
  int t_cur = millis();
  
  last_t_left = t_left;
  last_t_right = t_right;
  if(left == LOW && t_cur - t_left > HALL_DEBOUNCE) {
    t_left = t_cur - t_left;    //save time to complete one rev
  }
  if(right == LOW && t_cur - t_right > HALL_DEBOUNCE) {
    t_right = t_cur - t_right;  //save time to complete one rev
  }

  int cor = getHallCorrection();

  leftspeed  = clamp(MAX_SPEED + cor, 0, 255);
  rightspeed = clamp(MAX_SPEED - cor, 0, 255);

  writeMotorSpeed(LEFT_MOTOR,  LEFT_SPEED_PIN,  leftspeed);
  writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, rightspeed);
}

/*
 *  Calculate time based correction value for wheel speeds, do not correct
 *  if either wheel has not yet had 1 full revolution
 */
int getHallCorrection() {
  return last_t_left == 0 || last_t_right == 0 ? 0 : t_left > t_right ? -HALL_GAIN : HALL_GAIN;
}

//  int left  = digitalRead(HALL_EFFECT_LEFT);
//  int right = digitalRead(HALL_EFFECT_RIGHT);
//
//  run_left = 1; run_right = 1; // both on for HIGH HIGH && LOW LOW
//  if (left == LOW) {
//    if (right == HIGH) {       // LOW  HIGH
//      run_left = 0;
//    }
//  } else if (right == LOW) {   // HIGH LOW
//    run_right = 0;
//  }
//
//  Serial.print(last_left); Serial.print("\t"); Serial.println(last_right);
//  Serial.print(left); Serial.print("\t"); Serial.println(right);
//  Serial.print(run_left); Serial.print("\t"); Serial.println(run_right);
//  Serial.println();
//
//  leftspeed  = clamp(run_left  == 1 ? MAX_SPEED : 0, 0, 255);
//  rightspeed = clamp(run_right == 1 ? MAX_SPEED : 0, 0, 255);
//
//  writeMotorSpeed(LEFT_MOTOR,  LEFT_SPEED_PIN,  leftspeed);
//  writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, rightspeed);

void writeMotorSpeed(int motor, int motor_speed_pin, int vel) {
  digitalWrite(motor, vel < 0 ? BACKWARDS : FORWARDS);
  analogWrite(motor_speed_pin, vel);
}

/*
 * Clamp val between min_r and max_r
 */
int clamp(int val, int min_r, int max_r) {
  return val < max_r ? val > min_r ? val : min_r : max_r;
}
