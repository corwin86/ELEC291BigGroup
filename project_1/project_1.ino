#include <Servo.h>
#include <LiquidCrystal.h>;

//control pins !!!!!!add motor here!!!!!!------------------------------------------------------------------
#define FORWARDS        LOW  //motor direction forward
#define BACKWARDS       HIGH //motor direction backward
#define LEFT_MOTOR      4    //left motor pin
#define RIGHT_MOTOR     7    //right motor pin
#define LEFT_SPEED_PIN  5    //left motor speed setting pin
#define RIGHT_SPEED_PIN 6    //right motor speed setting pin
#define SWITCH1         9    //function switches
#define SWITCH2         8    //(s1 ^ !s2) = f1, (!s1 ^ s2) = f2, (s1 ^ s2) - f3, (!s2 ^ !s2) = off

#define LEFT 1
#define RIGHT 0

//Hall effect pins
#define HALL_EFFECT_LEFT 3
#define HALL_EFFECT_RIGHT 2
#define HALL_HIGH 600

//f1 pins
#define SERVO 10
#define TRIGGER 11
#define ECHO 12
#define TEMPERATURE 3

#define SLOW_DIST 60
#define STOP_DIST 4

//f2 pins
#define TAPE_LEFT 0
#define TAPE_RIGHT 1

//f3 pins
#define CRASH_SWITCH 13 //check which pin this uses

#define MAX_SPEED 255

// ==== VARIABLES ====

/***control variables***/
int function = 0;
//LiquidCrystal lcd(13,12,11,10,9,8); // these need to be changed
/***end control variables***/

/***collision (f1) variables***/
Servo myservo;  // create servo object to control a servo
float temp, pulse, distance, SpeedOfSound,
      rightDist, leftDist, sensorDist;
int   distCount = 0;
/***end servo variables***/

/***tapefollow (f2) variables***/

int kp = 80,
    kd = 0,
    vel = 175;
int last_error = 0, recent_error = 0;
int t_cur = 0, t_recent = 0;
int TAPE_THRESH = 600;
/***end tapefollow variables***/

/***roomba (f3) variables***/

/***end roomba variables***/

// == END VARIABLES ==

void setup() {
  // initialize serial monitor
  Serial.begin(9600);
  // attaches the servo on pin 10 to the servo object
  myservo.attach(SERVO);
  myservo.write(90);

  pinMode(CRASH_SWITCH, INPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);

  //setup lcd
  //lcd.begin(16,2);

  // rangefinder pin modes
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TEMPERATURE, INPUT);

  //crash switch pinmode
  pinMode(CRASH_SWITCH, INPUT_PULLUP);

  //set up motor pins and hall effect pins
  pinMode(RIGHT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  //set up hall effect pin
  pinMode(HALL_EFFECT_LEFT, INPUT);
  pinMode(HALL_EFFECT_RIGHT, INPUT);

}

//determines which function loop to run
void loop() {
  //updates the current function status, loops until valid function
  functionStatus();
  Serial.println(function);

  if (function == 1)
    f1_loop();
  else if (function == 2)
    f2_loop();
  else if (function == 3)
    f3_loop();
}

/**
   Loop for collision function
   Robot moves at max speed until it gets close to a wall/surface
   It then sweeps the area around it (180 degrees), checks if there is more room
   to the left or to the right, and then moves at max speed in that direction
*/
void f1_loop() {

  while (functionStatus() == 1) {
    sensorDist = ping();
    Serial.print("distance from sensor:");
    Serial.println(sensorDist);

    if (sensorDist > SLOW_DIST) {
      goForward(MAX_SPEED);
    }
    else if (distCount < 5) {
      distCount++;
    }
    else {
      decelerate();
      delay(500);
      if (sweep() == RIGHT) {
        turn90degrees(LEFT);
      }
      else {
        turn90degrees(RIGHT);
      }
      distCount = 0;
    }
  }
}

/**
   Loop for tape following function
   (follows a black line of tape)
*/
void f2_loop() {
  while (functionStatus() == 2) {
    Serial.println("in function 2");
    // read values from IR sensors
    int left  = analogRead(TAPE_LEFT);
    int right = analogRead(TAPE_RIGHT);

    //if error is unchanged, both are on tape
    int error = 0;
    if     ( onTape(left) && !onTape(right)) {
      error =  1;
    }
    else if (!onTape(left) &&  onTape(right)) {
      error = -1;
    }
    else if (!onTape(left) && !onTape(right)) {
      error = last_error < 0 ? -5 : 5;
    }

    // find time spent in current and previous error states (for D gain)
    if (error != last_error) {
      recent_error = last_error;
      t_recent = t_cur; //save time in recent error state
      t_cur = 1;        //begin counting new error state
    }

    // calculate gains & correction
    int p_gain = kp * error;
    int d_gain = (int)( (float)kd * (float)(error - recent_error) / (float)(t_cur++ + t_recent) );
    int correction = p_gain + d_gain;

    int leftspeed  = clamp(vel + correction, 0, 255);
    int rightspeed = clamp(vel - correction, 0, 255);

    writeMotorSpeed(LEFT_MOTOR , LEFT_SPEED_PIN , leftspeed);
    writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, rightspeed);

    // setup for next loop
    last_error = error;
  }
}

/**
   Loop for our third function
   IT'S A MYSTERY...
*/
void f3_loop() {
  Serial.println("in function 3");
  while (functionStatus() == 3) {
    spiral();

    if (sweep() == LEFT)
      turn90degrees(LEFT);
    else
      turn90degrees(RIGHT);
    goForward(MAX_SPEED);
    delay((int)ping() * 50);
    stop();
    // wallFollow();

  }
}

/**
   Does two things:
   1. updates the function variable to reflect current function status
   2. returns the function variable so the function can be called and used in one line
*/
int functionStatus() {
  int s1 = digitalRead(SWITCH1);
  int s2 = digitalRead(SWITCH2);

  if (s1 == LOW && s2 == HIGH)
    function = 1;
  else if (s1 == HIGH && s2 == LOW)
    function = 2;
  else if (s1 == LOW && s2 == LOW)
    function = 3;
  else
    function = 0;

  return function;
}

/**
  Moves the robot in a left spiral until it runs into something
*/
void spiral() {
  stop();
  int highSpeed = 220;
  int lowSpeed = 60;

  analogWrite(LEFT_SPEED_PIN, highSpeed);
  digitalWrite(LEFT_MOTOR, HIGH);
  while (digitalRead(CRASH_SWITCH) == HIGH) {
    analogWrite(RIGHT_SPEED_PIN, lowSpeed);
    digitalWrite(RIGHT_MOTOR, HIGH);
    delay(500);
    lowSpeed += 1;
  }
}

/**
   Sweeps the rangefinder from 90 degrees to 0, to 180, then back to 90,
   pinging the rangefinder at 0 and 180 degrees.

   Returns: 1 if the longest distance read is on the left, 0 if the longest distance is on the right
*/
int sweep() {
  myservo.write(0);
  delay(1000);
  leftDist = ping();

  myservo.write(180);
  delay(1000);
  rightDist = ping();

  Serial.print("distance to left: ");
  Serial.println(leftDist);
  Serial.print("distance to right: ");
  Serial.println(rightDist);


  myservo.write(90);
  if (leftDist > rightDist)
    return LEFT;
  else
    return RIGHT;
}

/**
   Checks all directions within 90 degrees of the front of the robot -------DO WE NEED THIS??-------

   param: direction - direction to sweep (left or right)
   returns: distance - the greatest distance away from the bot over 90 degrees
*/
//int checkDir(int direction){
//  int degrees = 85;
//  int distance;
//  while(degrees > 0 && degrees < 180){
//    myservo.write(degrees);
//    int distL = ping();
//    if (distL > distance)
//      distance = distL;
//    if (direction == LEFT)
//      degrees--;
//    else
//      degrees++;
//  }
//
//  return distance;
//}

/**
   Reads a value from the rangefinder.

   Returns: the distance away from the rangefinder, in cm
*/
float ping() {
  //Finding the temperature in Celcius
  temp = analogRead(TEMPERATURE) * (5000 / 10240);

  //Finding the speed of sound
  SpeedOfSound = 331.5 + (0.6 * temp);
  //Serial.println(SpeedOfSound);

  //Calculate the conversion in order to determine the distance
  float conversion = 1 / SpeedOfSound;
  conversion = conversion * 1000000 / 100;
  //Serial.println(conversion);

  //Send a pulse to the trigger
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  //delayMicroseconds(10);

  //Read from the trigger pin
  pulse = pulseIn(ECHO, HIGH);
  //Serial.println(pulse);

  //Calculate the distance
  distance = pulse / (2 * conversion);

  return distance;
}

/*go forward at a given speed

  param: speed (0-255) to set
*/
void goForward(int speed) {
  digitalWrite(LEFT_MOTOR, FORWARDS);
  analogWrite(LEFT_SPEED_PIN, speed);
  digitalWrite(RIGHT_MOTOR, FORWARDS);
  analogWrite(RIGHT_SPEED_PIN, speed);
}

/**Decelarate from cuurent speed to 0 in 2 seconds

*/
void decelerate() {
  //  int steps = speed / 10;
  //  for(int i = 0; i < 10; i++){
  //    analogWrite(RIGHT_SPEED_PIN, speed - steps * i);
  //    analogWrite(LEFT_SPEED_PIN, speed - steps * i);
  //    delay(200);
  //  }
  //  analogWrite(RIGHT_SPEED_PIN, 0);
  //  analogWrite(LEFT_SPEED_PIN, 0);
  int m_speed = MAX_SPEED;
  int p = STOP_DIST + 1;
  while(p > STOP_DIST) {
    p = ping();
    int c = MAX_SPEED * (SLOW_DIST - p) / SLOW_DIST - 50;

    m_speed = MAX_SPEED - c;
  
    writeMotorSpeed(LEFT_MOTOR, LEFT_SPEED_PIN, m_speed);
    writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, m_speed);
  }
  stop();
}

/*
   Stops and returns to default forward direction
*/
void stop() {
  analogWrite(RIGHT_SPEED_PIN, 0);
  digitalWrite(RIGHT_MOTOR, LOW);
  analogWrite(LEFT_SPEED_PIN, 0);
  digitalWrite(LEFT_MOTOR, LOW);
}

// ==============================
//     Tape following helpers
bool onTape(int reading) {
  return reading > TAPE_THRESH;
}

void writeMotorSpeed(int motor, int motor_speed_pin, int vel) {
  digitalWrite(motor, vel < 0 ? BACKWARDS : FORWARDS);
  analogWrite(motor_speed_pin, vel);
}

/*
   Clamp val between min_r and max_r
*/
int clamp(int val, int min_r, int max_r) {
  return val < max_r ? val > min_r ? val : min_r : max_r;
}
//   End Tape following helpers
// ==============================

/*
   This function displays the left motor speed and the right motor speed onto the LCD display
*/
//void printSpeed() {
//  int leftSpeed = 0; // this needs to be replaced
//  int rightSpeed = 0; // this needs to be replaced
//
//  lcd.clear();
//  lcd.home();
//  lcd.print("Left speed: ");
//  lcd.print(leftSpeed);
//  lcd.setCursor(0, 1);
//  lcd.print("Right speed: ");
//  lcd.print(rightSpeed);
//}

void turn90degrees(int direction) {
  stop();
  if (direction == 0) {
    analogWrite(RIGHT_SPEED_PIN, 100);
    digitalWrite(RIGHT_MOTOR, HIGH);
    analogWrite(LEFT_SPEED_PIN, 100);
    digitalWrite(LEFT_MOTOR, LOW);
  }
  else {
    analogWrite(RIGHT_SPEED_PIN, 100);
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(LEFT_SPEED_PIN, 100);
    digitalWrite(LEFT_MOTOR, HIGH);
  }
  delay(90 * 9.8);
  stop();
}

