#include <Servo.h>
#include <LiquidCrystal.h>;

//control pins
#define LEFT_MOTOR      4     //left motor pin
#define RIGHT_MOTOR     7     //right motor pin
#define LEFT_SPEED_PIN  5     //left motor speed setting pin
#define RIGHT_SPEED_PIN 6     //right motor speed setting pin
#define SWITCH1         9     //function switches
#define SWITCH2         8     //(s1^!s2)=>f1, (!s1^s2)=>f2, (s1^s2)=>f3, (!s2^!s2)=>off

//control defines
#define FORWARDS        LOW   //motor direction forward
#define BACKWARDS       HIGH  //motor direction backward
#define TURN_CONST      10.8   //experimentally derived, consistent up to ~180 degrees
#define MAX_SPEED       255
#define MIN_SPEED       80    //experimentally derived, min speed so that motor won't stall
#define SWIVEL_SPEED    100   //speed for the robot to turn at
#define SPIRAL_SPEED    200   //speed for the outside motor while spiralling
#define LEFT  1
#define RIGHT 0

//Hall effect pins
#define HALL_EFFECT_LEFT 3
#define HALL_EFFECT_RIGHT 2

//Hall effect defines
#define HALL_HIGH 600

//f1 pins
#define SERVO 10
#define TRIGGER 11
#define ECHO 12
#define TEMPERATURE 3

//f1 defines
#define SLOW_DIST 40
#define STOP_DIST 13

//f2 pins
#define TAPE_LEFT 0
#define TAPE_RIGHT 1

//f3 pins
#define BUMPER 13

//f3 defines
#define MIN_ANGLE   60
#define MAX_ANGLE   120
#define HALF_CONST  40    //time constant to move half of the distance from the wall

// ==== VARIABLES ====

/***control variables***/
int function = 0;
/***end control variables***/

/***collision (f1) variables***/
Servo myservo;  // create servo object to control a servo
float temp, pulse, distance, SpeedOfSound,
      rightDist, leftDist, sensorDist;
/***end collision variables***/

/***tapefollow (f2) variables***/
int kp = 80,
    kd = 0,
    vel = 175;
int last_error = 0, recent_error = 0;
int t_cur = 0, t_recent = 0;
int TAPE_THRESH = 600;
/***end tapefollow variables***/

/***roomba (f3) variables***/
int distCount = 0;
/***end roomba variables***/

// == END VARIABLES ==

void setup() {
  // initialize serial monitor
  Serial.begin(9600);
  // attaches the servo on pin 10 to the servo object
  myservo.attach(SERVO);
  myservo.write(90);

  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);

  // rangefinder pin modes
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TEMPERATURE, INPUT);

  //bumper switch pinmode
  pinMode(BUMPER, INPUT_PULLUP);

  //set up motor pins and hall effect pins
  pinMode(RIGHT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  //set up hall effect pin
  pinMode(HALL_EFFECT_LEFT, INPUT);
  pinMode(HALL_EFFECT_RIGHT, INPUT);
  delay(1500);
}

//determines which function loop to run
void loop() {
  //updates the current function status, loops until valid function
  functionStatus();

  if (function == 1)
    f1_loop();
  else if (function == 2)
    f2_loop();
  else if (function == 3)
    f3_loop();
}

/**
   Loop for collision avoiding function (f1)
   Robot moves at max speed until it gets close to a wall/surface
   It then sweeps the area around it (180 degrees) with the rangefinder, checks if there
   is more room to the left or to the right, and then moves at max speed in that direction
*/
void f1_loop() {
  while (functionStatus() == 1) {
    sensorDist = debouncePing();

    //as long as the robot has space, move at max speed
    if (sensorDist > SLOW_DIST) {
      goForward(MAX_SPEED);
    }
    //if it doesn't have space, slow down, look both ways, and turn
    //90 degrees in the direction with the most space and continue
    else {
      decelerate();
      delay(250);
      if (sweep() == LEFT) {
        turn(90, LEFT);
      }
      else {
        turn(90, RIGHT);
      }
    }
  }
}

/**
   Loop for tape following function
   (follows a black line of tape)
*/
void f2_loop() {
  while (functionStatus() == 2) {
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
   Loop for our third function - area covering
   Moves halfway to the wall, then spirals until it bumps into something.
   After bumping, it moves in a random direction (60 to 120 degrees away from wall)
   until bumping into something again 5 times, then goes back to spiral.  Repeats.
*/
void f3_loop() {
  int spiralCount = 0;
  spiral();
  while (functionStatus() == 3) {
    //every 5 moves, do a spiral
    if (spiralCount >= 5) {
      //moves halfway across the room before spiraling again
      goForward(MAX_SPEED);
      delay(debouncePing() * HALF_CONST);
      spiral();
      spiralCount = 0;
    }

    //then, bounce off walls for 5 moves
    //go at max speed until it hits something?
    goForward(MAX_SPEED);

    //slow down before collision
    while (debouncePing() < SLOW_DIST - 10) {
      delay(10);
    }
    goForward(MIN_SPEED);

    //if it hits something, stop
    while (true) {
      if (digitalRead(BUMPER) == LOW) {
        delay(10);
        if (digitalRead(BUMPER) == LOW) {
          stop();
          break;
        }
      }
    }

    //backs up briefly after a collision
    goForward(-100);
    delay(100);
    stop();

    //check which direction has the most space, then turn at a random
    //angle between 60 and 120 and move in that direction
    if (sweep() == LEFT) {
      turn(random(MIN_ANGLE, MAX_ANGLE), LEFT);
    }
    else {
      turn(random(MIN_ANGLE, MAX_ANGLE), RIGHT);
    }
    spiralCount++;
    //    goForward(MAX_SPEED);
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


// --------------Rangefind helpers-----------------
/**
   Sweeps the rangefinder from 90 degrees to 0, to 180, then back to 90,
   pinging the rangefinder at 0 and 180 degrees.

   Returns: 1 if the longest distance read is on the left, 0 if the longest distance is on the right
*/
int sweep() {
  myservo.write(180);
  delay(300);
  leftDist = debouncePing();
  delay(300);

  myservo.write(0);
  delay(300);
  rightDist = debouncePing();
  delay(300);

  myservo.write(90);
  delay(300);
  if (leftDist > rightDist)
    return LEFT;
  else
    return RIGHT;
}

/**
   Reduces inconsistent rangefinder values
*/
float debouncePing() {
  float ping1 = ping(); float ping2 = ping(); float ping3 = ping();
  if (ping1 - ping2 < 5 && ping2 - ping3 < 5)
    return (ping1 + ping2 + ping3) / 3;
  else
    return debouncePing();
}

/**
   Reads a value from the rangefinder - helper for debouncePing()

   returns: the distance away from the rangefinder, in cm
*/
float ping() {
  //Finding the temperature in Celcius
  temp = analogRead(TEMPERATURE) * (5000 / 10240);

  //Finding the speed of sound
  SpeedOfSound = 331.5 + (0.6 * temp);

  //Calculate the conversion in order to determine the distance
  float conversion = 1 / SpeedOfSound;
  conversion = conversion * 1000000 / 100;

  //Send a pulse to the trigger
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  //Read from the trigger pin
  pulse = pulseIn(ECHO, HIGH);

  //Calculate the distance
  distance = pulse / (2 * conversion);
  Serial.println(distance);
  return distance;
}
// --------------end rangefind helpers-----------------


// ------------------Motor helpers---------------------
/**
  Moves the robot in a left spiral until it runs into something
*/
void spiral() {
  stop();
  int highSpeed = SPIRAL_SPEED;
  int lowSpeed = MIN_SPEED;

  //set the outside motor speed
  writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, highSpeed);

  //gradually increase speed of inside motor
  distCount = 500; //check that it isn't used somewhere else
  while (true) {
    delay(10);
    distCount += 10;
    if (distCount >= 500) {
      lowSpeed += 1;
      writeMotorSpeed(LEFT_MOTOR, LEFT_SPEED_PIN, lowSpeed);
      distCount = 0;
    }

    //debouncing for switch
    if (digitalRead(BUMPER) == LOW) {
      delay(10);
      if (digitalRead(BUMPER) == LOW) {
        break;
      }
    }
  }
}

/**
  go forward at a given speed
  param: vel (0-255) to set velocity
*/
void goForward(int vel) {
  writeMotorSpeed(LEFT_MOTOR, LEFT_SPEED_PIN, vel);
  writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, vel);
}

/**
  Decelarate from cuurent speed to 0 in 2 seconds
*/
void decelerate() {
  int m_speed = MAX_SPEED;
  int p = STOP_DIST + 1;
  while (p > STOP_DIST) {
    p = debouncePing();
    int c = MAX_SPEED * (SLOW_DIST - p) / SLOW_DIST;

    m_speed = clamp(MAX_SPEED - c, MIN_SPEED, MAX_SPEED);

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
  digitalWrite(RIGHT_MOTOR, FORWARDS);
  analogWrite(LEFT_SPEED_PIN, 0);
  digitalWrite(LEFT_MOTOR, FORWARDS);
}


void turn(int degrees, int direction) {
  stop();
  if (direction == RIGHT) {
    writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, -SWIVEL_SPEED);
    writeMotorSpeed(LEFT_MOTOR, LEFT_SPEED_PIN, SWIVEL_SPEED);
  }
  else {
    writeMotorSpeed(RIGHT_MOTOR, RIGHT_SPEED_PIN, SWIVEL_SPEED);
    writeMotorSpeed(LEFT_MOTOR, LEFT_SPEED_PIN, -SWIVEL_SPEED);
  }
  delay(degrees * TURN_CONST);
  stop();
}

// -----------------end motor helpers-------------------

// --------------Tape following helpers-----------------
bool onTape(int reading) {
  return reading > TAPE_THRESH;
}

/**
   Writes a speed to a motor

   param motor - the motor to write to
   param motor_speed_pin - the speed pin to write to
   param vel - the speed to write to the motor
*/
void writeMotorSpeed(int motor, int motor_speed_pin, int vel) {
  digitalWrite(motor, vel < 0 ? BACKWARDS : FORWARDS);
  analogWrite(motor_speed_pin, abs(vel));
}

/*
   Clamp val between min_r and max_r
*/
int clamp(int val, int min_r, int max_r) {
  return val < max_r ? val > min_r ? val : min_r : max_r;
}
// ------------end tape following helpers---------------

