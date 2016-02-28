#include <Servo.h>

//control pins !!!!!!add motor here!!!!!!------------------------------------------------------------------
#define SWITCH1 9   //check which pin this uses
#define SWITCH2 8   //check which pin this uses

//f1 pins
#define SERVO 10
#define TRIGGER 13
#define ECHO 11
#define TEMPERATURE 3

//f2 pins
#define TAPE_LEFT 0
#define TAPE_RIGHT 1

//f3 pins
#define CRASH_SWITCH 11 //check which pin this uses

//f1 defines
#define SLOW_DIST 40  //cm away to start slowing down
#define STOP_DIST 15  //cm away to stop
#define STOP 0
#define MAX_SPEED 255
#define LEFT 1
#define RIGHT 0

//f2 defines

//f3 defines

// ==== VARIABLES ====

/***control variables***/
int function = 0;
/***end control variables***/

/***collision (f1) variables***/
Servo myservo;  // create servo object to control a servo
float temp, pulse, distance, SpeedOfSound, 
      rightDist, leftDist, sensorDist;
/***end servo variables***/

/***tapefollow (f2) variables***/
int left, right,
    leftspeed, rightspeed,
    m = 1, q = 0, // PID control variables, D gain counters
    p, d, con, //P correction, D correction, total correction
    error, lerr = 0, recerr = 0, // track current, last, and most recent (if not same as last)
    tape_thresh = 250;
    
//PID gains
int kp = 20,
    kd = 20,
    vel = 60;
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

  // rangefinder pin modes
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TEMPERATURE, INPUT);

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
 * Loop for collision function
 * Robot moves at max speed until it gets close to a wall/surface
 * It then sweeps the area around it (180 degrees), checks if there is more room
 * to the left or to the right, and then moves at max speed in that direction
 */
void f1_loop(){
  while(functionStatus() == 1){
    sensorDist = ping();
    
    if (dist > SLOW_DIST) 
      goForward(MAX_SPEED);
    else {
      decelerate(MAX_SPEED);
      delay(10);
    }
    
    if (sweep() == LEFT){
      turn(LEFT, 90);
    else
      turn(RIGHT, 90);
    }
  }
}

/**
 * Loop for tape following function
 * (follows a black line of tape)
 */
void f2_loop(){
  while(functionStatus() == 2){
    //tape follow
    left  = analogRead(TAPE_LEFT);
    right = analogRead(TAPE_RIGHT);
  
    if     (left > tape_thresh && right > tape_thresh) { error =  0; } // oo
    else if(left > tape_thresh && right < tape_thresh) { error = -1; } // ox
    else if(left < tape_thresh && right > tape_thresh) { error =  1; } // xo
    else if(left < tape_thresh && right < tape_thresh) {               // xx
      if(lerr > 0){ error =  5; }
      if(lerr < 0){ error = -5; }
    }
  
    if(error != lerr) {
      recerr = lerr;
      q = m;
      m = 1;
    }
  
    p = kp * error;
    d = (int)((float)kd * (float)(error - recerr)/(float)(q + m));
    con = p + d;
  
    m = m + 1;
  
    rightspeed = vel - con;
    leftspeed  = vel + con;
  
    Serial.print("Sensors: "); Serial.print(left); Serial.print(" | "); Serial.print(right); Serial.print("\tMotor: "); Serial.print(leftspeed); Serial.print(" | "); Serial.println(rightspeed);
    delay(150);
    //motor.speed(RIGHT_MOTOR, rightspeed < 255 ? rightspeed > 0 ? rightspeed : 0 : 255);
    //motor.speed(LEFT_MOTOR, leftspeed  < 255 ? leftspeed  > 0 ? leftspeed  : 0 : 255);
  
    lerr = error;
    // end tape following
  }
}

/**
 * Loop for our third function
 * IT'S A MYSTERY...
 */
void f3_loop(){
  while(functionStatus() == 3){
    spiral(LEFT);
<<<<<<< HEAD
//    if (sweep() == LEFT)
//      turn90degrees(LEFT);
//    else
//      turn90degrees(RIGHT);
//    moveForward(MAX_SPEED);
//    delay((int)ping() * 50);
//    stop();
    wallFollow();
=======
    if (sweep() == LEFT)
      turn90degrees(LEFT);
    else
      turn90degrees(RIGHT);
>>>>>>> 0b14eaf1f1d00326c165d42d7ea6d61e6e483a0a
  }
}

/**
 * Does two things:
 * 1. updates the function variable to reflect current function status
 * 2. returns the function variable so the function can be called and used in one line
 */
int functionStatus(){
  int s1 = digitalRead(SWITCH1); 
  int s2 = digitalRead(SWITCH2);
  
  if (s1 & !s2)
    function = 1;
  else if (!s1 & s2)
    function = 2;
  else if (s1 & s2)
    function = 3;
  else
    function = 0;
    
  return function;
}

/**
 * Moves the robot in a spiral until it runs into something
 * --------------------DO WE NEED TO HAVE A DIRECTION? OR ALWAYS SPIRAL LEFT??---------------------------
 */
void spiral(int direction){
  stop();
  int highSpeed = 220;
  int lowSpeed = 60;
  
  if(direction == LEFT){
    analogWrite(SPEED_B, highSpeed);
    digitalWrite(MOTOR_B, HIGH);
    //while (digitalRead(CRASH_SWITCH) == LOW){
    while(true){
      analogWrite(SPEED_A, lowSpeed); 
      digitalWrite(MOTOR_A, HIGH);
      delay(500);
      lowSpeed+=1;
    }
  }
  
  else{
    analogWrite(SPEED_A, highSpeed);
    digitalWrite(MOTOR_A, HIGH);
    //while (digitalRead(CRASH_SWITCH) == LOW){
    while(true){
      analogWrite(SPEED_B, lowSpeed); 
      digitalWrite(MOTOR_B, HIGH);
      delay(500);
      lowSpeed+=1;
    }
  }
  stop();
}

/**
 * Sweeps the rangefinder from 90 degrees to 0, to 180, then back to 90,
 * pinging the rangefinder at 0 and 180 degrees.
 * 
 * Returns: 1 if the longest distance read is on the left, 0 if the longest distance is on the right
 */
int sweep() {
  myservo.write(0);
  delay(1000);
  leftDist = ping();
  
  myservo.write(180);
  delay(1000);
  rightDist = ping();

  Serial.print(leftDist);
  Serial.print(rightDist);
  
  myservo.write(90);
  if (leftDist > rightDist)
    return LEFT;
  else
    return RIGHT;
}

/**
 * Checks all directions within 90 degrees of the front of the robot -------DO WE NEED THIS??-------
 * 
 * param: direction - direction to sweep (left or right)
 * returns: distance - the greatest distance away from the bot over 90 degrees
 */
int checkDir(int direction){
  int degrees = 85;
  int distance;
  while(degrees > 0 && degrees < 180){
    servo.write(degrees);
    int distL = ping();
    if (distL > distance)
      distance = distL;
    if (direction == LEFT)
      degrees--;
    else
      degrees++
  }

  return distance;
}

/**
 * Reads a value from the rangefinder.
 * 
 * Returns: the distance away from the rangefinder, in cm
 */
float ping(){
  //Finding the temperature in Celcius
  temp = analogRead(TEMPERATURE)*(5000/10240);
 
  //Finding the speed of sound
  SpeedOfSound = 331.5 + (0.6 * temp);
  //Serial.println(SpeedOfSound);

  //Calculate the conversion in order to determine the distance
  float conversion = 1/SpeedOfSound;
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
  distance = pulse/(2*conversion);

  return distance;
}

//robot moves forward at certain speed
void goForward(int speed){
  //skeleton function so code will compile
}

//robot pivots degrees specified in direction specified (RIGHT or LEFT)
void turn(int direction, int degrees){
  //skeleton function so code will compile
}

//decelerates to 0 from the speed specified
void decelerate(int speed){
  //skeleton function so code will compile
}

