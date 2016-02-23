#include <Servo.h>

//define pins
#define SERVO 10
#define TRIGGER 13
#define ECHO 11
#define TEMPERATURE 3
#define SWITCH1 
#define SWITCH2 

#define SLOW_DIST 40  //cm away to start slowing down
#define STOP_DIST 15  //cm away to stop
#define STOP 0
#define MAX_SPEED 255
#define SLOW_INC 50   //amount to decrement speed by when slowing down
#define LEFT -90
#define RIGHT 90

// ==== VARIABLES ====
Servo myservo;  // create servo object to control a servo

float temp;
float pulse;
float distance;
float SpeedOfSound;
int function = 0;
// == END VARIABLES ==

void setup() {
  // initialize serial monitor
  Serial.begin(9600);
  // attaches the servo on pin 10 to the servo object
  myservo.attach(SERVO);
  myservo.write(90);
    

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
  else
    continue;
}

/**
 * Loop for roomba function
 * Robot moves at max speed until it gets close to a wall/surface
 * It then sweeps the area around it (180 degrees), checks if there is more room
 * to the left or to the right, and then moves at max speed in that direction
 */
void f1_loop(){
  while(functionStatus() == 1){
    int dist = ping();
    
    if (dist > SLOW_DIST) 
      moveForward(MAX_SPEED);
      
    else if (dist > STOP_DIST){
      int myspeed = MAX_SPEED;
      while (ping() > STOP_DIST){
        moveForward(myspeed);
        myspeed -= SLOW_INC;
        delay(10);
      }
    }
    else {
      moveForward(STOP);
      if (sweep() == LEFT)
        turn(LEFT);
      else
        turn(RIGHT);
    }
    
  }
}

/**
 * Loop for tape following function
 * (follows a black line of tape)
 */
void f2_loop(){
  while(functionStatus() == 2){
      
  }
}

/**
 * Loop for our third function
 * IT'S A MYSTERY...
 */
void f3_loop(){
  while(functionStatus() == 3){
    
  }
}

/**
 * Does two things:
 * 1. updates the function variable to reflect current function status
 * 2. returns the function variable so the function can be called and used in one line
 */
int functionStatus(){
  boolean s1 = digitalRead(SWITCH1); 
  boolean s2 = digitalRead(SWITCH2);
  
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
 * Sweeps the rangefinder from 90 degrees to 0, to 180, then back to 90,
 * pinging the rangefinder at 0 and 180 degrees.
 * 
 * Returns: 1 if the longest distance read is on the left, 0 if the longest distance is on the right
 */
int sweep() {
  myservo.write(0);
  delay(1500);
  float leftDist = ping();
  
  myservo.write(180);
  delay(1500);
  float rightDist = ping();

  Serial.print("\nDistance to the left: ");
  Serial.print(leftDist);
  Serial.print("\nDistance to the right: ");
  Serial.print(rightDist);
  
  myservo.write(90);
  if (leftDist > rightDist)
    return LEFT;
  else
    return RIGHT;
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
  
  delayMicroseconds(100);

  return distance;
}

