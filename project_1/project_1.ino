#include <Servo.h>

//define pins
#define SERVO 10
#define TRIGGER 13
#define ECHO 11
#define TEMPERATURE 3

#define SLOW_DIST 30  //cm
#define STOP_DIST 10  //cm
#define STOP 0
//#define MAX_SPEED     //TBD by motor people
//#define SLOW_INC      //TBD by motor people
#define LEFT -90
#define RIGHT 90

// ==== VARIABLES ====
Servo myservo;  // create servo object to control a servo

float temp;
float pulse;
float distance;
float SpeedOfSound;
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

void loop() {
  f1_loop();
}

/**
 * Loop for function 1: roomba
 */
void f1_loop(){
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

  //possibly comment this out
  if (distance > 3000)
    return ping();
    
  else
    return distance;
}

