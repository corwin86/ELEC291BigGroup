#include <Servo.h>

#define TRIGGER
#define ECHO
#define TEMPERATURE

// ==== VARIABLES ====

/**** SENSOR CONTROL ****/
int rangePos = 0;    // variable to store the servo position for range finder

Servo myservo;  // create servo object to control a servo
/** END SENSOR CONTROL **/

// == END VARIABLES ==

void setup() {
  // initialize serial monitor
  Serial.begin(9600);
  // attaches the servo on pin 9 to the servo object
  myservo.attach(9);

  // rangefinder pin modes
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TEMPERATURE, INPUT);

}

void loop() {

  if (ping() < 30) 
    printf(sweep();
}

/**
 * Rotates the server to the specified position
 * 
 * Param: pos - the position to rotate to. For reference, when the rangefinder is facing
 * the same direction as the robot, the position is 90. When the rangefinder is facing to the
 * left, the position is 0, and when it is facing to the right, the position is 180.
 */
void moveSensor(int pos) {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

/**
 * Sweeps the rangefinder from 90 degrees to 0, to 180, then back to 90,
 * pinging the rangefinder at 0 and 180 degrees.
 * 
 * Returns: 1 if the longest distance read is on the left, 0 if the longest distance is on the right
 */
int sweep() {
  moveSensor(0);
  float leftDist = ping();
  
  moveSensor(180);
  float rightDist = ping();
  moveSensor(90);
  if (leftDist > rightDist)
    return 1;
  else
    return 0;
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

