#include <Servo.h>

// ==== PINOUTS ====
// Analog
const int TAPE_LEFT = 0, TAPE_RIGHT = 0;
// Digital
// !!! TBD
// == END PINOUTS ==

// ==== VARIABLES ====

/**** SENSOR CONTROL ****/
int sensorPos = 0;    // variable to store the servo position
Servo myservo;  // create servo object to control a servo

/**** PID GAINS ****/
int kp = 20,
    kd = 20,
    vel = 60;
/** END PID GAINS **/

int left, right,
    leftspeed, rightspeed,
    m = 1, q = 0, // PID control variables, D gain counters
    p, d, con, //P correction, D correction, total correction
    error, lerr = 0, recerr = 0, // track current, last, and most recent (if not same as last)
    tape_thresh = 250;
// == END VARIABLES ==

void setup() {
  // initialize serial monitor
  Serial.begin(9600);
  // attaches the servo on pin 9 to the servo object
  myservo.attach(9);
  
  // setup pin modes
  // !!! TBD
}

void loop() {
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
  
  Serial.print("Sensors: "); Serial.print(left); Serial.print(" | "); Serial.print(right); Serial.print("\tMotor: "); Serial.print(leftspeed); Serial.print(" | "); Serial.print(rightspeed);
  //motor.speed(RIGHT_MOTOR, rightspeed < 255 ? rightspeed > 0 ? rightspeed : 0 : 255);
  //motor.speed(LEFT_MOTOR, leftspeed  < 255 ? leftspeed  > 0 ? leftspeed  : 0 : 255);
  
  lerr = error;
  // end tape following
}

/**
 * Sweeps the servo from 90 degrees, to 0, to 180, then returns back to 90
 * ---------------------------NOT FINISHED----------------------------------
 * 
 * Returns: 0 if the bot should turn left, 1 if it should turn right
 */
int sweep {
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
