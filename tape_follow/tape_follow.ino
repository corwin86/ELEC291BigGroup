
// ==== PINOUTS ====
// Analog
const int TAPE_LEFT = 0, TAPE_RIGHT = 0;
// Digital
// !!! TBD
// == END PINOUTS ==

// ==== VARIABLES ====

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
  
  //motor.speed(RIGHT_MOTOR, rightspeed < 255 ? rightspeed > 0 ? rightspeed : 0 : 255);
  //motor.speed(LEFT_MOTOR, leftspeed  < 255 ? leftspeed  > 0 ? leftspeed  : 0 : 255);
  
  lerr = error;
  // end tape following
}
