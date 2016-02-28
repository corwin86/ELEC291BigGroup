#define MOTOR_A 4 //right
#define MOTOR_B 7 //left
#define BRAKE_A 
#define BRAKE_B 
#define SPEED_A 5 //right
#define SPEED_B 6 //left

#define LEFT 1
#define RIGHT 0

#define HALL_EFFECT_LEFT 8
#define HALL_EFFECT_RIGHT 9
#define HALL_LOW 300
#define HALL_HIGH 600

void setup() {
  //set up channels A & B with respecetive pins
  pinMode(SPEED_A, OUTPUT);  //Motor A pin
  pinMode(SPEED_B, OUTPUT);  //Motor B pin
  pinMode(HALL_EFFECT_LEFT, INPUT);
  pinMode(HALL_EFFECT_RIGHT, INPUT);

}

void loop() {

  spiral(LEFT);

  delay(3000);
}

/*
 * Performs a turn to the right or left, still need to be calibrated
 */
void slowTurn(int degree){
   
    stop();
    int slowSpeed = 250;

    long timeNeeded = degree * 50; // calibrate later
    
    if(degree < 0){     
         digitalWrite(MOTOR_A, HIGH);
         analogWrite(SPEED_A, slowSpeed);
         digitalWrite(MOTOR_B, LOW);
         analogWrite(SPEED_B, slowSpeed);  
         delay(timeNeeded);              
    }
    
    else {
      
         digitalWrite(MOTOR_A, LOW);
         analogWrite(SPEED_A, slowSpeed);
         digitalWrite(MOTOR_B, HIGH);
         analogWrite(SPEED_B, slowSpeed);  
         delay(timeNeeded);      
    }
    
    stop();  
    
}

/**
 * Makes robot move in a spiral until it hits something
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

/*
 * stops and returns to default forward direction
 */
void stop(){
   analogWrite(SPEED_A, 0);
   digitalWrite(MOTOR_A, LOW);
   analogWrite(SPEED_B, 0); 
   digitalWrite(MOTOR_B, LOW);
}

/*go forward at a given speed*/
void goForward(int speed){
  digitalWrite(MOTOR_A, HIGH);
  analogWrite(SPEED_A, speed);
  digitalWrite(MOTOR_B, HIGH);
  analogWrite(SPEED_B, speed);
  }

/*Decelarate from cuurent speed to 0 in 2 seconds*/
void decelerate(int speed){
  int steps = speed / 10; 
  for(int i = 0; i < 10; i++){
    digitalWrite(MOTOR_A, speed - steps * i);
    delay(200);
  }
  digitalWrite(MOTOR_A, 0);
}

void turn(int direction, int degree){}

void calibrate(int speed_left, int speed_right){
  //find time difference for sensor1
  analogWrite(SPEED_A, speed_right);
  analogWrite(SPEED_B, speed_left);
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
      analogWrite(SPEED_B, --speed_left);
    }
  }
  else{
    while(countRight < countLeft){
      analogWrite(SPEED_A, --speed_right);
    }
  }
}

void turn90degrees(int direction){
   stop();
   if(direction == 0){
    analogWrite(SPEED_A, 100);
    digitalWrite(MOTOR_A, HIGH);
    analogWrite(SPEED_B, 100); 
    digitalWrite(MOTOR_B, LOW);
   }
   else{
    analogWrite(SPEED_A, 100);
    digitalWrite(MOTOR_A, LOW);
    analogWrite(SPEED_B, 100); 
    digitalWrite(MOTOR_B, HIGH);
   }
   delay(90*9.8);
   stop();
}


   

