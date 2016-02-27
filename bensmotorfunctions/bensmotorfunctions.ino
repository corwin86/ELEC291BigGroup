
/*
 * Ben's functions, they work but need to be calibrated
 */

#define MOTOR_A 4 //right forward or backward
#define MOTOR_B 7 //left forward or backward
#define SPEED_A 5 //right speed
#define SPEED_B 6 // left speed
#define LEFT 1

void setup() {
  //set up channels A & B with respecetive pins
  pinMode(SPEED_A, OUTPUT);  //Motor A pin
  
  pinMode(SPEED_B, OUTPUT);  //Motor B pin
}

void loop() {

  delay(1000);
  
  spiral(LEFT);
  //spiral(0, 20);

  delay(2000);
 
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
   digitalWrite(MOTOR_A, HIGH);
   analogWrite(SPEED_B, 0); 
   digitalWrite(MOTOR_B, HIGH);
}




