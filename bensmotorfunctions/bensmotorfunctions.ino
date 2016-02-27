
/*
 * Ben's functions, they work but need to be calibrated
 */

#define MOTOR_A 4 //right forward or backward
#define MOTOR_B 7 //left forward or backward
#define SPEED_A 5 //right speed
#define SPEED_B 6 // left speed

void setup() {
  //set up channels A & B with respecetive pins
  pinMode(SPEED_A, OUTPUT);  //Motor A pin
  
  pinMode(SPEED_B, OUTPUT);  //Motor B pin
}

void loop() {

  delay(1000);
  
  slowTurn(50);
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

/*
 * stops and returns to default forward direction
 */
void stop(){
   analogWrite(SPEED_A, 0);
   digitalWrite(MOTOR_A, HIGH);
   analogWrite(SPEED_B, 0); 
   digitalWrite(MOTOR_B, HIGH);
}

/*
 * THIS IS NOT GOOD
 */
void spiral(int leftright, int numfeet){
    int timeToTurn = numfeet * 1000;

    if(leftright = 0){
      analogWrite(SPEED_A, 150);
      digitalWrite(MOTOR_A, HIGH);
      analogWrite(SPEED_B, 100); 
      digitalWrite(MOTOR_B, HIGH);
    }
    else{
      analogWrite(SPEED_A, 100);
      digitalWrite(MOTOR_A, HIGH);
      analogWrite(SPEED_B, 150); 
      digitalWrite(MOTOR_B, HIGH);
    }
    
    delay(timeToTurn);
    stop();
    
}




