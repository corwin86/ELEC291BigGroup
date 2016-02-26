#define MOTOR_A 5
#define MOTOR_B 6
#define BRAKE_A 
#define BRAKE_B 
#define SPEED_A 4
#define SPEED_B 7

#define LEFT 1
#define RIGHT 0

#define 

void setup() {
  //set up channels A & B with respecetive pins
  pinMode(MOTOR_A, OUTPUT);  //Motor A pin
  pinMode(MOTOR_B, OUTPUT);  //Motor B pin

}

void loop() {

  digitalWrite(MOTOR_A, HIGH);
  analogWrite(SPEED_A, 255);

  digitalWrite(MOTOR_B, HIGH);
  analogWrite(SPEED_B, 255);

  delay(3000);
}

void slowTurn(int direction){
    stop();
    int slowSpeed = 30;
    int reverseSlowSpeed = -slowSpeed;

    long startTime; 
    long newTime;

    long timeNeeded = direction * 50; // calibrate later

    newTime = millis();
    
    if(direction < 0){     
       while((millis() - newTime) < timeNeeded){         
         digitalWrite(MOTOR_A, HIGH);
         analogWrite(SPEED_A, slowSpeed);
         digitalWrite(MOTOR_B, HIGH);
         analogWrite(SPEED_B, reverseSlowSpeed);
       }
    }
    
    else {
      while((millis() - newTime) < timeNeeded){
         digitalWrite(MOTOR_A, HIGH);
         analogWrite(SPEED_A, reverseSlowSpeed);
         digitalWrite(MOTOR_B, HIGH);
         analogWrite(SPEED_B, slowSpeed);        
       }
    }
    stop();  
}

void stop(){
  digitalWrite(MOTOR_A, LOW);
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
    digitialWrite(MOTOR_A, speed - steps * i);
    delay(200);
  }
  digitalWrite(MOTOR_A, 0);
}

void turn(int direction, int degree){}

   

