#define MOTOR_A 4 #right
#define MOTOR_B 7 #left
#define BRAKE_A 
#define BRAKE_B 
#define SPEED_A 5 #right
#define SPEED_B 6 #left

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

void calibrate(int speed_left, int speed_right){
  //find time difference for sensor1
  analogWrite(MOTOR_A, speed_right);
  analogWrite(MOTOR_B, speed_left);
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
      analogWrite(MOTOR_B, --speed_left);
    }
  }
  else{
    while(countRight < countLeft){
      analogWrite(MOTOR_A, --speed_right);
    }
  }
}

   

