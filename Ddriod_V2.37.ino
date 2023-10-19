//Ddroid Smart Cleaning Robot v2.3 design by Nipuna Muhamdiram.
//Group project first year second semester.
//Univarsity of Colombo Faculty of Technology. 
//Ddroid GridMatrix Filling Implementation Test No: 34
//Motor check Test No :6
//Funtion Validation check Test No : 47
//Sensor check and validation check No : 17
//Humidity controlling check Test No : 7



#include <AFMotor.h>
#include <dht.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// const int relay1Pin = 34;
// const int relay2Pin = 35;
const int relay1Pin = 52;
const int relay2Pin = 53;

const int trigPin[] = {25, 22, 26,30, 28};
const int echoPin[] = {24, 23, 27,31, 29};

const int irpin = 32;
const int transisitor = 53;
#define outPin 36 

//L293D calibration
//Motor r1
const int motorPin1  = 40;  // Pin 14 of L293
const int motorPin2  = 41;  // Pin 10 of L293
//Motor r2
const int motorPin3  = 42; // Pin  7 of L293
const int motorPin4  = 43;  // Pin  2 of L293

// Define constants
// Minimum distance to consider an obstacle
// Duration for a turn (in milliseconds)
// Maximum number of turns to make to avoid an obstacle

const int MIN_DISTANCE = 5;  
const int TURN_DURATION = 1000;  
const int MAX_GO_ROUND = 2; 

dht DHT; 
int goRound =0;

void setup() {
  fan(0);
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  Serial.begin(9600);

    // Setting up the pins of relays
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);

  pinMode(transisitor, OUTPUT);
  
  pinMode(irpin, INPUT);
  // Initialize ultrasonic sensor pins with for loop
  for (int i = 0; i < 5; i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
  
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay1Pin, HIGH);
  digitalWrite(transisitor, HIGH);

}int a=0;

// ======= Code Section 2 ========
// define and implementing 10 by 10 grid matrix solving algorithm to navigate Ddroid easily with variuos floor structures
// Define grid size
const int GRID_SIZE = 10;  // 10x10 grid

// Define current position of the robot in the grid
int currentRow = 0;
int currentCol = 0;

// Define the grid map (obstacles= 1, free cells= 0)
int gridMap[GRID_SIZE][GRID_SIZE] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

void updateGridMap() {
  // Mark current position as visited and cleaned
  gridMap[currentRow][currentCol] = 1;
}

void moveForwardAndUpdateGridMap(int speed) {
  moveForward(speed, speed);
  delay(500); // Assuming the robot takes half a second to move one cell 
  updateGridMap();
}

void moveBackwardAndUpdateGridMap(int speed) {
  moveBackward(speed);
  delay(500); // Assuming the robot takes half a second to move one cell
  updateGridMap();
}

//  ------------- Extending loop section  ------------------
void loop() {
  
  int readData = DHT.read11(outPin);
  float t = DHT.temperature;        // Read temperature
	float h = DHT.humidity;           // Read humidity

// testing code block 
  Serial.print("Temperature = ");
	Serial.print(t);
	Serial.print("°C | ");
	Serial.print((t*9.0)/5.0+32.0);        // Convert celsius to fahrenheit
	Serial.println("°F ");
	Serial.print("Humidity = ");
	Serial.print(h);
	Serial.println("% ");
  irCheck();


// implementation code block
  if (Serial.available()) {
     fan(0);
      char command;
      Serial.println(command);
        command = Serial.read();
        executeCommand(command);
        
      }
  else if (irCheck()==1){
    fan(0);
    stopRobot();
    delay(1000);
    moveBackward(255);
    delay(300);
  }
    
  else {
        float distance[5];
        for (int i = 0; i < 5; i++) {
          fan(0);
          distance[i] = getDistance(trigPin[i], echoPin[i]);
        }
        float sensor1Detected = distance[0];
        float sensor2Detected = distance[1];
        float sensor3Detected = distance[2];
        float sensor4Detected = distance[3];
        float sensor5Detected = distance[4];

        if (distance[1] < 30|| distance[2] < 30 ||distance[3] < 30){
          stuckAvoid();
          //UPDATE loop 1
          if ( (distance[1] > 5 && distance[1] < 10) || (distance[2] > 5 && distance[2] < 10) ||(distance[3] > 5 && distance[3] < 10)){
            stuckAvoid();
            moveBackward(255);
            delay(300);}
          if (distance[1] < 5|| distance[2] < 5 ||distance[3] < 5){
            fan(0);
            Serial.println("EROR1");
            goRound =0;
            stopRobot();
            delay(500);
            moveBackward(255);
            delay(300);
          }
          if (distance[1] < 25 && distance[2] < 25 ){ 
            Serial.println("EROR2");
            goRound =0;
            stopRobot();
            delay(500);
            turnRight(255);
            delay(1000);}
          else if (distance[1] < 25 && distance[3] < 25 ){ 
            Serial.println("EROR3");
            goRound =0;
            stopRobot();
            delay(500);
            turnRight(255);
            delay(1000);}
          else if (distance[3] < 25 && distance[2] < 25 ){ 
            Serial.println("EROR4");
            goRound =0;
            stopRobot();
            delay(500);
            turnLeft(255);
            delay(1000);}
          else if (distance[1] < 25 && distance[3] < 25 && distance[2] < 25 ){ 
            Serial.println("EROR5");
            stopRobot();
            delay(500);
            if (distance[0] < 15 ){
              Serial.println("EROR6");
              if (goRound>2){
                // goRoundfunc();
                stuckAvoid();
              }
              goRound =goRound+1;
              turnRight(255);
              delay(1000);
              if (handleObs()==false ){
                fan(0);
                moveForward(170, 170);
                delay(100);
              }
              turnRight(255);
              delay(1000); 
              }
            else if (distance[4] < 15 ){
              Serial.println("EROR7");
              stopRobot();
              delay(500);
              if (goRound>2){
                // goRoundfunc();
                stuckAvoid();
              }
              goRound =goRound+1;
              turnLeft(255);delay(1000); 
              if (handleObs()==false ){
                fan(0);
                moveForward(170, 170);
                delay(100);
              }
              turnLeft(255);
              delay(1000);
              }
            else {
              Serial.println("EROR8");
            turnLeft(255);
            delay(1000);
            // moveForward(170, 170);
            // delay(500);
            turnLeft(255);
            delay(1000);
            }    }
          else if (sensor1Detected <25 && sensor3Detected <25) {
            Serial.println("EROR9");
            goRound =0;
            stopRobot();
            delay(500);
              turnRight(255);
              delay(1000);
                // continue
              }
          else if (sensor5Detected <25 && sensor3Detected <25) {
            Serial.println("EROR10");
            goRound =0;
            stopRobot();
            delay(500);
              turnLeft(255);
              delay(1000);
              // continue
              }
          else if (sensor1Detected <25 && sensor2Detected <25 ) {
            Serial.println("EROR11");
            goRound =0;
            stopRobot();
            delay(500);
              turnRight(255);
              delay(1000);
              }
          else if (sensor4Detected < 25 && sensor5Detected <25) {
            Serial.println("EROR12");
            goRound =0;
            stopRobot();
            delay(500);
              turnLeft(255);
              delay(1000);
              }
          else if (sensor3Detected<25) {
            fan(0);
            Serial.println("EROR13");
            stopRobot();
            delay(500);
              turnLeft(255);
              delay(1000);
              // continue
              }   }
// ------------------Section 3 ----------------------------------

        // Add grid-based movement
  if (currentRow < GRID_SIZE - 1) {
    // Move to the next row
    currentRow++;
    moveForwardAndUpdateGridMap(170); 
  } else if (currentCol < GRID_SIZE - 1) {
    // Move to the next column
    currentCol++;
    moveForwardAndUpdateGridMap(170); 
  } else {
    // Grid map complete, reset position to start
    currentRow = 0;
    currentCol = 0;
    // stopRobot();
  }
// -------------------section 3 end ------------------------------
        Serial.println("EROR14");
        fan(0);
        checkHumidityAndMove();
        zigzag();
        moveForward(170, 170);
        delay(100);
// test code block
        Serial.println("u1");
        Serial.println(sensor1Detected);
        Serial.println("u2");
        Serial.println(sensor2Detected);
        Serial.println("u3");
        Serial.println(sensor3Detected);
        Serial.println("u4");
        Serial.println(sensor4Detected);
        Serial.println("u5");
        Serial.println(sensor5Detected);
        // delay(500);
      }
    }
void zigzag(){
fan(0);
float distancex[5];
for (int i = 0; i < 5; i++) {
  fan(0);
  distancex[i] = getDistance(trigPin[i], echoPin[i]);
}
        float distance1 = distancex[0];
        float distance2 = distancex[1];
        float distance3 = distancex[2];
        float distance4 = distancex[3];
        float distance5 = distancex[4];

        if ((distance2 <70 && distance2 >40) && (distance3 <70 && distance3 >40) && (distance4 <70 && distance4 >40)){
          fan(0);
          stopRobot();
          if (distance1 >25 && distance5 >25) {
            fan(0);
          turnLeft(255);
          delay(1000);
          moveForward(170, 170);
          delay(100);
          turnLeft(255);
          delay(1000);
          }
          else if (distance1 >15 && distance5 >25) {
            fan(0);
          turnRight(255);
          delay(1000);
          moveForward(170, 170);
          delay(100);
          turnRight(255);
          delay(1000);
          }
          else if (distance1 >25 && distance5 >15) {
            fan(0);
          turnLeft(255);
          delay(1000);
          moveForward(170, 170);
          delay(100);
          turnLeft(255);
          delay(1000);
          }
          fan(0);
          moveForward(170, 170);
          delay(500);
        }
}
void rearClean(int states){
        if (states==1){
    //This code  will turn Motor A clockwise for 2 sec.
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(2000); 
        }
        else if (states==2){
             //This code will turn Motor A counter-clockwise for 2 sec.
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
    delay(2000);
        } 
}

bool irCheck(){
int sensorstatus=digitalRead(32);
    if(sensorstatus==1){
      Serial.println("distance is greater than 4cm / stair-step detected ");
        return 1;
    }
    else {
       Serial.println("stabalize distance in 0 - 4 cm range / stair step not detected");
        return 0;    
    }
}

// return (random(2) == 0) ? LEFT : RIGHT; // Randomly choose left or right turn
void goRoundfunc(){
  turnLeft(255);
  delay(2000);
}
int humidrotate =0;
void checkHumidityAndMove() {
  fan(0);
  // rearClean(1);
  if(humidrotate==3){
      stabalizeHumidity();
      humidrotate=0;
  }
  else if (humidrotate==0) {
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay1Pin, HIGH);
  }
  humidrotate+=1;
  float humidity = DHT.humidity;  // Read humidity
  // Check the humidity value and perform movement accordingly
  if (humidity < 30) {
    // Move forward     
      if (handleObs()==false ){
      moveForward(170, 170);
      delay(100);
      }
  } else if (humidity >= 30 && humidity < 60) {
    // Turn left
      if (handleObs()==false ){
      turnLeft(255);
      delay(1000);
                }  
  } else if (humidity >= 60 && humidity < 90) {
    // Turn right
      if (handleObs()==false ){
      turnRight(255);
      delay(1000);
            }
  }
}

void stuckAvoid() {
  fan(0);
  stopRobot();
  delay(500);
  moveBackward(255);
  delay(200);
  int goRoundCount = 0;
  while (handleObs() && goRoundCount < MAX_GO_ROUND) {
    turnRight(255);
    delay(TURN_DURATION);
    goRoundCount++;
  }
  if (goRoundCount >= MAX_GO_ROUND) {
    moveBackward(255);
    delay(300);
  }
}

void stabalizeHumidity(){
  float humidity = DHT.humidity;  // Read humidity
  float temperatures = DHT.temperature;
  Serial.println(" THIS IS HUMIDITI");

  if (humidity >= 50 && humidity < 80){
    Serial.println(" h1");
      digitalWrite(relay1Pin, HIGH);
  }
  else if (humidity >= 10 && humidity < 50 && temperatures < 36){
      digitalWrite(relay1Pin, HIGH);
      Serial.println(" h2");    
  }
  else if (humidity <= 20 && temperatures < 25){
    Serial.println(" h3");
      digitalWrite(relay1Pin, HIGH);
      digitalWrite(relay1Pin, LOW);
      digitalWrite(relay1Pin, HIGH);
      digitalWrite(relay1Pin, LOW);
  }
  else if (humidity >= 10 && temperatures < 27){
    Serial.println(" h4");
      digitalWrite(relay1Pin, HIGH);
      digitalWrite(relay1Pin, LOW);
  }
  else if (humidity >= 60 && temperatures > 32){
    Serial.println(" h5");
    digitalWrite(relay1Pin, LOW);
    delay(100);
    digitalWrite(relay1Pin, HIGH);   
  }
  else if(humidity < 90) {
      Serial.println(" h6");
    digitalWrite(relay1Pin, LOW);
    delay(100);
    digitalWrite(relay1Pin, HIGH); 
  }
}
void fan(int state) {
  if (state == 0) {
    // Turn on fan
    // digitalWrite(relay2Pin, HIGH);
    digitalWrite(relay2Pin, LOW);
  } else if (state == 1) {
    // Turn off fan
    // digitalWrite(relay2Pin, LOW);
    digitalWrite(relay2Pin, HIGH);
  } 
}
bool handleObs(){
float distance1[5];
for (int i = 0; i < 5; i++) {
  if (distance1[i] < 15) {
    return true;
  }
  return false;
}
}
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}
void executeCommand(char command) {
    switch (command) {
    case 'F':  // Move forward
      moveForward(200, 200);
      break;
    case 'B':  // Move backward
      moveBackward(200);
      break;
    case 'L':  // Turn left
      turnLeft(255);
      break;
    case 'R':  // Turn right
      turnRight(255);
      break;
    case 'S':  // Stop
      stopRobot();
      break;
    default:
      break;
  }     
    }
void stopRobot() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
void moveForward(int speedLeft, int speedRight) {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(speedLeft);
  motor2.setSpeed(speedRight);
  motor3.setSpeed(speedLeft);
  motor4.setSpeed(speedLeft);
}
void moveBackward(int speed) {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}
void turnLeft(int speed) {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}
int turnRight(int speed) {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}
// ------end of the code for Ddroid ----------