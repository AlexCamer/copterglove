/*
Authors: Ali Budak and Alexandru Camer
Code for a hand gesture controlled quadcopter (Fall 2018 / SE 101 Project)
*/
// Importing the gyroscope sensor library and the communication protocol library
#include <MPU9250.h>
#include <SPI.h>

// Initializing the sensors
MPU9250 IMU(Wire,0x68);
const int flexSensor = A0; 

// Initializing the global variables 
int lastFlex = 0;
int lastX = 0;
int lastY = 0;
int buttonSwitch = 0;
int buttonState = 0;
int buttonDelay = 0;   

//Initializing variables for the arduino pins
const int slaveSelectPin3 = 3;
const int slaveSelectPin5 = 5;
const int slaveSelectPin6 = 6;
const int slaveSelectPin9 = 9;
const int buttonPin = 7;
const int ledPinRed =  2;
const int ledPinGreen =  4;

byte address = 0x00;

int status;

// Setting up the pins and sensors
void setup() {
  Serial.begin(115200);
  pinMode(flexSensor, INPUT);
  pinMode(slaveSelectPin3, OUTPUT);
  pinMode(slaveSelectPin5, OUTPUT);
  pinMode(slaveSelectPin6, OUTPUT);
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(buttonPin, INPUT);
  digitalWrite(ledPinRed, HIGH);

  // Initializing the communication protocol
  SPI.begin();
  
  while(!Serial) {} 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful"); // Printing an error message if there are improper connections
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // Reading data from sensors
  IMU.readSensor();
  int inputFlex = analogRead(flexSensor);
  buttonState = digitalRead(buttonPin);

  // Getting the X,Y acceleration values from the gyroscope
  double inputX = IMU.getAccelX_mss();
  double inputY = IMU.getAccelY_mss();

  // Mapping the sensor data into values that are convenient to output
  int currentX = map(inputX, -10, 10, -100, 350);
  int currentY = map(inputY, -10, 10, -100, 350);
  int currentFlex = map(inputFlex, 600, 500, 250, 0);

  // Set the current x,y values using a compare function that allows optimal sensitivity
  currentX = compare((int)currentX, lastX, 25, 275, -25);
  currentY = compare((int)currentY, lastY, 25, 275, -25);
  currentFlex = compare(currentFlex, lastFlex, 25, 250, 0);

  // Adding a center safety interval
  int tempX = safety(currentX, 125, 25);
  int tempY = safety(currentY, 125, 25);
  int tempFlex = currentFlex;

  // Initialize an on/off switch 
  if(buttonState == HIGH && buttonDelay == 0){
    if(buttonSwitch == 0){
      digitalWrite(ledPinGreen, HIGH);
      digitalWrite(ledPinRed, LOW);
      buttonSwitch = 1;
    }
    else{
      digitalWrite(ledPinGreen, LOW);
      digitalWrite(ledPinRed, HIGH);
      buttonSwitch = 0;
    }
    buttonDelay += 25;
  }
  if(buttonDelay != 0){
    if(buttonDelay == 500){buttonDelay = 0;}
    else{buttonDelay += 25;}  
  }
  // Set the inputs to 0 if switches off
  if(buttonSwitch == 0){
    tempFlex = 0;
    tempX = 125;
    tempY = 125;
  }

  // Printing values to a serial monitor to check output data
  Serial.print("X: " + String(tempX));
  Serial.print("\t\t");
  Serial.print("Y: " + String(tempY));
  Serial.print("\t\t");
  Serial.print("FLEX: " + String(tempFlex));
  Serial.println();

  // Sending outputting data to digital potentiometers, allowing to move the quadcopter
  digitalPotWrite(slaveSelectPin3, tempX);
  digitalPotWrite(slaveSelectPin5, tempY);
  digitalPotWrite(slaveSelectPin6, tempFlex);
  
  lastX = currentX;
  lastY = currentY;
  lastFlex = currentFlex;
  delay(25);
}

// A function to send outputting to quadcopter
void digitalPotWrite(int pin, int value) {
  digitalWrite(pin, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(pin, HIGH);
}

// A compare function that allows an optimal sensitivity
int compare(int current, int last, int increment, int upperLimit, int lowerLimit){
  if(current - last > increment){                                                 
    current = last + increment;
  }
  else if(current - last < -1 * increment){
    current = last - increment;
  }
  else{current = last;}
  
  if(current < lowerLimit){
    current = lowerLimit;
  }
  else if(current > upperLimit){
    current = upperLimit;
  }
  return current;
}

// A function that sets a safe space around the starting values (at the beginning of the movement)
int safety(int current, int rest, int increment){
  if(current > rest){
    current -= increment;
  }
  else if(current < rest){
    current += increment;
  }
  return current;
}
