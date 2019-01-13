/*
Authors: Ali Budak and Alexandru Camer
We are attempting to convert a quadcopter remote into a glove so that we could control the quadcopter with the gestures
In our prototpye, we got our flex and gyroscope/accelerometer sensors to function properly and output converted position/flex data such that it could be used with the glove effectively.
*/
#include <MPU9250.h> //importing the library for gyroscope
#include <SPI.h> //importing the communication library

MPU9250 IMU(Wire,0x68); // gyroscope sensor named MPU9250 with address of 0x68
const int flexSensor = A0; 

int lastFlex = 0; // initializing global variables for storing previous data of from sensors
int lastX = 0;
int lastY = 0;


// set pin 10 as the slave select for the digital pot:
const int slaveSelectPin3 = 3;
const int slaveSelectPin5 = 5;
const int slaveSelectPin6 = 6;
int PotWiperVoltage = 1;
int RawVoltage = 0;
byte address = 0x00;

int status;

void setup() {
  Serial.begin(115200); //creating a console to display data

  pinMode(flexSensor, INPUT); //initializing the flex sensor at A0 on arduino
  pinMode(slaveSelectPin3, OUTPUT); // set the slaveSelectPin as an output
  pinMode(slaveSelectPin5, OUTPUT);
  pinMode(slaveSelectPin6, OUTPUT);

  SPI.begin();
  
  while(!Serial) {} 
  status = IMU.begin(); // start communication with MPU9250 sensor
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful"); // checking if gyro sensor is connected, if not, should return an error message
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  IMU.readSensor(); //reading data from sensors
  double inputX = IMU.getAccelX_mss(); // get the x, y positions of the gyroscope, using sensor library commands
  double inputY = IMU.getAccelY_mss();
  int inputFlex = analogRead(flexSensor);

  int currentX = map(inputX, -10, 10, -1000, 1000); // mapping data from sensors into a convenient range
  int currentY = map(inputY, -10, 10, -1000, 1000);
  int currentFlex = map(inputFlex, 600, 500, 250, 0);

  currentX = compare((int)currentX, lastX, 100, 1000, -1000); // putting mapped data through an algorithm to reduce erratic data, see function below
  currentY = compare((int)currentY, lastY, 100, 1000, -1000);
  currentFlex = compare(currentFlex, lastFlex, 25, 250, 0);

  Serial.print("X: " + String(currentX)); // print converted data to console
  Serial.print("\t\t");
  Serial.print("Y: " + String(currentY));
  Serial.print("\t\t");
  Serial.print("FLEX: " + String(currentFlex));
  Serial.println();

  digitalPotWrite(slaveSelectPin6, currentFlex);
  
  
  lastX = currentX; // storing previous data into global variables for comparison in the next iteration
  lastY = currentY;
  lastFlex = currentFlex;
  delay(50); // 50 millisecond delay between each iteration
}

void digitalPotWrite(int pin, int value) {
  digitalWrite(pin, LOW);  // take the SS pin low to select the chip:
  SPI.transfer(address);  // send in the address and value via SPI:
  SPI.transfer(value);
  digitalWrite(pin, HIGH);  // take the SS pin high to de-select the chip:
}

int compare(int current, int last, int increment, int upperLimit, int lowerLimit){
  // scaling converted position data into greater ranges in order to reduce sensitivity for hand gestures
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
