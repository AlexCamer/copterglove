#include <MPU9250.h>
#include <SPI.h>

MPU9250 IMU(Wire,0x68);
const int flexSensor = A0; 

int lastFlex = 0;
int lastX = 0;
int lastY = 0;
int PotWiperVoltage = 1;
int RawVoltage = 0;

const int slaveSelectPin3 = 3;
const int slaveSelectPin5 = 5;
const int slaveSelectPin6 = 6;
const int slaveSelectPin9 = 9;

byte address = 0x00;

int status;

void setup() {
  Serial.begin(115200);
  pinMode(flexSensor, INPUT);
  pinMode(slaveSelectPin3, OUTPUT);
  pinMode(slaveSelectPin5, OUTPUT);
  pinMode(slaveSelectPin6, OUTPUT);

  SPI.begin();
  
  while(!Serial) {} 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  IMU.readSensor();
  int inputFlex = analogRead(flexSensor);
  
  double inputX = IMU.getAccelX_mss();
  double inputY = IMU.getAccelY_mss();

  int currentX = map(inputX, -10, 10, -100, 350);
  int currentY = map(inputY, -10, 10, -100, 350);
  int currentFlex = map(inputFlex, 600, 500, 250, 0);

  currentX = compare((int)currentX, lastX, 25, 275, -25);
  currentY = compare((int)currentY, lastY, 25, 275, -25);
  currentFlex = compare(currentFlex, lastFlex, 25, 250, 0);

  int tempX = safety(currentX, 125, 25);
  int tempY = safety(currentY, 125, 25);
  
  Serial.print("X: " + String(tempX));
  Serial.print("\t\t");
  Serial.print("Y: " + String(tempY));
  Serial.print("\t\t");
  Serial.print("FLEX: " + String(currentFlex));
  Serial.println();
  
  digitalPotWrite(slaveSelectPin3, tempX); //tempX
  digitalPotWrite(slaveSelectPin5, tempY); //tempY
  digitalPotWrite(slaveSelectPin6, currentFlex);

  lastX = currentX;
  lastY = currentY;
  lastFlex = currentFlex;
  delay(25);
}

void digitalPotWrite(int pin, int value) {
  digitalWrite(pin, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(pin, HIGH);
}

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

int safety(int current, int rest, int increment){
  if(current > rest){
    current -= increment;
  }
  else if(current < rest){
    current += increment;
  }
  return current;
}
