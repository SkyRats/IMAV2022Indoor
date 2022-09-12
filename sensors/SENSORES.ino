#include <SoftwareSerial.h>
#include "Adafruit_CCS811.h"

SoftwareSerial mySerial(2, 3);

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int TempByte  = 0;
unsigned int Len  = 0;
unsigned int Temp  = 0;
Adafruit_CCS811 ccs;

void setup() {
  Serial.begin(9600); 
  if(!ccs.begin()){
    while(1);
  }
   while(!ccs.available());
  mySerial.begin(9600);
}

int temperatura(){
  mySerial.flush();
  mySerial.write(0X50);
  delay(50);  // trig US-100 begin to measure the distance
  Temp = mySerial.read() - 45;
  return Temp;
}

int distancia(){
  mySerial.flush();
  mySerial.write(0x55);
  delay(50); 
  HighByte = mySerial.read();
  LowByte  = mySerial.read();
  Len = HighByte*256 + LowByte;
  return Len;
}


int co2(){
  if(ccs.available()){
    if(!ccs.readData()){
      return (ccs.geteCO2());
    }
    else{
      while(1);
    }
  }
}

void loop() {
  while (mySerial.available()>0){
    mySerial.read();
  }
  
  int temperaturaVar = temperatura();
  // Serial.print("Temp: ");
  Serial.print(temperaturaVar);
  Serial.print(" ");
  delay(10);
  
  while (mySerial.available()>0){
    mySerial.read();
 }
 
  int distanciaVar = distancia();
  // Serial.print("Dist: ");
  Serial.print(distanciaVar);
  Serial.print(" ");
  delay(10);   


  int co2Var = co2();  
  // Serial.print("CO: ");
  Serial.println(co2Var);
  delay(10);   
                                                       
} 
