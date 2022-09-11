#include "Adafruit_CCS811.h"
 
Adafruit_CCS811 ccs;
 
void setup() {
  Serial.begin(115200); 
  if(!ccs.begin()){
    while(1);
  }
   while(!ccs.available());
}
 
void loop() {
  if(ccs.available()){
    if(!ccs.readData()){
      Serial.println(ccs.geteCO2());
    }
    else{
      Serial.println(0);
      while(1);
    }
  }
  delay(500);
}
