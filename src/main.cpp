#include <Arduino.h>
#include <myWifi.h>
// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}