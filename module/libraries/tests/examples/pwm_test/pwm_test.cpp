#include "Arduino.h"

void setup() {
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
}

void loop() {
  static uint16_t pwmVal[] = {0, 100, 400, 800, 1600, 3200, 6400, 12800};
  analogWrite(10, pwmVal[0]);
  analogWrite(11, pwmVal[1]);
  analogWrite(12, pwmVal[2]);
  analogWrite(13, pwmVal[3]);
  analogWrite(14, pwmVal[4]);
  analogWrite(20, pwmVal[5]);
  analogWrite(23, pwmVal[6]);
  analogWrite(24, pwmVal[7]);

  for (uint8_t i = 0; i < 8; i++) {
    pwmVal[i] += 100;
  }
  delay(10);

}