#include <Arduino.h>
#include "app/Config.h"
#include "drivers/SsiEncoder.h"

SsiEncoder encoder(PIN_ENC_CS, PIN_ENC_CLK, PIN_ENC_MISO);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("BOOT");

  if (!encoder.init()) {
    Serial.println("Encoder init failed!");
    while (1) delay(1000);
  }
}



void loop() {
  encoder.update();

  Serial.print("Angle: ");
  Serial.print(encoder.angleDegWrapped(), 2);
  Serial.print("\tVel (rad/s): ");
  Serial.println(encoder.velocityRadS(), 4);

  delay(100);
}
