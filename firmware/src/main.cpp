#include <Arduino.h>
#include "app/Config.h"
#include "drivers/SsiEncoder.h"
#include "drivers/MotorDriver.h"

SsiEncoder encoder(PIN_ENC_CS, PIN_ENC_CLK, PIN_ENC_MISO);
MotorDriver MtrDrv(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

void setup() {
  Serial.begin(115200);
  if (!encoder.init()) {
    Serial.println("Encoder init failed!");
    while (1) delay(1000);
  }
  Serial.println("Encoder initialized");

  if(!MtrDrv.init(VLIM, VSUP)) {
    Serial.println("Motor Driver init failed!");
    while (1) delay(1000);
  }
  MtrDrv.enable();
  Serial.println("Motor Driver initialized");
}

void loop() {
  encoder.update();
  Serial.print("Angle: ");
  Serial.print(encoder.angleDegWrapped(), 2);
  Serial.print("\tVel (rad/s): ");
  Serial.println(encoder.velocityRadS(), 4);

  // open loop motor control
  MtrDrv.setPWM(0.2f, 0.0f, 0.0f);
  delay(5);
  MtrDrv.setPWM(0.0f, 0.0f, 0.0f);
  delay(95);
}
