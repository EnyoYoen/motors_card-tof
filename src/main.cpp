#include "tof.h"

#include <Arduino.h>

void setup() {  
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  uint32_t res = init_tof();
  
  digitalWrite(LED_BUILTIN, LOW);

  std_msgs__msg__Int32 single_msg;
  single_msg.data = res;
  RCSOFTCHECK(rcl_publish(&distances_publisher, &single_msg, NULL));
}

void loop() {
  
}