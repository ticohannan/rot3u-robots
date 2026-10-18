// Host stand-in for the Servo library (compile only). Tests only.
#pragma once
#include <cstdint>
class Servo {
public:
  uint8_t attach(int p, int = 544, int = 2400) { pin = p; on = true; return 0; }
  void detach() { on = false; }
  void writeMicroseconds(int v) { us = v; }
  bool attached() { return on; }
  int pin = -1, us = 1500; bool on = false;
};
