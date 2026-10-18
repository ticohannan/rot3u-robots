// Host stand-in for EEPROM (4 KB, persisted by the harness between runs). Tests only.
#pragma once
#include <cstdint>
#include <cstring>
struct EEPROMShim {
  uint8_t mem[4096];
  EEPROMShim() { memset(mem, 0xFF, sizeof mem); }
  uint8_t read(int a) { return mem[a & 4095]; }
  void write(int a, uint8_t v) { mem[a & 4095] = v; }
  void update(int a, uint8_t v) { mem[a & 4095] = v; }
  uint16_t length() { return 4096; }
  template <typename T> T &get(int a, T &t) { memcpy(&t, mem + a, sizeof(T)); return t; }
  template <typename T> const T &put(int a, const T &t) { memcpy(mem + a, &t, sizeof(T)); return t; }
};
extern EEPROMShim EEPROM;
