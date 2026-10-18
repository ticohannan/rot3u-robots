// Host stand-in for Wire, emulating one PCA9685 (register file, auto-increment). Tests only.
#pragma once
#include <cstdint>
#include <vector>
struct WireShim {
  uint8_t reg[256]; uint8_t ptr = 0; int addr = 0; int devAddr = 0x40;
  std::vector<uint8_t> tx; std::vector<uint8_t> rx; size_t rxi = 0;
  WireShim() { for (int i = 0; i < 256; i++) reg[i] = 0; reg[0x00] = 0x11; reg[0xFE] = 30; }  // power-on: asleep
  void begin() {}
  void setClock(long) {}
  void beginTransmission(int a) { tx.clear(); addr = a; }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  uint8_t endTransmission(bool = true);
  uint8_t requestFrom(int a, int n) {
    rx.clear(); rxi = 0;
    if (a != devAddr) return 0;
    for (int i = 0; i < n; i++) rx.push_back(reg[(uint8_t)(ptr + i)]);
    return n;
  }
  int available() { return (int)(rx.size() - rxi); }
  int read() { return rxi < rx.size() ? rx[rxi++] : -1; }
};
extern WireShim Wire;
