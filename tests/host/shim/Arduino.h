// Host (PC) stand-in for the Arduino core, for tests only. Not used on the board.
#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cctype>
#include <string>
#include <deque>
#define PI 3.14159265358979f
#define HALF_PI 1.5707963267949f
#define DEG_TO_RAD 0.017453292519943f
#define RAD_TO_DEG 57.2957795130823f
#define HEX 16
#define DEC 10
#define PROGMEM
typedef uint8_t byte;
typedef bool boolean;
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
#define PSTR(s) (s)
inline char pgm_read_byte(const char *p) { return *p; }
inline const char *strcpy_P(char *d, const char *s) { return strcpy(d, s); }
template <class T, class L, class H> T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }
unsigned long millis();
void delay(unsigned long ms);
void yield();

class Print {
public:
  virtual size_t write(uint8_t c) = 0;
  size_t write(const char *s) { size_t n = 0; while (*s) n += write((uint8_t)*s++); return n; }
  size_t print(const char *s) { return write(s); }
  size_t print(const __FlashStringHelper *s) { return write(reinterpret_cast<const char*>(s)); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(const std::string &s) { return write(s.c_str()); }
  size_t printNum(long long v, int base) {
    char b[70];
    if (base == 16) snprintf(b, sizeof b, "%llX", (unsigned long long)v);
    else snprintf(b, sizeof b, "%lld", v);
    return write(b);
  }
  size_t print(unsigned char v, int base = 10) { return printNum(v, base); }
  size_t print(int v, int base = 10) { return printNum(base == 16 ? (long long)(unsigned)v : v, base); }
  size_t print(unsigned int v, int base = 10) { return printNum(v, base); }
  size_t print(long v, int base = 10) { return printNum(base == 16 ? (long long)(unsigned long)v : v, base); }
  size_t print(unsigned long v, int base = 10) { return printNum((long long)v, base); }
  size_t print(double v, int d = 2) {
    char b[64];
    if (std::isnan(v)) return write("nan");
    if (std::isinf(v)) return write("inf");
    snprintf(b, sizeof b, "%.*f", d, v);
    if (strcmp(b, "-0") == 0 || (b[0] == '-' && strspn(b + 1, "0.") == strlen(b + 1))) return write(b + 1);
    return write(b);
  }
  size_t println() { return write("\r\n"); }
  template <typename T> size_t println(T v) { size_t n = print(v); return n + println(); }
  template <typename T> size_t println(T v, int d) { size_t n = print(v, d); return n + println(); }
};

extern std::deque<char> g_in;
void g_char(char c);
struct SerialShim : public Print {
  void begin(long) {}
  int available() { return (int)g_in.size(); }
  int read() { if (g_in.empty()) return -1; char c = g_in.front(); g_in.pop_front(); return (unsigned char)c; }
  int peek() { return g_in.empty() ? -1 : (unsigned char)g_in.front(); }
  size_t write(uint8_t c) override { g_char((char)c); return 1; }
  using Print::write;
  operator bool() { return true; }
};
extern SerialShim Serial;
inline char *dtostrf(double v, signed char w, unsigned char p, char *out) {
  sprintf(out, "%*.*f", w, p, v);
  return out;
}
