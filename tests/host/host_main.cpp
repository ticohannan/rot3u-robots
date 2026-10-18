// Host test harness: runs one sketch on the PC with an emulated PCA9685 and EEPROM.
// Usage: host_<sketch> <log file> [state file]   (commands on stdin)
//   stdin lines are sent to the sketch as serial lines, except directives:
//     @run ms      run loop() for ms of simulated time
//     @keyat ms [c] inject a keypress ms of simulated time from now: c alone, or 'x' + newline
//     @idle text   send text as a line once the sketch has been quiet (no output, no servo
//                  writes) for 1 s of simulated time inside delay() - answers a WAIT prompt
// Log lines: C <command> | O <output line> | W <ch> <off counts> | T <ms>
// The state file (Wire registers + EEPROM) is loaded at start and saved at exit,
// so two runs in a row behave like a board reset with the PCA9685 still powered.
#include "Arduino.h"
#include "Wire.h"
#include "EEPROM.h"
#include <iostream>
std::deque<char> g_in; SerialShim Serial; WireShim Wire; EEPROMShim EEPROM;
static unsigned long g_ms = 0; static FILE *g_log; static std::string g_line;
static long g_keyAt = -1; static char g_key = 'x'; static bool g_any = false;
static std::deque<std::string> g_idle; static unsigned long g_act = 0;
void g_char(char c) {
  std::cout << c; g_any = true; g_act = g_ms;
  if (c == '\n') { fprintf(g_log, "O %s\n", g_line.c_str()); g_line.clear(); }
  else if (c != '\r') g_line += c;
}
static void tick(unsigned long ms) {
  g_ms += ms;
  if (g_keyAt >= 0 && (long)g_ms >= g_keyAt) { g_in.push_back(g_key); if (g_key == 'x') g_in.push_back('\n'); g_keyAt = -1; }
}
static void idleFeed() {
  if (!g_idle.empty() && g_in.empty() && g_ms - g_act > 1000) {
    for (char c : g_idle.front()) g_in.push_back(c);
    g_in.push_back('\n'); g_idle.pop_front(); g_act = g_ms;
  }
}
unsigned long millis() { return g_ms; }
void delay(unsigned long ms) { tick(ms); fprintf(g_log, "T %lu\n", g_ms); idleFeed(); }
void yield() {}
uint8_t WireShim::endTransmission(bool) {
  if (addr != devAddr) return 2;                    // NACK: no device
  if (tx.empty()) return 0;
  ptr = tx[0];
  for (size_t i = 1; i < tx.size(); i++) {
    uint8_t r = (uint8_t)(ptr + i - 1);
    if (r == 0xFE && !(reg[0] & 0x10)) continue;    // prescale only writable while asleep
    reg[r] = tx[i];
  }
  if (tx.size() >= 2 && tx[0] == 0x00) reg[0] &= 0x7F;   // RESTART bit reads back clear
  if (tx.size() == 2 && tx[0] == 0xFD) {                 // ALL_LED_OFF_H loads every channel's OFF_H
    for (int ch = 0; ch < 16; ch++) { reg[6 + 4 * ch + 3] = tx[1]; if (tx[1] & 0x10) fprintf(g_log, "W %d 4096 %lu ALL\n", ch, g_ms); }
  }
  if (tx.size() == 5 && ptr >= 6 && ptr < 6 + 4 * 16) {
    int ch = (ptr - 6) / 4; int off = tx[3] | (tx[4] << 8); g_act = g_ms;
    fprintf(g_log, "W %d %d %lu\n", ch, off, g_ms);
  }
  return 0;
}
#include "sketch_host.cpp"
static void loadState(const char *f) {
  FILE *s = fopen(f, "rb"); if (!s) return;
  if (fread(Wire.reg, 1, 256, s) != 256 || fread(EEPROM.mem, 1, 4096, s) != 4096) {}
  fclose(s);
}
static void saveState(const char *f) {
  FILE *s = fopen(f, "wb"); if (!s) return;
  fwrite(Wire.reg, 1, 256, s); fwrite(EEPROM.mem, 1, 4096, s); fclose(s);
}
static void runFor(unsigned long ms) {             // loop() with time advancing 1 ms per pass
  unsigned long end = g_ms + ms;
  while (g_ms < end) { loop(); tick(1); }
}
static void settle() {                             // run until 3 s of simulated time pass with no output
  unsigned long quiet = g_ms, cap = g_ms + 600000;
  while (g_ms < cap) {
    g_any = false; loop(); tick(1);
    if (g_any || !g_in.empty()) quiet = g_ms;
    if (g_ms - quiet > 3000) break;
  }
}
int main(int argc, char **argv) {
  g_log = fopen(argc > 1 ? argv[1] : "/dev/null", "w");
  if (argc > 2) loadState(argv[2]);
  setup(); settle();
  std::string line;
  while (std::getline(std::cin, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.rfind("@run ", 0) == 0) { runFor(atol(line.c_str() + 5)); continue; }
    if (line.rfind("@keyat ", 0) == 0) {          // @keyat ms [c]: c alone (no newline), default "x\n"
      char *e; g_keyAt = (long)g_ms + strtol(line.c_str() + 7, &e, 10);
      g_key = (*e == ' ' && e[1]) ? e[1] : 'x';
      continue;
    }
    if (line.rfind("@idle", 0) == 0) { g_idle.push_back(line.size() > 6 ? line.substr(6) : std::string()); continue; }
    std::cout << ">> " << line << "\n";
    fprintf(g_log, "C %s\n", line.c_str());
    for (char c : line) g_in.push_back(c);
    g_in.push_back('\n');
    settle();
  }
  if (argc > 2) saveState(argv[2]);
  fclose(g_log);
}
