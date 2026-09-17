// Host regression tests: execute the production Motor calibration against a fake bus.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <vector>

constexpr int HIGH = 1, LOW = 0, INPUT_PULLUP = 2, OUTPUT = 1, HEX = 16;
unsigned long clockMs = 0;
unsigned long millis() { return clockMs++; }
void delay(unsigned long ms) { clockMs += ms; }
int digitalRead(int) { return LOW; }
void digitalWrite(int, int) {}
void pinMode(int, int) {}

enum Scenario { SUCCESS, WRONG_TARGET, NOT_REACHED, WRONG_POSITION, STALLED,
                DISABLED, RESET_IGNORED, INVALID_FRAME, LOST_REPLY,
                FRACTIONAL_WRONG_TARGET, FRACTIONAL_WRONG_POSITION };
Scenario scenario = SUCCESS;
long targetTenths = 0;
unsigned targetReads = 0, positionReads = 0, reachedReads = 0, resets = 0;
std::vector<std::vector<uint8_t>> sent;

struct FakeSerial {
  std::deque<uint8_t> rx;
  void begin(unsigned long) {}
  void flush() {}
  template <typename T> void print(T) {}
  template <typename T> void print(T, int) {}
  template <typename T> void println(T) {}
  template <typename T> void println(T, int) {}
  void println() {}
  int available() { return (int)rx.size(); }
  int read() { int value = rx.front(); rx.pop_front(); return value; }
  void positionReply(uint8_t id, uint8_t function, long value) {
    uint32_t magnitude = value < 0 ? (uint32_t)-value : (uint32_t)value;
    uint8_t tail = scenario == INVALID_FRAME ? 0 : 0x6B;
    const uint8_t frame[] = {id, function, (uint8_t)(value < 0),
      (uint8_t)(magnitude >> 24), (uint8_t)(magnitude >> 16),
      (uint8_t)(magnitude >> 8), (uint8_t)magnitude, tail};
    rx.insert(rx.end(), frame, frame + sizeof(frame));
  }
  size_t write(const uint8_t *command, size_t size) {
    sent.emplace_back(command, command + size);
    const uint8_t id = command[0], function = command[1];
    if (function == 0xFD) {
      assert(size == 16 && command[13] == 1 && command[14] == 0);
      uint32_t magnitude = ((uint32_t)command[9] << 24) |
        ((uint32_t)command[10] << 16) | ((uint32_t)command[11] << 8) | command[12];
      targetTenths = command[2] ? -(long)magnitude : (long)magnitude;
    } else if (function == 0x0A) {
      // A reset before target and three reached confirmations is a regression.
      assert(targetReads > 0 && positionReads >= 3 && reachedReads >= 3);
      assert(command[2] == 0x6D && command[3] == 0x6B);
      ++resets;
    } else if (function == 0x33) {
      ++targetReads;
      if (scenario != LOST_REPLY) {
        positionReply(id, function, targetTenths + (scenario == WRONG_TARGET ? 1000 :
          scenario == FRACTIONAL_WRONG_TARGET ? 1 : 0));
      }
    } else if (function == 0x36) {
      ++positionReads;
      long actual = resets && scenario != RESET_IGNORED ? 0 : targetTenths;
      if (scenario == WRONG_POSITION) actual += 1000;
      if (scenario == FRACTIONAL_WRONG_POSITION) actual += 51;
      positionReply(id, function, actual);
    } else if (function == 0x3A) {
      uint8_t flag = 0x03;
      if (scenario == NOT_REACHED) flag = 0x01;
      if (scenario == STALLED) flag = 0x07;
      if (scenario == DISABLED) flag = 0x02;
      if (flag == 0x03) ++reachedReads;
      rx.insert(rx.end(), {id, function, flag, 0x6B});
    }
    // Motion/reset commands intentionally produce no automatic replies.
    return size;
  }
};
FakeSerial Serial, Serial1;

// Arduino normally generates these prototypes for the sketch.
void homing();
void updatePlateLogic();
void updateTrayLogic();
bool checkPlateEchoMatch();
#include "../../TF_test.ino"

void resetSimulation(Scenario next) {
  scenario = next;
  clockMs = 0;
  targetTenths = 0;
  targetReads = positionReads = reachedReads = resets = 0;
  sent.clear();
  Serial1.rx.clear();
}

void expectFailure(Scenario next, unsigned expectedResets, float offsetDeg = 500.0f) {
  resetSimulation(next);
  Motor motor(1);
  assert(!motor.calibrateZeroPosition(offsetDeg));
  assert(resets == expectedResets);
  assert(clockMs < 40000);
}

int main() {
  const long configuredTenths[] = {715, 735, 713, 561, 427, 442, 382, 500};
  for (uint8_t id = 1; id <= 8; ++id) {
    resetSimulation(SUCCESS);
    Motor motor(id);
    assert(motor.calibrateZeroPosition(HOMING_ZERO_OFFSETS_DEG[id - 1]));
    assert(resets == 1 && targetTenths == configuredTenths[id - 1]);
    for (const auto &frame : sent) assert(frame[0] == id);
  }
  // All addresses, signed offsets, zero offsets, and degree-to-wire conversion.
  const long offsets[] = {500, 4000, -500, 0, 250, 600, 700, 800};
  for (uint8_t id = 1; id <= 8; ++id) {
    resetSimulation(SUCCESS);
    Motor motor(id);
    assert(motor.calibrateZeroPosition(offsets[id - 1]));
    assert(resets == 1 && targetTenths == offsets[id - 1] * 10);
    for (const auto &frame : sent) assert(frame[0] == id);
  }
  expectFailure(WRONG_TARGET, 0);
  expectFailure(NOT_REACHED, 0);
  expectFailure(WRONG_POSITION, 0);
  expectFailure(STALLED, 0);
  expectFailure(DISABLED, 0);
  expectFailure(RESET_IGNORED, 1);
  expectFailure(INVALID_FRAME, 0);
  expectFailure(LOST_REPLY, 0);
  expectFailure(FRACTIONAL_WRONG_TARGET, 0, 71.3f);
  expectFailure(FRACTIONAL_WRONG_POSITION, 0, 71.3f);
  std::puts("PASS: 16 motor/offset cases and 10 failure cases; no premature reset.");
}
