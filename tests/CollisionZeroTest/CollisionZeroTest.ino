#include <Arduino.h>
#include <string.h>

#define ABS(x) ((x) > 0 ? (x) : -(x))

// Standalone X-branch experiment. Open this sketch, not the production sketch.
// Wire protocol follows TF_test.ino and the ZDT X-series V2 manual.
constexpr uint8_t HOME_MOTOR_COUNT = 8;
constexpr uint8_t TEST_MOTOR_COUNT = 6;
constexpr uint8_t RESET_MOTOR_COUNT = 3;
constexpr uint16_t ACC = 300;
constexpr uint16_t DECL = 300;
constexpr uint16_t VELOCITY_RPM = 200;
constexpr uint32_t TARGET_DEG = 500;
constexpr long TARGET_TENTHS = TARGET_DEG * 10L;  // Readback comparison only.
constexpr long POSITION_TOLERANCE = 50;  // 5 degrees, in 0.1-degree units.
constexpr long ZERO_TOLERANCE = 10;      // 1 degree.
constexpr unsigned long REPLY_TIMEOUT_MS = 300;
constexpr unsigned long STATE_TIMEOUT_MS = 2000;
constexpr unsigned long HOME_TIMEOUT_MS = 60000;
constexpr unsigned long MOVE_TIMEOUT_MS = 15000;
constexpr unsigned long OBSERVE_PAUSE_MS = 3000;
constexpr uint8_t OUTPUT_PINS[] = {22, 23, 24, 25, 26, 27, 35, 36};

long firstPosition[TEST_MOTOR_COUNT];
long beforeSecondMove[TEST_MOTOR_COUNT];
long finalPosition[TEST_MOTOR_COUNT];

// Consume stale replies before issuing the next request; never wait forever.
bool clearReceiveBuffer() {
  unsigned long start = millis();
  unsigned long lastByte = start;
  while (millis() - start < REPLY_TIMEOUT_MS) {
    if (Serial1.available()) {
      Serial1.read();
      lastByte = millis();
    } else if (millis() - lastByte >= 20) {
      return true;
    }
  }
  return false;
}

bool exchange(const uint8_t *command, uint8_t commandSize,
              uint8_t *reply, uint8_t replySize) {
  if (!clearReceiveBuffer()) return false;
  Serial1.write(command, commandSize);
  Serial1.flush();

  uint8_t received[64];
  uint8_t count = 0;
  unsigned long start = millis();
  unsigned long lastByte = start;
  while (millis() - start < REPLY_TIMEOUT_MS) {
    if (Serial1.available()) {
      int value = Serial1.read();
      if (count == sizeof(received)) return false;
      received[count++] = (uint8_t)value;
      lastByte = millis();
    } else if (count && millis() - lastByte >= 20) {
      break;
    }
  }

  // Match by length/address/function, not by the first 0x6B in the payload.
  for (uint8_t i = 0; i + replySize <= count; ++i) {
    if (received[i] == command[0] && received[i + 1] == command[1] &&
        received[i + replySize - 1] == 0x6B) {
      memcpy(reply, received + i, replySize);
      return true;
    }
  }
  Serial.print(F("No valid reply: motor="));
  Serial.print(command[0]);
  Serial.print(F(" function=0x"));
  Serial.print(command[1], HEX);
  Serial.print(F(" bytes="));
  for (uint8_t i = 0; i < count; ++i) {
    Serial.print(received[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  return false;
}

// Response=None: sending is not proof of execution. Poll the resulting state.
bool sendAction(const uint8_t *command, uint8_t size) {
  if (!clearReceiveBuffer()) return false;
  if (Serial1.write(command, size) != size) return false;
  Serial1.flush();
  delay(50);
  return true;
}

bool readFlag(uint8_t id, uint8_t function, uint8_t &flag) {
  const uint8_t command[] = {id, function, 0x6B};
  uint8_t reply[4];
  if (!exchange(command, sizeof(command), reply, sizeof(reply))) return false;
  flag = reply[2];
  return true;
}

bool readPosition(uint8_t id, uint8_t function, long &tenths) {
  const uint8_t command[] = {id, function, 0x6B};
  uint8_t reply[8];
  if (!exchange(command, sizeof(command), reply, sizeof(reply))) return false;
  uint32_t magnitude = ((uint32_t)reply[3] << 24) |
                       ((uint32_t)reply[4] << 16) |
                       ((uint32_t)reply[5] << 8) | reply[6];
  if (reply[2] > 1 || magnitude > 0x7FFFFFFFUL) return false;
  tenths = reply[2] ? -(long)magnitude : (long)magnitude;
  return true;
}

bool nearPosition(long actual, long expected, long tolerance) {
  // Widen before subtracting so an invalid extreme reading cannot overflow.
  int64_t difference = (int64_t)actual - expected;
  return difference >= -tolerance && difference <= tolerance;
}

void printPosition(uint8_t id, const char *stage, long tenths) {
  Serial.print(F("Motor "));
  Serial.print(id);
  Serial.print(' ');
  Serial.print(stage);
  Serial.print(F(": "));
  Serial.print(tenths / 10.0f, 1);
  Serial.println(F(" deg"));
}

void haltTest(uint8_t id, const char *reason) {
  Serial.print(F("TEST FAILED, motor="));
  Serial.print(id);
  Serial.print(F(": "));
  Serial.println(reason);
  // Stop requests are best effort if communications have failed.
  for (uint8_t motor = 1; motor <= HOME_MOTOR_COUNT; ++motor) {
    const uint8_t interrupt[] = {motor, 0x9C, 0x48, 0x6B};
    Serial1.write(interrupt, sizeof(interrupt));
    Serial1.flush();
    delay(30);
    const uint8_t stop[] = {motor, 0xFE, 0x98, 0, 0x6B};
    Serial1.write(stop, sizeof(stop));
    Serial1.flush();
    delay(30);
  }
  Serial.println(F("Stop requested for motors 1-8. Reset board to retry."));
  while (true) delay(1000);
}

void waitForEnabled(uint8_t id) {
  unsigned long start = millis();
  while (millis() - start < STATE_TIMEOUT_MS) {
    uint8_t status;
    if (!readFlag(id, 0x3A, status)) haltTest(id, "Cannot read enable status");
    if (status & 0x01) return;
    delay(50);
  }
  haltTest(id, "Enable status timeout");
}

bool waitForHoming(uint8_t id) {
  unsigned long start = millis();
  uint8_t settled = 0;
  bool observedRunning = false;
  while (millis() - start < HOME_TIMEOUT_MS) {
    uint8_t flag;
    if (!readFlag(id, 0x3B, flag)) haltTest(id, "Cannot read homing status");
    if (flag & 0x08) haltTest(id, "Homing failed");
    if ((flag & 0x03) != 0x03) haltTest(id, "Encoder/calibration not ready");
    if (flag & 0x80) haltTest(id, "Set S_PosTDP to Disable (0.1-degree commands)");
    if (flag & 0x04) observedRunning = true;
    settled = (flag & 0x04) ? 0 : settled + 1;
    if (settled >= 3) return observedRunning;
    delay(100);
  }
  haltTest(id, "Homing timeout");
  return false;
}

void collisionHomingPass(uint8_t pass) {
  Serial.print(F("Collision homing pass "));
  Serial.println(pass);
  for (uint8_t id = 1; id <= HOME_MOTOR_COUNT; ++id) {
    const uint8_t command[] = {id, 0x9A, 2, 0, 0x6B};
    if (!sendAction(command, sizeof(command))) haltTest(id, "Cannot send homing command");
    bool observedRunning = waitForHoming(id);
    long position;
    if (!readPosition(id, 0x36, position)) haltTest(id, "Cannot read homed position");
    if (!nearPosition(position, 0, ZERO_TOLERANCE)) haltTest(id, "Homing position is not zero");
    printPosition(id, "collision home status/position OK", position);
    if (!observedRunning) {
      Serial.println(F("Homing busy state not observed; verify this pass physically."));
    }
  }
}

// Copied from X:TF_test.ino; callers pass degrees and RPM unchanged.
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF) {
  uint8_t cmd[32] = { 0 };
  uint16_t vel = 0;
  uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);
  pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0] = addr;                   // 地址
  cmd[1] = 0xFD;                   // 功能码
  cmd[2] = dir;                    // 符号（方向）
  cmd[3] = (uint8_t)(acc >> 8);    // 加速加速度(RPM/s)高8位字节
  cmd[4] = (uint8_t)(acc >> 0);    // 加速加速度(RPM/s)低8位字节
  cmd[5] = (uint8_t)(dec >> 8);    // 减速加速度(RPM/s)高8位字节
  cmd[6] = (uint8_t)(dec >> 0);    // 减速加速度(RPM/s)低8位字节
  cmd[7] = (uint8_t)(vel >> 8);    // 最大速度(RPM)高8位字节
  cmd[8] = (uint8_t)(vel >> 0);    // 最大速度(RPM)低8位字节
  cmd[9] = (uint8_t)(pos >> 24);   // 位置(bit24 - bit31)
  cmd[10] = (uint8_t)(pos >> 16);  // 位置(bit16 - bit23)
  cmd[11] = (uint8_t)(pos >> 8);   // 位置(bit8  - bit15)
  cmd[12] = (uint8_t)(pos >> 0);   // 位置(bit0  - bit7 )
  cmd[13] = raf;                   // 相位位置/绝对位置标志
  cmd[14] = snF;                   // 多机同步运动标志
  cmd[15] = 0x6B;                  // 校验字节

  // 发送命令
  Serial1.write(cmd, 16);
}

void sendAbsolute500(uint8_t id) {
  if (!clearReceiveBuffer()) haltTest(id, "Cannot prepare bus for absolute move");
  ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VELOCITY_RPM, TARGET_DEG, 1, 0);
  Serial1.flush();
  delay(50);
  unsigned long start = millis();
  while (millis() - start < STATE_TIMEOUT_MS) {
    long target;
    if (!readPosition(id, 0x33, target)) haltTest(id, "Cannot read target position");
    if (target == TARGET_TENTHS) return;
    delay(50);
  }
  haltTest(id, "Target readback is not 500 degrees");
}

long waitForPosition(uint8_t id, long expected, long tolerance, bool requireReached) {
  unsigned long start = millis();
  uint8_t settled = 0;
  while (millis() - start < MOVE_TIMEOUT_MS) {
    long actual;
    uint8_t status;
    if (!readPosition(id, 0x36, actual) || !readFlag(id, 0x3A, status)) {
      haltTest(id, "Cannot read position/status");
    }
    if (!(status & 0x01) || (status & 0x0C)) haltTest(id, "Motor disabled or stalled");
    bool reached = !requireReached || (status & 0x02);
    settled = reached && nearPosition(actual, expected, tolerance) ? settled + 1 : 0;
    if (settled >= 3) return actual;
    delay(100);
  }
  haltTest(id, "Position verification timeout");
  return 0;
}

void runTest() {
  for (uint8_t id = 1; id <= HOME_MOTOR_COUNT; ++id) {
    const uint8_t enable[] = {id, 0xF3, 0xAB, 1, 0, 0x6B};
    if (!sendAction(enable, sizeof(enable))) haltTest(id, "Cannot send enable command");
    waitForEnabled(id);
    waitForHoming(id);  // Allow any power-on homing to finish first.
  }
  collisionHomingPass(1);
  collisionHomingPass(2);

  Serial.println(F("Stage 1: motors 1-6 -> absolute 500 deg"));
  for (uint8_t id = 1; id <= TEST_MOTOR_COUNT; ++id) {
    sendAbsolute500(id);
    firstPosition[id - 1] = waitForPosition(id, TARGET_TENTHS, POSITION_TOLERANCE, true);
    printPosition(id, "first 500", firstPosition[id - 1]);
  }
  delay(OBSERVE_PAUSE_MS);

  Serial.println(F("Stage 2: clear current angle on motors 1-3 ONLY"));
  for (uint8_t id = 1; id <= TEST_MOTOR_COUNT; ++id) {
    if (id <= RESET_MOTOR_COUNT) {
      // Section 5.2.3: current angle reset, NOT single-turn origin setting.
      const uint8_t reset[] = {id, 0x0A, 0x6D, 0x6B};
      if (!sendAction(reset, sizeof(reset))) haltTest(id, "Cannot send angle reset");
    }
    long expected = id <= RESET_MOTOR_COUNT ? 0 : TARGET_TENTHS;
    long tolerance = id <= RESET_MOTOR_COUNT ? ZERO_TOLERANCE : POSITION_TOLERANCE;
    beforeSecondMove[id - 1] = waitForPosition(id, expected, tolerance, false);
    printPosition(id, "before second move", beforeSecondMove[id - 1]);
  }
  delay(OBSERVE_PAUSE_MS);

  Serial.println(F("Stage 3: motors 1-6 -> absolute 500 deg again"));
  for (uint8_t id = 1; id <= TEST_MOTOR_COUNT; ++id) {
    sendAbsolute500(id);
    finalPosition[id - 1] = waitForPosition(id, TARGET_TENTHS, POSITION_TOLERANCE, true);
    printPosition(id, "second 500", finalPosition[id - 1]);
  }

  Serial.println(F("Readback checks passed. Verify physical travel separately."));
  Serial.println(F("motor,reset,first_deg,before_second_deg,final_deg,second_delta_deg"));
  for (uint8_t i = 0; i < TEST_MOTOR_COUNT; ++i) {
    Serial.print(i + 1);
    Serial.print(',');
    Serial.print(i < RESET_MOTOR_COUNT ? 1 : 0);
    Serial.print(',');
    Serial.print(firstPosition[i] / 10.0f, 1);
    Serial.print(',');
    Serial.print(beforeSecondMove[i] / 10.0f, 1);
    Serial.print(',');
    Serial.print(finalPosition[i] / 10.0f, 1);
    Serial.print(',');
    Serial.println((finalPosition[i] - beforeSecondMove[i]) / 10.0f, 1);
  }
  Serial.println(F("Expected: 1-3 move about +500 deg again; 4-6 stay put."));
  Serial.println(F("DONE. Motors 7-8 remain at collision home. Reset board to rerun."));
}

void setup() {
  for (uint8_t pin : OUTPUT_PINS) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
  Serial1.begin(19200);
  delay(5000);
  Serial.println(F("CollisionZeroTest: motors 1-8 home twice, 1-6 compare angle reset."));
  Serial.println(F("Uses Response=None, checksum=0x6B, S_PosTDP=Disable. Queries verify state."));
  Serial.println(F("Send s to start once. Robot outputs remain LOW."));
}

void loop() {
  static bool started = false;
  if (!started && Serial.available() && Serial.read() == 's') {
    started = true;
    runTest();
  }
  delay(10);
}
