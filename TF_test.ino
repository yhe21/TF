// ====================== 公共宏 & 类型 ======================
#define ABS(x) ((x) > 0 ? (x) : -(x))

// 与 Arduino1 相同的系统参数枚举
typedef enum {
  S_VER = 0,    /* 读取固件版本和对应的硬件版本 */
  S_RL = 1,     /* 读取读取相电阻和相电感 */
  S_PID = 2,    /* 读取PID参数 */
  S_ORG = 3,    /* 读取回零参数 */
  S_VBUS = 4,   /* 读取总线电压 */
  S_CBUS = 5,   /* 读取总线电流 */
  S_CPHA = 6,   /* 读取相电流 */
  S_ENC = 7,    /* 读取编码器原始值 */
  S_CPUL = 8,   /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
  S_ENCL = 9,   /* 读取经过线性化校准后的编码器值 */
  S_TPUL = 10,  /* 读取输入脉冲数 */
  S_TPOS = 11,  /* 读取电机目标位置 */
  S_OPOS = 12,  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
  S_VEL = 13,   /* 读取电机实时转速 */
  S_CPOS = 14,  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
  S_PERR = 15,  /* 读取电机位置误差 */
  S_TEMP = 16,  /* 读取电机实时温度 */
  S_SFLAG = 17, /* 读取状态标志位 */
  S_OFLAG = 18, /* 读取回零状态标志位 */
  S_Conf = 19,  /* 读取驱动参数 */
  S_State = 20, /* 读取系统状态参数 */
} SysParams_t;

// ====================== ZDT_X42 函数原型 ======================
// ⚠️ 这里仅声明，具体实现直接复用你 Arduino1 文件中那一整段 ZDT_X42_* 函数

void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s);
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir,
                                      uint16_t acc, uint16_t dec,
                                      float velocity, float position,
                                      uint8_t raf, uint8_t snF);
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);

// 如需使能/回零等，也可以加上：
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF);
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// ...

// ====================== 通讯缓冲区 ======================
uint8_t g_rxCmd[128];
uint8_t g_rxCount = 0;

// ====================== 运动通用参数 ======================
const uint16_t ACC = 10000;
const uint16_t DECL = 6000;
const float    VEL  = 2000.0f;  // RPM（按你那套参数来）

// ====================== 电机与角度定义 ======================
// Motor 9：工装/治具旋转
// Motor 10：防漏挡板

constexpr uint8_t MOTOR_FIXTURE = 9;
constexpr uint8_t MOTOR_SHIELD  = 10;

// ⭐ 下面这些全部用「度」为单位，后面如果要改机械位置，改这里就行
const long POS_FIXTURE_OK_DEG     = 400;    // 工装 OK 位置
const long POS_FIXTURE_STAMP_DEG  = 2400;   // 冲压位置
const long POS_FIXTURE_GLUE1_DEG  = 3100;  // 胶位1
const long POS_FIXTURE_GLUE2_DEG  = 4100;  // 胶位2

const long POS_SHIELD_CLOSE_DEG   = 10;    // 挡板关闭（防漏）
const long POS_SHIELD_OPEN_DEG    = 90;   // 挡板打开（可出胶）

// ====================== IO 定义 ======================
// 输入（来自 T6 / 传感器）
constexpr int IN_STAMP_GLUE   = 44;  // Stamp + Glue 请求（高电平有效，来自机器人）
constexpr int IN_STAMP_ONLY   = 45;  // 仅 Stamp 请求（高电平有效，来自机器人）
constexpr int IN_LIMIT_SW   = 46;  // 回零limit switch
constexpr int IN_STAMP_SENSOR = 47;  // 冲压上死点传感器（传感器，高电平有效）
constexpr int IN_PURGE = 30;
// 输出（给 T6 / 电磁阀）
constexpr int OUT_FIXTURE_OK  = 48;  // Fixture at OK 信号
constexpr int OUT_GLUE_SOL    = 49;  // Glue Head 电磁阀
constexpr int OUT_STAMP_SOL   = 50;  // Stamping 电磁阀

// ====================== 时间参数（可调） ======================
const unsigned long ACK_TIMEOUT_MS      = 200;    // Ack 等待超时
const unsigned long REACHED_TIMEOUT_MS  = 3000;  // 到位等待超时
const unsigned long MOVE_RETRY_DELAY_MS = 20;     // 重发命令间隔

const unsigned long GLUE_ON_TIME_MS     = 200;    // 每次出胶时间
const unsigned long STAMP_MAX_TIME_MS   = 2000;   // 冲压最大等待时间
enum HomingStatus {
  HOMING_IN_PROGRESS,
  HOMING_FAILED,
  HOMING_SUCCESS,
  HOMING_INVALID
};
const char* fatalError(const char* msg) {
  Serial.println();
  Serial.println(F("========== FATAL ERROR =========="));
  Serial.println(msg);
  Serial.println(F("System halted."));
  Serial.println(F("================================="));

  // 如果你想用板载 LED 闪烁报警，可以在这里加：
  // pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(200);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(200);
  }

  // 理论上永远到不了这里，但保留 return 满足“返回字符串”的需求
  return msg;
}



HomingStatus checkHomingStatus(uint8_t id) {
  memset(g_rxCmd, 0, sizeof(g_rxCmd));
  g_rxCount = 0;

  // 读取回零状态标志 (S_OFLAG，对应功能码 0x3B)
  ZDT_X42_V2_Read_Sys_Params(id, S_OFLAG);
  ZDT_X42_V2_Receive_Data(g_rxCmd, &g_rxCount);

  if (g_rxCount != 4 || g_rxCmd[0] != id || g_rxCmd[1] != 0x3B || g_rxCmd[g_rxCount - 1] != 0x6B) {
    Serial.print("checkHomingStatus: 无效的回零状态返回帧, id=");
    Serial.println(id);
    return HOMING_INVALID;
  }

  uint8_t flag = g_rxCmd[2];

  Serial.print("Motor ");
  Serial.print(id);
  Serial.print(" Homing Flag = 0x");
  Serial.println(flag, HEX);

  bool Enc_Rdy = flag & 0x01;
  bool Cal_Rdy = flag & 0x02;
  bool Org_SF  = flag & 0x04;
  bool Org_CF  = flag & 0x08;
  bool Otp_TF  = flag & 0x10;
  bool Ocp_TF  = flag & 0x20;

  Serial.print("  Enc_Rdy=");
  Serial.print(Enc_Rdy);
  Serial.print(" Cal_Rdy=");
  Serial.print(Cal_Rdy);
  Serial.print(" Org_SF=");
  Serial.print(Org_SF);
  Serial.print(" Org_CF=");
  Serial.print(Org_CF);
  Serial.print(" Otp_TF=");
  Serial.print(Otp_TF);
  Serial.print(" Ocp_TF=");
  Serial.println(Ocp_TF);

  uint8_t org_state = flag & 0x0C;
  if (org_state == 0x04) {
    Serial.println("  Homing: 正在回零...");
    return HOMING_IN_PROGRESS;
  } else if (org_state == 0x08) {
    Serial.println("  Homing: 回零失败！");
    return HOMING_FAILED;
  } else if (org_state == 0x00) {
    Serial.println("  Homing: 回零成功！");
    return HOMING_SUCCESS;
  } else {
    Serial.println("  Homing: 未知状态");
    return HOMING_INVALID;
  }
}
// ====================== 检查 Ack / Reached（0.1° 风格） ======================

// 检查「目标位置」是否已正确设定 (S_TPOS / 功能码 0x33)
// 参数 expectedTargetDeg 使用「度」
bool checkAckDeg(uint8_t id, long expectedTargetDeg) {
  memset(g_rxCmd, 0, sizeof(g_rxCmd));
  g_rxCount = 0;

  // 读取目标位置 (功能码 0x33)
  ZDT_X42_V2_Read_Sys_Params(id, S_TPOS);
  ZDT_X42_V2_Receive_Data(g_rxCmd, &g_rxCount);

  // 基本帧校验
  if (g_rxCount != 8 || g_rxCmd[0] != id || g_rxCmd[1] != 0x33) {
    Serial.println("checkAckDeg: 无效返回帧");
    return false;
  }

  // 提取符号和原始位置（驱动内部为 0.1°）
  bool negative = (g_rxCmd[2] != 0);
  uint32_t posRaw =
    ((uint32_t)g_rxCmd[3] << 24) |
    ((uint32_t)g_rxCmd[4] << 16) |
    ((uint32_t)g_rxCmd[5] << 8)  |
    (uint32_t)g_rxCmd[6];

  // 和你原来的写法保持一致：0.1° → 度
  long curDeg = (long)(posRaw * 0.1f);
  if (negative) curDeg = -curDeg;

  long diff = curDeg - expectedTargetDeg;
  if (diff < 0) diff = -diff;

  //Serial.print("checkAckDeg: Motor ");
  //Serial.print(id);
  //Serial.print(" 目标角度设定 = ");
  //Serial.print(curDeg);
  //Serial.print(" deg (期望 ");
  //Serial.print(expectedTargetDeg);
  //Serial.print(") 差值=");
  //Serial.println(diff);

  // 容差 ±1°
  if (diff <= 1) {
    //Serial.println("checkAckDeg: 设定目标位置确认 OK");
    return true;
  }

  return false;
}

// 检查「实时位置」是否到达目标 (S_CPOS / 功能码 0x36)
bool checkReachedDeg(uint8_t id, long expectedTargetDeg) {
  memset(g_rxCmd, 0, sizeof(g_rxCmd));
  g_rxCount = 0;

  // 读取实时位置 (功能码 0x36)
  ZDT_X42_V2_Read_Sys_Params(id, S_CPOS);
  ZDT_X42_V2_Receive_Data(g_rxCmd, &g_rxCount);

  // 基本帧校验
  if (g_rxCount != 8 || g_rxCmd[0] != id || g_rxCmd[1] != 0x36) {
    Serial.println("checkReachedDeg: 无效返回帧");
    return false;
  }

  bool negative = (g_rxCmd[2] != 0);
  uint32_t posRaw =
    ((uint32_t)g_rxCmd[3] << 24) |
    ((uint32_t)g_rxCmd[4] << 16) |
    ((uint32_t)g_rxCmd[5] << 8)  |
    (uint32_t)g_rxCmd[6];

  long curDeg = (long)(posRaw * 0.1f);  // 0.1° → 度
  if (negative) curDeg = -curDeg;

  long diff = curDeg - expectedTargetDeg;
  if (diff < 0) diff = -diff;

  //Serial.print("checkReachedDeg: Motor ");
  //Serial.print(id);
  //Serial.print(" 当前角度 = ");
  //Serial.print(curDeg);
  //Serial.print(" deg (期望 ");
  //Serial.print(expectedTargetDeg);
  //Serial.print(") 差值=");
  //Serial.println(diff);

  if (diff <= 5) {
    //Serial.println("checkReachedDeg: 到位确认 OK");
    return true;
  }

  return false;
}

// ====================== 封装 moveMotorDeg ======================
// 发送一个目标角度（单位：度），自动循环：Ack 检查 + Reached 检查
// Ack 超时 200ms，Reached 超时 20000ms
bool moveMotorDeg(uint8_t id, long targetDeg) {
  //Serial.print("moveMotorDeg: Motor ");
  //Serial.print(id);
  //Serial.print(" 目标 ");
  //Serial.print(targetDeg);
  //Serial.println(" deg");

  unsigned long start = millis();

  // ---------- 阶段1：确认目标角度已经正确写入 (Ack) ----------
  while (millis() - start < ACK_TIMEOUT_MS) {
    // 发送位置命令（position 参数为“度”，底层再 *10）
    ZDT_X42_V2_Traj_Position_Control(
      id,
      0,           // dir: 0 = CW，暂用固定方向，如需双向自己扩展
      ACC,
      DECL,
      VEL,
      (float)targetDeg, // position: 单位“度”
      1,            // raf: 绝对位置
      0             // snF: 不使用多机同步
    );

    delay(MOVE_RETRY_DELAY_MS);  // 20ms 间隔再查 Ack

    if (checkAckDeg(id, targetDeg)) {
      // Ack OK，跳出阶段1
      break;
    }
  }

  if (millis() - start >= ACK_TIMEOUT_MS) {
    Serial.print("moveMotorDeg: Motor ");
    Serial.print(id);
    Serial.println(" Ack 超时！");
    return false;
  }

  // ---------- 阶段2：等待运动到位 ----------
  unsigned long startReached = millis();
  while (millis() - startReached < REACHED_TIMEOUT_MS) {
    if (checkReachedDeg(id, targetDeg)) {
      //Serial.print("moveMotorDeg: Motor ");
      //Serial.print(id);
      //Serial.println(" Reached OK");
      return true;
    }
    delay(MOVE_RETRY_DELAY_MS);  // MOVE_RETRY_DELAY_MSms 轮询一次实时位置
  }

  Serial.print("moveMotorDeg: Motor ");
  Serial.print(id);
  Serial.println(" Reached 超时！");
  return false;
}



// ====================== Homing 函数 ======================
// 流程：
// 1) 先确认当前回零都结束且成功
// 2) 触发一次多圈碰撞回零 (o_mode = 2)，等待结束并确认成功
// 3) 再触发一次单圈就近回零 (o_mode = 0)，等待结束并确认成功
// 4) 此时 limit switch (D46) 应该为 LOW
// 5) 将治具电机转到 OK 位置，检查 D46 应为 HIGH
void homing() {
  Serial.println("=== Arduino2 Homing 开始 ===");
  HomingStatus status;

  // ---------- 阶段 0：确认当前无回零进行中 ----------
  for (uint8_t id = MOTOR_FIXTURE; id <= MOTOR_FIXTURE; ++id) {
    do {
      status = checkHomingStatus(id);
      delay(300);
    } while (status == HOMING_IN_PROGRESS);

    if (status != HOMING_SUCCESS) {
      fatalError("Homing Phase0: 初始状态非 SUCCESS");
    }
  }

  // ---------- 阶段 1：触发多圈碰撞回零 (o_mode = 2) ----------
  Serial.println("Homing: 触发多圈碰撞回零 (o_mode=2)");
  ZDT_X42_V2_Origin_Trigger_Return(0, 2, 0);
  delay(50);

  for (uint8_t id = MOTOR_FIXTURE; id <= MOTOR_FIXTURE; ++id) {
    do {
      status = checkHomingStatus(id);
      delay(50);
    } while (status == HOMING_IN_PROGRESS);

    if (status != HOMING_SUCCESS) {
      fatalError("Homing Phase1: 碰撞回零失败");
    }
  }

  // ---------- 阶段 2：触发单圈就近回零 (o_mode = 0) ----------
  Serial.println("Homing: 触发单圈就近回零 (o_mode=0)");
  ZDT_X42_V2_Origin_Trigger_Return(0, 0, 0);
  delay(1000);



  // ---------- 阶段 3：limit switch 检查 ----------
  delay(200); // 给机械一点 settle 时间

  // 进行完就近回零后，limit switch 应该为 LOW
  int limState = digitalRead(IN_LIMIT_SW);
  Serial.print("Homing: 限位开关状态(就近回零后) = ");
  Serial.println(limState == LOW ? "LOW" : "HIGH");
  if (limState == LOW) {
    fatalError("Homing: 完成就近回零后，限位开关不是 LOW");
  }

  // 将治具电机转到 OK 位置
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_OK_DEG)) {
    fatalError("Homing: 移动到 OK 位置失败");
  }

  delay(2000);
  limState = digitalRead(IN_LIMIT_SW);
  Serial.print("Homing: 限位开关状态(OK 位置) = ");
  Serial.println(limState == LOW ? "LOW" : "HIGH");
  if (limState == HIGH) {
    fatalError("Homing: 在 OK 位置时，限位开关不是 HIGH");
  }

  Serial.println("=== Arduino2 Homing 完成 ===");
}
// ====================== 工艺流程函数 ======================

// 简单封装：让挡板开 / 关
bool shieldOpen() {
  return moveMotorDeg(MOTOR_SHIELD, POS_SHIELD_OPEN_DEG);
}
bool shieldClose() {
  return moveMotorDeg(MOTOR_SHIELD, POS_SHIELD_CLOSE_DEG);
}

// 胶一次（在当前位置附近），只管开关胶阀和时间
void doOneGlueShot() {
  digitalWrite(OUT_GLUE_SOL, HIGH);
  delay(GLUE_ON_TIME_MS);
  digitalWrite(OUT_GLUE_SOL, LOW);
}

// 一次完整的「Stamp + Glue」循环（阻塞式）
void runStampAndGlueCycle() {
  Serial.println("=== Cycle: Stamp + Glue 开始 ===");

  // 确保输出初始状态
  digitalWrite(OUT_GLUE_SOL, LOW);
  digitalWrite(OUT_STAMP_SOL, LOW);

  // 1) 确保治具先回到 OK
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_OK_DEG)) fatalError("Fixture not at OK");
  digitalWrite(OUT_FIXTURE_OK, LOW);

  // 2) 去 Stamp 位置
  if(digitalRead(IN_STAMP_SENSOR) == LOW) fatalError("stamp sensor not at up location");
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_STAMP_DEG)) fatalError("Fixture not at Stamp when stamp and glue");
  
  // 3) 冲压：开启冲压电磁阀，等到冲压传感器（高电平有效）或超时
  digitalWrite(OUT_STAMP_SOL, HIGH);
  {
    unsigned long t0 = millis();
    while (digitalRead(IN_STAMP_SENSOR) == LOW) {
      delay(10);
    }
    delay(400);t0 = millis();
    digitalWrite(OUT_STAMP_SOL, LOW);
    while (digitalRead(IN_STAMP_SENSOR) == LOW &&(millis() - t0 < STAMP_MAX_TIME_MS)) {
      delay(10);
    }
    if(millis() - t0 > STAMP_MAX_TIME_MS) fatalError("stamp sensor not recover in 2000ms from stroke");
  }
  

  // 4) 去 Glue1
  
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_GLUE1_DEG)) fatalError("Fixture cannot go to glue 1");

  // 打开挡板，开始出胶
  if (!shieldOpen()) fatalError("glue shield not open");
  doOneGlueShot();

  // 5) 去 Glue2
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_GLUE2_DEG)) fatalError("Fixture cannot go to glue 2");
  doOneGlueShot();

  // 6) 关闭挡板
  if (!shieldClose()) fatalError("glue shield not closed");
  // 7) 回到OK
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_OK_DEG)) fatalError("Cannot return to OK");
  digitalWrite(OUT_FIXTURE_OK, HIGH);

  Serial.println("=== Cycle: Stamp + Glue 完成 ===");
}

// 仅 Stamp 循环（不出胶，不动挡板）——简单版
void runStampOnlyCycle() {
  Serial.println("=== Cycle: Stamp Only 开始 ===");

  // 1) 确保治具先回到 OK
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_OK_DEG)) fatalError("Fixture not at OK");
  digitalWrite(OUT_FIXTURE_OK, LOW);

  // 2) 去 Stamp 位置
  if(digitalRead(IN_STAMP_SENSOR) == LOW) fatalError("stamp sensor not at up location");
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_STAMP_DEG)) fatalError("Fixture not at Stamp when stamp and glue");
  
  // 3) 冲压：开启冲压电磁阀，等到冲压传感器（高电平有效）或超时
  digitalWrite(OUT_STAMP_SOL, HIGH);
  {
    unsigned long t0 = millis();
    while (digitalRead(IN_STAMP_SENSOR) == HIGH) {
      delay(10);
    }
    delay(400);t0 = millis();
    digitalWrite(OUT_STAMP_SOL, LOW);
    while (digitalRead(IN_STAMP_SENSOR) == LOW &&(millis() - t0 < STAMP_MAX_TIME_MS)) {
      delay(10);
    }
    if(millis() - t0 > STAMP_MAX_TIME_MS) fatalError("stamp sensor not recover in 2000ms from stroke");
  }
  // 4) 回到OK
  if (!moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_OK_DEG)) fatalError("Cannot return to OK");
  digitalWrite(OUT_FIXTURE_OK, HIGH);

  Serial.println("=== Cycle: Stamp Only 完成 ===");
}

// 读取 S_SFLAG（状态标志位），判断电机是否在线 + 返回 flag
bool readStatusFlag(uint8_t id, uint8_t &flag) {
  uint8_t rxCmd[16] = {0};
  uint8_t rxCount = 0;

  // 发读取状态标志命令
  ZDT_X42_V2_Read_Sys_Params(id, S_SFLAG);  // S_SFLAG 在你的枚举里对应功能码 0x3A
  ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

  // 正常的返回帧应该是: [addr][0x3A][flag][0x6B]
  if (rxCount != 4 || rxCmd[0] != id || rxCmd[1] != 0x3A || rxCmd[rxCount - 1] != 0x6B) {
    // 没有收到有效帧，认为当前电机不在线
    return false;
  }

  flag = rxCmd[2];
  return true;
}

// 根据状态标志位判断该电机是否已使能
bool isMotorEnabledFromFlag(uint8_t flag) {
  // ⚠️ 下面的掩码需要根据 ZDT 手册里 S_SFLAG 的具体定义来调整
  // 这里先假设 bit0 = 1 表示电机使能（伺服 ON），你可以改成正确的 mask
  const uint8_t ENABLE_MASK = 0x01;

  return (flag & ENABLE_MASK) != 0;
}


void preHoming_EnableAndCollisionHome() {
  Serial.println("=== Pre-homing: enable all motors & run collision homing ===");

  for (uint8_t id = 9; id <= 10; ++id) {
    Serial.println();
    Serial.print(">> Motor ");
    Serial.print(id);
    Serial.println(" pre-homing start");

    // ========== 1. 等电机上线：循环读取 S_SFLAG ==========
    uint8_t flag = 0;

    while (true) {
      bool ok = readStatusFlag(id, flag);
      if (ok) {
        Serial.print("Motor ");
        Serial.print(id);
        Serial.println(" is online (S_SFLAG 有返回)");
        break;
      }
      Serial.print("Motor ");
      Serial.print(id);
      Serial.println(" offline or no response, retry after 200ms...");
      delay(200);
    }

    // ========== 2. 发送使能，并确认已经使能 ==========
    const int MAX_ENABLE_RETRY = 5;
    bool enabled = false;

    for (int retry = 0; retry < MAX_ENABLE_RETRY; ++retry) {
      Serial.print("Enabling motor ");
      Serial.print(id);
      Serial.print(" (retry ");
      Serial.print(retry);
      Serial.println(")");

      // 使能该电机：state = true，snF = 0（暂不做多机同步）
      ZDT_X42_V2_En_Control(id, true, 0);
      delay(50);  // 稍微等一下再读状态

      if (!readStatusFlag(id, flag)) {
        Serial.println("readStatusFlag after En_Control failed, will retry enable...");
        delay(100);
        continue;
      }

      if (isMotorEnabledFromFlag(flag)) {
        Serial.print("Motor ");
        Serial.print(id);
        Serial.println(" ENABLED OK (状态标志已变为使能)");
        enabled = true;
        break;
      } else {
        Serial.print("Motor ");
        Serial.print(id);
        Serial.println(" still not enabled, retry...");
        delay(100);
      }
    }

    if (!enabled) {
      // 如果你已经在这个 Arduino 文件里也有 fatalError，可以直接调用：
      // fatalError("Enable motor failed", id);
      Serial.print("FATAL: failed to enable motor ");
      Serial.println(id);
      while (true) {
        // 死循环报警，你也可以闪灯
        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
      }
    }

    // ========== 3. 对该电机做一次“无限位碰撞回零”(o_mode = 2) ==========
    Serial.print("Motor ");
    Serial.print(id);
    Serial.println(" start multi-turn collision homing (o_mode = 2)...");

    // o_mode = 2: 多圈无限位碰撞回零; snF = 0 不做多机同步
    ZDT_X42_V2_Origin_Trigger_Return(id, 2, 0);
    delay(50);

  }

  Serial.println("=== Pre-homing done for all motors ===");
}


// ====================== Arduino 标准入口 ======================
void setup() {
  delay(500);
  Serial.begin(115200);
  Serial1.begin(19200);  // 和你 Arduino1 一样的串口参数

  delay(1000);

  // 输入：机器人请求信号，高电平有效
  pinMode(IN_STAMP_GLUE,   INPUT_PULLUP);   // 注意：外部应保证高=请求，低=无
  pinMode(IN_STAMP_ONLY,   INPUT_PULLUP);
  // 传感器：低电平有效
  pinMode(IN_LIMIT_SW,   INPUT_PULLUP);
  pinMode(IN_PURGE, INPUT_PULLUP);
  pinMode(IN_STAMP_SENSOR, INPUT_PULLUP);

  // 输出：给机器人 / 电磁阀，高电平有效
  pinMode(OUT_FIXTURE_OK, OUTPUT);
  pinMode(OUT_GLUE_SOL,   OUTPUT);
  pinMode(OUT_STAMP_SOL,  OUTPUT);

  digitalWrite(OUT_FIXTURE_OK, LOW);
  digitalWrite(OUT_GLUE_SOL,   LOW);
  digitalWrite(OUT_STAMP_SOL,  LOW);

  Serial.println("Arduino2 初始化完成");

  preHoming_EnableAndCollisionHome();
  // 如需回零，可在此处调用你的 Origin_Trigger + homing 逻辑
  homing();
  shieldClose();
}

void loop() {
  // 简单轮询请求信号
  bool stampGlueReq = (digitalRead(IN_STAMP_GLUE) == HIGH);  // 来自机器人，高电平有效
  bool stampOnlyReq = (digitalRead(IN_STAMP_ONLY) == HIGH);
  bool purgeSwitch=(digitalRead(IN_PURGE)==LOW);//todo add name and wire
  if (stampGlueReq) {
    runStampAndGlueCycle();
    // 等待请求信号撤销，避免重复触发
    while (digitalRead(IN_STAMP_GLUE) == HIGH) {
      delay(10);
    }
  } else if (stampOnlyReq) {
    runStampOnlyCycle();
    while (digitalRead(IN_STAMP_ONLY) == HIGH) {
      delay(10);
    }
  } else if(purgeSwitch) {
    while(digitalRead(IN_PURGE)==LOW){
      if (!shieldOpen()) fatalError("glue shield not open when purge");
      delay(100);
    }
    if (!shieldClose()) fatalError("glue shield not open when purge");
    
  } else{
    if(moveMotorDeg(MOTOR_FIXTURE, POS_FIXTURE_OK_DEG)) digitalWrite(OUT_FIXTURE_OK, HIGH);
  }


  delay(10);
}

void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址
  cmd[1] = 0x0A;  // 功能码
  cmd[2] = 0x6D;  // 辅助码
  cmd[3] = 0x6B;  // 校验字节

  // 发送命令
  Serial1.write(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址
  cmd[1] = 0x0E;  // 功能码
  cmd[2] = 0x52;  // 辅助码
  cmd[3] = 0x6B;  // 校验字节

  // 发送命令
  Serial1.write(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址

  switch (s)  // 功能码
  {
    case S_VER: cmd[1] = 0x1F; break;   /* 读取固件版本和对应的硬件版本 */
    case S_RL: cmd[1] = 0x20; break;    /* 读取读取相电阻和相电感 */
    case S_PID: cmd[1] = 0x21; break;   /* 读取PID参数 */
    case S_ORG: cmd[1] = 0x22; break;   /* 读取回零参数 */
    case S_VBUS: cmd[1] = 0x24; break;  /* 读取总线电压 */
    case S_CBUS: cmd[1] = 0x26; break;  /* 读取总线电流 */
    case S_CPHA: cmd[1] = 0x27; break;  /* 读取相电流 */
    case S_ENC: cmd[1] = 0x29; break;   /* 读取编码器原始值 */
    case S_CPUL: cmd[1] = 0x30; break;  /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
    case S_ENCL: cmd[1] = 0x31; break;  /* 读取经过线性化校准后的编码器值 */
    case S_TPUL: cmd[1] = 0x32; break;  /* 读取输入脉冲数 */
    case S_TPOS: cmd[1] = 0x33; break;  /* 读取电机目标位置 */
    case S_OPOS: cmd[1] = 0x34; break;  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
    case S_VEL: cmd[1] = 0x35; break;   /* 读取电机实时转速 */
    case S_CPOS: cmd[1] = 0x36; break;  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
    case S_PERR: cmd[1] = 0x37; break;  /* 读取电机位置误差 */
    case S_TEMP: cmd[1] = 0x39; break;  /* 读取电机实时温度 */
    case S_SFLAG: cmd[1] = 0x3A; break; /* 读取状态标志位 */
    case S_OFLAG: cmd[1] = 0x3B; break; /* 读取回零状态标志位 */
    case S_Conf:
      cmd[1] = 0x42;
      cmd[2] = 0x6C;
      break; /* 读取驱动参数 */
    case S_State:
      cmd[1] = 0x43;
      cmd[2] = 0x7A;
      break; /* 读取系统状态参数 */
    default: break;
  }

  // 发送命令
  if (s >= S_Conf) {
    cmd[3] = 0x6B;
    Serial1.write(cmd, 4);
  } else {
    cmd[2] = 0x6B;
    Serial1.write(cmd, 3);
  }
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;       // 地址
  cmd[1] = 0x46;       // 功能码
  cmd[2] = 0x69;       // 辅助码
  cmd[3] = svF;        // 是否存储标志，false为不存储，true为存储
  cmd[4] = ctrl_mode;  // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
  cmd[5] = 0x6B;       // 校验字节

  // 发送命令
  Serial1.write(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;            // 地址
  cmd[1] = 0xF3;            // 功能码
  cmd[2] = 0xAB;            // 辅助码
  cmd[3] = (uint8_t)state;  // 使能状态
  cmd[4] = snF;             // 多机同步运动标志
  cmd[5] = 0x6B;            // 校验字节

  // 发送命令
  Serial1.write(cmd, 6);
}

/**
  * @brief    力矩模式
  * @param    addr  ：电机地址
  * @param    sign  ：符号         ，0为正，其余值为负
  * @param    t_ramp：斜率(Ma/s)   ，范围0 - 65535Ma/s
  * @param    torque：力矩(Ma)     ，范围0 - 4000Ma
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;                    // 地址
  cmd[1] = 0xF5;                    // 功能码
  cmd[2] = sign;                    // 符号（方向）
  cmd[3] = (uint8_t)(t_ramp >> 8);  // 力矩斜率(Ma/s)高8位字节
  cmd[4] = (uint8_t)(t_ramp >> 0);  // 力矩斜率(Ma/s)低8位字节
  cmd[5] = (uint8_t)(torque >> 8);  // 力矩(Ma)高8位字节
  cmd[6] = (uint8_t)(torque >> 0);  // 力矩(Ma)低8位字节
  cmd[7] = snF;                     // 多机同步运动标志
  cmd[8] = 0x6B;                    // 校验字节

  // 发送命令
  Serial1.write(cmd, 9);
}

/**
  * @brief    速度模式
  * @param    addr  ：电机地址
  * @param    dir     ：方向         ，0为CW，其余值为CCW
  * @param    v_ramp  ：斜率(RPM/s)  ，范围0 - 65535RPM/s
  * @param    velocity：速度(RPM)    ，范围0.0 - 4000.0RPM
  * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF) {
  uint8_t cmd[16] = { 0 };
  uint16_t vel = 0;

  // 将速度放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);

  // 装载命令
  cmd[0] = addr;                    // 地址
  cmd[1] = 0xF6;                    // 功能码
  cmd[2] = dir;                     // 符号（方向）
  cmd[3] = (uint8_t)(v_ramp >> 8);  // 速度斜率(RPM/s)高8位字节
  cmd[4] = (uint8_t)(v_ramp >> 0);  // 速度斜率(RPM/s)低8位字节
  cmd[5] = (uint8_t)(vel >> 8);     // 速度(RPM)高8位字节
  cmd[6] = (uint8_t)(vel >> 0);     // 速度(RPM)低8位字节
  cmd[7] = snF;                     // 多机同步运动标志
  cmd[8] = 0x6B;                    // 校验字节

  // 发送命令
  Serial1.write(cmd, 9);
}

/**
  * @brief    直通限速位置模式
  * @param    addr  ：电机地址
  * @param    dir     ：方向                   ，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)          ，范围0.0 - 4000.0RPM
  * @param    position：位置(°)                ，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志  ，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志           ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF) {
  uint8_t cmd[16] = { 0 };
  uint16_t vel = 0;
  uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);
  pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0] = addr;                  // 地址
  cmd[1] = 0xFB;                  // 功能码
  cmd[2] = dir;                   // 符号（方向）
  cmd[3] = (uint8_t)(vel >> 8);   // 最大速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0);   // 最大速度(RPM)低8位字节
  cmd[5] = (uint8_t)(pos >> 24);  // 位置(bit24 - bit31)
  cmd[6] = (uint8_t)(pos >> 16);  // 位置(bit16 - bit23)
  cmd[7] = (uint8_t)(pos >> 8);   // 位置(bit8  - bit15)
  cmd[8] = (uint8_t)(pos >> 0);   // 位置(bit0  - bit7 )
  cmd[9] = raf;                   // 相位位置/绝对位置标志
  cmd[10] = snF;                  // 多机同步运动标志
  cmd[11] = 0x6B;                 // 校验字节

  // 发送命令
  Serial1.write(cmd, 12);
}

/**
  * @brief    梯形曲线位置模式
  * @param    addr  ：电机地址
  * @param    dir     ：方向                   ，0为CW，其余值为CCW
  * @param    acc     ：加速加速度(RPM/s)     ，0为CW，其余值为CCW
  * @param    dec     ：减速加速度(RPM/s)     ，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)          ，范围0.0 - 4000.0RPM
  * @param    position：位置(°)                ，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志  ，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志           ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
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

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址
  cmd[1] = 0xFE;  // 功能码
  cmd[2] = 0x98;  // 辅助码
  cmd[3] = snF;   // 多机同步运动标志
  cmd[4] = 0x6B;  // 校验字节

  // 发送命令
  Serial1.write(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址
  cmd[1] = 0xFF;  // 功能码
  cmd[2] = 0x66;  // 辅助码
  cmd[3] = 0x6B;  // 校验字节

  // 发送命令
  Serial1.write(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址
  cmd[1] = 0x93;  // 功能码
  cmd[2] = 0x88;  // 辅助码
  cmd[3] = svF;   // 是否存储标志，false为不存储，true为存储
  cmd[4] = 0x6B;  // 校验字节

  // 发送命令
  Serial1.write(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF) {
  uint8_t cmd[32] = { 0 };

  // 装载命令
  cmd[0] = addr;                     // 地址
  cmd[1] = 0x4C;                     // 功能码
  cmd[2] = 0xAE;                     // 辅助码
  cmd[3] = svF;                      // 是否存储标志，false为不存储，true为存储
  cmd[4] = o_mode;                   // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] = o_dir;                    // 回零方向
  cmd[6] = (uint8_t)(o_vel >> 8);    // 回零速度(RPM)高8位字节
  cmd[7] = (uint8_t)(o_vel >> 0);    // 回零速度(RPM)低8位字节
  cmd[8] = (uint8_t)(o_tm >> 24);    // 回零超时时间(bit24 - bit31)
  cmd[9] = (uint8_t)(o_tm >> 16);    // 回零超时时间(bit16 - bit23)
  cmd[10] = (uint8_t)(o_tm >> 8);    // 回零超时时间(bit8  - bit15)
  cmd[11] = (uint8_t)(o_tm >> 0);    // 回零超时时间(bit0  - bit7 )
  cmd[12] = (uint8_t)(sl_vel >> 8);  // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] = (uint8_t)(sl_vel >> 0);  // 无限位碰撞回零检测转速(RPM)低8位字节
  cmd[14] = (uint8_t)(sl_ma >> 8);   // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] = (uint8_t)(sl_ma >> 0);   // 无限位碰撞回零检测电流(Ma)低8位字节
  cmd[16] = (uint8_t)(sl_ms >> 8);   // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] = (uint8_t)(sl_ms >> 0);   // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] = potF;                    // 上电自动触发回零，false为不使能，true为使能
  cmd[19] = 0x6B;                    // 校验字节

  // 发送命令
  Serial1.write(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;    // 地址
  cmd[1] = 0x9A;    // 功能码
  cmd[2] = o_mode;  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] = snF;     // 多机同步运动标志，false为不启用，true为启用
  cmd[4] = 0x6B;    // 校验字节

  // 发送命令
  Serial1.write(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr) {
  uint8_t cmd[16] = { 0 };

  // 装载命令
  cmd[0] = addr;  // 地址
  cmd[1] = 0x9C;  // 功能码
  cmd[2] = 0x48;  // 辅助码
  cmd[3] = 0x6B;  // 校验字节

  // 发送命令
  Serial1.write(cmd, 4);
}

/**
  * @brief    接收数据
  * @param    rxCmd   : 接收到的数据缓存在该数组
  * @param    rxCount : 接收到的数据长度
  * @retval   无
  */
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount) {
  int i = 0;
  unsigned long lTime;  // 上一时刻的时间
  unsigned long cTime;  // 当前时刻的时间

  // 记录当前的时间
  lTime = cTime = millis();

  // 开始接收数据
  while (1) {
    if (Serial1.available() > 0)  // 串口有数据进来
    {
      if (i <= 128)  // 防止数组溢出，该值需要小于数组的长度
      {
        rxCmd[i++] = Serial1.read();  // 接收数据

        lTime = millis();  // 更新上一时刻的时间
      }
    } else  // 串口有没有数据
    {
      cTime = millis();  // 获取当前时刻的时间

      if ((int)(cTime - lTime) > 20)  // 100毫秒内串口没有数据进来，就判定一帧数据接收结束
      {
        *rxCount = i;  // 数据长度

        break;  // 退出while(1)循环
      }
    }
  }
}