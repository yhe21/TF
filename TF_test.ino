#define ABS(x) ((x) > 0 ? (x) : -(x))

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


void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr);                                                                                                                                      // 将当前位置清零
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr);                                                                                                                                            // 解除堵转保护
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s);                                                                                                                            // 读取参数
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);                                                                                                             // 发送命令切换开环/闭环控制模式
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF);                                                                                                                       // 电机使能控制
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF);                                                                               // 力矩模式控制
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF);                                                                               // 速度模式控制
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF);                                                         // 直通限速位置模式控制
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF);                                  // 梯形曲线加减速位置模式控制
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF);                                                                                                                                     // 让电机立即停止运动
void ZDT_X42_V2_Synchronous_motion(uint8_t addr);                                                                                                                                        // 触发多机同步开始运动
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF);                                                                                                                                    // 设置单圈回零的零点位置
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);  // 修改回零参数
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);                                                                                                           // 发送命令触发回零
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr);                                                                                                                                          // 强制中断并退出回零
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);                                                                                                                          // 返回数据接收函数
// ======== 测试参数 ========
const uint8_t MOTOR_ID = 1;
const uint16_t ACC = 300;
const uint16_t DECL = 300;
const float VEL = 150.0f;  // 最大速度 (RPM)
const uint32_t POS_0 = 0;  // 0.0° → 0 (单位 0.1°)
const uint32_t POS_FIRST = 300;
const uint32_t POS_SECOND = 1500;
const uint32_t POS_4000 = 4000;  // 4000.0° → 4000 (单位 0.1°)
const uint32_t POS_HOMING = 78;
const uint32_t POS_HOMING_END = 6800;
const uint32_t POS_END = 6600;
const uint32_t POS_MID = 3800;

// ===== Button Pins =====
constexpr int BTN_1 = 2;
constexpr int BTN_2 = 3;
constexpr int BTN_3 = 4;
constexpr int BTN_4 = 5;
constexpr int BTN_5 = 6;
constexpr int BTN_6 = 7;
constexpr int BTN_7 = 8;
constexpr int BTN_8 = 9;

// ===== Sensor Pins =====
constexpr int SEN_1 = 10;
constexpr int SEN_2 = 11;
constexpr int SEN_3 = 12;


// Optional arrays
constexpr int BTN_PINS[8] = { BTN_1, BTN_2, BTN_3, BTN_4, BTN_5, BTN_6, BTN_7, BTN_8 };
constexpr int SEN_PINS[3] = { SEN_1, SEN_2, SEN_3 };

// ===== OUTPUT PINS =====
constexpr int OUT_OK = 22;       // OK 信号输出
constexpr int OUT_PLATE_0 = 23;  // plate bit0
constexpr int OUT_PLATE_1 = 24;  // plate bit1
constexpr int OUT_PLATE_2 = 25;  // plate bit2
constexpr int OUT_SECOND = 26;   // second place 输出
constexpr int OUT_EJECT = 27;    // ejecting 输出
constexpr int OUT_TRAY_1 = 35;
constexpr int OUT_TRAY_2 = 36;
// 也可以用数组管理 plate bit 输出
constexpr int PLATE_OUT_PINS[3] = { OUT_PLATE_0, OUT_PLATE_1, OUT_PLATE_2 };

// ===== INPUT PINS =====
constexpr int IN_PLATE_ECHO_0 = 28;  // plate echo bit0
constexpr int IN_PLATE_ECHO_1 = 29;  // plate echo bit1
constexpr int IN_PLATE_ECHO_2 = 30;  // plate echo bit2
constexpr int IN_SECOND_REQ = 31;    // second place 请求
constexpr int IN_EJECT_REQ = 32;     // eject 请求
constexpr int IN_TRAY_1_EJECT_REQ = 33;//nameplate tray eject 
constexpr int IN_TRAY_2_EJECT_REQ = 34;
// plate echo 输入数组
constexpr int PLATE_ECHO_PINS[3] = { IN_PLATE_ECHO_0, IN_PLATE_ECHO_1, IN_PLATE_ECHO_2 };

class Motor {
public:
  // ===== 状态定义 =====
  enum State {
    ST1_UNSENT,
    ST1_SENT,
    ST1_ACKED,
    ST1_REACHED,
    ST2_UNSENT,
    ST2_SENT,
    ST2_ACKED,
    ST2_REACHED,
    ST3_UNSENT,
    ST3_SENT,
    ST3_ACKED,
    ST3_REACHED
  };
  enum HomingStatus {
    HOMING_IN_PROGRESS,
    HOMING_FAILED,
    HOMING_SUCCESS,
    HOMING_INVALID
  };


  uint8_t id;  // 电机编号
  int btnPin;
  State state;              // 当前状态
  unsigned long lastCmdTm;  // 该电机上次命令时间（可选）
  bool acked = false;
  bool reached = false;
  uint8_t rxCmd[128];
  uint8_t rxCount;

  // 👉 所有电机共享的全局命令时间戳（节流控制）
  static unsigned long lastGlobalCmdTm;
  static uint8_t plate;


  // ===== 构造 =====
  Motor(uint8_t motorId)
    : id(motorId) {
    state = ST1_UNSENT;
    lastCmdTm = 0;
    btnPin = BTN_PINS[motorId - 1];
  }

  void test_run() {
    if (checkAck(id, 5000)) {
      Serial.println("ok");

    } else {
      Serial.println("not ok");
    }
  }
  HomingStatus checkHomingStatus() {

    memset(rxCmd, 0, sizeof(rxCmd));
    rxCount = 0;
    // 1️⃣ 发送读取回零状态标志命令
    ZDT_X42_V2_Read_Sys_Params(id, S_OFLAG);  // 功能码 0x3B
    ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);
    // 2️⃣ 基本校验
    if (rxCount != 4 || rxCmd[0] != id || rxCmd[1] != 0x3B || rxCmd[rxCount - 1] != 0x6B) {
      Serial.println("无效的回零状态返回帧");
      return HOMING_INVALID;
    }
    // 3️⃣ 提取状态标志位（第3个字节）
    uint8_t flag = rxCmd[2];
    // 可选调试打印
    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" Homing Flag = 0x");
    Serial.println(flag, HEX);

    // 4️⃣ 判断各标志
    bool Enc_Rdy = flag & 0x01;
    bool Cal_Rdy = flag & 0x02;
    bool Org_SF = flag & 0x04;
    bool Org_CF = flag & 0x08;
    bool Otp_TF = flag & 0x10;
    bool Ocp_TF = flag & 0x20;

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

    // 5️⃣ 回零状态判断
    uint8_t org_state = flag & 0x0C;
    if (org_state == 0x04) {
      Serial.println("正在回零...");
      return HOMING_IN_PROGRESS;
    } else if (org_state == 0x08) {
      Serial.println("回零失败！");
      return HOMING_FAILED;
    } else if (org_state == 0x00) {
      Serial.println("回零成功！");
      return HOMING_SUCCESS;
    } else {
      Serial.println("未知状态");
      return HOMING_INVALID;
    }
  }
  bool isSecondPlaceRequested() {
    // 假设高电平表示“请求 second place”
    if(digitalRead(IN_SECOND_REQ) == HIGH){
      digitalWrite(OUT_SECOND,HIGH);
      return true;
    }
    else{
      digitalWrite(OUT_SECOND,LOW);
      return false;
    }
    
  }
  bool checkAvailable() {
    if (state == ST2_REACHED) {
      return true;
    } else {
      return false;
    }
  }
  // ===== 主运行函数（状态机核心） =====
  void run(unsigned long now) {

    switch (state) {
      // ==================== 工位1 ====================
      case ST1_UNSENT:
        if (canSend(now)) {
          sendCommand(1, now);
          ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_END, 1, 0);
          state = ST1_SENT;
        }
        break;

      case ST1_SENT:
        if (checkAck(id, POS_END)) {
          state = ST1_ACKED;
        } else {
          state = ST2_UNSENT;
        }

        break;

      case ST1_ACKED:
        if (checkReached(id, POS_END)) state = ST1_REACHED;

        break;

      case ST1_REACHED:
        //delay(50);
        if (digitalRead(btnPin) == LOW) { state = ST2_UNSENT; }

        break;

      // ==================== 工位2 ====================
      case ST2_UNSENT:
        if (canSend(now)) {
          ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_MID, 1, 0);
          sendCommand(2, now);
          state = ST2_SENT;
        }
        break;

      case ST2_SENT:
        if (checkAck(id, POS_MID)) {
          state = ST2_ACKED;
        } else {
          state = ST2_UNSENT;
        }

        break;

      case ST2_ACKED:
        if (checkReached(id, POS_MID)) state = ST2_REACHED;

        break;

      case ST2_REACHED:
        //delay(50);
        if (id == plate) {
          state = ST3_UNSENT;
        } else if (digitalRead(btnPin) == LOW) {
          state = ST1_UNSENT;
        }

        break;

      // ==================== 工位3 ====================
      case ST3_UNSENT:
        {

          if (!canSend(now)) break;
          if (id != plate) break;
          bool second = isSecondPlaceRequested();
          uint32_t target = second ? POS_SECOND : POS_FIRST;
          st3Target = target;  // 记录这次工位3的目标
          ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, st3Target, 1, 0);
          sendCommand(3, now);
          state = ST3_SENT;
          break;
        }

      case ST3_SENT:
        if (checkAck(id, st3Target)) {
          state = ST3_ACKED;
        } else {
          state = ST2_UNSENT;
        }

        break;

      case ST3_ACKED:
        if (checkReached(id, st3Target)) state = ST3_REACHED;

        break;

      case ST3_REACHED:
        {
          //delay(50);
          //change output
          bool second = isSecondPlaceRequested();
          uint32_t expectedTarget = second ? POS_SECOND : POS_FIRST;
          if (expectedTarget != st3Target) {
            state = ST3_UNSENT;
            digitalWrite(OUT_OK, LOW);
            break;
          }
          //eject信号：
          if (digitalRead(IN_EJECT_REQ) == HIGH) {

            digitalWrite(OUT_EJECT, HIGH);
            digitalWrite(OUT_OK, LOW);
            while (digitalRead(IN_EJECT_REQ) == HIGH) { delay(1); }
            digitalWrite(OUT_EJECT, LOW);
            state = ST1_UNSENT;
          }
          //手动eject信号：
          else if (digitalRead(btnPin) == LOW) {
            int btnTmr = 0;
            while (digitalRead(btnPin) == LOW && btnTmr < 50) {
              btnTmr = btnTmr + 1;
              delay(5);
            }
            if (btnTmr >= 40) {
              digitalWrite(OUT_EJECT, HIGH);
              while (digitalRead(IN_EJECT_REQ) == LOW) { delay(1); }
              digitalWrite(OUT_EJECT, LOW);
              digitalWrite(OUT_OK, LOW);
              state = ST1_UNSENT;
            }
          }  // 循环回工位1
          //change output
          break;
        }
    }
    lastGlobalCmdTm = now;
    
  }
  void run2(unsigned long now) {
    uint32_t pos_end=4000;
    uint32_t pos_first=300;
    switch (state) {
      // ==================== 工位1 ====================
      case ST1_UNSENT:
        if (canSend(now)) {
          sendCommand(1, now);
          ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, pos_end, 1, 0);
          state = ST1_SENT;
        }
        break;

      case ST1_SENT:
        if (checkAck(id, pos_end)) {
          state = ST1_ACKED;
        } else {
          state = ST2_UNSENT;
        }

        break;

      case ST1_ACKED:
        if (checkReached(id, pos_end)) state = ST1_REACHED;

        break;

      case ST1_REACHED:
        //delay(50);
        if (digitalRead(btnPin) == LOW) { state = ST2_UNSENT; }

        break;

      // ==================== 工位2 ====================
      case ST2_UNSENT:
        if (canSend(now)) {
          ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, pos_first, 1, 0);
          sendCommand(2, now);
          state = ST2_SENT;
        }
        break;

      case ST2_SENT:
        if (checkAck(id, pos_first)) {
          state = ST2_ACKED;
        } else {
          state = ST2_UNSENT;
        }
        break;

      case ST2_ACKED:
        if (checkReached(id, pos_first)) state = ST2_REACHED;

        break;

      case ST2_REACHED:
        //delay(50);
        
        if (digitalRead(btnPin) == LOW) //triggered
        {
          state = ST1_UNSENT;
          if(id==7){digitalWrite(OUT_TRAY_1,LOW);break;}
          else{digitalWrite(OUT_TRAY_2,LOW);break;}
        }
        else if (digitalRead(IN_TRAY_1_EJECT_REQ)==HIGH && id==7){
          state = ST1_UNSENT;
          digitalWrite(OUT_TRAY_1,LOW);
          while(digitalRead(IN_TRAY_1_EJECT_REQ)==HIGH){delay(1);}
          break;
        }
        else if (digitalRead(IN_TRAY_2_EJECT_REQ)==HIGH && id==8){
          state = ST1_UNSENT;
          digitalWrite(OUT_TRAY_2,LOW);
          while(digitalRead(IN_TRAY_2_EJECT_REQ)==HIGH){delay(1);}
          break;
        }
        if(id==7){digitalWrite(OUT_TRAY_1,HIGH);break;}
        else{digitalWrite(OUT_TRAY_2,HIGH);break;}
        
    }
    lastGlobalCmdTm = now;
    
  }

private:
  uint32_t st3Target = POS_FIRST;
  // ===== 工具函数 =====
  bool canSend(unsigned long now) {
    // 限制：所有电机共享全局节流 ≥10ms
    return (now - lastGlobalCmdTm) >= 10;
  }

  void sendCommand(uint8_t station, unsigned long now) {
    lastCmdTm = now;
    lastGlobalCmdTm = now;  // 更新全局时间戳
    // === TODO: 替换为实际运动命令 ===
    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" -> send to STATION ");
    Serial.println(station);
  }

  bool checkAck(uint8_t id, long expectedTarget01deg) {
    memset(rxCmd, 0, sizeof(rxCmd));
    rxCount = 0;

    // 读取目标位置 (功能码 0x33)
    ZDT_X42_V2_Read_Sys_Params(id, S_TPOS);
    ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

    // 校验基本字段
    if (rxCount != 8 || rxCmd[0] != id || rxCmd[1] != 0x33) {
      Serial.println(" 无效返回帧");
      return false;
    }

    // 提取符号和位置
    bool negative = rxCmd[2];
    uint32_t pos = ((uint32_t)rxCmd[3] << 24) | ((uint32_t)rxCmd[4] << 16) | ((uint32_t)rxCmd[5] << 8) | (uint32_t)rxCmd[6];
    long cpos = (long)(pos * 0.1f);
    if (negative) cpos = -cpos;

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" 设定目标角度: ");
    Serial.println(cpos);

    // 判断是否设定目标（允许±5）
    if (abs(cpos - expectedTarget01deg) <= 5) {
      Serial.println("设定目标位置确认");
      return true;
    }

    return false;
  }

  bool checkReached(uint8_t id, long expectedTarget01deg) {
    memset(rxCmd, 0, sizeof(rxCmd));
    rxCount = 0;

    // 读取实时位置 (功能码 0x36)
    ZDT_X42_V2_Read_Sys_Params(id, S_CPOS);
    ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

    // 校验基本字段
    if (rxCount != 8 || rxCmd[0] != id || rxCmd[1] != 0x36) {
      Serial.println(" 无效返回帧");
      return false;
    }

    // 提取符号和位置
    bool negative = rxCmd[2];
    uint32_t pos = ((uint32_t)rxCmd[3] << 24) | ((uint32_t)rxCmd[4] << 16) | ((uint32_t)rxCmd[5] << 8) | (uint32_t)rxCmd[6];
    long cpos = (long)(pos * 0.1f);
    if (negative) cpos = -cpos;

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" 当前角度: ");
    Serial.println(cpos);

    // 判断是否到达目标（允许±5）
    if (abs(cpos - expectedTarget01deg) <= 5) {
      Serial.println(" 到位确认：当前位置与目标接近");
      return true;
    }

    return false;
  }
};

// ===== 类外定义全局静态变量（必须写这一行） =====
unsigned long Motor::lastGlobalCmdTm = 0;
uint8_t Motor::plate = 0;
Motor motors[8] = { Motor(1), Motor(2), Motor(3), Motor(4), Motor(5), Motor(6), Motor(7), Motor(8) };

void setup() {



  delay(500);
  Serial.begin(115200);
  Serial1.begin(19200);
  delay(5000);
  //   ===== INPUT PINS =====
  pinMode(IN_PLATE_ECHO_0, INPUT_PULLUP);
  pinMode(IN_PLATE_ECHO_1, INPUT_PULLUP);
  pinMode(IN_PLATE_ECHO_2, INPUT_PULLUP);
  pinMode(IN_SECOND_REQ, INPUT_PULLUP);
  pinMode(IN_EJECT_REQ, INPUT_PULLUP);
  pinMode(IN_TRAY_1_EJECT_REQ, INPUT_PULLUP);
  pinMode(IN_TRAY_2_EJECT_REQ, INPUT_PULLUP);

  // ===== OUTPUT PINS =====
  pinMode(OUT_OK, OUTPUT);
  pinMode(OUT_PLATE_0, OUTPUT);
  pinMode(OUT_PLATE_1, OUTPUT);
  pinMode(OUT_PLATE_2, OUTPUT);
  pinMode(OUT_SECOND, OUTPUT);
  pinMode(OUT_EJECT, OUTPUT);
  pinMode(OUT_TRAY_1, OUTPUT);
  pinMode(OUT_TRAY_2, OUTPUT);

  // ===== 初始化这些输出为低电平（根据你的实际需求可修改） =====
  digitalWrite(OUT_OK, LOW);
  digitalWrite(OUT_PLATE_0, LOW);
  digitalWrite(OUT_PLATE_1, LOW);
  digitalWrite(OUT_PLATE_2, LOW);
  digitalWrite(OUT_SECOND, LOW);
  digitalWrite(OUT_EJECT, LOW);

  // ===== BTN and SEN PINS =====
  pinMode(SEN_1, INPUT_PULLUP);
  pinMode(SEN_2, INPUT_PULLUP);
  pinMode(SEN_3, INPUT_PULLUP);

  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(BTN_5, INPUT_PULLUP);
  pinMode(BTN_6, INPUT_PULLUP);
  pinMode(BTN_7, INPUT_PULLUP);
  pinMode(BTN_8, INPUT_PULLUP);
  // 执行一次就近单圈回零（o_mode=2 单圈就近回零）
  //
  //ZDT_X42_V2_Origin_Trigger_Return(0, 0, 0);
  //waitUntilInPosition();  // 等待回零完成
  //delay(3000);
  //ZDT_X42_V2_Traj_Position_Control(1, 0, ACC, DECL, VEL, 500, 1, 0);
  homing();
  delay(10);
}

void loop() {
  updatePlateLogic();
  unsigned long now = millis();
  for (int i = 0; i < 6; i++) {
    now = millis();
    motors[i].run(now);
    delay(10);  // 轮询节拍
  }
  for (int i = 6; i < 8; i++) {
    now = millis();
    motors[i].run2(now);
    delay(10);  // 轮询节拍
  }

}
void homing() {
  Motor::HomingStatus status;
  for (uint8_t id = 1; id <= 8; ++id) {

    // 轮询直到回零结束（不再是“正在回零”）
    do {
      status = motors[id - 1].checkHomingStatus();
      delay(50);  // 稍微等一下，避免总线太频繁
    } while (status == Motor::HomingStatus::HOMING_IN_PROGRESS);

    // 一旦不是成功，就报错退出
    if (status != Motor::HomingStatus::HOMING_SUCCESS) {
      Serial.print("Homing failed on motor ");
      Serial.println(id);
      while (1) {}
    }
  }
  ZDT_X42_V2_Origin_Trigger_Return(0, 2, 0);
  delay(50);
  for (uint8_t id = 1; id <= 8; ++id) {

    // 轮询直到回零结束（不再是“正在回零”）
    do {
      status = motors[id - 1].checkHomingStatus();
      delay(50);  // 稍微等一下，避免总线太频繁
    } while (status == Motor::HomingStatus::HOMING_IN_PROGRESS);

    // 一旦不是成功，就报错退出
    if (status != Motor::HomingStatus::HOMING_SUCCESS) {
      Serial.print("Homing failed on motor ");
      Serial.println(id);
      while (1) {}
    }
  }

  ZDT_X42_V2_Origin_Trigger_Return(0, 0, 0);
  delay(2000);
  int trigger, sen_1, sen_2,sen_3;
  sen_1 = SEN_1;
  sen_2 = SEN_2;
  sen_3= SEN_3;
  for (uint8_t id = 1; id <= 8; ++id) {
    ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_FIRST, 1, 0);
    delay(200);
  }
  delay(2000);
  for (uint8_t id = 1; id <= 6; ++id) {
    trigger = digitalRead(sen_1);
    if (trigger == LOW) {
      Serial.print("Motor:");
      Serial.print(id);
      Serial.println("start to trigger sen_1");
      ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_HOMING, 1, 0);
      delay(1000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("triggered sen_1 before move");
      while (1) {}
    }
    trigger = digitalRead(sen_1);
    if (digitalRead(sen_1) == HIGH) {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("trigger sen_1 OK.");
      ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_FIRST, 1, 0);
      delay(1000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("Does not trigger sen_1");
      while (1) {}
    }
    trigger = digitalRead(sen_1);
    if (trigger == LOW) {
      Serial.print("Motor:");
      Serial.print(id);
      Serial.println("Recovered from sen_1,start to trigger sen_2");
      ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_HOMING_END, 1, 0);
      delay(8000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("Does not recover from sen_1");
      while (1) {}
    }
    if (digitalRead(sen_2) == HIGH) {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("trigger sen_2 OK.");
      ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_END, 1, 0);
      delay(1000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("Does not trigger sen_2");
      while (1) {}
    }
  }
  //trigger sen_3 for motor 7 and 8
  delay(1000);
  for (uint8_t id = 7; id <= 8; ++id) {
    
  trigger = digitalRead(sen_3);
    if (trigger == LOW) {
      Serial.print("Motor:");
      Serial.print(id);
      Serial.println("start to trigger sen_3");
      ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_HOMING, 1, 0);
      delay(1000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("triggered sen_3 before move");
      while (1) {}
    }
    trigger = digitalRead(sen_3);
    if (digitalRead(sen_3) == HIGH) {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("trigger sen_3 OK.");
      ZDT_X42_V2_Traj_Position_Control(id, 0, ACC, DECL, VEL, POS_FIRST, 1, 0);
      delay(1000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("Does not trigger sen_1");
      while (1) {}
    }
    trigger = digitalRead(sen_3);
    if (trigger == LOW) {
      Serial.print("Motor:");
      Serial.print(id);
      Serial.println("Recovered from sen_3");

      delay(1000);
    } else {
      Serial.print("Motor:");
      Serial.print("id");
      Serial.println("Does not recover from sen_1");
      while (1) {}
    }
  }

}
bool anyMotorInStation3() {
  for (int i = 0; i < 6; ++i) {
    if (motors[i].state == Motor::ST3_UNSENT || motors[i].state == Motor::ST3_SENT || motors[i].state == Motor::ST3_ACKED || motors[i].state == Motor::ST3_REACHED) {
      
      if(motors[i].state==Motor::ST3_REACHED&&checkPlateEchoMatch()){digitalWrite(OUT_OK,HIGH);}

      return true;
    }
  }
  return false;
}

bool motorIsReadyForPlate(uint8_t id) {
  Motor &m = motors[id - 1];
  // 定义哪些状态认为“有资格成为新的 plate”
  return (m.state == Motor::ST2_REACHED);  // 例如：在工位2等待中
}
bool checkPlateEchoMatch() {
  if (digitalRead(IN_PLATE_ECHO_0) != digitalRead(OUT_PLATE_0)) return false;
  if (digitalRead(IN_PLATE_ECHO_1) != digitalRead(OUT_PLATE_1)) return false;
  if (digitalRead(IN_PLATE_ECHO_2) != digitalRead(OUT_PLATE_2)) return false;
  return true;
}
void updatePlateLogic() {
  // 如果已有电机进入工位3流程，不允许切换 plate
  if (anyMotorInStation3()) return;

  uint8_t start = Motor::plate;  // 当前 plate
  uint8_t id = start % 6 + 1;    // 下一个电机（1~6 循环）

  for (int i = 0; i < 6; i++) {      // 最多查 6 次
    if (motorIsReadyForPlate(id)) {  // 如果这台电机已准备好
      Motor::plate = id;
      digitalWrite(OUT_PLATE_0, (Motor::plate & 0b001) ? HIGH : LOW);
      digitalWrite(OUT_PLATE_1, (Motor::plate & 0b010) ? HIGH : LOW);
      digitalWrite(OUT_PLATE_2, (Motor::plate & 0b100) ? HIGH : LOW);
      Serial.print("Plate switched to motor ");
      Serial.println(id);
      return;
    }

    // 下一个 id（循环 1~8）
    id = id % 6 + 1;
  }

  // 一圈都没找到 → 保持原样
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