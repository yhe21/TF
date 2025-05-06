/**********************************************************
*** ZDT_X42_V2.0步进闭环控制例程
*** 编写作者：ZHANGDATOU
*** 技术支持：张大头闭环伺服
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：http s://blog.csdn.net/zhangdatou666
*** qq交流群：262438510
**********************************************************/

// 注意：串口和USB下载口共用串口（0,1），USB上传程序时，先拔掉串口线.

#define    ABS(x)    ((x) > 0 ? (x) : -(x)) 

typedef enum {
  S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
  S_RL    = 1,      /* 读取读取相电阻和相电感 */
  S_PID   = 2,      /* 读取PID参数 */
  S_ORG   = 3,      /* 读取回零参数 */
  S_VBUS  = 4,      /* 读取总线电压 */
  S_CBUS  = 5,      /* 读取总线电流 */
  S_CPHA  = 6,      /* 读取相电流 */
  S_ENC   = 7,      /* 读取编码器原始值 */
  S_CPUL  = 8,      /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
  S_ENCL  = 9,      /* 读取经过线性化校准后的编码器值 */
  S_TPUL  = 10,     /* 读取输入脉冲数 */
  S_TPOS  = 11,     /* 读取电机目标位置 */
  S_OPOS  = 12,     /* 读取电机实时设定的目标位置（开环模式的实时位置） */
  S_VEL   = 13,     /* 读取电机实时转速 */
  S_CPOS  = 14,     /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
  S_PERR  = 15,     /* 读取电机位置误差 */
  S_TEMP  = 16,     /* 读取电机实时温度 */
  S_SFLAG = 17,     /* 读取状态标志位 */
  S_OFLAG = 18,     /* 读取回零状态标志位 */
  S_Conf  = 19,     /* 读取驱动参数 */
  S_State = 20,     /* 读取系统状态参数 */
}SysParams_t;

void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr); // 将当前位置清零
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr); // 解除堵转保护
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s); // 读取参数
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // 发送命令切换开环/闭环控制模式
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF); // 电机使能控制
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF); // 力矩模式控制
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF); // 速度模式控制
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF); // 直通限速位置模式控制
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF); // 梯形曲线加减速位置模式控制
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF); // 让电机立即停止运动
void ZDT_X42_V2_Synchronous_motion(uint8_t addr); // 触发多机同步开始运动
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF); // 设置单圈回零的零点位置
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // 发送命令触发回零
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr); // 强制中断并退出回零
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount); // 返回数据接收函数

void setup() {
  // put your setup code here, to run once:

  // 初始化LED灯
  pinMode(LED_BUILTIN, OUTPUT);

  // 初始化串口
  Serial.begin(115200);

  // 等待串口初始化完成
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // 上电延时2秒等待ZDT_X42_V2闭环初始化完毕
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // 定义接收数据数组、接收数据长度
  uint8_t rxCmd[128] = {0}; uint8_t rxCount = 0;
 
  /**********************************************************
  ***  梯形曲线位置模式：
    *   加速加速度   ：1000RPM/s
    *   减速加速度   ：1000RPM/s
    *   最大速度      ：2000RPM
    *   相对位置运动  ：-36000.0°
  **********************************************************/ 
  ZDT_X42_V2_Traj_Position_Control(1, 1, 1000, 1000, 2000.0f, 500.0f, 1, 0);
  ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);
  // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
  ZDT_X42_V2_Receive_Data(rxCmd, &rxCount);

  // 验证校验字节，验证成功则点亮LED灯，否则熄灭LED灯
  if(rxCmd[rxCount - 1] == 0x6B) { digitalWrite(LED_BUILTIN, HIGH); } else { digitalWrite(LED_BUILTIN, LOW); }

  // 调试使用，打印ZDT_X42_V2闭环返回的数据到串口
  // for(int i = 0; i < rxCount; i++) { Serial.write(rxCmd[i] + 1); } // 因为和USB下载口共用串口，所以让每个数据加1再发送出来，防止和电机地址冲突

  // 停止发送命令
  while(1);
}

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址

  switch(s)                             // 功能码
  {
    case S_VER   : cmd[1] = 0x1F; break;                  /* 读取固件版本和对应的硬件版本 */
    case S_RL    : cmd[1] = 0x20; break;                  /* 读取读取相电阻和相电感 */
    case S_PID   : cmd[1] = 0x21; break;                  /* 读取PID参数 */
    case S_ORG   : cmd[1] = 0x22; break;                  /* 读取回零参数 */
    case S_VBUS  : cmd[1] = 0x24; break;                  /* 读取总线电压 */
    case S_CBUS  : cmd[1] = 0x26; break;                  /* 读取总线电流 */
    case S_CPHA  : cmd[1] = 0x27; break;                  /* 读取相电流 */
    case S_ENC   : cmd[1] = 0x29; break;                  /* 读取编码器原始值 */
    case S_CPUL  : cmd[1] = 0x30; break;                  /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
    case S_ENCL  : cmd[1] = 0x31; break;                  /* 读取经过线性化校准后的编码器值 */
    case S_TPUL  : cmd[1] = 0x32; break;                  /* 读取输入脉冲数 */
    case S_TPOS  : cmd[1] = 0x33; break;                  /* 读取电机目标位置 */
    case S_OPOS  : cmd[1] = 0x34; break;                  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
    case S_VEL   : cmd[1] = 0x35; break;                  /* 读取电机实时转速 */
    case S_CPOS  : cmd[1] = 0x36; break;                  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
    case S_PERR  : cmd[1] = 0x37; break;                  /* 读取电机位置误差 */
    case S_TEMP  : cmd[1] = 0x39; break;                  /* 读取电机实时温度 */
    case S_SFLAG : cmd[1] = 0x3A; break;                  /* 读取状态标志位 */
    case S_OFLAG : cmd[1] = 0x3B; break;                  /* 读取回零状态标志位 */
    case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;   /* 读取驱动参数 */
    case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;   /* 读取系统状态参数 */
    default: break;
  }

  // 发送命令
  if(s >= S_Conf)
  {
    cmd[3] = 0x6B; Serial.write(cmd, 4);
  }
  else
  {
    cmd[2] = 0x6B; Serial.write(cmd, 3);
  }
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 6);
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
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF5;                       // 功能码
  cmd[2] =  sign;                       // 符号（方向）
  cmd[3] =  (uint8_t)(t_ramp >> 8);     // 力矩斜率(Ma/s)高8位字节
  cmd[4] =  (uint8_t)(t_ramp >> 0);     // 力矩斜率(Ma/s)低8位字节
  cmd[5] =  (uint8_t)(torque >> 8);     // 力矩(Ma)高8位字节
  cmd[6] =  (uint8_t)(torque >> 0);     // 力矩(Ma)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 9);
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
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0;

  // 将速度放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 符号（方向）
  cmd[3] =  (uint8_t)(v_ramp >> 8);     // 速度斜率(RPM/s)高8位字节
  cmd[4] =  (uint8_t)(v_ramp >> 0);     // 速度斜率(RPM/s)低8位字节
  cmd[5] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[6] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 9);
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
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFB;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[5]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[6]  =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[7]  =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[8]  =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[9]  =  raf;                       // 相位位置/绝对位置标志
  cmd[10] =  snF;                       // 多机同步运动标志
  cmd[11] =  0x6B;                      // 校验字节
  
  // 发送命令
  Serial.write(cmd, 12);
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
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(acc >> 8);       // 加速加速度(RPM/s)高8位字节
  cmd[4]  =  (uint8_t)(acc >> 0);       // 加速加速度(RPM/s)低8位字节  
  cmd[5]  =  (uint8_t)(dec >> 8);       // 减速加速度(RPM/s)高8位字节
  cmd[6]  =  (uint8_t)(dec >> 0);       // 减速加速度(RPM/s)低8位字节  
  cmd[7]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[8]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[9]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[10] =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[11] =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[12] =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[13] =  raf;                       // 相位位置/绝对位置标志
  cmd[14] =  snF;                       // 多机同步运动标志
  cmd[15] =  0x6B;                      // 校验字节
  
  // 发送命令
  Serial.write(cmd, 16);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 5);
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
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  Serial.write(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial.write(cmd, 4);
}

/**
  * @brief    接收数据
  * @param    rxCmd   : 接收到的数据缓存在该数组
  * @param    rxCount : 接收到的数据长度
  * @retval   无
  */
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
  int i = 0;
  unsigned long lTime;                    // 上一时刻的时间
  unsigned long cTime;                    // 当前时刻的时间

  // 记录当前的时间
  lTime = cTime = millis();

  // 开始接收数据
  while(1)
  {
    if(Serial.available() > 0)            // 串口有数据进来
    {
      if(i <= 128)                        // 防止数组溢出，该值需要小于数组的长度
      {
        rxCmd[i++] = Serial.read();       // 接收数据

        lTime = millis();                 // 更新上一时刻的时间
      }
    }
    else                                  // 串口有没有数据
    {
      cTime = millis();                   // 获取当前时刻的时间

      if((int)(cTime - lTime) > 100)      // 100毫秒内串口没有数据进来，就判定一帧数据接收结束
      {
        *rxCount = i;                     // 数据长度
        
        break;                            // 退出while(1)循环
      }
    }
  }
}
