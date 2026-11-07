#include <Arduino.h> 
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "InlineCurrent.h"   //引入在线电流检测
#include "DengFOC.h"

uint32_t SMO_last_time;
float Ealpha,Ebeta;
float Est_Ialpha, Est_Ibeta;
float Ealpha_flt,Ebeta_flt;
float SMO_Est_theta;
float Rs = 8.25;
float Ls = 0.004256;
float h = 0.3;
float SMO_Ualpha,SMO_Ubeta;

// SMO Vel
long SMO_angle_prev_ts;
float SMO_angle_prev;
int32_t SMO_full_rotations;
int32_t SMO_vel_full_rotations;
long SMO_vel_angle_prev_ts;
float SMO_vel_angle_prev;
float SMO_Vel;
float SMO_Theta;
long SMO_theta_prev;
LowPassFilter SMO_Vel_Flter = LowPassFilter(0.05);
float Ts;

//SMO 锁相环（PLL）：从Eα/Eβ中提取平滑的电角度和转速
float SMO_PLL_theta = 0.0f;
float SMO_PLL_speed = 0.0f;      //电角速度 rad/s
float SMO_PLL_integral = 0.0f;
bool  SMO_PLL_valid = false;


#define _sqrt(a) (_sqrtApprox(a))

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_power_supply;
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f
#define _2PI 6.28318530718f
//AS5600 I2C速率：400k；若现场线长/上拉较弱仍偶发错误，可降到200000或100000
#define AS5600_I2C_CLOCK 400000UL

float zero_electric_angle=0;
int PP=1,DIR=1;
int pwmA = 32;
int pwmB = 33;
int pwmC = 25;

//低通滤波初始化
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); // Tf = 10ms   //M0速度环
LowPassFilter M0_Curr_Flt = LowPassFilter(0.002); // Tf = 2ms   //M0电流环（与12_SMO_Observer一致）
//PID
PIDController vel_loop_M0 = PIDController{vel_loop_M0.P = 2, 
                                          vel_loop_M0.I = 0, 
                                          vel_loop_M0.D = 0, 
                                          vel_loop_M0. output_ramp= 100000, 
                                          vel_loop_M0.limit = voltage_power_supply/2};

PIDController angle_loop_M0 = PIDController{angle_loop_M0.P = 2,
                                          angle_loop_M0 .I = 0, 
                                          angle_loop_M0.D = 0, 
                                          angle_loop_M0.output_ramp = 100000, 
                                          angle_loop_M0.limit = 100};
PIDController current_loop_M0 = PIDController{current_loop_M0.P = 1.2,
                                          current_loop_M0.I = 0, 
                                          current_loop_M0.D = 0,
                                          current_loop_M0.output_ramp = 100000, 
                                          current_loop_M0.limit = 12.6};

//AS5600
Sensor_AS5600 S0=Sensor_AS5600(0);
TwoWire S0_I2C = TwoWire(0);

//根号定义
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f

//初始化电流闭环
CurrSense CS_M0= CurrSense(0);

//M0编码器对齐完成标志；未对齐或I2C故障时禁止闭环输出
bool DFOC_M0_aligned = false;

static inline bool M0_sensor_ok() {
  return DFOC_M0_aligned && S0.isReady();
}



//=================PID 设置函数=================
//速度PID
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp,float limit)   //M0角度环PID设置
{
  vel_loop_M0.P=P;
  vel_loop_M0.I=I;
  vel_loop_M0.D=D;
  vel_loop_M0.output_ramp=ramp;
  vel_loop_M0.limit=limit;
}
//角度PID
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp,float limit)   //M0角度环PID设置
{
  angle_loop_M0.P=P;
  angle_loop_M0.I=I;
  angle_loop_M0.D=D;
  angle_loop_M0.output_ramp=ramp;
  angle_loop_M0.limit=limit;
}
void DFOC_M0_SET_CURRENT_PID(float P,float I,float D,float ramp)    //M0电流环PID设置
{
  current_loop_M0.P=P;
  current_loop_M0.I=I;
  current_loop_M0.D=D;
  current_loop_M0.output_ramp=ramp;
  //电流环输出是Uq，限幅应与实际可输出电压一致，避免积分饱和
  current_loop_M0.limit = voltage_power_supply / 2;
}


//M0速度PID接口
float DFOC_M0_VEL_PID(float error)   //M0速度环
{
   return vel_loop_M0(error);
   
}
//M0角度PID接口
float DFOC_M0_ANGLE_PID(float error)
{
  return angle_loop_M0(error);
}


//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。1


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //写入PWM到PWM 0 1 2 通道
  ledcWrite(0, dc_a*255);
  ledcWrite(1, dc_b*255);
  ledcWrite(2, dc_c*255);
}

// // SPWM六步换向实现
void setTorque(float Uq,float angle_el) {
  Uq=_constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 
  //把实际施加的αβ电压送进SMO观测器（之前漏了这两行，观测器一直用U=0，
  //反电动势实际上变成了“电流的镜像”，会把速度环的低频摆动当成转速锁住）
  SMO_Ualpha = Ualpha;
  SMO_Ubeta  = Ubeta;

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}


// //SVPWM实现
// void setTorque(float Uq, float angle_el)
// {
//   if (Uq < 0)
//     angle_el += _PI;
//   Uq = abs(Uq);
// //angle_el 是 d-q 坐标系里 q 轴的角度，但 SVPWM 的 6 个扇区是按 α-β 坐标系排的。
// //帕克逆变换里 Uα=-Uq·sin, Uβ=Uq·cos 导致矢量比 angle_el 实际超前 90°，所以加 _PI_2 把坐标对齐，扇区才判对。
//   angle_el = _normalizeAngle(angle_el + _PI_2);
//   int sector = floor(angle_el / _PI_3) + 1;
//   // calculate the duty cycles
//   float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
//   float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
//   float T0 = 1 - T1 - T2;

//   float Ta, Tb, Tc;
//   switch (sector)
//   {
//   case 1:
//     Ta = T1 + T2 + T0 / 2;
//     Tb = T2 + T0 / 2;
//     Tc = T0 / 2;
//     break;
//   case 2:
//     Ta = T1 + T0 / 2;
//     Tb = T1 + T2 + T0 / 2;
//     Tc = T0 / 2;
//     break;
//   case 3:
//     Ta = T0 / 2;
//     Tb = T1 + T2 + T0 / 2;
//     Tc = T2 + T0 / 2;
//     break;
//   case 4:
//     Ta = T0 / 2;
//     Tb = T1 + T0 / 2;
//     Tc = T1 + T2 + T0 / 2;
//     break;
//   case 5:
//     Ta = T2 + T0 / 2;
//     Tb = T0 / 2;
//     Tc = T1 + T2 + T0 / 2;
//     break;
//   case 6:
//     Ta = T1 + T2 + T0 / 2;
//     Tb = T0 / 2;
//     Tc = T1 + T0 / 2;
//     break;
//   default:
//     Ta = 0;
//     Tb = 0;
//     Tc = 0;
//   }

//   float Ua = Ta * voltage_power_supply;
//   float Ub = Tb * voltage_power_supply;
//   float Uc = Tc * voltage_power_supply;

//    setPwm(Ua,Ub,Uc);
// }

void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;
  //SMO滑模增益必须大于电机最大反电动势，同时兼顾1~2ms的采样周期
  h = voltage_power_supply / 2.5f;
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  ledcSetup(0, 30000, 8);  //pwm频道, 频率, 精度
  ledcSetup(1, 30000, 8);  //pwm频道, 频率, 精度
  ledcSetup(2, 30000, 8);  //pwm频道, 频率, 精度
  ledcAttachPin(pwmA, 0);
  ledcAttachPin(pwmB, 1);
  ledcAttachPin(pwmC, 2);
  Serial.println("完成PWM初始化设置");

  //AS5600
  S0_I2C.begin(19,18, AS5600_I2C_CLOCK);
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
  Serial.println("编码器加载完毕");

  //PID 加载
  // vel_loop_M0 = PIDController{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = voltage_power_supply/2};

 vel_loop_M0 = PIDController{vel_loop_M0.P = 2, 
                             vel_loop_M0.I = 0, 
                             vel_loop_M0.D = 0, 
                             vel_loop_M0. output_ramp= 100000, 
                             vel_loop_M0.limit = voltage_power_supply/2};


  //初始化电流传感器
  CS_M0.init();
 }


float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * S0.getMechanicalAngle()-zero_electric_angle);
}


void DFOC_alignSensor(int _PP,int _DIR)
{ 
  PP=_PP;
  DIR=_DIR;
  DFOC_M0_aligned = false;

  if (!S0.isReady()) {
    Serial.println("M0 AS5600 I2C未就绪，跳过对齐，电机不会输出");
    setTorque(0, _3PI_2);
    return;
  }

  setTorque(3, _3PI_2);  //起劲
  delay(1000);

  if (!S0.Sensor_update() || !S0.isReady()) {  //更新角度，方便下面电角度读取
    Serial.println("M0 AS5600 对齐读取失败，跳过对齐，电机不会输出");
    setTorque(0, _3PI_2);
    return;
  }

  zero_electric_angle=_electricalAngle();
  setTorque(0, _3PI_2);  //松劲（解除校准）
  DFOC_M0_aligned = true;
  Serial.print("0电角度：");Serial.println(zero_electric_angle);
}

float DFOC_M0_Angle()
{
  return DIR*S0.getAngle();
}


//=========================电流读取=========================

//通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
float cal_Iq_Id(float current_a,float current_b,float angle_el)
{
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = cos(angle_el);
  float st = sin(angle_el);
  //float I_d = I_alpha * ct + I_beta * st;
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}
float DFOC_M0_Current()
{  
  float I_q_M0_ori=cal_Iq_Id(CS_M0.current_a,CS_M0.current_b,_electricalAngle());
  float I_q_M0_flit=M0_Curr_Flt(I_q_M0_ori);
  return I_q_M0_flit;  
}

//=========================电流读取=========================
//有滤波
float DFOC_M0_Velocity()
{
  //获取速度数据并滤波
  float vel_M0_ori=S0.getVelocity();
  float vel_M0_flit=M0_Vel_Flt(DIR*vel_M0_ori);
  return vel_M0_flit;   //考虑方向
}

//==============串口接收==============
float motor_target;
int commaPosition;
String serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {
      
      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');//检测字符串中的逗号
      if(commaPosition != -1)//如果有逗号存在就向下执行
      {
          String cmd = command.substring(0,commaPosition);
          cmd.trim();
          if (cmd.length() > 0) {
            motor_target = cmd.toDouble();            //电机角度/速度目标
          }
          // 不回显目标值，避免在VOFA的三列数据中插入额外数字
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}


float serial_motor_target()
{
  return motor_target;
}



//================简易接口函数================
void DFOC_M0_setTorque(float Target)            //电流力矩环
{
  if (!M0_sensor_ok()) { setTorque(0, _electricalAngle()); return; }
  setTorque(current_loop_M0(Target-DFOC_M0_Current()),_electricalAngle());
}

void DFOC_M0_set_Velocity_Angle(float Target)   //角度-速度-力 位置闭环
{
 //setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());        //改进前
 if (!M0_sensor_ok()) { setTorque(0, _electricalAngle()); return; }
 DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)-DFOC_M0_Velocity()));  //改进后
}

void DFOC_M0_setVelocity(float Target)          //速度闭环
{
  //setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //改进前
  if (!M0_sensor_ok()) { setTorque(0, _electricalAngle()); return; }
  DFOC_M0_setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI));                //改进后
}

void DFOC_M0_set_Force_Angle(float Target)      //力位闭环
{
  //setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());   //改进前
  if (!M0_sensor_ok()) { setTorque(0, _electricalAngle()); return; }
  DFOC_M0_setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI));                //改进后
}

float sat(float err, float limits) {
  if (err > limits) return 1;
  else if (err < -limits) return -1;
  else
    return err / limits;
}



//归一化锁相环：err≈sin(theta-true_theta)，PI输出电角速度，积分得到电角度
static float SMO_low_mag_time = 0.0f;   //反电动势幅值持续过低的计时
static float SMO_bad_phase_time = 0.0f; //PLL相位误差持续过大的计时

static void SMO_PLL_update(float Ts) {
  const float wn = 80.0f;       //PLL自然频率 rad/s
  const float zeta = 0.707f;    //阻尼比
  const float kp = 2.0f * zeta * wn;
  const float ki = wn * wn;

  float mag2 = Ealpha_flt * Ealpha_flt + Ebeta_flt * Ebeta_flt;
  float mag = sqrtf(mag2);

  //反电动势幅值短暂跌落（电流毛刺、过零点）时不要清零PLL：
  //保持当前速度继续积分角度，等E恢复后直接续上，避免“掉到0再慢慢爬”
  if (mag < 0.3f) {
    SMO_low_mag_time += Ts;
    if (SMO_low_mag_time > 0.15f) {     //真的没有反电动势了才缓慢归零
      SMO_PLL_speed *= 0.9f;
      SMO_PLL_integral = SMO_PLL_speed / ki;
    }
    SMO_PLL_theta = _normalizeAngle(SMO_PLL_theta + SMO_PLL_speed * Ts);
    SMO_Est_theta = SMO_PLL_theta;
    return;
  }
  SMO_low_mag_time = 0.0f;

  if (!SMO_PLL_valid) {         //首次进入或从低速恢复：用atan2初始化角度
    SMO_PLL_theta = _normalizeAngle(atan2(-Ealpha_flt, Ebeta_flt));
    SMO_PLL_speed = 0.0f;
    SMO_PLL_integral = 0.0f;
    SMO_PLL_valid = true;
  }

  float ct = cosf(SMO_PLL_theta);
  float st = sinf(SMO_PLL_theta);
  // 相位误差：-Eα*cosθ - Eβ*sinθ = |E|*sin(theta_true - theta_est)
  float err_raw = (-Ealpha_flt * ct - Ebeta_flt * st) / (mag + 1e-3f);

  //相位误差持续接近±1，说明已经失锁（例如E矢量被毛刺翻了一下），
  //直接用atan2重新对准，比让PI慢慢爬回来快得多
  if (fabs(err_raw) > 0.8f) {
    SMO_bad_phase_time += Ts;
    if (SMO_bad_phase_time > 0.03f) {
      SMO_PLL_theta = _normalizeAngle(atan2(-Ealpha_flt, Ebeta_flt));
      SMO_PLL_integral = SMO_PLL_speed / ki;
      SMO_bad_phase_time = 0.0f;
      err_raw = 0.0f;
    }
  } else {
    SMO_bad_phase_time = 0.0f;
  }

  // 限幅，避免个别异常反电动势采样把PLL相位误差拉得过大
  float err = err_raw;
  if (err > 0.5f) err = 0.5f;
  else if (err < -0.5f) err = -0.5f;

  float speed_new = kp * err + ki * SMO_PLL_integral;
  // 限制速度变化率，抑制PLL突跳/失锁；正常电机加速远小于此值
  const float max_delta = 20000.0f * Ts;
  if (speed_new > SMO_PLL_speed + max_delta) speed_new = SMO_PLL_speed + max_delta;
  else if (speed_new < SMO_PLL_speed - max_delta) speed_new = SMO_PLL_speed - max_delta;
  SMO_PLL_speed = speed_new;

  SMO_PLL_integral += err * Ts;
  if (SMO_PLL_integral > 1.0f) SMO_PLL_integral = 1.0f;
  else if (SMO_PLL_integral < -1.0f) SMO_PLL_integral = -1.0f;

  if (SMO_PLL_speed > 2000.0f) SMO_PLL_speed = 2000.0f;
  else if (SMO_PLL_speed < -2000.0f) SMO_PLL_speed = -2000.0f;

  SMO_PLL_theta = _normalizeAngle(SMO_PLL_theta + SMO_PLL_speed * Ts);
  SMO_Est_theta = SMO_PLL_theta;
}


void SMO_position_estimate(){
  uint32_t now_time = micros();
  Ts = (now_time - SMO_last_time) * 1e-6f;
  SMO_last_time = now_time;
  if (Ts <= 0) Ts = 1e-3f;

  //串口/VOFA卡顿会让这一拍的时间突然变长（几十ms）。
  //这种时候不能按1ms硬算：PLL角度只推进1ms，实际转子已经转了几十圈，必然失锁。
  //这里只按真实时间推进PLL角度并保持速度，跳过本拍观测器，等下一拍正常再继续。
  if (Ts > 5e-3f) {
    SMO_PLL_theta = _normalizeAngle(SMO_PLL_theta + SMO_PLL_speed * Ts);
    SMO_Est_theta = SMO_PLL_theta;
    return;
  }


  float Ialpha = CS_M0.current_a;
  float Ibeta = _1_SQRT3 * CS_M0.current_a + _2_SQRT3 * CS_M0.current_b;

  //后向欧拉离散：Ts较大（1ms以上）时仍稳定，显式欧拉在Rs/Ls*Ts>1时会发散
  float obs_k = 1.0f / (1.0f + Ts * Rs / Ls);
  Est_Ialpha = (Est_Ialpha + Ts / Ls * (SMO_Ualpha - Ealpha)) * obs_k;//Iα的估计值
  Est_Ibeta  = (Est_Ibeta  + Ts / Ls * (SMO_Ubeta  - Ebeta))  * obs_k;//Iβ的估计值

  float Ialpha_Err = Est_Ialpha - Ialpha;
  float Ibeta_Err = Est_Ibeta - Ibeta;

  Ealpha = h * sat(Ialpha_Err, 0.5f);
  Ebeta = h * sat(Ibeta_Err, 0.5f);
//简单滤波
  Ealpha_flt = 0.1 * Ealpha_flt + 0.9 * Ealpha;
  Ebeta_flt = 0.1 * Ebeta_flt + 0.9 * Ebeta;

  if (!isfinite(Ealpha_flt) || !isfinite(Ebeta_flt)) {
    Ealpha_flt = 0.0f;
    Ebeta_flt = 0.0f;
  }

  //用PLL从反电动势中提取平滑的角度/速度；低速反电动势不足时自动失效
  SMO_PLL_update(Ts);
}


void runFOC() {
  //====传感器更新====
  S0.Sensor_update();
  CS_M0.getPhaseCurrents();

  //====SMO观测器更新====
  SMO_position_estimate();
  SMOThetaUpdate();
  SMO_Vel = SMO_Vel_Flter(getSMOVel());
  if (!isfinite(SMO_Vel)) {
    SMO_Vel_Flter.reset();
    SMO_Vel = 0.0f;
  }
  getSMOTheta();
}




// SMO Vel test
//SMO_full_rotations滑膜观测的圈数
//SMO_vel_angle_prev 上一次速度计算时的角度

float getSMOVel(){
  //PLL锁定后直接输出PLL电角速度，避免对角度做差分放大噪声
  if (SMO_PLL_valid) {
    float vel = SMO_PLL_speed / PP;
    if (!isfinite(vel)) {
      SMO_PLL_valid = false;
      return 0.0f;
    }
    if (fabs(vel) < 2.0f) vel = 0.0f;   //静止残余死区
    //同步角度差分的备用状态，防止PLL失效切换时出现速度跳变
    SMO_vel_angle_prev = SMO_angle_prev;
    SMO_vel_full_rotations = SMO_full_rotations;
    SMO_vel_angle_prev_ts = SMO_angle_prev_ts;
    return vel;
  }

  //NaN保护：一旦上一次角度被污染，重置观测角度状态
  if (!isfinite(SMO_angle_prev) || !isfinite(SMO_vel_angle_prev)) {
    if (!isfinite(SMO_angle_prev)) SMO_angle_prev = 0.0f;
    SMO_vel_angle_prev = SMO_angle_prev;
    SMO_vel_full_rotations = SMO_full_rotations;
    SMO_vel_angle_prev_ts = SMO_angle_prev_ts;
    return 0.0f;
  }

  float Ts = (SMO_angle_prev_ts - SMO_vel_angle_prev_ts)*1e-6;
  if(Ts <= 0) Ts = 1e-3f;
  //离散得速度
  float vel = ( (float)(SMO_full_rotations - SMO_vel_full_rotations)*_2PI + (SMO_angle_prev - SMO_vel_angle_prev) ) / Ts; 
  //SMO观测的是完整0~2π电角度，除以PP得到机械角速度
  vel = vel / PP;
  if (!isfinite(vel)) vel = 0.0f;
  SMO_vel_angle_prev = SMO_angle_prev;
  SMO_vel_full_rotations = SMO_full_rotations;
  SMO_vel_angle_prev_ts = SMO_angle_prev_ts;
  return vel;
}

void getSMOTheta(){
  long currTime = micros();
  float ts = (currTime - SMO_theta_prev) *1e-6;
  if(ts <= 0) ts = 1e-3f;
  if (!isfinite(SMO_Vel)) SMO_Vel = 0.0f;
  SMO_Theta += ts * SMO_Vel;
  if (!isfinite(SMO_Theta)) SMO_Theta = 0.0f;
  SMO_Theta = _normalizeAngle(SMO_Theta);
  SMO_theta_prev = currTime;
}
//对SMO_position_estimate（）观测出来的电角度进行更新,计算圈数
void SMOThetaUpdate(){
  float val = SMO_Est_theta;  
  if (!isfinite(val)) val = 0.0f;
  if (!isfinite(SMO_angle_prev)) {
    SMO_angle_prev = 0.0f;
    SMO_full_rotations = 0;
  }
  SMO_angle_prev_ts = micros();
  float d_angle = val - SMO_angle_prev;
  if(abs(d_angle) > (0.8f*_2PI) ) SMO_full_rotations += ( d_angle > 0 ) ? -1 : 1; 
  SMO_angle_prev = val;
}
