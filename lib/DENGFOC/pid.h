#ifndef PID_H
#define PID_H

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator() (float error);
    void reset();   //清零积分/输出（目标速度大幅变化时用，避免带着旧积分冲过头）

    float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_ramp; 
    float limit; 
protected:
    float error_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
    unsigned long timestamp_prev; //!< 上次执行时间戳
};

#endif
