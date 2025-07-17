#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "ENCODER.h"
#include "Motor.h"
#include "math.h"

// --- 定义俯仰、翻滚、偏航角度变量 ---
float Pitch, Roll, Yaw;
float g_target_pitch = 0.0f; // 目标俯仰角度

// --- 内环PID控制器参数 ---
float Kp_bal = 20, Ki_bal = 0, Kd_bal = 5; // 恢复Kp_bal，赋予直立环力量

// --- 外环PID控制器参数 ---
float Kp_pos = 0.8, Ki_pos = 0, Kd_pos = 0.1; // 外环PID控制器的P、I、D参数

long g_position_error_sum = 0; // 当前位置误差的累积和

/**
  * @brief  内环PID控制器计算
  * @param  Angle 当前俯仰角度
  * @retval 返回速度调整值
  */
int Balance_PID_Calc(float Angle)
{
    static float Error_Last = 0, Error_Integral = 0;
    float Error;
    int Speed;

    Error = g_target_pitch - Angle; // 计算当前俯仰角度与目标俯仰角度的误差

    if(abs(Error) < 20) { Error_Integral += Error; } // 如果误差小于20，则累积误差
    else { Error_Integral = 0; } // 否则将误差累积和重置为0

    Speed = Kp_bal * Error + Ki_bal * Error_Integral + Kd_bal * (Error - Error_Last); // 计算速度调整值
    Error_Last = Error; // 更新上次误差

    return Speed;
}

/**
  * @brief  外环PID控制器计算
  * @param  Encoder_L 左侧编码器读数
  * @param  Encoder_R 右侧编码器读数
  * @retval 返回速度修正值
  */
int Position_PID_Calc(int Encoder_L, int Encoder_R)
{
    static long last_pos_error = 0;
    long current_pos = Encoder_L + Encoder_R;
    long pos_error = 0 - current_pos; // 计算当前位置误差

    g_position_error_sum = pos_error; // 记录当前误差用于显示

    // 位置误差的PD计算
    int speed_correction = Kp_pos * pos_error + Kd_pos * (pos_error - last_pos_error);
    last_pos_error = pos_error; // 更新上次位置误差

    return speed_correction;
}

int main(void)
{
    // 1. 初始化外设
    OLED_Init();
    MPU6050_Init();
    MPU6050_DMP_Init();
    Motor_Init();
    Encoder_Init();
    OLED_ShowNum(4,1,g_target_pitch,4);
    // 延时500ms等待MPU6050稳定
    Delay_ms(500);
    // 获取初始的俯仰角度
    MPU6050_DMP_Get_Data(&g_target_pitch, &Roll, &Yaw);

    // 2. 清除编码器计数
    Clear_Encoder_Count();

    int encoder_l, encoder_r;
    int speed_from_balance, speed_from_position, final_speed;

    while (1)
    {
        // 获取传感器数据
        MPU6050_DMP_Get_Data(&Pitch, &Roll, &Yaw);
        encoder_l = Read_Left_Encoder(); // 读取左侧编码器值
        encoder_r = Read_Right_Encoder(); // 读取右侧编码器值

        // PID控制器计算
        // 内环PID控制器计算速度
        speed_from_balance = Balance_PID_Calc(Pitch);

        // 外环PID控制器计算速度修正
        speed_from_position = Position_PID_Calc(encoder_l, encoder_r);

        // 合并速度调整和修正 (已校正符号)
        final_speed = -speed_from_balance + speed_from_position;

        // 速度限制
        if(final_speed > 1000) final_speed = 1000;
        if(final_speed < -1000) final_speed = -1000;

        // 设置电机速度
        MotorA_SetSpeed(final_speed);
        MotorB_SetSpeed(final_speed);

        // --- OLED显示 ---
        OLED_ShowString(1, 1, "Pitch:");
        OLED_ShowSignedNum(1, 8, (int)Pitch, 4);
        OLED_ShowString(2, 1, "Speed:");
        OLED_ShowSignedNum(2, 8, final_speed, 4);
        OLED_ShowString(3, 1, "PosE:"); // Position Error
        OLED_ShowSignedNum(3, 7, g_position_error_sum, 5);
        OLED_ShowString(4, 1, "Tgt:"); 
        OLED_ShowSignedNum(4, 5, (int)g_target_pitch, 4);
    }
}
