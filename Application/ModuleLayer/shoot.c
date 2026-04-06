/**
  ******************************************************************************
  * File Name          : shoot.c
  * Description        : 摩擦轮控制实现 — 上主控专用
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RM_Project.
  ******************************************************************************
*/

#include "shoot.h"
#include "rp_math.h"

/* Private function prototypes -----------------------------------------------*/
static void Fric_Extern_Update(fric_t *fric);                                    // 更新外部输入信息
static void Fric_Check_HighPriority(fric_t *fric);                              // 检查全局最高优先级并跳转
static void Fric_Check_Block(fric_t *fric);                                     // 检查堵转并更新计数
static void Fric_Speed_Pid(Motor_RM_t *motor, int16_t target_speed, fric_t *fric);  // 速度PID计算
static void Fric_Output_Local(fric_t *fric);                                   // 输出到电机

/* 私有函数前向声明 */
static void Fric_Init_impl(fric_t *fric);           // 初始化实现
static void Fric_Work_impl(fric_t *fric);           // 主处理函数实现
static void Fric_State_Stopping(fric_t *fric);      // 停止状态处理
static void Fric_State_Sleep(fric_t *fric);         // 睡眠状态处理
static void Fric_State_Run(fric_t *fric);          // 运行状态处理
static void Fric_State_Reverse(fric_t *fric);      // 反转状态处理

/* Private variables ---------------------------------------------------------*/
fric_t fric = {
    .work = Fric_Work_impl,
};  // 摩擦轮全局实例

/* Public functions --------------------------------------------------------*/
/**
 * @brief  摩擦轮初始化函数（绑定函数指针）
 * @param  fric: 摩擦轮句柄
 */
void Fric_Init(fric_t *fric)
{
    fric->init = Fric_Init_impl;
    fric->work = Fric_Work_impl;
}

/* Private member functions ---------------------------------------------------*/
/**
 * @brief  初始化实现
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Init_impl(fric_t *fric)
{
    // PID参数配置
    fric->cfg.pid.kp = 22.0f;
    fric->cfg.pid.ki = 0.35f;
    fric->cfg.pid.kd = 0.0f;
    fric->cfg.pid.integral_max = 7000.0f;
    fric->cfg.pid.out_max = 12000.0f;

    // 停止检测配置
    fric->cfg.stop.speed_threshold = 100.0f;    // 停转判定速度阈值
    fric->cfg.stop.confirm_ms = 1000;          // 停转确认持续时间

    // 堵转检测配置
    fric->cfg.block.current_threshold = 5000.0f;  // 堵转判定电流阈值
    fric->cfg.block.speed_threshold = 100.0f;     // 堵转判定速度阈值
    fric->cfg.block.confirm_ms = 400;             // 堵转确认持续时间

    // 反转参数配置
    fric->cfg.reverse.speed = -1000;    // 反转目标转速
    fric->cfg.reverse.duration_ms = 500;  // 反转持续时间

    // 方向配置
    fric->cfg.dir.L_direction = 1;   // 左摩擦轮正转
    fric->cfg.dir.R_direction = 0;   // 右摩擦轮反转（镜像放置）

    // 初始状态
    fric->state = FRIC_STATE_STOPPING;
    fric->state_enter_tick = 0;
    fric->block_confirm_tick = 0;
}

/**
 * @brief  主处理函数实现
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Work_impl(fric_t *fric)
{
    // 更新外部输入信息（解耦）
    Fric_Extern_Update(fric);

    // 全局最高优先级检查：任何状态下，rc_offline或目标速度为0则跳转停止
    Fric_Check_HighPriority(fric);

    // 根据状态执行对应动作
    switch (fric->state) {
        case FRIC_STATE_STOPPING:
            Fric_State_Stopping(fric);
            break;

        case FRIC_STATE_SLEEP:
            Fric_State_Sleep(fric);
            break;

        case FRIC_STATE_RUN:
            Fric_State_Run(fric);
            break;

        case FRIC_STATE_REVERSE:
            Fric_State_Reverse(fric);
            break;

        default:
            fric->state = FRIC_STATE_STOPPING;
            fric->state_enter_tick = HAL_GetTick();
            break;
    }

    // 输出到电机
    Fric_Output_Local(fric);
}

/**
 * @brief  停止状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Stopping(fric_t *fric)
{
    uint32_t now = HAL_GetTick();

    // 两个摩擦轮转速绝对值均小于阈值
    uint8_t L_near_stop = (my_abs(fric->info.L_speed) < fric->cfg.stop.speed_threshold);
    uint8_t R_near_stop = (my_abs(fric->info.R_speed) < fric->cfg.stop.speed_threshold);

    if (L_near_stop && R_near_stop) {
        // 速度已降至阈值，检查持续时间
        if ((now - fric->state_enter_tick) >= fric->cfg.stop.confirm_ms) {
            fric->state = FRIC_STATE_SLEEP;
            fric->state_enter_tick = now;
        }
    } else {
        // 速度还没降到阈值，重置进入时刻
        fric->state_enter_tick = now;
    }

    // PID控制到转速0
    Fric_Speed_Pid(&rm_motor[L_Fric], 0, fric);
    Fric_Speed_Pid(&rm_motor[R_Fric], 0, fric);
}

/**
 * @brief  睡眠状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Sleep(fric_t *fric)
{
    // 卸力（输出为0）
    rm_motor[L_Fric].tx_info->torque = 0;
    rm_motor[R_Fric].tx_info->torque = 0;

    // 目标速度不为0 且 rc_online=true，跳转至运行状态
    if ((my_abs(fric->info.target_speed) >= 1.0f) && (fric->info.rc_online == 1)) {
        fric->state = FRIC_STATE_RUN;
        fric->state_enter_tick = HAL_GetTick();
        fric->block_confirm_tick = 0;
    }
}

/**
 * @brief  运行状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Run(fric_t *fric)
{
    // 检查堵转
    Fric_Check_Block(fric);

    // 堵转确认超时，跳转至反转状态
    if (fric->block_confirm_tick >= fric->cfg.block.confirm_ms) {
        fric->state = FRIC_STATE_REVERSE;
        fric->state_enter_tick = HAL_GetTick();
    }

    // 考虑方向配置的目标速度
    int16_t L_target = fric->cfg.dir.L_direction ? (int16_t)fric->info.target_speed : (int16_t)(-fric->info.target_speed);
    int16_t R_target = fric->cfg.dir.R_direction ? (int16_t)fric->info.target_speed : (int16_t)(-fric->info.target_speed);

    Fric_Speed_Pid(&rm_motor[L_Fric], L_target, fric);
    Fric_Speed_Pid(&rm_motor[R_Fric], R_target, fric);
}

/**
 * @brief  反转状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Reverse(fric_t *fric)
{
    uint32_t now = HAL_GetTick();

    // 持续超过reverse_duration_ms，跳转回运行状态
    if ((now - fric->state_enter_tick) >= fric->cfg.reverse.duration_ms) {
        fric->state = FRIC_STATE_RUN;
        fric->state_enter_tick = now;
        fric->block_confirm_tick = 0;
    }

    // 反转目标转速
    int16_t L_target = fric->cfg.dir.L_direction ? fric->cfg.reverse.speed : -fric->cfg.reverse.speed;
    int16_t R_target = fric->cfg.dir.R_direction ? fric->cfg.reverse.speed : -fric->cfg.reverse.speed;

    Fric_Speed_Pid(&rm_motor[L_Fric], L_target, fric);
    Fric_Speed_Pid(&rm_motor[R_Fric], R_target, fric);
}

/**
 * @brief  更新外部输入信息（解耦用）
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Extern_Update(fric_t *fric)
{
    // 更新目标转速（来自下主控Board_Rx_Info.fric_target_speed）
    fric->info.target_speed = Board_Rx_Info.fric_target_speed;

    // 更新遥控器在线状态
    fric->info.rc_online = Board_Rx_Info.flag.bit.is_rc_online;

    // 更新左摩擦轮速度
    fric->info.L_speed = rm_motor[L_Fric].rx_info->encoder_speed;

    // 更新右摩擦轮速度
    fric->info.R_speed = rm_motor[R_Fric].rx_info->encoder_speed;

    // 更新左摩擦轮电流
    fric->info.L_current = (int16_t)rm_motor[L_Fric].rx_info->torque_current;

    // 更新右摩擦轮电流
    fric->info.R_current = (int16_t)rm_motor[R_Fric].rx_info->torque_current;
}

/**
 * @brief  检查全局最高优先级规则并跳转
 * @param  fric: 摩擦轮句柄
 * @note   任何状态下，rc_offline或目标速度为0则跳转至停止状态
 */
static void Fric_Check_HighPriority(fric_t *fric)
{
    // 遥控器不在线 或 目标速度为0
    uint8_t trigger = (fric->info.rc_online == 0) || (my_abs(fric->info.target_speed) < 1.0f);

    if (trigger && (fric->state != FRIC_STATE_STOPPING)) {
        fric->state = FRIC_STATE_STOPPING;
        fric->state_enter_tick = HAL_GetTick();
    }
}

/**
 * @brief  检查堵转并更新计数
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Check_Block(fric_t *fric)
{
    // 检查左摩擦轮：电流大于阈值 且 速度小于阈值
    uint8_t L_block = (my_abs(fric->info.L_current) > fric->cfg.block.current_threshold) &&
                      (my_abs(fric->info.L_speed) < fric->cfg.block.speed_threshold);

    // 检查右摩擦轮：电流大于阈值 且 速度小于阈值
    uint8_t R_block = (my_abs(fric->info.R_current) > fric->cfg.block.current_threshold) &&
                      (my_abs(fric->info.R_speed) < fric->cfg.block.speed_threshold);

    if (L_block || R_block) {
        fric->block_confirm_tick++;
    } else {
        fric->block_confirm_tick = 0;
    }
}

/**
 * @brief  摩擦轮速度PID计算
 * @param  motor: 摩擦轮电机句柄
 * @param  target_speed: 目标速度
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Speed_Pid(Motor_RM_t *motor, int16_t target_speed, fric_t *fric)
{
    pid_ctrl_t *speed_ctrl = motor->ctrl->speed_ctrl;

    // 设置目标值和测量值
    speed_ctrl->target = (float)target_speed;
    speed_ctrl->measure = motor->rx_info->encoder_speed;

    // 计算当前误差
    float err = speed_ctrl->target - speed_ctrl->measure;

    // 比例项
    float p_out = fric->cfg.pid.kp * err;

    // 积分项
    float i_out = fric->cfg.pid.ki * err;
    if (i_out > fric->cfg.pid.integral_max) {
        i_out = fric->cfg.pid.integral_max;
    } else if (i_out < -fric->cfg.pid.integral_max) {
        i_out = -fric->cfg.pid.integral_max;
    }

    // 微分项
    float d_out = fric->cfg.pid.kd * (err - speed_ctrl->last_err);

    // 总输出限幅
    float total_out = p_out + i_out + d_out;
    if (total_out > fric->cfg.pid.out_max) {
        total_out = fric->cfg.pid.out_max;
    } else if (total_out < -fric->cfg.pid.out_max) {
        total_out = -fric->cfg.pid.out_max;
    }

    // 保存误差
    speed_ctrl->last_err = err;

    // 输出扭矩
    motor->tx_info->torque = (int16_t)total_out;
}

/**
 * @brief  输出到电机（保存到输出结构体便于外部监控）
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Output_Local(fric_t *fric)
{
    fric->output.L_output = rm_motor[L_Fric].tx_info->torque;
    fric->output.R_output = rm_motor[R_Fric].tx_info->torque;
}