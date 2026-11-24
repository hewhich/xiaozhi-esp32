#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>
#include <driver/ledc.h>

// ============================================================================
// 单板四轮差速机器狗  硬件引脚定义  (ESP32-S3)
// 说明:
//  - 底盘: 左右两组电机, 每组由一路 DRV8833 驱动 (DIR 方向 + PWM 调速)
//  - 腰部: 一路旋转电机 (DIR + PWM), 左右各一个限位开关
//  - 音频: I2S 外接 codec (ES8311 兼容), 若你的板子没有外部 codec,
//          可在 board.cc 中把 GetAudioCodec() 切换为 DummyAudioCodec。
// 板级修订: initial release (ESP32-S3 四轮差速机器狗)
// ============================================================================

// ------------------------- 音频 I2S ----------------------------------------
#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_GPIO_MCLK      GPIO_NUM_10
#define AUDIO_I2S_GPIO_WS        GPIO_NUM_11
#define AUDIO_I2S_GPIO_BCLK      GPIO_NUM_12
#define AUDIO_I2S_GPIO_DIN       GPIO_NUM_13
#define AUDIO_I2S_GPIO_DOUT      GPIO_NUM_14

// codec 控制总线 (ES8311 I2C) —— 用户未指定, 这里预留 S3 空闲 IO,
// 请按实际录音/放音模块核对; 若使用 NoAudioCodec 可忽略。
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_4
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_5
#define AUDIO_CODEC_PA_PIN       GPIO_NUM_NC
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR

// ------------------------- 底盘 DRV8833 差速 -------------------------------
// 左轮组 (左前 + 左后并联)
#define MOTOR_L_DIR              GPIO_NUM_15
#define MOTOR_L_PWM              GPIO_NUM_16
// 右轮组 (右前 + 右后并联)
#define MOTOR_R_DIR              GPIO_NUM_17
#define MOTOR_R_PWM              GPIO_NUM_18

// ------------------------- 腰部旋转电机 ------------------------------------
#define WAIST_DIR                GPIO_NUM_19
#define WAIST_PWM                GPIO_NUM_20
#define WAIST_LIMIT_L            GPIO_NUM_21   // 左限位 (低电平触发, 内部上拉)
#define WAIST_LIMIT_R            GPIO_NUM_22   // 右限位 (低电平触发, 内部上拉)

// ------------------------- 电机 / PWM 参数 --------------------------------
#define MOTOR_PWM_FREQ_HZ        20000          // 电机 PWM 频率 20kHz, 静音
#define MOTOR_PWM_DUTY_RES       LEDC_TIMER_13_BIT
#define MOTOR_PWM_MAX_DUTY       8191          // 2^13 - 1
#define MOTOR_DEFAULT_SPEED      60             // 默认速度 0-100 (%)

// 限位触发电平: 开关接 GND 时为低电平
#define LIMIT_ACTIVE_LEVEL       0

// 状态机节拍 (ms) —— 非阻塞, 由 esp_timer 周期回调驱动
#define DOG_TICK_PERIOD_MS       20

#endif // _BOARD_CONFIG_H_
