// ============================================================================
// 单板四轮差速机器狗 (ESP32-S3)  板级实现
//  - WifiBoard 继承
//  - LEDC PWM 驱动左右轮组 + 腰部旋转电机
//  - 左右限位开关保护: 触限立刻切断腰部 PWM, 防止堵转
//  - MCP 工具: 前进/后退/左转/右转/停止/腰部左转/腰部右转/踏步走路/特技跳舞
//  - 组合动作为非阻塞 esp_timer 状态机, 全程不使用 vTaskDelay / delay
// ============================================================================

#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "mcp_server.h"
#include "application.h"
#include "config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/i2c_master.h>
#include <cstdlib>
#include <cstring>
#include <algorithm>

#define TAG "My4WdDog"

// ---------------------------------------------------------------------------
// 底盘/腰部动作阶段表 (非阻塞状态机)
// ---------------------------------------------------------------------------
struct ChassisPhase {
    int left;    // 左轮组速度 -100 ~ 100, 正=前进
    int right;   // 右轮组速度 -100 ~ 100
    int waist;   // -1 左, 0 停, +1 右
    int duration_ms;
};

// 踏步走路: 原地交替迈步 + 小步前进/收回, 防止跑飞
static const ChassisPhase kWalkPhases[] = {
    {  50,   0,  0, 220 },
    {   0,  50,  0, 220 },
    {  60,  60,  0, 260 },
    { -40, -40,  0, 180 },
};
static const int kWalkLoopCount = 4;

// 特技跳舞: 点动 + 原地转向 + 腰部摆动 + 左右摇摆
static const ChassisPhase kDancePhases[] = {
    {  50,  50,  0, 250 },
    { -50, -50,  0, 250 },
    { -60,  60,  0, 300 },
    {  60, -60,  0, 300 },
    {   0,   0, -1, 350 },
    {   0,   0, +1, 350 },
    {  50, -50,  0, 200 },
    { -50,  50,  0, 200 },
};
static const int kDanceLoopCount = 2;

enum class Motion {
    Idle,
    Walking,
    Dancing,
};

class My4WdDogBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_ = nullptr;

    // ---- 电机 / 状态机内部状态 ----
    Motion motion_ = Motion::Idle;
    const ChassisPhase* phase_table_ = nullptr;
    int phase_count_ = 0;
    int phase_index_ = 0;
    int loop_left_ = 0;
    int64_t phase_start_us_ = 0;

    int waist_dir_level_ = 0;   // 当前腰部方向: 1 正转, 0 反转, -1 未驱动
    bool waist_running_ = false;

    esp_timer_handle_t tick_timer_ = nullptr;

    // ------------------------------------------------------------------ I2C
    void InitializeI2c() {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_0;
        bus_cfg.sda_io_num = AUDIO_CODEC_I2C_SDA_PIN;
        bus_cfg.scl_io_num = AUDIO_CODEC_I2C_SCL_PIN;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &codec_i2c_bus_));
    }

    // ------------------------------------------------------- GPIO / LEDC 初始化
    void InitializeMotorPins() {
        // 方向引脚: 推挽输出
        gpio_config_t dir_cfg = {};
        dir_cfg.pin_bit_mask = (1ULL << MOTOR_L_DIR) | (1ULL << MOTOR_R_DIR) |
                               (1ULL << WAIST_DIR);
        dir_cfg.mode = GPIO_MODE_OUTPUT;
        dir_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        dir_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        dir_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&dir_cfg));

        // 限位引脚: 输入 + 内部上拉 (开关触发接 GND)
        gpio_config_t limit_cfg = {};
        limit_cfg.pin_bit_mask = (1ULL << WAIST_LIMIT_L) | (1ULL << WAIST_LIMIT_R);
        limit_cfg.mode = GPIO_MODE_INPUT;
        limit_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        limit_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        limit_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&limit_cfg));

        // LEDC 定时器: 三路 PWM 共用
        ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution = MOTOR_PWM_DUTY_RES;
        timer_cfg.timer_num = LEDC_TIMER_0;
        timer_cfg.freq_hz = MOTOR_PWM_FREQ_HZ;
        timer_cfg.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        ledc_channel_config_t ch_cfg = {};
        ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_cfg.timer_sel = LEDC_TIMER_0;
        ch_cfg.intr_type = LEDC_INTR_DISABLE;
        ch_cfg.duty = 0;
        ch_cfg.hpoint = 0;

        ch_cfg.channel = LEDC_CHANNEL_0;
        ch_cfg.gpio_num = MOTOR_L_PWM;
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        ch_cfg.channel = LEDC_CHANNEL_1;
        ch_cfg.gpio_num = MOTOR_R_PWM;
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        ch_cfg.channel = LEDC_CHANNEL_2;
        ch_cfg.gpio_num = WAIST_PWM;
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    // ------------------------------------------------------- 底层电机输出
    static int SpeedToDuty(int speed_percent) {
        int s = std::max(0, std::min(100, speed_percent));
        return s * MOTOR_PWM_MAX_DUTY / 100;
    }

    // 单路电机: dir_gpio / pwm_channel, speed 正负代表方向
    void DriveMotor(gpio_num_t dir_gpio, ledc_channel_t channel, int speed) {
        if (speed > 0) {
            gpio_set_level(dir_gpio, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, SpeedToDuty(speed));
        } else if (speed < 0) {
            gpio_set_level(dir_gpio, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, SpeedToDuty(-speed));
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }

    void SetChassis(int left, int right) {
        DriveMotor(MOTOR_L_DIR, LEDC_CHANNEL_0, left);
        DriveMotor(MOTOR_R_DIR, LEDC_CHANNEL_1, right);
    }

    void StopChassis() {
        SetChassis(0, 0);
    }

    void DriveWaist(int direction, int speed) {
        // direction: -1 左, +1 右
        if (direction == 0) {
            WaistStop();
            return;
        }
        waist_running_ = true;
        waist_dir_level_ = (direction > 0) ? 1 : 0;
        gpio_set_level(WAIST_DIR, waist_dir_level_);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, SpeedToDuty(speed));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }

    void WaistStop() {
        waist_running_ = false;
        waist_dir_level_ = -1;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }

    // ------------------------------------------------------- 限位保护
    bool LimitLeftPressed() {
        return gpio_get_level(WAIST_LIMIT_L) == LIMIT_ACTIVE_LEVEL;
    }
    bool LimitRightPressed() {
        return gpio_get_level(WAIST_LIMIT_R) == LIMIT_ACTIVE_LEVEL;
    }

    void CheckWaistLimit() {
        if (!waist_running_) return;
        // 当前腰部正转(dir=1)对应右向; 反转(dir=0)对应左向
        bool hit_left = (waist_dir_level_ == 0) && LimitLeftPressed();
        bool hit_right = (waist_dir_level_ == 1) && LimitRightPressed();
        if (hit_left || hit_right) {
            ESP_LOGW(TAG, "Waist limit hit (L=%d R=%d), cut PWM",
                     LimitLeftPressed(), LimitRightPressed());
            WaistStop();
        }
    }

    // ------------------------------------------------------- 状态机节拍
    void ApplyPhase(const ChassisPhase& p) {
        SetChassis(p.left, p.right);
        if (p.waist == 0) {
            if (waist_running_) WaistStop();
        } else {
            DriveWaist(p.waist, MOTOR_DEFAULT_SPEED);
        }
    }

    void Tick() {
        CheckWaistLimit();  // 限位保护优先, 每拍都查

        if (motion_ == Motion::Idle || phase_table_ == nullptr) return;

        int64_t now_us = esp_timer_get_time();
        if (now_us - phase_start_us_ >= phase_table_[phase_index_].duration_ms * 1000LL) {
            phase_index_++;
            if (phase_index_ >= phase_count_) {
                phase_index_ = 0;
                loop_left_--;
                if (loop_left_ <= 0) {
                    // 动作结束, 回Idle并停车
                    motion_ = Motion::Idle;
                    phase_table_ = nullptr;
                    StopChassis();
                    WaistStop();
                    ESP_LOGI(TAG, "Motion finished.");
                    return;
                }
            }
            phase_start_us_ = now_us;
        }
        ApplyPhase(phase_table_[phase_index_]);
    }

    static void OnTickTimer(void* arg) {
        static_cast<My4WdDogBoard*>(arg)->Tick();
    }

    void StartMotion(Motion motion, const ChassisPhase* table, int count, int loops) {
        motion_ = motion;
        phase_table_ = table;
        phase_count_ = count;
        phase_index_ = 0;
        loop_left_ = loops;
        phase_start_us_ = esp_timer_get_time();
        ApplyPhase(table[0]);
        ESP_LOGI(TAG, "Start motion: %s", motion == Motion::Walking ? "walk" : "dance");
    }

    // ------------------------------------------------------- MCP 工具
    int GetSpeed(const PropertyList& properties, int def) {
        // 遍历参数列表取 speed; 取不到或类型不符时回退默认值
        for (const auto& prop : properties) {
            if (prop.name() == "speed") {
                try {
                    return prop.value<int>();
                } catch (...) {
                    return def;
                }
            }
        }
        return def;
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.dog.forward", "机器狗向前直行",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 0, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                StopMotionIfActive();
                int s = GetSpeed(p, MOTOR_DEFAULT_SPEED);
                SetChassis(s, s);
                return true;
            });

        mcp.AddTool("self.dog.backward", "机器狗向后直行",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 0, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                StopMotionIfActive();
                int s = GetSpeed(p, MOTOR_DEFAULT_SPEED);
                SetChassis(-s, -s);
                return true;
            });

        mcp.AddTool("self.dog.turn_left", "机器狗原地左转 (差速转向)",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 0, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                StopMotionIfActive();
                int s = GetSpeed(p, MOTOR_DEFAULT_SPEED);
                SetChassis(-s, s);
                return true;
            });

        mcp.AddTool("self.dog.turn_right", "机器狗原地右转 (差速转向)",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 0, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                StopMotionIfActive();
                int s = GetSpeed(p, MOTOR_DEFAULT_SPEED);
                SetChassis(s, -s);
                return true;
            });

        mcp.AddTool("self.dog.stop", "机器狗底盘与腰部全部停止",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StopMotionIfActive();
                StopChassis();
                WaistStop();
                return true;
            });

        mcp.AddTool("self.dog.waist_left", "腰部向左旋转, 碰到左限位自动停",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 0, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                int s = GetSpeed(p, MOTOR_DEFAULT_SPEED);
                DriveWaist(-1, s);
                return true;
            });

        mcp.AddTool("self.dog.waist_right", "腰部向右旋转, 碰到右限位自动停",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 0, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                int s = GetSpeed(p, MOTOR_DEFAULT_SPEED);
                DriveWaist(+1, s);
                return true;
            });

        mcp.AddTool("self.dog.walk", "启动非阻塞踏步走路组合动作 (状态机自动走完)",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartMotion(Motion::Walking, kWalkPhases,
                            sizeof(kWalkPhases) / sizeof(kWalkPhases[0]),
                            kWalkLoopCount);
                return true;
            });

        mcp.AddTool("self.dog.dance", "启动非阻塞特技跳舞组合动作 (状态机自动走完)",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartMotion(Motion::Dancing, kDancePhases,
                            sizeof(kDancePhases) / sizeof(kDancePhases[0]),
                            kDanceLoopCount);
                return true;
            });
    }

    void StopMotionIfActive() {
        if (motion_ != Motion::Idle) {
            motion_ = Motion::Idle;
            phase_table_ = nullptr;
        }
    }

public:
    My4WdDogBoard() {
        InitializeI2c();
        InitializeMotorPins();
        InitializeTools();

        // 周期节拍定时器: 驱动非阻塞状态机 + 限位巡检
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = OnTickTimer;
        timer_args.arg = this;
        timer_args.name = "dog_tick";
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &tick_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer_, DOG_TICK_PERIOD_MS * 1000ULL));

        ESP_LOGI(TAG, "My4WdDogBoard init done.");
    }

    ~My4WdDogBoard() {
        if (tick_timer_) {
            esp_timer_stop(tick_timer_);
            esp_timer_delete(tick_timer_);
        }
    }

    AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            codec_i2c_bus_,
            I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }
};

DECLARE_BOARD(My4WdDogBoard);
