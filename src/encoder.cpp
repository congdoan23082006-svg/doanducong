#include "encoder.h"
#include "config.h"
#include <Arduino.h>
#include <driver/pcnt.h>

#define VELOCITY_FILTER_ALPHA  0.75f // Hệ số lọc EMA: 75% xung tức thời, 25% xung quá khứ (làm phẳng bước nhảy)

#define PCNT_UNIT_L PCNT_UNIT_0
#define PCNT_UNIT_R PCNT_UNIT_1

static int8_t s_dir_enc_L = DEFAULT_DIR_ENC_L;
static int8_t s_dir_enc_R = DEFAULT_DIR_ENC_R;

static float s_ppr_wheel_L = DEFAULT_PPR_WHEEL_L;
static float s_ppr_wheel_R = DEFAULT_PPR_WHEEL_R;

static volatile int64_t s_total_ticks_L = 0;
static volatile int64_t s_total_ticks_R = 0;

static int16_t s_last_raw_pcnt_L = 0;
static int16_t s_last_raw_pcnt_R = 0;

static volatile float s_current_rpm_L = 0;
static volatile float s_current_rpm_R = 0;
static volatile float s_current_mms_L = 0;
static volatile float s_current_mms_R = 0;

// Spinlock khóa ngắt siêu ngắn (chỉ 10ns) để bảo vệ việc đọc/ghi biến 64-bit an toàn tuyệt đối đa nhân (Dual-Core)
static portMUX_TYPE s_encoder_mux = portMUX_INITIALIZER_UNLOCKED;

static void init_single_pcnt_unit(int pinA, int pinB, pcnt_unit_t unit) {
    // Kích hoạt điện trở kéo lên nội Pull-up cho cả 2 kênh Encoder để nhận xung sườn đanh rõ nét
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    // Kênh 0: Cạnh xung chân A, mức điều khiển chân B
    pcnt_config_t pcnt_config_a = {
        .pulse_gpio_num = pinA,
        .ctrl_gpio_num = pinB,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_a);

    // Kênh 1: Cạnh xung chân B, mức điều khiển chân A (Cấu hình chuẩn Quadrature x4 giải mã 4 cạnh xung)
    pcnt_config_t pcnt_config_b = {
        .pulse_gpio_num = pinB,
        .ctrl_gpio_num = pinA,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_DEC, // Đồng pha đếm với kênh 0, không bị triệt tiêu xung lẫn nhau
        .neg_mode = PCNT_COUNT_INC, // Đồng pha đếm với kênh 0
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&pcnt_config_b);

    // Lọc xung nhiễu tia lửa điện động cơ dưới 1000ns (clock 80MHz: 100 chu kỳ = 1.25us)
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void encoder_init(void) {
    init_single_pcnt_unit(ENC_L_A, ENC_L_B, PCNT_UNIT_L);
    init_single_pcnt_unit(ENC_R_A, ENC_R_B, PCNT_UNIT_R);
    encoder_reset();
}

void encoder_reset(void) {
    portENTER_CRITICAL(&s_encoder_mux);
    pcnt_counter_clear(PCNT_UNIT_L);
    pcnt_counter_clear(PCNT_UNIT_R);
    s_last_raw_pcnt_L = 0;
    s_last_raw_pcnt_R = 0;
    s_total_ticks_L = 0;
    s_total_ticks_R = 0;
    s_current_rpm_L = 0;
    s_current_rpm_R = 0;
    s_current_mms_L = 0;
    s_current_mms_R = 0;
    portEXIT_CRITICAL(&s_encoder_mux);
}

IRAM_ATTR void encoder_update(float dt_sec) {
    int16_t cur_raw_L = 0, cur_raw_R = 0;
    pcnt_get_counter_value(PCNT_UNIT_L, &cur_raw_L);
    pcnt_get_counter_value(PCNT_UNIT_R, &cur_raw_R);

    int delta_L = (cur_raw_L - s_last_raw_pcnt_L) * s_dir_enc_L;
    int delta_R = (cur_raw_R - s_last_raw_pcnt_R) * s_dir_enc_R;
    s_last_raw_pcnt_L = cur_raw_L;
    s_last_raw_pcnt_R = cur_raw_R;

    // Cập nhật an toàn số xung 64-bit
    portENTER_CRITICAL(&s_encoder_mux);
    s_total_ticks_L += delta_L;
    s_total_ticks_R += delta_R;
    portEXIT_CRITICAL(&s_encoder_mux);

    if (dt_sec > 0.0001f) {
        float rps_L = (delta_L / s_ppr_wheel_L) / dt_sec;
        float rps_R = (delta_R / s_ppr_wheel_R) / dt_sec;

        float raw_rpm_L = rps_L * 60.0f;
        float raw_rpm_R = rps_R * 60.0f;
        float raw_mms_L = rps_L * WHEEL_CIRCUMFERENCE_MM;
        float raw_mms_R = rps_R * WHEEL_CIRCUMFERENCE_MM;

        // Bộ lọc EMA siêu nhẹ (~15ns FPU): Làm mịn triệt để bước nhảy rời rạc của vận tốc
        s_current_rpm_L = VELOCITY_FILTER_ALPHA * raw_rpm_L + (1.0f - VELOCITY_FILTER_ALPHA) * s_current_rpm_L;
        s_current_rpm_R = VELOCITY_FILTER_ALPHA * raw_rpm_R + (1.0f - VELOCITY_FILTER_ALPHA) * s_current_rpm_R;

        s_current_mms_L = VELOCITY_FILTER_ALPHA * raw_mms_L + (1.0f - VELOCITY_FILTER_ALPHA) * s_current_mms_L;
        s_current_mms_R = VELOCITY_FILTER_ALPHA * raw_mms_R + (1.0f - VELOCITY_FILTER_ALPHA) * s_current_mms_R;
    }
}

int64_t encoder_get_ticks_L(void) {
    portENTER_CRITICAL(&s_encoder_mux);
    int64_t ticks = s_total_ticks_L;
    portEXIT_CRITICAL(&s_encoder_mux);
    return ticks;
}

int64_t encoder_get_ticks_R(void) {
    portENTER_CRITICAL(&s_encoder_mux);
    int64_t ticks = s_total_ticks_R;
    portEXIT_CRITICAL(&s_encoder_mux);
    return ticks;
}

float encoder_get_dist_L_mm(void) {
    return (encoder_get_ticks_L() / s_ppr_wheel_L) * WHEEL_CIRCUMFERENCE_MM;
}

float encoder_get_dist_R_mm(void) {
    return (encoder_get_ticks_R() / s_ppr_wheel_R) * WHEEL_CIRCUMFERENCE_MM;
}

float encoder_get_dist_avg_mm(void) {
    return (encoder_get_dist_L_mm() + encoder_get_dist_R_mm()) * 0.5f;
}

float encoder_get_speed_L_mms(void) {
    return s_current_mms_L;
}

float encoder_get_speed_R_mms(void) {
    return s_current_mms_R;
}

float encoder_get_rpm_L(void) {
    return s_current_rpm_L;
}

float encoder_get_rpm_R(void) {
    return s_current_rpm_R;
}

void encoder_set_ppr(float ppr_L, float ppr_R) {
    s_ppr_wheel_L = ppr_L;
    s_ppr_wheel_R = ppr_R;
}

void encoder_get_ppr(float *ppr_L, float *ppr_R) {
    if (ppr_L) *ppr_L = s_ppr_wheel_L;
    if (ppr_R) *ppr_R = s_ppr_wheel_R;
}

void encoder_set_dir(int8_t dir_L, int8_t dir_R) {
    s_dir_enc_L = dir_L;
    s_dir_enc_R = dir_R;
}

int8_t encoder_get_dir_L(void) {
    return s_dir_enc_L;
}

int8_t encoder_get_dir_R(void) {
    return s_dir_enc_R;
}
