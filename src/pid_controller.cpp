#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include <Arduino.h>

// ================================================================
// 5. PID & THAM SỐ (TỐI ƯU HÌNH HỌC CÁNH CUNG 16 MẮT)
// ================================================================
static float KP = 0.04f, KD = 0.3f;
static unsigned long autoStartMs = 0;

// Bảng tọa độ hình học 16 mắt cánh cong (Tâm Setpoint = 7500)
// - 8 mắt giữa (4..11): bước mịn (500-900) để xe đi thẳng đầm chắc, triệt tiêu lắc đuôi cá
// - 8 mắt cánh (0..3 & 12..15): uốn cong lùi và xòe rộng, bước lũy tiến (1200-1300) để giật cua gắt
static const int SENSOR_POS[16] = {
        0,  1200,  2500,  3800, // Cánh trái (mắt 0..3)
     4800,  5700,  6600,  7100, // Nửa giữa trái (mắt 4..7)
     7900,  8400,  9300, 10200, // Nửa giữa phải (mắt 8..11)
    11200, 12500, 13800, 15000  // Cánh phải (mắt 12..15)
};

// Cua góc vuông: bánh ngoài = BASE + TURN_OUT, bánh trong = BASE - TURN_IN
// Đã hiệu chuẩn: với BASE=100 -> Ngoài=180, Trong=-20 (đảo chiều nhẹ xoay compa quanh trục cực gọn)
static int TURN_OUT = 80, TURN_IN = 120;
static const int BOOST = 20, BOOST_THRESHOLD = 150;

// Đoạn nét đứt (GAP) và vạch kết thúc (Full đen)
#define GAP_STRAIGHT_MS 100
#define GAP_MAX_COUNT   5
#define GAP_ERR_MAX     1500
#define END_BLACK_COUNT 2
#define END_CONFIRM     3

static int cnt = 0;
static int cntDen = 0;
static bool inGap = false, inFullBlack = false;
static int fullBlackFrames = 0;
static unsigned long gapStart = 0;

static int lastError = 0;
static int lastLinePos = 7500; // Tâm Setpoint của 16 mắt (0..15000)

static PID_Config_t pidConfig = {
    .Kp = 0.04f,
    .Ki = 0.0f,
    .Kd = 0.3f,
    .baseSpeed = 100,
    .maxSpeed = 220,
    .minSpeed = -115,
    .turnSpeed = 130
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static uint8_t s_sector_action = 0; // 0: Normal, 1: Force Left, 2: Force Right, 3: Lock Straight, 4: Brake

static inline int16_t clamp_int16(int16_t v, int16_t min_v, int16_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void move(int left, int right) {
    left = clamp_int16(left, pidConfig.minSpeed, pidConfig.maxSpeed);
    right = clamp_int16(right, pidConfig.minSpeed, pidConfig.maxSpeed);
    motor_set_speeds(left, right);
}

static int autoBaseSpeed(void) {
    return pidConfig.baseSpeed;
}

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    KP = kp;
    KD = kd;
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = baseSpeed;
    lastError = 0;
    lastLinePos = 7500;
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    cnt = 0;
    cntDen = 0;
    inGap = false;
    inFullBlack = false;
    fullBlackFrames = 0;
}

void pid_set_tunings(float kp, float ki, float kd) {
    KP = kp;
    KD = kd;
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}

void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = clamp_int16(speed, 0, 255);
}

void pid_set_turn_speed(int16_t speed) {
    pidConfig.turnSpeed = clamp_int16(speed, 60, 220);
    TURN_OUT = (int)(pidConfig.turnSpeed * 80 / 130);
    TURN_IN  = (int)(pidConfig.turnSpeed * 120 / 130);
}

int16_t pid_get_turn_speed(void) {
    return pidConfig.turnSpeed;
}

void pid_set_turn_params(int out, int in) {
    TURN_OUT = clamp_int16(out, 0, 255);
    TURN_IN  = clamp_int16(in, 0, 255);
    pidConfig.turnSpeed = TURN_OUT;
}

int pid_get_turn_out(void) {
    return TURN_OUT;
}

int pid_get_turn_in(void) {
    return TURN_IN;
}

void pid_set_sector_action(uint8_t action) {
    s_sector_action = action;
}

uint8_t pid_get_sector_action(void) {
    return s_sector_action;
}

void pid_set_auto_mode(bool enable) {
    autoMode = enable;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
    } else {
        autoStartMs = millis();
        lastError = 0;
        lastLinePos = 7500;
        cnt = 0;
        cntDen = 0;
        inGap = false;
        fullBlackFrames = 0;
        inFullBlack = false;
        currentCase = LINE_CASE_NORMAL_PID;
    }
}

bool pid_is_auto_mode(void) {
    return autoMode;
}

/**
 * @brief Đọc vị trí tim vạch bằng thuật toán chuẩn của code mẫu
 * 16 mắt: dải 0..15000, Setpoint = 7500
 */
bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    sensor_read_all();

    long sum = 0;
    long weighted = 0;
    bool seen = false;
    uint8_t count = 0;

    int start_i = 0;
    int end_i = N_SENSORS - 1;
    if (s_sector_action == 1) { // ACTION_FORCE_LEFT: Ép nhánh trái
        end_i = 8;
    } else if (s_sector_action == 2) { // ACTION_FORCE_RIGHT: Ép nhánh phải
        start_i = 7;
    }

    for (int i = start_i; i <= end_i; i++) {
        uint16_t v = sensor_get_calibrated(i);
        if (v > 200) seen = true;
        if (v > 500) count++;
        if (v > 50) {
            weighted += (long)v * SENSOR_POS[i];
            sum += v;
        }
    }
    *outBlackCount = count;

    if (!seen || sum == 0) {
        if (lastLinePos < 3500)
            lastLinePos = 0;
        else if (lastLinePos > 11500)
            lastLinePos = 15000;
        // Trả ra vị trí scale cho Web BLE gauge (-75.0 .. +75.0)
        *outPosition = (float)(lastLinePos - 7500) / 100.0f;
        return false;
    } else {
        lastLinePos = (int)(weighted / sum);
    }

    *outPosition = (float)(lastLinePos - 7500) / 100.0f;
    return true;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady) return LINE_CASE_STOPPED;
    uint16_t mask = sensor_get_line_bits();
    bool wings_both = ((mask & 0x000F) != 0) && ((mask & 0xF000) != 0);
    if (blackCount >= 9 || (wings_both && blackCount >= 6) || mask == 0xFFFF) return LINE_CASE_CROSS_LINE;
    // Cua phải: Cụm cánh phải (12..15) đen, cụm cánh trái (0..2) trắng
    if (s_sector_action != 1 && (mask & 0xF000) && !(mask & 0x0007) && blackCount <= 8) return LINE_CASE_CORNER_RIGHT;
    // Cua trái: Cụm cánh trái (0..3) đen, cụm cánh phải (13..15) trắng
    if (s_sector_action != 2 && (mask & 0x000F) && !(mask & 0xE000) && blackCount <= 8) return LINE_CASE_CORNER_LEFT;
    if (blackCount == 0) return LINE_CASE_LOST_LINE;
    return LINE_CASE_NORMAL_PID;
}

void pid_line_follow_step(void) {
    if (!autoMode) {
        return;
    }

    int base = autoBaseSpeed();

    // Sa bàn: Ép chạy thẳng vượt vạch nét đứt
    if (s_sector_action == 3) {
        currentCase = LINE_CASE_DASHED_LINE;
        move(base, base);
        return;
    }
    // Sa bàn: Phanh dừng chốt đích
    if (s_sector_action == 4) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
        return;
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);
    uint16_t currentPattern = sensor_get_line_bits();

    // 1. Vạch kết thúc / ngã tư lớn (Tối ưu cho cánh cong)
    // Khi cán vạch ngang, cả 2 cánh chạm đen hoặc tổng số mắt đen >= 9
    bool wings_both_black = ((currentPattern & 0x000F) != 0) && ((currentPattern & 0xF000) != 0);
    if (blackCount >= 9 || (wings_both_black && blackCount >= 6) || currentPattern == 0xFFFF) {
        if (fullBlackFrames < END_CONFIRM)
            fullBlackFrames++;
        if (fullBlackFrames >= END_CONFIRM && !inFullBlack) {
            inFullBlack = true;
            cntDen++;
            if (cntDen >= END_BLACK_COUNT) {
                currentCase = LINE_CASE_CROSS_LINE;
                motor_stop();
                return;
            }
        }
    } else {
        fullBlackFrames = 0;
        inFullBlack = false;
    }

    if (blackCount != 0)
        inGap = false;

    // 2. Cua 90 độ (Tối ưu cho cánh cong: một cánh chạm đen, cánh đối diện hoàn toàn trắng)
    // Cua phải 90°: Cánh phải (mắt 12..15) dính đen, cánh trái (mắt 0..2) trắng (Bỏ qua nếu đang Ép Trái)
    if (s_sector_action != 1 && (currentPattern & 0xF000) && !(currentPattern & 0x0007) && blackCount <= 8) {
        currentCase = LINE_CASE_CORNER_RIGHT;
        move(base + TURN_OUT, base - TURN_IN);
        return;
    }
    // Cua trái 90°: Cánh trái (mắt 0..3) dính đen, cánh phải (mắt 13..15) trắng (Bỏ qua nếu đang Ép Phải)
    if (s_sector_action != 2 && (currentPattern & 0x000F) && !(currentPattern & 0xE000) && blackCount <= 8) {
        currentCase = LINE_CASE_CORNER_LEFT;
        move(base - TURN_IN, base + TURN_OUT);
        return;
    }

    // 3. Mất line / Vạch nét đứt (GAP)
    if (blackCount == 0) {
        if (!inGap && cnt < GAP_MAX_COUNT && abs(lastError) <= GAP_ERR_MAX) {
            inGap = true;
            gapStart = millis();
            cnt++;
        }
        if (inGap && millis() - gapStart < GAP_STRAIGHT_MS) {
            currentCase = LINE_CASE_DASHED_LINE;
            move(base, base);
            return;
        }
        currentCase = LINE_CASE_LOST_LINE;
        if (lastError > 0)
            move(base + BOOST, -base);
        else
            move(-base, base + BOOST);
        return;
    }

    // 4. PID bám line (công thức trực tiếp của code mẫu + Deadband nhỏ cho cánh cong)
    currentCase = LINE_CASE_NORMAL_PID;
    int error = lastLinePos - 7500; // SETPOINT = 7500
    if (abs(error) < 120) {
        error = 0; // Deadband nhỏ triệt tiêu rung lắc ở đường thẳng
    }

    int adjust = (int)(error * KP) + (int)(KD * (error - lastError));
    lastError = (abs(error) < 120) ? 0 : error;

    int L = base + adjust;
    int R = base - adjust;
    if (adjust > BOOST_THRESHOLD)
        L += BOOST;
    if (adjust < -BOOST_THRESHOLD)
        R += BOOST;

    move(L, R);
}

LineCase_t pid_get_current_case(void) {
    return currentCase;
}

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID:   return "PID_FOLLOW";
        case LINE_CASE_LOST_LINE:    return "LOST_LINE";
        case LINE_CASE_CROSS_LINE:   return "CROSS_LINE";
        case LINE_CASE_DASHED_LINE:  return "GAP_STRAIGHT";
        case LINE_CASE_CORNER_LEFT:  return "TURN_LEFT";
        case LINE_CASE_CORNER_RIGHT: return "TURN_RIGHT";
        case LINE_CASE_STOPPED:      return "STOPPED";
        default: return "UNKNOWN";
    }
}

float pid_get_last_error(void) {
    return (float)lastError / 100.0f;
}

float pid_get_correction(void) {
    return (float)lastError / 100.0f;
}

float pid_get_kp(void) {
    return KP;
}

float pid_get_ki(void) {
    return 0.0f;
}

float pid_get_kd(void) {
    return KD;
}

int16_t pid_get_base_speed(void) {
    return pidConfig.baseSpeed;
}
