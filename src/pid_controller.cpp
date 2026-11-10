// ================================================================
// XE DÒ LINE 16 MẮT 2026 - PHIÊN BẢN HOÀN HẢO (CAR CHALLENGE 2026)
// Bộ điều khiển PID 100 PWM, Continuous Deadband, State Machine 90°
// ================================================================
#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include <Arduino.h>

// ================================================================
// PID & THAM SỐ (TỐI ƯU HÌNH HỌC CÁNH CUNG 16 MẮT)
// ================================================================
static float KP = 0.04f, KD = 0.35f;
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

// Cua góc vuông: Bánh ngoài tiến (p_out = 90), bánh trong lùi (p_in = 70) tạo tâm xoay compa cực êm
static int TURN_OUT = 90, TURN_IN = 70;

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
static float s_filtered_dError = 0.0f;
static int lastLinePos = 7500; // Tâm Setpoint của 16 mắt (0..15000)
static unsigned long s_last_center_ms = 0;

// ================================================================
// STATE MACHINE CUA 90 ĐỘ (CHỐT XOAY COMPA KHÔNG VĂNG)
// ================================================================
// 0: Không cua (PID bình thường)
// 1: Đang xoay góc vuông Trái 90°
// 2: Đang xoay góc vuông Phải 90°
static uint8_t s_corner_state = 0;
static unsigned long s_corner_start_ms = 0;
static unsigned long s_last_corner_end_ms = 0; // Chống kích hoạt đúp khi vừa thoát góc vuông
static bool s_corner_cleared_old = false;

static PID_Config_t pidConfig = {
    .Kp = 0.04f,
    .Ki = 0.0f,
    .Kd = 0.35f,
    .baseSpeed = 100,
    .maxSpeed = 220,
    .minSpeed = -120,
    .turnSpeed = 90
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
    s_filtered_dError = 0.0f;
    lastLinePos = 7500;
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    s_corner_state = 0;
    s_corner_start_ms = 0;
    s_last_corner_end_ms = 0;
    s_corner_cleared_old = false;
    s_last_center_ms = millis();
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
    pidConfig.turnSpeed = clamp_int16(speed, 50, 220);
    TURN_OUT = clamp_int16(speed, 50, 220);
    TURN_IN  = (int)(TURN_OUT * 70 / 90);
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
        s_corner_state = 0;
        s_corner_cleared_old = false;
        motor_stop();
    } else {
        autoStartMs = millis();
        lastError = 0;
        s_filtered_dError = 0.0f;
        lastLinePos = 7500;
        cnt = 0;
        cntDen = 0;
        inGap = false;
        fullBlackFrames = 0;
        inFullBlack = false;
        s_corner_state = 0;
        s_corner_start_ms = 0;
        s_last_corner_end_ms = 0;
        s_corner_cleared_old = false;
        s_last_center_ms = millis();
        currentCase = LINE_CASE_NORMAL_PID;
    }
}

bool pid_is_auto_mode(void) {
    return autoMode;
}

/**
 * @brief Đọc vị trí tim vạch bằng thuật toán chuẩn Smooth Centroid
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
    if (s_corner_state == 2) return LINE_CASE_CORNER_RIGHT;
    if (s_corner_state == 1) return LINE_CASE_CORNER_LEFT;
    uint16_t mask = sensor_get_line_bits();
    bool wings_both = ((mask & 0x000F) != 0) && ((mask & 0xF000) != 0);
    if (blackCount >= 9 || (wings_both && blackCount >= 6) || mask == 0xFFFF) return LINE_CASE_CROSS_LINE;
    if (s_sector_action != 1 && (mask & 0xE000) && !(mask & 0x0007) && blackCount <= 8) return LINE_CASE_CORNER_RIGHT;
    if (s_sector_action != 2 && (mask & 0x0007) && !(mask & 0xE000) && blackCount <= 8) return LINE_CASE_CORNER_LEFT;
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
        s_corner_state = 0;
        move(base, base);
        return;
    }
    // Sa bàn: Phanh dừng chốt đích
    if (s_sector_action == 4) {
        currentCase = LINE_CASE_STOPPED;
        s_corner_state = 0;
        motor_stop();
        return;
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);
    uint16_t currentPattern = sensor_get_line_bits();

    // Ghi nhận thời điểm mắt trung tâm (5..10) nhìn thấy line
    if ((currentPattern & 0x07E0) != 0) {
        s_last_center_ms = millis();
    }

    // 1. Vạch kết thúc / ngã tư lớn
    // Chỉ kích hoạt khi CẢ 2 CÁNH ĐỀU ĐEN (vạch ngang trọn vẹn) hoặc gần như toàn bộ dàn mắt đen (>= 13 mắt)
    bool wings_both_black = ((currentPattern & 0x000F) != 0) && ((currentPattern & 0xF000) != 0);
    if ((wings_both_black && blackCount >= 6) || blackCount >= 13 || currentPattern == 0xFFFF) {
        if (fullBlackFrames < END_CONFIRM)
            fullBlackFrames++;
        if (fullBlackFrames >= END_CONFIRM && !inFullBlack) {
            inFullBlack = true;
            cntDen++;
            if (cntDen >= END_BLACK_COUNT) {
                currentCase = LINE_CASE_CROSS_LINE;
                s_corner_state = 0;
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

    // 2. TÍNH TOÁN LỰC XOAY COMPA CUA 90 ĐỘ (BÁNH NGOÀI 90, BÁNH TRONG 70)
    // Bánh ngoài tiến (p_out = 90): xoay êm ái, bám sát vạch, không bị văng trớn
    // Bánh trong lùi (p_in = 70): ghìm cứng làm tâm compa chuẩn xác
    int p_out = constrain(TURN_OUT, 60, 130);
    int p_in  = constrain(TURN_IN, 40, 110);

    // 2A. NẾU ĐANG TRONG TRẠNG THÁI XOAY 90 ĐỘ (STATE MACHINE ACTIVE)
    if (s_corner_state == 2) { // Đang xoay Phải 90°
        currentCase = LINE_CASE_CORNER_RIGHT;
        move(p_out, -p_in); // Bánh trái tiến, bánh phải lùi ghìm tâm

        unsigned long elapsed = millis() - s_corner_start_ms;
        // Kiểm tra xem vạch cũ đã rời khỏi cụm mắt giữa chưa
        if ((currentPattern & 0x03C0) == 0) {
            s_corner_cleared_old = true;
        }

        // Thoát cua khi: Vạch cũ đã rời đi VÀ mắt giữa bắt được vạch mới (hoặc sau 40ms nếu xoay nhanh)
        if ((s_corner_cleared_old || elapsed >= 40) && ((currentPattern & 0x03C0) != 0)) {
            s_corner_state = 0;
            s_corner_cleared_old = false;
            s_last_corner_end_ms = millis();
            lastError = 0;
            s_filtered_dError = 0.0f;
            return;
        }

        // Timeout bảo hiểm tối đa 350ms
        if (elapsed > 350) {
            s_corner_state = 0;
            s_corner_cleared_old = false;
            s_last_corner_end_ms = millis();
            lastError = 0;
            s_filtered_dError = 0.0f;
            return;
        }
        return;
    }

    if (s_corner_state == 1) { // Đang xoay Trái 90°
        currentCase = LINE_CASE_CORNER_LEFT;
        move(-p_in, p_out); // Bánh trái lùi ghìm tâm, bánh phải tiến

        unsigned long elapsed = millis() - s_corner_start_ms;
        if ((currentPattern & 0x03C0) == 0) {
            s_corner_cleared_old = true;
        }

        if ((s_corner_cleared_old || elapsed >= 40) && ((currentPattern & 0x03C0) != 0)) {
            s_corner_state = 0;
            s_corner_cleared_old = false;
            s_last_corner_end_ms = millis();
            lastError = 0;
            s_filtered_dError = 0.0f;
            return;
        }

        if (elapsed > 350) {
            s_corner_state = 0;
            s_corner_cleared_old = false;
            s_last_corner_end_ms = millis();
            lastError = 0;
            s_filtered_dError = 0.0f;
            return;
        }
        return;
    }

    // 2B. NHẬN DIỆN BẮT ĐẦU VÀO GÓC VUÔNG 90 ĐỘ / NGÃ 3 NHÁNH RẼ (MỞ RỘNG VÙNG MẮT QUÉT)
    // Đếm số mắt đen ở 2 cánh để nhận diện sớm ngã 3 ngay từ mắt 10-12 (không cần chờ mắt rìa 15)
    uint8_t right_branch_cnt = 0;
    if (currentPattern & (1 << 10)) right_branch_cnt++;
    if (currentPattern & (1 << 11)) right_branch_cnt++;
    if (currentPattern & (1 << 12)) right_branch_cnt++;
    if (currentPattern & (1 << 13)) right_branch_cnt++;
    if (currentPattern & (1 << 14)) right_branch_cnt++;
    if (currentPattern & (1 << 15)) right_branch_cnt++;

    uint8_t left_branch_cnt = 0;
    if (currentPattern & (1 << 0)) left_branch_cnt++;
    if (currentPattern & (1 << 1)) left_branch_cnt++;
    if (currentPattern & (1 << 2)) left_branch_cnt++;
    if (currentPattern & (1 << 3)) left_branch_cnt++;
    if (currentPattern & (1 << 4)) left_branch_cnt++;
    if (currentPattern & (1 << 5)) left_branch_cnt++;

    bool corner_cooldown_ok = (millis() - s_last_corner_end_ms > 70) || (s_sector_action == 1) || (s_sector_action == 2);
    bool recently_centered = ((currentPattern & 0x07E0) != 0) || ((millis() - s_last_center_ms) < 70);

    // Cua phải 90° / Ngã 3 rẽ phải:
    // - Nhánh phải bắt được (từ 2 mắt đen ở cánh phải trở lên hoặc mắt 13..15 chạm đen)
    // - Cánh trái (mắt 0..2) hoàn toàn TRẮNG (loại trừ ngã tư)
    bool right_branch_detected = (right_branch_cnt >= 2) || ((currentPattern & 0xE000) != 0);
    bool left_wing_clear = !(currentPattern & 0x0007);

    if (s_sector_action != 1 && corner_cooldown_ok && right_branch_detected && left_wing_clear && blackCount <= 12 && recently_centered) {
        s_corner_state = 2; // Khóa trạng thái cua Phải
        s_corner_start_ms = millis();
        s_corner_cleared_old = false;
        currentCase = LINE_CASE_CORNER_RIGHT;
        move(p_out, -p_in);
        return;
    }

    // Cua trái 90° / Ngã 3 rẽ trái:
    // - Nhánh trái bắt được (từ 2 mắt đen ở cánh trái trở lên hoặc mắt 0..2 chạm đen)
    // - Cánh phải (mắt 13..15) hoàn toàn TRẮNG
    bool left_branch_detected = (left_branch_cnt >= 2) || ((currentPattern & 0x0007) != 0);
    bool right_wing_clear = !(currentPattern & 0xE000);

    if (s_sector_action != 2 && corner_cooldown_ok && left_branch_detected && right_wing_clear && blackCount <= 12 && recently_centered) {
        s_corner_state = 1; // Khóa trạng thái cua Trái
        s_corner_start_ms = millis();
        s_corner_cleared_old = false;
        currentCase = LINE_CASE_CORNER_LEFT;
        move(-p_in, p_out);
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
            move(base + 20, -base);
        else
            move(-base, base + 20);
        return;
    }

    // 4. PID BÁM LINE ĐẦM CHẮC (DEADBAND LIÊN TỤC + D-TERM FILTER)
    currentCase = LINE_CASE_NORMAL_PID;
    int raw_error = lastLinePos - 7500; // SETPOINT = 7500

    // Vùng chết liên tục (Continuous Deadband 70): Triệt tiêu rung lắc ở đường thẳng
    // nhưng KHÔNG BAO GIỜ bị giật bước đột ngột (Zero Derivative Kick)
    int error = 0;
    const int DEADBAND = 70;
    if (raw_error > DEADBAND) {
        error = raw_error - DEADBAND;
    } else if (raw_error < -DEADBAND) {
        error = raw_error + DEADBAND;
    } else {
        error = 0;
    }

    // Bộ lọc thông thấp cho thành phần vi phân D (70% cũ, 30% mới) để dập tắt nhiễu ADC 1000Hz
    float raw_dError = (float)(error - lastError);
    s_filtered_dError = s_filtered_dError * 0.7f + raw_dError * 0.3f;
    lastError = error;

    int adjust = (int)(error * KP + s_filtered_dError * KD);

    int L = base + adjust;
    int R = base - adjust;

    // Thuật toán Bù lực đẩy bánh ngoài khi vào cung uốn lượn (High-Speed Curve Hugging)
    // Khi sai số |error| > 1000 (đang vào cung uốn lượn), tự động bơm thêm lực đẩy bánh ngoài lên tới +25 PWM
    // Giúp xe lướt tốc độ cao qua các đoạn uốn cong ngọt ngào như xe đua F1 có lực ép khí động học
    int abs_err = abs(error);
    if (abs_err > 1000) {
        int curve_boost = (int)(((long)(abs_err - 1000) * 25) / 6500);
        if (adjust > 0) {
            L += curve_boost; // Cung uốn sang phải -> Bơm thêm lực bánh ngoài (Trái)
        } else if (adjust < 0) {
            R += curve_boost; // Cung uốn sang trái -> Bơm thêm lực bánh ngoài (Phải)
        }
    }

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
