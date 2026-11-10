// ================================================================
// XE DÒ LINE 16 MẮT 2026 - PHIÊN BẢN HOÀN HẢO (CAR CHALLENGE 2026)
// Quét 16 mắt 4000Hz, MUX 8µs, Lọc IIR 92/8 siêu tốc
// ================================================================
#include "sensor.h"
#include "config.h"
#include <Arduino.h>

uint16_t sensorRaw[N_SENSORS] = {0};
uint16_t minValues[N_SENSORS] = {0};
uint16_t maxValues[N_SENSORS] = {0};
uint16_t thresholds[N_SENSORS] = {0};
bool blackIsHigher[N_SENSORS] = {false};

bool isCalibrating = false;
bool calibrationReady = true; // Mặc định sẵn sàng để xe có thể chạy ngay khi bật nguồn

static uint32_t s_calib_start_time = 0;
static uint32_t s_calib_duration_ms = 4000;
static bool s_global_black_is_higher = true; // ĐÃ CHỐT: Mắt đọc Vạch ĐEN = ADC CAO, Nền TRẮNG = ADC THẤP
static int16_t s_last_line_pos = 7500;

void sensor_init(void) {
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    pinMode(PIN_MUX_D, OUTPUT);

    digitalWrite(PIN_MUX_A, LOW);
    digitalWrite(PIN_MUX_B, LOW);
    digitalWrite(PIN_MUX_C, LOW);
    digitalWrite(PIN_MUX_D, LOW);

    pinMode(PIN_LINE_SIG, INPUT);
    analogReadResolution(12); // ADC 12-bit: 0 - 4095
    analogSetAttenuation(ADC_11db); // Dải đo đầy đủ 0 - 3.3V

    for (int i = 0; i < N_SENSORS; i++) {
        minValues[i] = 4095;
        maxValues[i] = 0;
        thresholds[i] = 2048;
        blackIsHigher[i] = true;
    }
    calibrationReady = true;
}

void sensor_mux_select(uint8_t channel) {
    channel &= 0x0F;
    digitalWrite(PIN_MUX_A, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_MUX_B, (channel & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_MUX_C, (channel & 0x04) ? HIGH : LOW);
    digitalWrite(PIN_MUX_D, (channel & 0x08) ? HIGH : LOW);
    delayMicroseconds(8); // Tối ưu tần số siêu cao 8µs (tăng gấp đôi tốc độ quét 16 mắt)
}

uint16_t sensor_read_mux(uint8_t channel) {
    sensor_mux_select(channel);
    // Đọc ADC trực tiếp nhanh chuẩn xác
    return (uint16_t)analogRead(PIN_LINE_SIG);
}

static uint32_t s_last_read_us = 0;

void sensor_read_all(void) {
    uint32_t now = (uint32_t)esp_timer_get_time();
    // Cho phép quét tần số cực cao (> 3500Hz), chỉ khóa nếu khoảng cách < 250µs
    if ((now - s_last_read_us) < 250 && !isCalibrating && calibrationReady) {
        return;
    }
    s_last_read_us = now;

    for (uint8_t i = 0; i < N_SENSORS; i++) {
        uint16_t raw = sensor_read_mux(i);
        if (isCalibrating) {
            sensorRaw[i] = raw;
        } else {
            // Bộ lọc IIR phản ứng cực nhạy (92% mới, 8% cũ), bắt vạch tức thời trong < 1ms
            sensorRaw[i] = (uint16_t)(((uint32_t)raw * 92 + (uint32_t)sensorRaw[i] * 8) / 100);
        }
    }
}

void sensor_start_sweep_calibration(uint32_t duration_ms) {
    s_calib_duration_ms = duration_ms;
    s_calib_start_time = millis();
    isCalibrating = true;
    calibrationReady = false;

    for (int i = 0; i < N_SENSORS; i++) {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }
}

void sensor_update_calibration_loop(void) {
    if (!isCalibrating) return;

    sensor_read_all();
    for (int i = 0; i < N_SENSORS; i++) {
        if (sensorRaw[i] < minValues[i]) minValues[i] = sensorRaw[i];
        if (sensorRaw[i] > maxValues[i]) maxValues[i] = sensorRaw[i];
    }

    if (millis() - s_calib_start_time >= s_calib_duration_ms) {
        sensor_finish_sweep_calibration();
    }
}

void sensor_finish_sweep_calibration(void) {
    isCalibrating = false;
    calibrationReady = true;

    for (int i = 0; i < N_SENSORS; i++) {
        thresholds[i] = (minValues[i] + maxValues[i]) / 2;
        if ((maxValues[i] - minValues[i]) >= MIN_CALIBRATION_DELTA) {
            blackIsHigher[i] = s_global_black_is_higher;
        }
    }
}

bool sensor_is_calibrating(void) {
    return isCalibrating;
}

bool sensor_is_button_pressed(void) {
    static int last_reading = HIGH;
    static unsigned long last_time = 0;
    static bool pressed = false;

    int reading = digitalRead(PIN_CALIB);
    if (reading != last_reading) {
        last_time = millis();
        last_reading = reading;
    }

    if ((millis() - last_time) > BUTTON_DEBOUNCE_MS) {
        if (reading == LOW && !pressed) {
            pressed = true;
            return true;
        } else if (reading == HIGH) {
            pressed = false;
        }
    }
    return false;
}

void sensor_handle_calibration_step(void) {
    if (!isCalibrating) {
        sensor_start_sweep_calibration(4000);
    } else {
        sensor_finish_sweep_calibration();
    }
}

uint16_t sensor_get_calibrated(uint8_t index) {
    if (index >= N_SENSORS) return 0;
    if (!calibrationReady) {
        if (s_global_black_is_higher) {
            return (sensorRaw[index] * 1000UL) / 4095;
        } else {
            return ((4095 - sensorRaw[index]) * 1000UL) / 4095;
        }
    }

    uint16_t raw = sensorRaw[index];
    uint16_t mn = minValues[index];
    uint16_t mx = maxValues[index];

    if (mx <= mn || (mx - mn) < 150) return 0; // Kênh chưa calib hoặc hỏng (QTR_MIN_RANGE)

    uint16_t range = mx - mn;
    long v = 0;
    if (s_global_black_is_higher) {
        v = ((long)(raw - min(raw, mn)) * 1000) / range;
    } else {
        v = ((long)(mx - min(raw, mx)) * 1000) / range;
    }
    v = constrain(v, 0, 1000);
    return (uint16_t)v;
}

bool sensor_is_black(uint8_t index) {
    if (index >= N_SENSORS) return false;
    return (sensor_get_calibrated(index) >= 500);
}

uint16_t sensor_get_line_bits(void) {
    uint16_t bits = 0;
    for (uint8_t i = 0; i < N_SENSORS; i++) {
        if (sensor_is_black(i)) {
            bits |= (1 << i);
        }
    }
    return bits;
}

void sensor_set_polarity(bool black_is_higher) {
    s_global_black_is_higher = black_is_higher;
    for (int i = 0; i < N_SENSORS; i++) {
        blackIsHigher[i] = black_is_higher;
    }
}

bool sensor_get_polarity(void) {
    return s_global_black_is_higher;
}

/**
 * @brief Đọc vị trí tim vạch bằng thuật toán QTR Weighted Centroid (0..15000, Setpoint: 7500)
 */
int16_t sensor_get_line_position_analog(void) {
    sensor_read_all();

    long sum = 0;
    long weighted = 0;
    bool seen = false;

    for (uint8_t i = 0; i < N_SENSORS; i++) {
        uint16_t v = sensor_get_calibrated(i);
        if (v > 200) seen = true;
        if (v > 50) {
            weighted += (long)v * i * 1000;
            sum += v;
        }
    }

    if (!seen || sum == 0) {
        // Mất line: giữ lại biên gần nhất theo chuẩn QTR
        if (s_last_line_pos < 4000) {
            s_last_line_pos = 0; // Kịch biên trái
        } else if (s_last_line_pos > 11000) {
            s_last_line_pos = 15000; // Kịch biên phải
        }
    } else {
        s_last_line_pos = (int16_t)(weighted / sum);
    }

    return s_last_line_pos;
}

int16_t sensor_get_line_position_binary(void) {
    sensor_read_all();

    uint32_t weighted_sum = 0;
    uint32_t count = 0;

    for (uint8_t i = 0; i < N_SENSORS; i++) {
        if (sensor_is_black(i)) {
            weighted_sum += (uint32_t)i * 1000UL;
            count++;
        }
    }

    if (count == 0) {
        return NO_LINE;
    }

    return (int16_t)(weighted_sum / count);
}

int16_t sensor_get_line_position(void) {
    return sensor_get_line_position_analog();
}

// =================== CÁC HÀM NHẬN DIỆN ĐỊA HÌNH VẠCH ĐẶC BIỆT ===================

uint8_t sensor_get_black_count(void) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < N_SENSORS; i++) {
        if (sensor_is_black(i)) count++;
    }
    return count;
}

bool sensor_is_crossline(void) {
    // Vạch ngang: Có từ 11 mắt trở lên cùng thấy vạch đen
    return (sensor_get_black_count() >= 11);
}

bool sensor_is_out_of_line(void) {
    // Mất line: Không có mắt nào thấy vạch đen
    return (sensor_get_black_count() == 0);
}

bool sensor_is_90_left(void) {
    // Góc vuông trái: Mắt 0, 1 hoặc 2 thấy đen VÀ các mắt trung tâm (6, 7, 8) cũng thấy đen
    bool outer_left = sensor_is_black(0) || sensor_is_black(1);
    bool center = sensor_is_black(6) || sensor_is_black(7) || sensor_is_black(8);
    return (outer_left && center);
}

bool sensor_is_90_right(void) {
    // Góc vuông phải: Mắt 14 hoặc 15 thấy đen VÀ các mắt trung tâm (7, 8, 9) cũng thấy đen
    bool outer_right = sensor_is_black(14) || sensor_is_black(15);
    bool center = sensor_is_black(7) || sensor_is_black(8) || sensor_is_black(9);
    return (outer_right && center);
}

void sensor_print_states(void) {
    Serial.print("BITS: [");
    for (int i = 0; i < N_SENSORS; i++) {
        Serial.print(sensor_is_black(i) ? "1" : "0");
        if (i < N_SENSORS - 1) Serial.print(" ");
    }
    Serial.println("]");
}
