// ================================================================
// XE DÒ LINE 16 MẮT 2026 - PHIÊN BẢN HOÀN HẢO (CAR CHALLENGE 2026)
// Giao tiếp Bluetooth Low Energy (NUS), Telemetry 150ms chuẩn mượt
// ================================================================
#include "ble_handler.h"
#include "map_manager.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"
#include "encoder.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLEServer *pServer = NULL;
static BLECharacteristic *pTxCharacteristic = NULL;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;
static CarRunMode_t s_car_mode = CAR_MODE_STOP;
static int16_t s_scan_speed = 100; // Tốc độ nền mặc định 100 để xe chạy ổn định chuẩn xác

void car_set_run_mode(CarRunMode_t mode) {
    s_car_mode = mode;
    if (mode == CAR_MODE_STOP) {
        pid_set_auto_mode(false);
        motor_stop();
    } else if (mode == CAR_MODE_ALIGN) {
        pid_set_auto_mode(false); // Ở chế độ căn tâm, controlTask trực tiếp điều khiển lực xoay nhẹ quanh tâm
    } else {
        // KHÔNG reset xung ở đây! Xung chỉ reset khi người dùng bấm nút CMD:RESET_ENC
        pid_set_auto_mode(true);
    }
}

CarRunMode_t car_get_run_mode(void) {
    return s_car_mode;
}

void car_set_scan_speed(int16_t speed) {
    s_scan_speed = constrain(speed, 50, 220);
}

int16_t car_get_scan_speed(void) {
    return s_scan_speed;
}

// Callbacks xử lý kết nối BLE
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("\n>> 🔵 [BLE] ĐÃ KẾT NỐI VỚI WEB!");
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("\n>> 🔴 [BLE] ĐÃ NGẮT KẾT NỐI KHỎI WEB!");
        // Bật lại quảng bá để Web có thể quét kết nối lại bất cứ lúc nào
        pServer->startAdvertising();
    }
};

// Callbacks nhận lệnh từ Web BLE
class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rawVal = pCharacteristic->getValue();
        if (rawVal.length() == 0) return;

        String rxValue = String(rawVal.c_str());

        rxValue.trim();
        Serial.printf(">> 📥 [WEB LỆNH]: %s\n", rxValue.c_str());

        // 1. Lệnh điều khiển chạy (Hỗ trợ cả giao diện Web BLE và App Serial Terminal)
        if (rxValue == "CMD:SCAN") {
            // LƯỢT 1: Quét map chậm (bò tốc độ an toàn 70 để lấy mốc)
            car_set_run_mode(CAR_MODE_SCAN);
            ble_send_string("ACK:SCAN_MODE_STARTED");
        } else if (rxValue == "CMD:RACE" || rxValue == "CMD:START" || rxValue == "AUTO" || rxValue == "RUN") {
            // LƯỢT 2: Bung tốc độ đua theo sa bàn / Bật Auto phóng xe
            car_set_run_mode(CAR_MODE_RACE);
            ble_send_string("ACK:RACE_MODE_STARTED");
        } else if (rxValue == "CMD:STOP" || rxValue == "MANUAL" || rxValue == "STOP") {
            car_set_run_mode(CAR_MODE_STOP);
            ble_send_string("ACK:STOPPED");
        } else if (rxValue == "CMD:CALIB" || rxValue == "CAL") {
            car_set_run_mode(CAR_MODE_STOP);
            sensor_start_sweep_calibration(4000);
            ble_send_string("ACK:CALIB_STARTED");
        }
        else if (rxValue.startsWith("CMD:SCAN_SPD:")) {
            int16_t spd = rxValue.substring(13).toInt();
            car_set_scan_speed(spd);
            ble_send_string("ACK:SCAN_SPD_SET");
        }
        else if (rxValue == "CMD:RESET_ENC") {
            encoder_reset();
            ble_send_string("ACK:ENC_RESET");
        }
        else if (rxValue.startsWith("CMD:PID:") || rxValue.startsWith("PID:")) {
            // Hỗ trợ cả CMD:PID:kp:ki:kd:spd và PID:kp,kd,spd
            String pStr = rxValue.startsWith("CMD:PID:") ? rxValue.substring(8) : rxValue.substring(4);
            pStr.replace(',', ':');
            int p1 = pStr.indexOf(':');
            int p2 = pStr.indexOf(':', p1 + 1);
            int p3 = pStr.indexOf(':', p2 + 1);
            if (p1 > 0 && p2 > 0) {
                float kp = pStr.substring(0, p1).toFloat();
                float p2_val = pStr.substring(p1 + 1, p2).toFloat();
                if (p3 > 0) {
                    // Cú pháp 4 tham số: kp:ki:kd:spd
                    float kd = pStr.substring(p2 + 1, p3).toFloat();
                    int16_t spd = pStr.substring(p3 + 1).toInt();
                    pid_set_tunings(kp, p2_val, kd);
                    pid_set_base_speed(spd);
                } else {
                    // Cú pháp 3 tham số: kp:kd:spd (chuẩn code mẫu)
                    float kd = p2_val;
                    int16_t spd = pStr.substring(p2 + 1).toInt();
                    pid_set_tunings(kp, 0.0f, kd);
                    pid_set_base_speed(spd);
                }
                ble_send_string("ACK:PID_UPDATED");
            }
        } else if (rxValue == "GETPID") {
            char pidBuf[96];
            snprintf(pidBuf, sizeof(pidBuf), "PID:%.3f,%.3f,%d,TURN:%d,%d", 
                     pid_get_kp(), pid_get_kd(), pid_get_base_speed(),
                     pid_get_turn_out(), pid_get_turn_in());
            ble_send_string(pidBuf);
        } else if (rxValue.startsWith("CMD:TURN:") || rxValue.startsWith("TURN:")) {
            // Cú pháp: CMD:TURN:<out>:<in> hoặc TURN:<out>,<in>
            String tStr = rxValue.startsWith("CMD:TURN:") ? rxValue.substring(9) : rxValue.substring(5);
            tStr.replace(',', ':');
            int sep = tStr.indexOf(':');
            if (sep > 0) {
                int t_out = tStr.substring(0, sep).toInt();
                int t_in = tStr.substring(sep + 1).toInt();
                pid_set_turn_params(t_out, t_in);
                char ackBuf[48];
                snprintf(ackBuf, sizeof(ackBuf), "ACK:TURN_SET:%d,%d", t_out, t_in);
                ble_send_string(ackBuf);
            }
        } else if (rxValue == "GETTURN") {
            char tBuf[48];
            snprintf(tBuf, sizeof(tBuf), "TURN:%d,%d", pid_get_turn_out(), pid_get_turn_in());
            ble_send_string(tBuf);
        }
        // 2. Chọn map & làn
        else if (rxValue.startsWith("CMD:MAP:")) {
            uint8_t m = rxValue.substring(8).toInt();
            map_select(m);
            ble_send_map_data(m);
        } else if (rxValue.startsWith("CMD:LANE:")) {
            bool right = (rxValue.substring(9).toInt() == 1);
            map_set_right_lane(right);
            ble_send_string(right ? "ACK:LANE_RIGHT" : "ACK:LANE_LEFT");
        }
        // 3. Quản lý Waypoint
        else if (rxValue.startsWith("CMD:WP_SET:")) {
            // Cú pháp: CMD:WP_SET:<map>:<idx>:<ticks>:<speed>:<action>
            int p1 = rxValue.indexOf(':', 11);
            int p2 = rxValue.indexOf(':', p1 + 1);
            int p3 = rxValue.indexOf(':', p2 + 1);
            int p4 = rxValue.indexOf(':', p3 + 1);

            if (p1 > 0 && p2 > 0 && p3 > 0 && p4 > 0) {
                uint8_t m = rxValue.substring(11, p1).toInt();
                uint8_t idx = rxValue.substring(p1 + 1, p2).toInt();
                uint32_t ticks = rxValue.substring(p2 + 1, p3).toInt();
                int16_t spd = rxValue.substring(p3 + 1, p4).toInt();
                uint8_t act = rxValue.substring(p4 + 1).toInt();

                map_set_waypoint(m, idx, ticks, spd, act);
                ble_send_string("ACK:WP_SET_OK");
            }
        } else if (rxValue.startsWith("CMD:WP_CLEAR:")) {
            uint8_t m = rxValue.substring(13).toInt();
            map_clear_waypoints(m);
            ble_send_string("ACK:WP_CLEARED");
        } else if (rxValue == "CMD:SAVE") {
            map_save_to_flash();
            ble_send_string("ACK:SAVED_FLASH");
        } else if (rxValue == "CMD:RESET_DEF") {
            map_load_defaults();
            map_save_to_flash();
            ble_send_map_data(map_get_selected());
            ble_send_string("ACK:DEFAULTS_RESTORED");
        } else if (rxValue.startsWith("CMD:GET_MAP:")) {
            uint8_t m = rxValue.substring(12).toInt();
            ble_send_map_data(m);
        }
        else if (rxValue == "CMD:POLARITY:TOGGLE") {
            bool cur = sensor_get_polarity();
            sensor_set_polarity(!cur);
            ble_send_string(!cur ? "ACK:POLARITY:HIGHER" : "ACK:POLARITY:LOWER");
        }
        else if (rxValue == "CMD:POLARITY:GET") {
            ble_send_string(sensor_get_polarity() ? "ACK:POLARITY:HIGHER" : "ACK:POLARITY:LOWER");
        }
    }
};

void ble_init(const char *device_name) {
    BLEDevice::init(device_name);
    BLEDevice::setMTU(256);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // TX Characteristic (Notify to Web)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Write from Web)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();

    // Bắt đầu quảng bá (Advertising)
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Tăng tốc độ nhận diện
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

bool ble_is_connected(void) {
    return deviceConnected;
}

void ble_send_string(const char *str) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue((uint8_t*)str, strlen(str));
        pTxCharacteristic->notify();
        delay(5);
    }
    Serial.printf(">> 📤 [BLE PHẢN HỒI]: %s\n", str);
}

void ble_send_telemetry(int32_t ticks_l, int32_t ticks_r, float line_pos, 
                        int16_t pwm_l, int16_t pwm_r, 
                        uint8_t map_idx, uint8_t wp_idx, uint8_t car_state,
                        uint16_t sensor_mask) {
    if (!deviceConnected || !pTxCharacteristic) return;

    // Định dạng gói tin CSV chuẩn Web:
    // T:<ticks_L>,<ticks_R>,<pos*10>,<pwm_L>,<pwm_R>,<map>,<wp>,<state>,<mask_int>
    char buf[96];
    int16_t pos_scaled = (int16_t)roundf(line_pos * 10.0f);
    snprintf(buf, sizeof(buf), "T:%ld,%ld,%d,%d,%d,%u,%u,%u,%u",
             (long)ticks_l, (long)ticks_r, pos_scaled,
             pwm_l, pwm_r, map_idx, wp_idx, car_state, (unsigned int)sensor_mask);

    pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
    pTxCharacteristic->notify();
    delay(5);

    // In Serial mỗi 500ms để kiểm tra tận mắt dữ liệu gửi lên Web
    static uint32_t s_last_dbg = 0;
    if (millis() - s_last_dbg > 500) {
        s_last_dbg = millis();
        Serial.printf(">> 📡 [BLE GỬI]: %s\n", buf);
    }
}

void ble_send_map_data(uint8_t map_idx) {
    if (!deviceConnected || !pTxCharacteristic) return;

    uint8_t count = map_get_waypoint_count(map_idx);
    char buf[64];
    snprintf(buf, sizeof(buf), "MAP_INFO:%u,%u,%u", map_idx, count, map_is_right_lane() ? 1 : 0);
    ble_send_string(buf);
    delay(10);

    for (uint8_t i = 0; i < count; i++) {
        const Waypoint_t *wp = map_get_waypoint(map_idx, i);
        if (wp) {
            snprintf(buf, sizeof(buf), "WP:%u,%u,%lu,%d,%u",
                     map_idx, i, (unsigned long)wp->end_ticks, wp->target_speed, wp->action);
            ble_send_string(buf);
            delay(15); // Khoảng nghỉ nhỏ để tránh tràn bộ đệm BLE Notify
        }
    }
}
