/**
 * android_uorb_publishers.cpp — Phase 3
 * الجسر بين SharedSensorData (native_sensor_reader) و PX4 uORB
 * يقرأ من SharedSensorData وينشر على uORB topics
 */
#include "android_uorb_publishers.h"
#include "shared_sensor_data.h"
#include <parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>  // param_find / param_get

#include <thread>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <unistd.h>

// PX4 Device ID format: (devtype << 16) | (address << 8) | (bus << 3) | bus_type
// bus_type=4 (SIMULATION), bus=0, address=0
// devtype: من drv_sensor.h — أنواع الحساسات المحاكية
#define ANDROID_DEVID_IMU   ((0x14 << 16) | 4)  // DRV_IMU_DEVTYPE_SIM + BusType_SIMULATION
#define ANDROID_DEVID_MAG   ((0x03 << 16) | 4)  // DRV_MAG_DEVTYPE_MAGSIM
#define ANDROID_DEVID_BARO  ((0x65 << 16) | 4)  // DRV_BARO_DEVTYPE_BAROSIM

extern SharedSensorData g_sensor_data;

static std::atomic<bool> s_running{false};
static std::thread s_publisher_thread;

// Mag values for publishing (only when new data arrives)
static float s_mag_x = 0.0f;
static float s_mag_y = 0.0f;
static float s_mag_z = 0.0f;

// === Atomic cache for UI reads (no semaphore needed from JNI) ===
// publisher_loop updates these from uORB; JNI getters read them lock-free.
static std::atomic<float> s_cached_roll{0.0f};
static std::atomic<float> s_cached_pitch{0.0f};
static std::atomic<float> s_cached_yaw{0.0f};
static std::atomic<float> s_cached_altitude{0.0f};
static std::atomic<bool>  s_cached_armed{false};
static std::atomic<int>   s_cached_nav_state{0};
static std::atomic<int>   s_cached_ekf_status{0};

// === Publishers (كتابة على uORB) ===
static uORB::Publication<sensor_accel_s> s_accel_pub{ORB_ID(sensor_accel)};
static uORB::Publication<sensor_gyro_s>  s_gyro_pub{ORB_ID(sensor_gyro)};
static uORB::Publication<sensor_baro_s>  s_baro_pub{ORB_ID(sensor_baro)};
static uORB::Publication<sensor_mag_s>   s_mag_pub{ORB_ID(sensor_mag)};
// === Subscribers (قراءة من uORB — لعرض الحالة في الواجهة) ===
static uORB::Subscription s_att_sub{ORB_ID(vehicle_attitude)};
static uORB::Subscription s_status_sub{ORB_ID(vehicle_status)};
static uORB::Subscription s_local_pos_sub{ORB_ID(vehicle_local_position)};

// Convert Android sensor timestamp (ns, BOOTTIME) to PX4 hrt_abstime (µs).
//
// المشكلة: Android BOOTTIME وPX4 HRT ليسا متطابقين تماماً — يوجد انجراف
// (clock drift) تراكمي يبلغ ~10-50 µs/s. إذا حُسب الـ offset مرة واحدة
// فقط عند البداية، يتراكم الخطأ مع الوقت حتى يتجاوز timestamp_sample
// قيمة timestamp، فيظهر الخطأ المتواصل في الكونسول:
//   "accel/gyro timestamp < timestamp_sample"
// وهذا يؤثر على دقة EKF لأنه يعتمد على الفرق بين الـ timestamps
// لحساب sensor latency الحقيقي.
//
// الحل: تحديث الـ offset كل 5 ثوانٍ (بدلاً من مرة واحدة) لمواجهة الـ drift
// باستمرار. هذا هو نفس المبدأ المستخدم في PX4 drivers الحقيقية.
// تمت الموافقة على هذا التعديل بتاريخ 2026-03-18.
static hrt_abstime s_clock_offset_us    = 0;
static bool        s_clock_offset_valid = false;
static hrt_abstime s_last_sync_us       = 0;

// فترة إعادة المزامنة: 5 ثوانٍ (بالميكروثانية)
static constexpr hrt_abstime CLOCK_SYNC_INTERVAL_US = 5000000ULL;

static hrt_abstime sensor_ts_to_hrt(int64_t android_ns) {
    const hrt_abstime now       = hrt_absolute_time();
    const hrt_abstime sensor_us = (hrt_abstime)(android_ns / 1000);

    // أعد حساب الـ offset عند أول استدعاء أو بعد مرور 5 ثوانٍ
    if (!s_clock_offset_valid || (now - s_last_sync_us) > CLOCK_SYNC_INTERVAL_US) {
        s_clock_offset_us   = now - sensor_us;
        s_clock_offset_valid = true;
        s_last_sync_us       = now;
    }

    return sensor_us + s_clock_offset_us;
}

static void publisher_loop() {
    // HITL: skip phone accel/gyro so sensor_combined uses HIL_SENSOR data only.
    // Without this, phone sensors get instance 0 and rocket_gnc reads desk accel
    // instead of simulated thrust → launch never detected → fins stay zero.
    int32_t sys_hitl = 0;
    param_t p_hitl = param_find("SYS_HITL");
    if (p_hitl != PARAM_INVALID) { param_get(p_hitl, &sys_hitl); }
    const bool skip_imu = (sys_hitl == 1);

    while (s_running.load()) {

        // ========== Accelerometer (independent) ==========
        if (!skip_imu && g_sensor_data.accel.has_new_data.load()) {
            std::lock_guard<std::mutex> lock(g_sensor_data.accel_mutex);

            // timestamp_sample = وقت استقبال event الهاردوير (مسجَّل في native_sensor_reader لحظة الوصول)
            // timestamp        = وقت النشر على uORB — دائماً >= timestamp_sample
            // الفرق = latency حقيقي يستخدمه EKF بدون drift أو تحويل ساعات
            const hrt_abstime sample_ts = g_sensor_data.accel.hrt_receipt_us;
            const hrt_abstime now       = hrt_absolute_time();

            sensor_accel_s accel{};
            accel.timestamp          = now;
            accel.timestamp_sample   = sample_ts;
            accel.device_id          = ANDROID_DEVID_IMU;
            accel.x                  = g_sensor_data.accel.data[0];
            accel.y                  = g_sensor_data.accel.data[1];
            accel.z                  = g_sensor_data.accel.data[2];
            accel.temperature        = g_sensor_data.accel.temperature;
            accel.samples            = 1;
            s_accel_pub.publish(accel);

            g_sensor_data.accel.has_new_data.store(false);
        }

        // ========== Gyroscope (independent) ==========
        if (!skip_imu && g_sensor_data.gyro.has_new_data.load()) {
            std::lock_guard<std::mutex> lock(g_sensor_data.gyro_mutex);

            // timestamp_sample = وقت استقبال event الهاردوير (مسجَّل في native_sensor_reader لحظة الوصول)
            // timestamp        = وقت النشر على uORB — دائماً >= timestamp_sample
            const hrt_abstime sample_ts = g_sensor_data.gyro.hrt_receipt_us;
            const hrt_abstime now       = hrt_absolute_time();

            sensor_gyro_s gyro{};
            gyro.timestamp           = now;
            gyro.timestamp_sample    = sample_ts;
            gyro.device_id           = ANDROID_DEVID_IMU;
            gyro.x                   = g_sensor_data.gyro.data[0];
            gyro.y                   = g_sensor_data.gyro.data[1];
            gyro.z                   = g_sensor_data.gyro.data[2];
            gyro.temperature         = g_sensor_data.gyro.temperature;
            gyro.samples             = 1;
            s_gyro_pub.publish(gyro);

            g_sensor_data.gyro.has_new_data.store(false);
        }

        // ========== Barometer ==========
        {
            std::lock_guard<std::mutex> lock(g_sensor_data.baro_mutex);
            if (g_sensor_data.baro.count > 0) {
                sensor_baro_s baro{};
                baro.timestamp       = hrt_absolute_time();
                baro.timestamp_sample = baro.timestamp;
                baro.device_id       = ANDROID_DEVID_BARO;
                // sum_pressure already in Pa (converted in native_sensor_reader.cpp: hPa × 100)
                baro.pressure        = (float)(g_sensor_data.baro.sum_pressure / g_sensor_data.baro.count);
                baro.temperature     = (float)(g_sensor_data.baro.sum_temperature / g_sensor_data.baro.count);
                s_baro_pub.publish(baro);

                // صفّر العداد
                g_sensor_data.baro.sum_pressure = 0;
                g_sensor_data.baro.sum_temperature = 0;
                g_sensor_data.baro.count = 0;
            }
        }

        // ========== Magnetometer (publish only when new data arrives) ==========
        {
            std::lock_guard<std::mutex> lock(g_sensor_data.mag_mutex);
            if (g_sensor_data.mag.count > 0) {
                s_mag_x = (float)(g_sensor_data.mag.sum_field[0] / g_sensor_data.mag.count) / 1000.0f;
                s_mag_y = (float)(g_sensor_data.mag.sum_field[1] / g_sensor_data.mag.count) / 1000.0f;
                s_mag_z = (float)(g_sensor_data.mag.sum_field[2] / g_sensor_data.mag.count) / 1000.0f;
                g_sensor_data.mag.sum_field[0] = 0;
                g_sensor_data.mag.sum_field[1] = 0;
                g_sensor_data.mag.sum_field[2] = 0;
                g_sensor_data.mag.count = 0;

                sensor_mag_s mag{};
                mag.timestamp        = hrt_absolute_time();
                mag.timestamp_sample = mag.timestamp;
                mag.device_id        = ANDROID_DEVID_MAG;
                mag.x = s_mag_x;
                mag.y = s_mag_y;
                mag.z = s_mag_z;
                s_mag_pub.publish(mag);
            }
        }

        // === Update cached state for UI (every ~50ms = 20 Hz is enough for display) ===
        static hrt_abstime s_last_ui_update = 0;
        const hrt_abstime now_ui = hrt_absolute_time();
        if ((now_ui - s_last_ui_update) >= 50000) { // 50 ms
            s_last_ui_update = now_ui;

            vehicle_attitude_s att{};
            if (s_att_sub.copy(&att)) {
                float sinr = 2.0f * (att.q[0] * att.q[1] + att.q[2] * att.q[3]);
                float cosr = 1.0f - 2.0f * (att.q[1] * att.q[1] + att.q[2] * att.q[2]);
                s_cached_roll.store(atan2f(sinr, cosr) * 57.2957795f, std::memory_order_relaxed);

                float sinp = 2.0f * (att.q[0] * att.q[2] - att.q[3] * att.q[1]);
                s_cached_pitch.store(asinf(fmaxf(-1.0f, fminf(1.0f, sinp))) * 57.2957795f, std::memory_order_relaxed);

                float siny = 2.0f * (att.q[0] * att.q[3] + att.q[1] * att.q[2]);
                float cosy = 1.0f - 2.0f * (att.q[2] * att.q[2] + att.q[3] * att.q[3]);
                s_cached_yaw.store(atan2f(siny, cosy) * 57.2957795f, std::memory_order_relaxed);
            }

            vehicle_local_position_s pos{};
            if (s_local_pos_sub.copy(&pos)) {
                s_cached_altitude.store(-pos.z, std::memory_order_relaxed);
            }

            vehicle_status_s status{};
            if (s_status_sub.copy(&status)) {
                s_cached_armed.store(status.arming_state == 2, std::memory_order_relaxed);
                s_cached_nav_state.store(status.nav_state, std::memory_order_relaxed);
            }
        }

        usleep(2500);  // ~400 Hz polling rate
    }
}

void start_uorb_publishers() {
    s_running.store(true);
    s_publisher_thread = std::thread(publisher_loop);
}

void stop_uorb_publishers() {
    s_running.store(false);
    if (s_publisher_thread.joinable()) {
        s_publisher_thread.join();
    }
}

// === قراءة حالة المركبة من الـ atomic cache (بدون uORB semaphore — آمن للاستدعاء من أي thread) ===

float get_vehicle_roll()     { return s_cached_roll.load(std::memory_order_relaxed); }
float get_vehicle_pitch()    { return s_cached_pitch.load(std::memory_order_relaxed); }
float get_vehicle_yaw()      { return s_cached_yaw.load(std::memory_order_relaxed); }
float get_vehicle_altitude() { return s_cached_altitude.load(std::memory_order_relaxed); }
bool  get_vehicle_armed()    { return s_cached_armed.load(std::memory_order_relaxed); }
int   get_vehicle_nav_state(){ return s_cached_nav_state.load(std::memory_order_relaxed); }
int   get_ekf_status()       { return s_cached_ekf_status.load(std::memory_order_relaxed); }

int get_airframe_id() {
    int32_t val = 0;
    param_t p = param_find("SYS_AUTOSTART");
    if (p != PARAM_INVALID) { param_get(p, &val); }
    return val;
}

// Change SYS_AUTOSTART from Kotlin UI.  Caller is responsible for restarting
// PX4 modules (via stopPX4 + startPX4) so the new airframe defaults are applied
// by the boot-time logic in px4_jni.cpp.
bool set_airframe_id(int id) {
    param_t p = param_find("SYS_AUTOSTART");
    if (p == PARAM_INVALID) { return false; }
    int32_t val = (int32_t)id;
    if (param_set(p, &val) != 0) { return false; }
    // Persist to parameter storage so the value survives the restart.
    param_save_default(true);
    return true;
}

// Board stubs for Android (مطلوبة من commander)
#include <px4_platform_common/board_common.h>

extern "C" {

int board_power_off(int status)
{
    return 0;
}

int board_register_power_state_notification_cb(power_button_state_notification_t cb)
{
    return 0;
}

} // extern "C"
