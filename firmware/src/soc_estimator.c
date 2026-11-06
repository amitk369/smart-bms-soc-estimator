/**
 * @file soc_estimator.c
 * @brief Coulomb Counting + OCV Lookup with Drift Compensation
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "soc_estimator.h"
#include <stddef.h>
#include <math.h>

// 21-point Empirical OCV vs SoC Calibration Table (Samsung 18650 Li-ion 2600mAh, 25°C)
typedef struct {
    float soc; // 0.0 to 100.0%
    float ocv; // Volts
} ocv_point_t;

static const ocv_point_t OCV_TABLE[] = {
    {0.0f,   3.000f},
    {5.0f,   3.380f},
    {10.0f,  3.520f},
    {15.0f,  3.605f},
    {20.0f,  3.660f},
    {25.0f,  3.695f},
    {30.0f,  3.720f},
    {35.0f,  3.742f},
    {40.0f,  3.765f},
    {45.0f,  3.790f},
    {50.0f,  3.818f},
    {55.0f,  3.848f},
    {60.0f,  3.885f},
    {65.0f,  3.925f},
    {70.0f,  3.970f},
    {75.0f,  4.015f},
    {80.0f,  4.062f},
    {85.0f,  4.105f},
    {90.0f,  4.148f},
    {95.0f,  4.185f},
    {100.0f, 4.200f}
};

#define OCV_TABLE_LEN (sizeof(OCV_TABLE) / sizeof(ocv_point_t))

float ocv_to_soc_lookup(float ocv_volts) {
    if (ocv_volts <= OCV_TABLE[0].ocv) return 0.0f;
    if (ocv_volts >= OCV_TABLE[OCV_TABLE_LEN - 1].ocv) return 100.0f;

    for (size_t i = 0; i < OCV_TABLE_LEN - 1; i++) {
        if (ocv_volts >= OCV_TABLE[i].ocv && ocv_volts <= OCV_TABLE[i + 1].ocv) {
            float v_low = OCV_TABLE[i].ocv;
            float v_high = OCV_TABLE[i + 1].ocv;
            float s_low = OCV_TABLE[i].soc;
            float s_high = OCV_TABLE[i + 1].soc;
            
            // Piecewise linear interpolation
            return s_low + (ocv_volts - v_low) * (s_high - s_low) / (v_high - v_low);
        }
    }
    return 50.0f;
}

float soc_to_ocv_lookup(float soc_percent) {
    if (soc_percent <= 0.0f) return OCV_TABLE[0].ocv;
    if (soc_percent >= 100.0f) return OCV_TABLE[OCV_TABLE_LEN - 1].ocv;

    for (size_t i = 0; i < OCV_TABLE_LEN - 1; i++) {
        if (soc_percent >= OCV_TABLE[i].soc && soc_percent <= OCV_TABLE[i + 1].soc) {
            float s_low = OCV_TABLE[i].soc;
            float s_high = OCV_TABLE[i + 1].soc;
            float v_low = OCV_TABLE[i].ocv;
            float v_high = OCV_TABLE[i + 1].ocv;

            return v_low + (soc_percent - s_low) * (v_high - v_low) / (s_high - s_low);
        }
    }
    return 3.7f;
}

void soc_estimator_init(bms_pack_t *pack, float initial_avg_voltage) {
    float estimated_soc = ocv_to_soc_lookup(initial_avg_voltage);
    pack->soc.soc_percent = estimated_soc;
    pack->soc.soc_coulomb_percent = estimated_soc;
    pack->soc.soc_ocv_percent = estimated_soc;
    pack->soc.coulombic_efficiency = 0.985f; // 98.5% charge efficiency
    pack->soc.accumulated_coulombs = (estimated_soc / 100.0f) * (BMS_NOMINAL_CAPACITY_AH * 3600.0f);
    pack->soc.rest_timer_sec = 300; // Assume rested at boot
    pack->soc.ocv_valid = true;
}

void soc_estimator_update(bms_pack_t *pack, float dt_seconds) {
    float current_a = pack->pack_current_a;
    float avg_voltage = pack->pack_voltage_v / (float)BMS_NUM_CELLS;
    
    // Efficiency factor: 1.0 on discharge, ~0.985 on charge
    float eta = (current_a >= 0.0f) ? pack->soc.coulombic_efficiency : 1.0f;
    
    // Coulomb integration: delta_Q = eta * I * dt (Ampere-seconds)
    float delta_coulombs = eta * current_a * dt_seconds;
    pack->soc.accumulated_coulombs += delta_coulombs;

    // Capacity bound check
    float max_coulombs = BMS_NOMINAL_CAPACITY_AH * 3600.0f;
    if (pack->soc.accumulated_coulombs < 0.0f) {
        pack->soc.accumulated_coulombs = 0.0f;
    } else if (pack->soc.accumulated_coulombs > max_coulombs) {
        pack->soc.accumulated_coulombs = max_coulombs;
    }

    pack->soc.soc_coulomb_percent = (pack->soc.accumulated_coulombs / max_coulombs) * 100.0f;

    // Check for rest conditions (|current| < 50 mA)
    if (fabsf(current_a) < 0.050f) {
        // Increment rest timer (sample period converted to seconds)
        static float accumulated_rest_dt = 0.0f;
        accumulated_rest_dt += dt_seconds;
        if (accumulated_rest_dt >= 1.0f) {
            pack->soc.rest_timer_sec++;
            accumulated_rest_dt -= 1.0f;
        }
    } else {
        pack->soc.rest_timer_sec = 0;
        pack->soc.ocv_valid = false;
    }

    // After 5 minutes of rest, cell polarization decays and terminal voltage ~= OCV
    if (pack->soc.rest_timer_sec >= SOC_OCV_REST_TIME_SEC) {
        pack->soc.ocv_valid = true;
        pack->soc.soc_ocv_percent = ocv_to_soc_lookup(avg_voltage);

        // Extended Kalman / Complementary gentle drift correction:
        // Blend Coulomb counting toward OCV reference with smoothing constant alpha
        const float alpha = 0.02f; // Smooth 2% step correction per update
        pack->soc.soc_percent = (1.0f - alpha) * pack->soc.soc_coulomb_percent + alpha * pack->soc.soc_ocv_percent;
        
        // Re-anchor Coulomb counter to prevent cumulative integration drift
        pack->soc.accumulated_coulombs = (pack->soc.soc_percent / 100.0f) * max_coulombs;
    } else {
        // In dynamic discharge/charge, follow high-bandwidth Coulomb Counting
        pack->soc.soc_percent = pack->soc.soc_coulomb_percent;
    }
}
