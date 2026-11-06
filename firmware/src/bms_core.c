/**
 * @file bms_core.c
 * @brief Top-Level BMS Control Logic Implementation
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "bms_core.h"

void bms_core_init(bms_pack_t *pack) {
    pack->uptime_ms = 0;
    pack->pack_voltage_v = 0.0f;
    pack->pack_current_a = 0.0f;
    pack->state = BMS_STATE_INIT;
    
    for (int i = 0; i < BMS_NUM_CELLS; i++) {
        pack->cells[i].voltage_v = BMS_NOMINAL_VOLTAGE_V;
        pack->cells[i].filtered_voltage_v = BMS_NOMINAL_VOLTAGE_V;
        pack->cells[i].temperature_c = 25.0f;
        pack->cells[i].internal_resistance_mOhm = 28.0f;
        pack->cells[i].balancing_active = false;
        pack->cells[i].balancing_pwm_duty = 0;
        pack->pack_voltage_v += BMS_NOMINAL_VOLTAGE_V;
    }

    protection_init(pack);
    cell_balancer_init(pack);
    soc_estimator_init(pack, BMS_NOMINAL_VOLTAGE_V);
    
    pack->state = BMS_STATE_STANDBY;
}

void bms_core_step(bms_pack_t *pack, float cell_voltages[BMS_NUM_CELLS], float pack_current_a, float cell_temps[BMS_NUM_CELLS], float dt_seconds) {
    pack->uptime_ms += (uint32_t)(dt_seconds * 1000.0f);
    pack->pack_current_a = pack_current_a;
    pack->pack_voltage_v = 0.0f;

    float t_max = -100.0f;
    float t_min = 100.0f;

    // Filter sensor readings (exponential moving average for ADC noise rejection)
    const float ema_alpha = 0.25f;
    for (int i = 0; i < BMS_NUM_CELLS; i++) {
        pack->cells[i].voltage_v = cell_voltages[i];
        pack->cells[i].filtered_voltage_v = (ema_alpha * cell_voltages[i]) + ((1.0f - ema_alpha) * pack->cells[i].filtered_voltage_v);
        pack->cells[i].temperature_c = cell_temps[i];
        pack->pack_voltage_v += cell_voltages[i];

        if (cell_temps[i] > t_max) t_max = cell_temps[i];
        if (cell_temps[i] < t_min) t_min = cell_temps[i];
    }

    pack->max_temperature_c = t_max;
    pack->min_temperature_c = t_min;

    // Step 1: Run Safety Protection Check
    protection_check(pack);

    // Step 2: Update SoC Estimation
    soc_estimator_update(pack, dt_seconds);

    // Step 3: Run Passive Balancing State Machine
    cell_balancer_update(pack);
}
