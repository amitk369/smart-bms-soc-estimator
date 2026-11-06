/**
 * @file cell_balancer.c
 * @brief Automated Passive Resistive Cell Balancing Engine
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "cell_balancer.h"

void cell_balancer_init(bms_pack_t *pack) {
    cell_balancer_disable_all(pack);
}

void cell_balancer_disable_all(bms_pack_t *pack) {
    for (int i = 0; i < BMS_NUM_CELLS; i++) {
        pack->cells[i].balancing_active = false;
        pack->cells[i].balancing_pwm_duty = 0;
    }
}

void cell_balancer_update(bms_pack_t *pack) {
    // Cell balancing is only permitted when pack is healthy and not in fault
    if (pack->fault_mask != FAULT_NONE) {
        cell_balancer_disable_all(pack);
        return;
    }

    // Do not balance during heavy discharge to avoid unnecessary heat
    if (pack->pack_current_a < -0.200f) {
        cell_balancer_disable_all(pack);
        return;
    }

    // Find min and max cell voltages
    float min_v = pack->cells[0].voltage_v;
    float max_v = pack->cells[0].voltage_v;

    for (int i = 1; i < BMS_NUM_CELLS; i++) {
        if (pack->cells[i].voltage_v < min_v) min_v = pack->cells[i].voltage_v;
        if (pack->cells[i].voltage_v > max_v) max_v = pack->cells[i].voltage_v;
    }

    pack->min_cell_voltage_v = min_v;
    pack->max_cell_voltage_v = max_v;
    pack->cell_delta_v = max_v - min_v;

    // Balancing condition:
    // Minimum cell voltage must be above BALANCING_MIN_VOLTAGE_V (3.80V)
    // AND delta_v must exceed threshold (15 mV)
    if (min_v < BALANCING_MIN_VOLTAGE_V || pack->cell_delta_v < BALANCING_DELTA_THRESH_V) {
        cell_balancer_disable_all(pack);
        return;
    }

    // Thermal throttling: If pack temperature > 42°C, scale down balancing duty cycle
    float thermal_duty_scaler = 1.0f;
    if (pack->max_temperature_c > 42.0f) {
        thermal_duty_scaler = (CELL_OT_CHARGE_LIMIT_C - pack->max_temperature_c) / 3.0f;
        if (thermal_duty_scaler < 0.0f) thermal_duty_scaler = 0.0f;
    }

    // Engage passive bleed resistors for higher cells
    for (int i = 0; i < BMS_NUM_CELLS; i++) {
        float cell_excess = pack->cells[i].voltage_v - min_v;
        if (cell_excess > BALANCING_DELTA_THRESH_V) {
            pack->cells[i].balancing_active = true;
            
            // Proportional PWM duty cycle: 50% at 15mV, up to 80% max at 50mV excess
            float duty_norm = 0.5f + (cell_excess - BALANCING_DELTA_THRESH_V) * (0.30f / 0.035f);
            if (duty_norm > BALANCING_DUTY_CYCLE_MAX) duty_norm = BALANCING_DUTY_CYCLE_MAX;
            
            duty_norm *= thermal_duty_scaler;
            pack->cells[i].balancing_pwm_duty = (uint8_t)(duty_norm * 255.0f);
        } else {
            pack->cells[i].balancing_active = false;
            pack->cells[i].balancing_pwm_duty = 0;
        }
    }
}
