/**
 * @file protection.c
 * @brief Safety & Fault Protection State Machine Implementation
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "protection.h"
#include "cell_balancer.h"

void protection_init(bms_pack_t *pack) {
    pack->fault_mask = FAULT_NONE;
    pack->charge_contactor_closed = true;
    pack->discharge_contactor_closed = true;
}

bool protection_is_fault_active(const bms_pack_t *pack) {
    return pack->fault_mask != FAULT_NONE;
}

void protection_check(bms_pack_t *pack) {
    uint32_t active_faults = FAULT_NONE;

    // 1. Cell Voltage Protection
    for (int i = 0; i < BMS_NUM_CELLS; i++) {
        float v = pack->cells[i].voltage_v;

        // Over-Voltage (OVP)
        if (v >= CELL_OVP_THRESHOLD_V) {
            active_faults |= FAULT_CELL_OVP;
        } else if (pack->fault_mask & FAULT_CELL_OVP) {
            // Keep fault until cell drops below hysteresis release voltage
            if (v > CELL_OVP_RELEASE_V) {
                active_faults |= FAULT_CELL_OVP;
            }
        }

        // Under-Voltage (UVP)
        if (v <= CELL_UVP_THRESHOLD_V) {
            active_faults |= FAULT_CELL_UVP;
        } else if (pack->fault_mask & FAULT_CELL_UVP) {
            // Keep fault until cell recovers above hysteresis release voltage
            if (v < CELL_UVP_RELEASE_V) {
                active_faults |= FAULT_CELL_UVP;
            }
        }
    }

    // 2. Current Protection
    float i_pack = pack->pack_current_a;

    // Short circuit detection (instant trip)
    if (i_pack <= -SHORT_CIRCUIT_CURRENT_A) {
        active_faults |= FAULT_SHORT_CIRCUIT;
    }

    // Over-current discharge
    if (i_pack <= -MAX_CONTINUOUS_DISCHARGE_A) {
        active_faults |= FAULT_OVER_CURRENT_DISCHG;
    }

    // Over-current charge
    if (i_pack >= MAX_CONTINUOUS_CHARGE_A) {
        active_faults |= FAULT_OVER_CURRENT_CHARGE;
    }

    // 3. Thermal Protection
    float t_max = pack->max_temperature_c;
    float t_min = pack->min_temperature_c;

    // Discharge over-temperature (Thermal Runaway Prevention)
    if (t_max >= CELL_OT_DISCHARGE_LIMIT_C) {
        active_faults |= FAULT_OVER_TEMP_DISCHG;
    } else if (pack->fault_mask & FAULT_OVER_TEMP_DISCHG) {
        if (t_max > (CELL_OT_DISCHARGE_LIMIT_C - CELL_TEMP_RELEASE_DELTA_C)) {
            active_faults |= FAULT_OVER_TEMP_DISCHG;
        }
    }

    // Charge over-temperature
    if (i_pack > 0.050f && t_max >= CELL_OT_CHARGE_LIMIT_C) {
        active_faults |= FAULT_OVER_TEMP_CHARGE;
    }

    // Charge under-temperature (Lithium plating hazard below 0°C)
    if (i_pack > 0.050f && t_min <= CELL_UT_CHARGE_LIMIT_C) {
        active_faults |= FAULT_UNDER_TEMP_CHARGE;
    }

    // Update fault mask
    pack->fault_mask = active_faults;

    // Contactors & Switch Safety Actuation
    if (pack->fault_mask & (FAULT_CELL_OVP | FAULT_OVER_CURRENT_CHARGE | FAULT_OVER_TEMP_CHARGE | FAULT_UNDER_TEMP_CHARGE)) {
        pack->charge_contactor_closed = false; // Isolate charger
    } else {
        pack->charge_contactor_closed = true;
    }

    if (pack->fault_mask & (FAULT_CELL_UVP | FAULT_OVER_CURRENT_DISCHG | FAULT_SHORT_CIRCUIT | FAULT_OVER_TEMP_DISCHG)) {
        pack->discharge_contactor_closed = false; // Isolate load
    } else {
        pack->discharge_contactor_closed = true;
    }

    // If fatal fault, disable balancing immediately
    if (pack->fault_mask != FAULT_NONE) {
        pack->state = BMS_STATE_FAULT;
        cell_balancer_disable_all(pack);
    } else {
        if (pack->pack_current_a > 0.05f) {
            pack->state = BMS_STATE_CHARGING;
        } else if (pack->pack_current_a < -0.05f) {
            pack->state = BMS_STATE_DISCHARGING;
        } else {
            pack->state = BMS_STATE_STANDBY;
        }
    }
}
