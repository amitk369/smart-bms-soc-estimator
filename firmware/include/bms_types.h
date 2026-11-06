/**
 * @file bms_types.h
 * @brief Data Structures, Enums, and State Definitions
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

// System Operating States
typedef enum {
    BMS_STATE_INIT = 0,
    BMS_STATE_STANDBY,
    BMS_STATE_CHARGING,
    BMS_STATE_DISCHARGING,
    BMS_STATE_BALANCING,
    BMS_STATE_FAULT
} bms_state_t;

// Fault Bitmask Flags
typedef enum {
    FAULT_NONE                  = 0x0000,
    FAULT_CELL_OVP              = 0x0001,
    FAULT_CELL_UVP              = 0x0002,
    FAULT_OVER_CURRENT_CHARGE   = 0x0004,
    FAULT_OVER_CURRENT_DISCHG   = 0x0008,
    FAULT_SHORT_CIRCUIT         = 0x0010,
    FAULT_OVER_TEMP_CHARGE      = 0x0020,
    FAULT_OVER_TEMP_DISCHG      = 0x0040,
    FAULT_UNDER_TEMP_CHARGE     = 0x0080,
    FAULT_CELL_IMBALANCE        = 0x0100,
    FAULT_COMMUNICATION_LOSS    = 0x0200
} bms_fault_flags_t;

// Individual Cell Telemetry & Status
typedef struct {
    float voltage_v;
    float filtered_voltage_v;
    float temperature_c;
    float internal_resistance_mOhm;
    bool balancing_active;
    uint8_t balancing_pwm_duty; // 0-255
} cell_status_t;

// State of Charge Estimation Data
typedef struct {
    float soc_percent;          // Filtered hybrid estimate (0 - 100%)
    float soc_coulomb_percent;  // Pure Coulomb counting integrator
    float soc_ocv_percent;      // OCV table lookup result
    float accumulated_coulombs; // Ampere-seconds integrated
    float coulombic_efficiency; // Typical: 0.985 for Li-ion
    uint32_t rest_timer_sec;    // Seconds current has been near zero (< 50mA)
    bool ocv_valid;             // True if pack has relaxed enough for reliable OCV
} soc_estimator_t;

// Master BMS Pack State
typedef struct {
    cell_status_t cells[BMS_NUM_CELLS];
    float pack_voltage_v;
    float pack_current_a;       // Positive = charging, Negative = discharging
    float max_cell_voltage_v;
    float min_cell_voltage_v;
    float cell_delta_v;
    float max_temperature_c;
    float min_temperature_c;
    
    bms_state_t state;
    uint32_t fault_mask;
    
    soc_estimator_t soc;
    
    bool charge_contactor_closed;
    bool discharge_contactor_closed;
    
    uint32_t uptime_ms;
} bms_pack_t;

#endif // BMS_TYPES_H
