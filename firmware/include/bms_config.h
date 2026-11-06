/**
 * @file bms_config.h
 * @brief Hardware & Operational Calibration Constants for 4S Li-ion BMS
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#define BMS_NUM_CELLS             4       // 4S Li-ion Battery Pack
#define BMS_NOMINAL_CAPACITY_AH   2.60f   // Samsung 18650-26J nominal capacity (2600 mAh)
#define BMS_NOMINAL_VOLTAGE_V     3.70f   // Nominal cell voltage
#define BMS_PACK_NOMINAL_V        14.80f  // 4 * 3.7V

// Voltage Protection Thresholds (in Volts)
#define CELL_OVP_THRESHOLD_V      4.25f   // Over-Voltage Protection (Hard Cutoff)
#define CELL_OVP_RELEASE_V        4.15f   // Over-Voltage Release Hysteresis
#define CELL_UVP_THRESHOLD_V      2.80f   // Under-Voltage Protection (Hard Cutoff)
#define CELL_UVP_RELEASE_V        3.00f   // Under-Voltage Release Hysteresis

// Temperature Protection Limits (in Celsius)
#define CELL_OT_CHARGE_LIMIT_C    45.0f   // Over-temperature limit during charge
#define CELL_OT_DISCHARGE_LIMIT_C 60.0f   // Over-temperature limit during discharge
#define CELL_UT_CHARGE_LIMIT_C     0.0f   // Under-temperature limit during charge (prevent dendrites)
#define CELL_TEMP_RELEASE_DELTA_C  5.0f   // Thermal hysteresis margin

// Current Protection Thresholds (in Amperes)
#define MAX_CONTINUOUS_DISCHARGE_A 10.0f  // Max continuous discharge current
#define SHORT_CIRCUIT_CURRENT_A    25.0f  // Instantaneous hardware trip current
#define MAX_CONTINUOUS_CHARGE_A    3.0f   // Max continuous charge current

// Passive Balancing Thresholds
#define BALANCING_MIN_VOLTAGE_V   3.80f   // Only balance when cells are near top-of-charge
#define BALANCING_DELTA_THRESH_V  0.015f  // 15 mV threshold between max and min cell
#define BALANCING_BLEED_RESISTOR  39.0f   // 39 Ohm shunt bleed resistor (approx 100mA bleed)
#define BALANCING_DUTY_CYCLE_MAX  0.80f   // 80% max PWM duty cycle to manage thermal dissipation

// Timing Configurations
#define BMS_SAMPLE_PERIOD_MS      50      // 20 Hz primary loop
#define TELEMETRY_INTERVAL_MS     1000    // 1 Hz external telemetry rate
#define SOC_OCV_REST_TIME_SEC     300     // 5 minutes rest required for stable OCV update

#endif // BMS_CONFIG_H
