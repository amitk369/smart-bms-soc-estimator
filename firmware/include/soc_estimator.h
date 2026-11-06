/**
 * @file soc_estimator.h
 * @brief State-of-Charge (SoC) Estimator Header
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

#include "bms_types.h"

// Initialize the SoC estimator with initial cell voltage
void soc_estimator_init(bms_pack_t *pack, float initial_avg_voltage);

// Periodic update step (called at BMS_SAMPLE_PERIOD_MS)
void soc_estimator_update(bms_pack_t *pack, float dt_seconds);

// Lookup SoC percentage given Open-Circuit Voltage
float ocv_to_soc_lookup(float ocv_volts);

// Inverse lookup: OCV given SoC percentage
float soc_to_ocv_lookup(float soc_percent);

#endif // SOC_ESTIMATOR_H
