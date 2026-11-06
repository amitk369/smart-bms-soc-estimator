/**
 * @file bms_core.h
 * @brief Top-Level BMS Controller Engine Header
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef BMS_CORE_H
#define BMS_CORE_H

#include "bms_types.h"
#include "soc_estimator.h"
#include "cell_balancer.h"
#include "protection.h"

void bms_core_init(bms_pack_t *pack);
void bms_core_step(bms_pack_t *pack, float cell_voltages[BMS_NUM_CELLS], float pack_current_a, float cell_temps[BMS_NUM_CELLS], float dt_seconds);

#endif // BMS_CORE_H
