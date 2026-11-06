/**
 * @file cell_balancer.h
 * @brief Automated Passive Resistive Cell Balancing Header
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef CELL_BALANCER_H
#define CELL_BALANCER_H

#include "bms_types.h"

void cell_balancer_init(bms_pack_t *pack);
void cell_balancer_update(bms_pack_t *pack);
void cell_balancer_disable_all(bms_pack_t *pack);

#endif // CELL_BALANCER_H
