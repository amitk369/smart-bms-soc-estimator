/**
 * @file protection.h
 * @brief Safety & Fault Protection State Machine Header
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#include "bms_types.h"

void protection_init(bms_pack_t *pack);
void protection_check(bms_pack_t *pack);
bool protection_is_fault_active(const bms_pack_t *pack);

#endif // PROTECTION_H
