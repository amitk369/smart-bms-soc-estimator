/**
 * @file test_bms.c
 * @brief Unit Tests for BMS Core: SoC, Cell Balancing & Protection
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../firmware/include/bms_core.h"

void test_soc_lookup() {
    printf("[TEST] Testing OCV to SoC lookup...\n");
    float soc_empty = ocv_to_soc_lookup(3.00f);
    assert(fabsf(soc_empty - 0.0f) < 0.1f);

    float soc_full = ocv_to_soc_lookup(4.20f);
    assert(fabsf(soc_full - 100.0f) < 0.1f);

    float soc_nominal = ocv_to_soc_lookup(3.720f);
    assert(fabsf(soc_nominal - 30.0f) < 1.0f);
    printf("  -> OCV table lookup PASSED.\n");
}

void test_cell_balancing() {
    printf("[TEST] Testing Passive Cell Balancing...\n");
    bms_pack_t pack;
    bms_core_init(&pack);

    // Set unbalanced cells above 3.80V
    float cell_v[4] = { 4.10f, 4.02f, 4.08f, 4.02f }; // Min is 4.02V, Cell 0 & Cell 2 exceed delta (15mV)
    float cell_t[4] = { 25.0f, 25.0f, 25.0f, 25.0f };

    bms_core_step(&pack, cell_v, 0.0f, cell_t, 0.05f);

    assert(pack.cells[0].balancing_active == true);
    assert(pack.cells[1].balancing_active == false); // At minimum, should not bleed
    assert(pack.cells[2].balancing_active == true);
    assert(pack.cells[3].balancing_active == false);
    printf("  -> Passive cell balancing trigger PASSED.\n");
}

void test_protection_ovp_uvp() {
    printf("[TEST] Testing Over-Voltage & Under-Voltage Protection...\n");
    bms_pack_t pack;
    bms_core_init(&pack);

    // Test OVP: Cell 1 hits 4.28V (> 4.25V threshold)
    float ovp_v[4] = { 4.28f, 4.10f, 4.10f, 4.10f };
    float cell_t[4] = { 25.0f, 25.0f, 25.0f, 25.0f };

    bms_core_step(&pack, ovp_v, 1.0f, cell_t, 0.05f);
    assert(pack.fault_mask & FAULT_CELL_OVP);
    assert(pack.charge_contactor_closed == false); // Must disconnect charger

    // Test UVP: Cell 2 drops to 2.75V (< 2.80V threshold)
    bms_core_init(&pack);
    float uvp_v[4] = { 3.50f, 2.75f, 3.50f, 3.50f };
    bms_core_step(&pack, uvp_v, -2.0f, cell_t, 0.05f);
    assert(pack.fault_mask & FAULT_CELL_UVP);
    assert(pack.discharge_contactor_closed == false); // Must disconnect load
    printf("  -> OVP/UVP contactor cutoff PASSED.\n");
}

void test_thermal_runaway_cutoff() {
    printf("[TEST] Testing Thermal Runaway Cutoff...\n");
    bms_pack_t pack;
    bms_core_init(&pack);

    float norm_v[4] = { 3.80f, 3.80f, 3.80f, 3.80f };
    float hot_t[4] = { 62.0f, 30.0f, 30.0f, 30.0f }; // Exceeds 60°C discharge limit

    bms_core_step(&pack, norm_v, -4.0f, hot_t, 0.05f);
    assert(pack.fault_mask & FAULT_OVER_TEMP_DISCHG);
    assert(pack.discharge_contactor_closed == false);
    printf("  -> Thermal runaway isolation PASSED.\n");
}

int main() {
    printf("=========================================\n");
    printf(" Running BMS Firmware Unit Test Harness \n");
    printf("=========================================\n");
    test_soc_lookup();
    test_cell_balancing();
    test_protection_ovp_uvp();
    test_thermal_runaway_cutoff();
    printf("=========================================\n");
    printf(" ALL BMS UNIT TESTS PASSED SUCCESSFULLY! \n");
    printf("=========================================\n");
    return 0;
}
