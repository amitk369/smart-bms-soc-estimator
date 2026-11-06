#!/usr/bin/env python3
"""
Dynamic Simulation & Verification Engine for 4S Li-ion Battery Management System (BMS)
Equivalent Circuit Model (Thevenin 1-RC) + Coulomb Counting & OCV Lookup Validation
Author: Amit Kumar (NIT Patna - Electrical Engineering)
"""

import math
import json
import os

class LiIonCellModel:
    """Thevenin 1-RC Equivalent Circuit Model of a Li-ion Cell."""
    def __init__(self, capacity_ah=2.6, initial_soc=1.0, r0=0.035, r1=0.025, c1=1500.0):
        self.capacity_as = capacity_ah * 3600.0  # Capacity in Ampere-seconds
        self.soc = initial_soc                   # True SoC [0.0 - 1.0]
        self.r0 = r0                             # Ohmic internal resistance (Ohms)
        self.r1 = r1                             # Polarization resistance (Ohms)
        self.c1 = c1                             # Polarization capacitance (Farads)
        self.v_rc = 0.0                          # Polarization capacitor voltage
        self.temperature_c = 25.0

    def ocv_lookup(self, soc):
        """Standard NMC 18650 Open-Circuit Voltage polynomial fit."""
        s = min(max(soc, 0.0), 1.0)
        # 6th-order empirical OCV curve
        return (3.00 + 1.25 * s - 0.75 * (s**2) + 1.85 * (s**3) 
                - 1.82 * (s**4) + 0.67 * (s**5))

    def step(self, current_a, dt):
        """Simulate one time step with given load current (positive = charge, negative = discharge)."""
        # True Coulomb integration
        delta_q = current_a * dt
        self.soc += delta_q / self.capacity_as
        self.soc = min(max(self.soc, 0.0), 1.0)

        # Differential equation for RC parallel branch: dV_rc/dt = -V_rc/(R1*C1) + I/C1
        tau = self.r1 * self.c1
        self.v_rc = self.v_rc * math.exp(-dt / tau) + (current_a * self.r1) * (1.0 - math.exp(-dt / tau))

        # Terminal voltage
        v_ocv = self.ocv_lookup(self.soc)
        v_terminal = v_ocv + (current_a * self.r0) + self.v_rc

        # Thermal dissipation: Q_gen = I^2 * R0 + V_rc^2 / R1
        p_loss = (current_a**2) * self.r0 + (self.v_rc**2) / max(self.r1, 1e-4)
        c_th = 45.0  # Thermal heat capacity (J/K)
        r_th = 18.0  # Thermal resistance to ambient (K/W)
        self.temperature_c += (dt / c_th) * (p_loss - (self.temperature_c - 25.0) / r_th)

        return v_terminal, v_ocv

def run_simulation(duration_sec=3600, dt=0.5):
    """Run dynamic discharge profile with pulse loads and rest periods."""
    cells = [
        LiIonCellModel(capacity_ah=2.60, initial_soc=0.98),
        LiIonCellModel(capacity_ah=2.58, initial_soc=0.97),
        LiIonCellModel(capacity_ah=2.62, initial_soc=0.99),
        LiIonCellModel(capacity_ah=2.55, initial_soc=0.96)
    ]

    # Coulomb counter state
    est_coulombs = 0.98 * 2.60 * 3600.0
    nominal_coulombs = 2.60 * 3600.0
    rest_seconds = 0
    
    time_pts = []
    true_soc_pts = []
    est_soc_pts = []
    errors = []
    current_pts = []
    pack_v_pts = []

    steps = int(duration_sec / dt)
    print(f"Running Li-ion BMS Dynamic Simulation: {steps} steps ({duration_sec}s)...")

    for k in range(steps):
        t = k * dt

        # Dynamic discharge profile:
        # Alternating pulses of 2.0A, 5.0A, 0.5A, with occasional 300s rest periods
        cycle_phase = (t % 600.0)
        if cycle_phase < 120.0:
            i_load = -2.5  # 2.5A discharge
        elif cycle_phase < 240.0:
            i_load = -4.8  # High dynamic pulse
        elif cycle_phase < 360.0:
            i_load = -1.2  # Moderate load
        elif cycle_phase < 450.0:
            i_load = 0.0   # Rest period 1
        elif cycle_phase < 550.0:
            i_load = 1.5   # Regenerative braking pulse
        else:
            i_load = 0.0   # Rest period 2

        # Step cell physics
        v_terms = []
        for cell in cells:
            vt, _ = cell.step(i_load, dt)
            v_terms.append(vt)

        pack_v = sum(v_terms)
        avg_v = pack_v / 4.0
        true_pack_soc = (sum(c.soc for c in cells) / 4.0) * 100.0

        # Coulomb Counting with intentional simulated sensor noise & drift
        current_meas = i_load + 0.012  # 12mA ADC offset drift
        est_coulombs += current_meas * dt
        est_coulombs = max(0.0, min(est_coulombs, nominal_coulombs))
        est_soc_pure = (est_coulombs / nominal_coulombs) * 100.0

        if abs(i_load) < 0.05:
            rest_seconds += dt
        else:
            rest_seconds = 0

        # OCV Lookup & Drift Correction during rest
        if rest_seconds >= 200.0:
            # Cell has relaxed; calibrate Coulomb counter to OCV
            ocv_estimated_soc = min(max((avg_v - 3.0) / 1.2 * 100.0, 0.0), 100.0)
            est_coulombs = 0.95 * est_coulombs + 0.05 * (ocv_estimated_soc / 100.0 * nominal_coulombs)

        est_soc = (est_coulombs / nominal_coulombs) * 100.0
        err = abs(est_soc - true_pack_soc)

        time_pts.append(t)
        true_soc_pts.append(true_pack_soc)
        est_soc_pts.append(est_soc)
        errors.append(err)
        current_pts.append(i_load)
        pack_v_pts.append(pack_v)

    max_err = max(errors)
    mean_err = sum(errors) / len(errors)

    print("\n--- SIMULATION RESULTS ---")
    print(f"Total Simulation Time: {duration_sec} s ({duration_sec/3600:.1f} hours)")
    print(f"Mean SoC Estimation Error: {mean_err:.2f}%")
    print(f"Maximum SoC Estimation Error: {max_err:.2f}% (Target: < 3.0%)")
    assert max_err < 3.0, f"Error {max_err}% exceeded 3% specification!"
    print("SUCCESS: SoC Estimation verified within <3% error bound under dynamic discharge!")

    # Save summary report
    summary = {
        "status": "PASSED",
        "duration_sec": duration_sec,
        "max_error_pct": round(max_err, 3),
        "mean_error_pct": round(mean_err, 3),
        "target_error_pct": "< 3.0%",
        "final_pack_voltage_v": round(pack_v, 2),
        "cells": [round(c.soc * 100, 1) for c in cells]
    }
    with open(os.path.join(os.path.dirname(__file__), "sim_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

if __name__ == "__main__":
    run_simulation()
