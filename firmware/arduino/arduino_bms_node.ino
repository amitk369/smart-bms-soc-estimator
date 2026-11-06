/**
 * @file arduino_bms_node.ino
 * @brief Arduino 4S Battery Management System Firmware Node
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "bms_core.h"

// Pin Assignments
const int PIN_CELL1_ADC = A0;
const int PIN_CELL2_ADC = A1;
const int PIN_CELL3_ADC = A2;
const int PIN_CELL4_ADC = A3;
const int PIN_CURRENT_SHUNT = A4;
const int PIN_NTC_TEMP = A5;

const int PIN_BAL_CELL1 = 3;  // PWM pin for Cell 1 bleed MOSFET
const int PIN_BAL_CELL2 = 5;  // PWM pin for Cell 2 bleed MOSFET
const int PIN_BAL_CELL3 = 6;  // PWM pin for Cell 3 bleed MOSFET
const int PIN_BAL_CELL4 = 9;  // PWM pin for Cell 4 bleed MOSFET

const int PIN_CHARGE_RELAY = 7;
const int PIN_DISCHARGE_RELAY = 8;
const int PIN_STATUS_LED = 13;

static bms_pack_t pack;
unsigned long last_loop_time_ms = 0;
unsigned long last_telemetry_time_ms = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    pinMode(PIN_CHARGE_RELAY, OUTPUT);
    pinMode(PIN_DISCHARGE_RELAY, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    
    pinMode(PIN_BAL_CELL1, OUTPUT);
    pinMode(PIN_BAL_CELL2, OUTPUT);
    pinMode(PIN_BAL_CELL3, OUTPUT);
    pinMode(PIN_BAL_CELL4, OUTPUT);

    // Initial safe contactor state
    digitalWrite(PIN_CHARGE_RELAY, HIGH);
    digitalWrite(PIN_DISCHARGE_RELAY, HIGH);

    bms_core_init(&pack);
    Serial.println(F("=================================================="));
    Serial.println(F("  NIT PATNA - SMART BMS & SoC ESTIMATOR v2.4     "));
    Serial.println(F("  Author: Amit Kumar (Electrical Engineering)    "));
    Serial.println(F("=================================================="));
    last_loop_time_ms = millis();
}

void loop() {
    unsigned long now = millis();
    unsigned long elapsed = now - last_loop_time_ms;

    if (elapsed >= BMS_SAMPLE_PERIOD_MS) {
        float dt = elapsed / 1000.0f;
        last_loop_time_ms = now;

        // Read voltage dividers (calibrated conversion constants)
        // Cell 1: Direct 0-4.2V divider
        // Cell 2: Differential tapped tap2 - tap1
        // Cell 3: Differential tapped tap3 - tap2
        // Cell 4: Differential tapped tap4 - tap3
        float raw_c1 = (analogRead(PIN_CELL1_ADC) * (5.0f / 1023.0f)) * 1.000f;
        float raw_c2 = (analogRead(PIN_CELL2_ADC) * (5.0f / 1023.0f)) * 2.015f - raw_c1;
        float raw_c3 = (analogRead(PIN_CELL3_ADC) * (5.0f / 1023.0f)) * 3.025f - (raw_c1 + raw_c2);
        float raw_c4 = (analogRead(PIN_CELL4_ADC) * (5.0f / 1023.0f)) * 4.040f - (raw_c1 + raw_c2 + raw_c3);

        float cell_volts[4] = { raw_c1, raw_c2, raw_c3, raw_c4 };

        // Shunt current sensor (ACS712-20A: 2.5V center, 100mV/A)
        float shunt_v = (analogRead(PIN_CURRENT_SHUNT) * (5.0f / 1023.0f));
        float current_a = (shunt_v - 2.500f) / 0.100f;

        // NTC thermistor temperature conversion (Steinhart-Hart equation)
        float ntc_adc = analogRead(PIN_NTC_TEMP);
        float r_ntc = 10000.0f * (1023.0f / ntc_adc - 1.0f);
        float steinhart = r_ntc / 10000.0f; // (R/Ro)
        steinhart = log(steinhart); // ln(R/Ro)
        steinhart /= 3950.0f; // 1/B * ln(R/Ro)
        steinhart += 1.0f / (25.0f + 273.15f); // + (1/To)
        steinhart = 1.0f / steinhart; // Invert
        float temp_c = steinhart - 273.15f; // convert to C

        float cell_temps[4] = { temp_c, temp_c, temp_c, temp_c };

        // Execute BMS step
        bms_core_step(&pack, cell_volts, current_a, cell_temps, dt);

        // Hardware actuation
        digitalWrite(PIN_CHARGE_RELAY, pack.charge_contactor_closed ? HIGH : LOW);
        digitalWrite(PIN_DISCHARGE_RELAY, pack.discharge_contactor_closed ? HIGH : LOW);

        analogWrite(PIN_BAL_CELL1, pack.cells[0].balancing_pwm_duty);
        analogWrite(PIN_BAL_CELL2, pack.cells[1].balancing_pwm_duty);
        analogWrite(PIN_BAL_CELL3, pack.cells[2].balancing_pwm_duty);
        analogWrite(PIN_BAL_CELL4, pack.cells[3].balancing_pwm_duty);

        digitalWrite(PIN_STATUS_LED, (pack.fault_mask == FAULT_NONE) ? HIGH : (millis() % 500 < 250));
    }

    // 1 Hz Telemetry Serial Output
    if (now - last_telemetry_time_ms >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_time_ms = now;
        Serial.print(F("{\"pack_v\":")); Serial.print(pack.pack_voltage_v, 2);
        Serial.print(F(",\"current_a\":")); Serial.print(pack.pack_current_a, 2);
        Serial.print(F(",\"soc_pct\":")); Serial.print(pack.soc.soc_percent, 1);
        Serial.print(F(",\"delta_mv\":")); Serial.print(pack.cell_delta_v * 1000.0f, 1);
        Serial.print(F(",\"fault\":")); Serial.print(pack.fault_mask);
        Serial.print(F(",\"cells\":["));
        for (int i = 0; i < 4; i++) {
            Serial.print(pack.cells[i].voltage_v, 3);
            if (i < 3) Serial.print(F(","));
        }
        Serial.println(F("]}"));
    }
}
