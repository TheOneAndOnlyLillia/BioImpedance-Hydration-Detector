#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const uint8_t  pin_adc          = A0;
const uint8_t  pin_led          = 13;
const uint8_t  pin_ad9833_fsync = 10;
const uint8_t  lcd_i2c_addr     = 0x27;
const uint8_t  lcd_cols         = 16;
const uint8_t  lcd_rows         = 2;

const float    ad9833_ref_hz    = 25000000.0f;
const float    excitation_hz    = 50000.0f;

const uint16_t meas_averages    = 32;
const uint16_t report_period_ms = 1000;

const float    vref_volt         = 5.0f;
const uint16_t adc_max           = 1023;
const float    ad8226_gain       = 100.0f;
const float    howland_current_a = 50e-6f;
const float    detector_scale    = 1.0f;

float          baseline_volt     = 2.5f;

const float z_at_fully_hydrated = 250.0f;
const float z_at_dehydrated     = 700.0f;
const float hydration_high_pct  = 65.0f;
const float hydration_low_pct   = 50.0f;

unsigned long last_report_ms = 0;
LiquidCrystal_I2C lcd(lcd_i2c_addr, lcd_cols, lcd_rows);
uint8_t spinner_frame = 0;

float dbg_raw_volt    = 0;
float dbg_signal_volt = 0;
float dbg_v_body      = 0;

byte bar_char1[8] = {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000};
byte bar_char2[8] = {0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000};
byte bar_char3[8] = {0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100};
byte bar_char4[8] = {0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110};
byte bar_char5[8] = {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};

void upload_bar_graph_chars() {
    lcd.createChar(1, bar_char1);
    lcd.createChar(2, bar_char2);
    lcd.createChar(3, bar_char3);
    lcd.createChar(4, bar_char4);
    lcd.createChar(5, bar_char5);
    lcd.home();
}

void draw_bar_row(uint8_t row, float percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    uint8_t total_pixels = (uint8_t)((percent / 100.0f) * (lcd_cols * 5) + 0.5f);
    uint8_t full_cells   = total_pixels / 5;
    uint8_t partial      = total_pixels - full_cells * 5;
    lcd.setCursor(0, row);
    for (uint8_t i = 0; i < lcd_cols; i++) {
        if (i < full_cells)                      lcd.write((uint8_t)5);
        else if (i == full_cells && partial > 0) lcd.write((uint8_t)partial);
        else                                     lcd.write(' ');
    }
}

void ad9833_write_word(uint16_t w) {
    digitalWrite(pin_ad9833_fsync, LOW);
    SPI.transfer((uint8_t)(w >> 8));
    SPI.transfer((uint8_t)(w & 0xFF));
    digitalWrite(pin_ad9833_fsync, HIGH);
}

void ad9833_set_frequency_sine(float freq_hz) {
    uint32_t freq_reg = (uint32_t)((freq_hz * 268435456.0f / ad9833_ref_hz) + 0.5f);
    uint16_t lsb14 = (uint16_t)(freq_reg & 0x3FFF);
    uint16_t msb14 = (uint16_t)((freq_reg >> 14) & 0x3FFF);

    ad9833_write_word(0x2100);
    ad9833_write_word(0x4000 | lsb14);
    ad9833_write_word(0x4000 | msb14);
    ad9833_write_word(0xC000);
    ad9833_write_word(0x2000);
}

void setup_ad9833() {
    pinMode(pin_ad9833_fsync, OUTPUT);
    digitalWrite(pin_ad9833_fsync, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE2));
    ad9833_set_frequency_sine(excitation_hz);
    SPI.endTransaction();
}

float read_detector_volts() {
    uint32_t acc = 0;
    for (uint16_t i = 0; i < meas_averages; i++) {
        acc += analogRead(pin_adc);
        delayMicroseconds(200);
    }
    float counts = (float)acc / meas_averages;
    return (counts / (float)adc_max) * vref_volt;
}

void tare_baseline() {
    baseline_volt = read_detector_volts();
}

float measure_impedance_ohms() {
    float v_detector = read_detector_volts();
    dbg_raw_volt = v_detector;

    float v_signal = v_detector - baseline_volt;
    if (v_signal < 0) v_signal = 0;
    dbg_signal_volt = v_signal;

    float v_buffer_out = v_signal / detector_scale;
    float v_body = v_buffer_out / ad8226_gain;
    dbg_v_body = v_body;

    float z_ohms = v_body / howland_current_a;
    return z_ohms;
}

float impedance_to_hydration_pct(float z_ohms) {
    float slope = (hydration_high_pct - hydration_low_pct) /
                  (z_at_fully_hydrated - z_at_dehydrated);
    float pct = hydration_low_pct + slope * (z_ohms - z_at_dehydrated);
    if (pct < 30.0f) pct = 30.0f;
    if (pct > 80.0f) pct = 80.0f;
    return pct;
}

void setup() {
    Serial.begin(115200);

    lcd.init();
    lcd.backlight();
    upload_bar_graph_chars();
    lcd.clear();



    setup_ad9833();

    
    delay(1500);
    tare_baseline();
    Serial.print(F("Baseline (no-signal) = "));
    Serial.print(baseline_volt, 3);
    Serial.println(F(" V"));

    if (howland_current_a == 50e-6f) {
        Serial.println(F("WARNING: injected current is an ESTIMATE."));
        Serial.println(F("Measure it on a 1k load and dont forget to update howland_current_a."));
    }

    Serial.println(F("Calibration points (edit in code after calibrating):"));
    Serial.print(F("  ")); Serial.print(z_at_fully_hydrated, 0);
    Serial.print(F(" ohm -> ")); Serial.print(hydration_high_pct, 1); Serial.println(F("% (hydrated)"));
    Serial.print(F("  ")); Serial.print(z_at_dehydrated, 0);
    Serial.print(F(" ohm -> ")); Serial.print(hydration_low_pct, 1); Serial.println(F("% (dehydrated)"));
    Serial.println();

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(F("Place electrodes"));
    lcd.setCursor(0, 1); lcd.print(F("on forearm..."));
    delay(1500);
}

void loop() {
    if (millis() - last_report_ms < report_period_ms) return;
    last_report_ms = millis();

    float z_avg = measure_impedance_ohms();
    float hydration_pct = impedance_to_hydration_pct(z_avg);

    float hydration_score = ((hydration_pct - hydration_low_pct) /
                             (hydration_high_pct - hydration_low_pct)) * 100.0f;
    if (hydration_score < 0)   hydration_score = 0;
    if (hydration_score > 100) hydration_score = 100;

    const char *status;
    if      (hydration_pct >= 60) status = "Hydrated";
    else if (hydration_pct >= 55) status = "Ok";
    else if (hydration_pct >= 50) status = "LOW";
    else                          status = "DEHYDRATED";

    Serial.print(F("Vraw="));    Serial.print(dbg_raw_volt, 3);
    Serial.print(F("V  Vsig=")); Serial.print(dbg_signal_volt, 3);
    Serial.print(F("V  Vbody="));Serial.print(dbg_v_body * 1000.0f, 2);
    Serial.print(F("mV  |Z|=")); Serial.print(z_avg, 1);
    Serial.print(F("ohm  H2O="));Serial.print(hydration_pct, 1);
    Serial.print(F("%  ["));     Serial.print(status);
    Serial.println(F("]"));

    const char spinner[] = {'|', '/', '-', '\\'};
    char spinner_ch = spinner[spinner_frame];
    char status_ch  = status[0];
    spinner_frame = (spinner_frame + 1) & 0x03;

    char row0buf[17];
    snprintf(row0buf, sizeof(row0buf), "H2O %d%% %dohm %c%c",
             (int)round(hydration_pct), (int)round(z_avg), status_ch, spinner_ch);
    lcd.setCursor(0, 0);
    lcd.print(row0buf);
    for (int i = strlen(row0buf); i < lcd_cols; i++) lcd.print(' ');

    draw_bar_row(1, hydration_score);
}
