#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const uint8_t  pin_excitation = 9;
const uint8_t  pin_adc        = A0;
const uint8_t  pin_led        = 13;
const uint8_t  lcd_i2c_addr   = 0x27;
const uint8_t  lcd_cols       = 16;
const uint8_t  lcd_rows       = 2;

const float    excitation_hz    = 50000.0f;
const uint16_t samples_per_meas = 256;
const uint16_t meas_averages    = 8;
const uint16_t report_period_ms = 1000;

const float    vref_volt         = 5.0f;
const uint16_t adc_max           = 1023;
const float    ad8226_gain       = 100.0f;
const float    howland_current_a = 25e-6f;

const float z_at_fully_hydrated = 250.0f;
const float z_at_dehydrated     = 700.0f;
const float hydration_high_pct  = 65.0f;
const float hydration_low_pct   = 50.0f;

uint16_t adc_samples[samples_per_meas];
unsigned long last_report_ms = 0;
LiquidCrystal_I2C lcd(lcd_i2c_addr, lcd_cols, lcd_rows);
uint8_t spinner_frame = 0;

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
        if (i < full_cells) {
            lcd.write((uint8_t)5);
        } else if (i == full_cells && partial > 0) {
            lcd.write((uint8_t)partial);
        } else {
            lcd.write(' ');
        }
    }
}

void setup_50khz_pwm() {
    pinMode(pin_excitation, OUTPUT);

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    TCCR1A |= (1 << COM1A1);

    TCCR1B |= (1 << CS10);

    ICR1  = 319;
    OCR1A = 160;
}

void setup_fast_adc() {
    ADMUX  = (1 << REFS0) | (pin_adc - A0);

    ADCSRA = (1 << ADEN) | (1 << ADPS2);

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
}

inline uint16_t read_adc_fast() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

float dbg_mean_adc = 0;
uint16_t dbg_min_adc = 0, dbg_max_adc = 0;
float dbg_mag_counts = 0;

float measure_impedance_ohms() {
    noInterrupts();
    unsigned long t0 = micros();
    for (uint16_t i = 0; i < samples_per_meas; i++) {
        adc_samples[i] = read_adc_fast();
    }
    unsigned long t1 = micros();
    interrupts();

    uint16_t mn = 1023, mx = 0;
    for (uint16_t i = 0; i < samples_per_meas; i++) {
        if (adc_samples[i] < mn) mn = adc_samples[i];
        if (adc_samples[i] > mx) mx = adc_samples[i];
    }
    dbg_min_adc = mn;
    dbg_max_adc = mx;

    float dt = (float)(t1 - t0) / samples_per_meas * 1e-6f;
    if (dt <= 0) dt = 13e-6f;

    float mean_adc = 0;
    for (uint16_t i = 0; i < samples_per_meas; i++) mean_adc += adc_samples[i];
    mean_adc /= samples_per_meas;

    float i_sum = 0, q_sum = 0;
    float omega = 2.0f * PI * excitation_hz;
    for (uint16_t i = 0; i < samples_per_meas; i++) {
        float t = i * dt;
        float s = (float)adc_samples[i] - mean_adc;
        i_sum += s * sinf(omega * t);
        q_sum += s * cosf(omega * t);
    }
    i_sum /= samples_per_meas;
    q_sum /= samples_per_meas;

    float mag_counts = 2.0f * sqrtf(i_sum * i_sum + q_sum * q_sum);
    dbg_mean_adc = mean_adc;
    dbg_mag_counts = mag_counts;

    float v_buffer_out = (mag_counts / (float)adc_max) * vref_volt;

    float v_body = v_buffer_out / ad8226_gain;

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
    pinMode(pin_led, OUTPUT);

    lcd.init();
    lcd.backlight();
    upload_bar_graph_chars();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.setCursor(0, 1);

    Serial.println(F("Starting 50 kHz excitation on D9..."));
    setup_50khz_pwm();
    setup_fast_adc();

    Serial.println(F("Calibration points (edit in code):"));
    Serial.print(F("  "));   Serial.print(z_at_fully_hydrated, 0);
    Serial.print(F(" ohm -> ")); Serial.print(hydration_high_pct, 1); Serial.println(F("% (hydrated)"));
    Serial.print(F("  "));   Serial.print(z_at_dehydrated, 0);
    Serial.print(F(" ohm -> ")); Serial.print(hydration_low_pct, 1); Serial.println(F("% (dehydrated)"));
    Serial.println(F("Place electrodes on inner forearm and wait..."));
    Serial.println();

    delay(1500);
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(F("Place electrodes"));
    lcd.setCursor(0, 1); lcd.print(F("on forearm..."));
    delay(1500);
}

void lcd_print_row(uint8_t row, const String &text) {
    String padded = text;
    while (padded.length() < lcd_cols) padded += ' ';
    if (padded.length() > lcd_cols) padded = padded.substring(0, lcd_cols);
    lcd.setCursor(0, row);
    lcd.print(padded);
}

void loop() {
    if (millis() - last_report_ms < report_period_ms) return;
    last_report_ms = millis();

    digitalWrite(pin_led, HIGH);

    float z_sum = 0;
    for (uint16_t i = 0; i < meas_averages; i++) {
        z_sum += measure_impedance_ohms();
    }
    float z_avg = z_sum / meas_averages;
    float hydration_pct = impedance_to_hydration_pct(z_avg);

    float hydration_score = ((hydration_pct - hydration_low_pct) /
                             (hydration_high_pct - hydration_low_pct)) * 100.0f;
    if (hydration_score < 0)   hydration_score = 0;
    if (hydration_score > 100) hydration_score = 100;

    const char *status;
    if      (hydration_pct >= 60) status = "HYDRATED";
    else if (hydration_pct >= 55) status = "OK";
    else if (hydration_pct >= 50) status = "LOW";
    else                          status = "DEHYDRATED";

    Serial.print(F("|Z|="));
    Serial.print(z_avg, 1);
    Serial.print(F("ohm  H2O="));
    Serial.print(hydration_pct, 1);
    Serial.print(F("%  ADC mean="));
    Serial.print(dbg_mean_adc, 0);
    Serial.print(F(" min="));
    Serial.print(dbg_min_adc);
    Serial.print(F(" max="));
    Serial.print(dbg_max_adc);
    Serial.print(F(" p-p="));
    Serial.print(dbg_max_adc - dbg_min_adc);
    Serial.print(F(" magCounts="));
    Serial.print(dbg_mag_counts, 2);
    Serial.print(F("  ["));
    Serial.print(status);
    Serial.println(F("]"));

    // this is just for fun, a little loading spinner
    const char spinner[] = {'|', '/', '-', '\\'};
    char spinner_ch = spinner[spinner_frame];
    char status_ch  = status[0];
    spinner_frame = (spinner_frame + 1) & 0x03;

    char row0buf[17];
    snprintf(row0buf, sizeof(row0buf),
             "H2O %d%% %dohm %c%c",
             (int)round(hydration_pct),
             (int)round(z_avg),
             status_ch,
             spinner_ch);

    lcd.setCursor(0, 0);
    lcd.print(row0buf);
    for (int i = strlen(row0buf); i < lcd_cols; i++) lcd.print(' ');

    draw_bar_row(1, hydration_score);

    digitalWrite(pin_led, LOW);
}
