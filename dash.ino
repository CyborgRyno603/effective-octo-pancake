#include <Wire.h>
#include <Adafruit_NeoPixel.h>

const uint8_t PIN_NEOPIXEL = 6;
const uint8_t PIN_VOLTAGE_SENSE = A2; 
const uint8_t PIN_SR_DAT = 8;
const uint8_t PIN_SR_CLK = 9;
const uint8_t PIN_SR_LAT = 10;
const uint8_t PIN_DIG_SPD_TENS = 2;
const uint8_t PIN_DIG_SPD_ONES = 3;
const uint8_t PIN_DIG_AMP_TENS = 4;
const uint8_t PIN_DIG_AMP_ONES = 5;

const uint8_t MOTOR_CONTROLLER_ADDR = 0x08;
Adafruit_NeoPixel pixels(7, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// --- Tuning Variables ---
const float ADC_REF_VOLTAGE = 5.0;
const float CURRENT_ZERO_OFFSET_V = 2.5; // Update per current sensor datasheet
const float CURRENT_SENSITIVITY_V_PER_A = 0.04; 
const float TIRE_CIRCUMFERENCE_METERS = 1.55; 
const float MIN_COMPETITION_SPEED_KPH = 25.0; 
const float EFFICIENT_MAX_AMPS = 15.0; 
const float VOLTAGE_DIVIDER_RATIO = 11.0; 
const float BATT_MAX_VOLTAGE = 54.6; 
const float BATT_MIN_VOLTAGE = 39.0; 
const float EMA_ALPHA = 0.15; // Smoothing weight

const uint8_t digitFont[11] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 
    0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111, 0b00000000
};

unsigned int rawRPM = 0;
int rawCurrent = 512; // Initialize to zero-current state
float actualAmps = 0.0;
float actualSpeedKPH = 0.0;
float packVoltage = 0.0;
int batteryPercent = 0;

uint8_t displayDigits[4] = {10, 10, 10, 10}; 
uint8_t currentDigit = 0;
unsigned long lastMultiplexTime = 0;

void setup() {
    Wire.begin(); 
    Wire.setWireTimeout(25000, true); 
    
    pinMode(PIN_VOLTAGE_SENSE, INPUT);
    pinMode(PIN_SR_DAT, OUTPUT); pinMode(PIN_SR_CLK, OUTPUT); pinMode(PIN_SR_LAT, OUTPUT);
    pinMode(PIN_DIG_SPD_TENS, OUTPUT); pinMode(PIN_DIG_SPD_ONES, OUTPUT);
    pinMode(PIN_DIG_AMP_TENS, OUTPUT); pinMode(PIN_DIG_AMP_ONES, OUTPUT);

    digitalWrite(PIN_DIG_SPD_TENS, LOW); digitalWrite(PIN_DIG_SPD_ONES, LOW);
    digitalWrite(PIN_DIG_AMP_TENS, LOW); digitalWrite(PIN_DIG_AMP_ONES, LOW);

    pixels.begin(); pixels.setBrightness(150); pixels.show(); 
}

void loop() {
    if (micros() - lastMultiplexTime >= 2500) {
        multiplexDisplay();
        lastMultiplexTime = micros();
    }

    static unsigned long lastDataTime = 0;
    if (millis() - lastDataTime >= 100) {
        requestMotorData();
        calculateMetrics();
        updateDigitBuffer();
        updateNeoPixels();
        lastDataTime = millis();
    }
}

void multiplexDisplay() {
    digitalWrite(PIN_DIG_SPD_TENS, LOW); digitalWrite(PIN_DIG_SPD_ONES, LOW);
    digitalWrite(PIN_DIG_AMP_TENS, LOW); digitalWrite(PIN_DIG_AMP_ONES, LOW);

    digitalWrite(PIN_SR_LAT, LOW);
    shiftOut(PIN_SR_DAT, PIN_SR_CLK, MSBFIRST, digitFont[displayDigits[currentDigit]]);
    digitalWrite(PIN_SR_LAT, HIGH);

    switch(currentDigit) {
        case 0: digitalWrite(PIN_DIG_SPD_TENS, HIGH); break;
        case 1: digitalWrite(PIN_DIG_SPD_ONES, HIGH); break;
        case 2: digitalWrite(PIN_DIG_AMP_TENS, HIGH); break;
        case 3: digitalWrite(PIN_DIG_AMP_ONES, HIGH); break;
    }
    currentDigit = (currentDigit + 1) % 4;
}

void updateDigitBuffer() {
    int spd = constrain((int)actualSpeedKPH, 0, 99);
    displayDigits[0] = (spd >= 10) ? (spd / 10) : 10; 
    displayDigits[1] = spd % 10;                      

    int amps = constrain(abs((int)actualAmps), 0, 99);
    displayDigits[2] = (amps >= 10) ? (amps / 10) : 10; 
    displayDigits[3] = amps % 10;                       
}

void requestMotorData() {
    Wire.requestFrom(MOTOR_CONTROLLER_ADDR, (uint8_t)4);
    if (Wire.available() == 4) {
        uint8_t rpmHigh = Wire.read();
        uint8_t rpmLow = Wire.read();
        uint8_t curHigh = Wire.read();
        uint8_t curLow = Wire.read();
        rawRPM = (rpmHigh << 8) | rpmLow;
        rawCurrent = (curHigh << 8) | curLow;
    }

    if (Wire.getWireTimeoutFlag()) {
        Wire.clearWireTimeoutFlag();
        rawRPM = 0;
        rawCurrent = 512; // Assume 0A if connection lost
    }
}

void calculateMetrics() {
    float currentSensorVolts = (rawCurrent / 1023.0) * ADC_REF_VOLTAGE;
    float instantaneousAmps = (currentSensorVolts - CURRENT_ZERO_OFFSET_V) / CURRENT_SENSITIVITY_V_PER_A;
    actualAmps = (EMA_ALPHA * instantaneousAmps) + ((1.0 - EMA_ALPHA) * actualAmps);

    actualSpeedKPH = (rawRPM * TIRE_CIRCUMFERENCE_METERS * 60.0) / 1000.0;
    
    float pinVolts = (analogRead(PIN_VOLTAGE_SENSE) / 1023.0) * ADC_REF_VOLTAGE;
    float instantaneousVolts = pinVolts * VOLTAGE_DIVIDER_RATIO;
    packVoltage = (EMA_ALPHA * instantaneousVolts) + ((1.0 - EMA_ALPHA) * packVoltage);
    
    float constrainedVolts = constrain(packVoltage, BATT_MIN_VOLTAGE, BATT_MAX_VOLTAGE);
    batteryPercent = map(constrainedVolts * 10, BATT_MIN_VOLTAGE * 10, BATT_MAX_VOLTAGE * 10, 0, 100);
}

void updateNeoPixels() {
    pixels.clear();
    uint32_t battColor = (batteryPercent > 75) ? pixels.Color(0, 255, 0) :
                         (batteryPercent > 50) ? pixels.Color(150, 255, 0) :
                         (batteryPercent > 25) ? pixels.Color(255, 165, 0) : pixels.Color(255, 0, 0);
    
    int numLedsOn = (batteryPercent > 75) ? 4 : (batteryPercent > 50) ? 3 : (batteryPercent > 25) ? 2 : 1;
    for (int i = 0; i < numLedsOn; i++) pixels.setPixelColor(i, battColor);

    uint32_t effColor = (actualAmps > EFFICIENT_MAX_AMPS) ? pixels.Color(255, 0, 0) :
                        (actualSpeedKPH > 0 && actualSpeedKPH < MIN_COMPETITION_SPEED_KPH) ? pixels.Color(255, 255, 0) :
                        (actualSpeedKPH >= MIN_COMPETITION_SPEED_KPH && actualAmps <= EFFICIENT_MAX_AMPS) ? pixels.Color(0, 255, 0) : pixels.Color(0, 0, 50);
    
    for (int i = 4; i <= 6; i++) pixels.setPixelColor(i, effColor);
    pixels.show();
}
