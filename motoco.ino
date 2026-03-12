#include <Wire.h>

const uint8_t PIN_THROTTLE = A0;
const uint8_t PIN_CURRENT  = A1; 
const uint8_t PIN_HALL_A = 2;
const uint8_t PIN_HALL_B = 3;
const uint8_t PIN_HALL_C = 4;
const uint8_t PIN_HIN_A = 9;  
const uint8_t PIN_HIN_B = 10; 
const uint8_t PIN_HIN_C = 11; 
const uint8_t PIN_LIN_A = 5;
const uint8_t PIN_LIN_B = 6;
const uint8_t PIN_LIN_C = 7;
const uint8_t PIN_SD_A = 8;
const uint8_t PIN_SD_B = 12;
const uint8_t PIN_SD_C = 13;

const uint8_t I2C_SLAVE_ADDR = 0x08; 
const uint8_t POLE_PAIRS = 15; // TUNING REQUIRED

volatile uint8_t hallState = 0;
volatile uint8_t previousHallState = 0;
volatile unsigned long lastCommutationTime = 0;
volatile unsigned long commutationInterval = 0;
volatile unsigned int sharedMotorRPM = 0;
volatile int sharedCurrentRaw = 0;

uint8_t lastAppliedState = 0;
int lastAppliedDuty = -1;
int throttleValue = 0;
int pwmDuty = 0;

void setup() {
    pinMode(PIN_THROTTLE, INPUT); pinMode(PIN_CURRENT, INPUT);
    pinMode(PIN_HALL_A, INPUT_PULLUP); pinMode(PIN_HALL_B, INPUT_PULLUP); pinMode(PIN_HALL_C, INPUT_PULLUP);
    pinMode(PIN_HIN_A, OUTPUT); pinMode(PIN_LIN_A, OUTPUT); pinMode(PIN_SD_A, OUTPUT);
    pinMode(PIN_HIN_B, OUTPUT); pinMode(PIN_LIN_B, OUTPUT); pinMode(PIN_SD_B, OUTPUT);
    pinMode(PIN_HIN_C, OUTPUT); pinMode(PIN_LIN_C, OUTPUT); pinMode(PIN_SD_C, OUTPUT);

    disableAllPhases();

    TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
    TCCR2B = (TCCR2B & 0b11111000) | 0x01; 

    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onRequest(i2cRequestEvent); 

    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20);
    updateHallState();
}

void loop() {
    throttleValue = analogRead(PIN_THROTTLE);
    pwmDuty = (throttleValue < 50) ? 0 : map(throttleValue, 50, 1023, 0, 240); 
    
    int currentReading = analogRead(PIN_CURRENT); 

    noInterrupts(); 
    unsigned long currentInterval = commutationInterval;
    unsigned long timeSinceLast = micros() - lastCommutationTime;
    interrupts();

    unsigned int calcRPM = 0;
    if (timeSinceLast < 100000 && currentInterval > 0) { 
        calcRPM = (10000000UL / currentInterval) / POLE_PAIRS;
    }

    noInterrupts();
    sharedMotorRPM = calcRPM;
    sharedCurrentRaw = currentReading;
    interrupts();

    if (pwmDuty == 0) {
        if (lastAppliedDuty != 0) {
            disableAllPhases();
            lastAppliedDuty = 0; lastAppliedState = 0; 
        }
    } else if (hallState != lastAppliedState || abs(pwmDuty - lastAppliedDuty) > 2) {
        commute(hallState, pwmDuty);
        lastAppliedState = hallState;
        lastAppliedDuty = pwmDuty;
    }
}

ISR(PCINT2_vect) {
    updateHallState();
}

void updateHallState() {
    uint8_t hA = digitalRead(PIN_HALL_A);
    uint8_t hB = digitalRead(PIN_HALL_B);
    uint8_t hC = digitalRead(PIN_HALL_C);
    hallState = (hA << 2) | (hB << 1) | hC;

    if (hallState != previousHallState) {
        unsigned long now = micros();
        commutationInterval = now - lastCommutationTime;
        lastCommutationTime = now;
        previousHallState = hallState;
    }
}

void commute(uint8_t state, int duty) {
    switch(state) {
        case 1: 
            digitalWrite(PIN_SD_C, HIGH); digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_SD_A, LOW); digitalWrite(PIN_SD_B, LOW);
            digitalWrite(PIN_LIN_B, HIGH); analogWrite(PIN_HIN_A, duty); break;
        case 2: 
            digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_SD_B, LOW); digitalWrite(PIN_SD_C, LOW);
            digitalWrite(PIN_LIN_C, HIGH); analogWrite(PIN_HIN_B, duty); break;
        case 3: 
            digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_SD_A, LOW); digitalWrite(PIN_SD_C, LOW);
            digitalWrite(PIN_LIN_C, HIGH); analogWrite(PIN_HIN_A, duty); break;
        case 4: 
            digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_SD_C, LOW); digitalWrite(PIN_SD_A, LOW);
            digitalWrite(PIN_LIN_A, HIGH); analogWrite(PIN_HIN_C, duty); break;
        case 5: 
            digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_SD_C, LOW); digitalWrite(PIN_SD_B, LOW);
            digitalWrite(PIN_LIN_B, HIGH); analogWrite(PIN_HIN_C, duty); break;
        case 6: 
            digitalWrite(PIN_SD_C, HIGH); digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_SD_B, LOW); digitalWrite(PIN_SD_A, LOW);
            digitalWrite(PIN_LIN_A, HIGH); analogWrite(PIN_HIN_B, duty); break;
        default:
            disableAllPhases(); break;
    }
}

void disableAllPhases() {
    digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_SD_C, HIGH);
    analogWrite(PIN_HIN_A, 0); analogWrite(PIN_HIN_B, 0); analogWrite(PIN_HIN_C, 0);
    digitalWrite(PIN_LIN_A, LOW); digitalWrite(PIN_LIN_B, LOW); digitalWrite(PIN_LIN_C, LOW);
}

void i2cRequestEvent() {
    uint8_t payload[4];
    unsigned int txRPM = sharedMotorRPM;
    int txCurrent = sharedCurrentRaw;

    payload[0] = (txRPM >> 8) & 0xFF; payload[1] = txRPM & 0xFF;        
    payload[2] = (txCurrent >> 8) & 0xFF; payload[3] = txCurrent & 0xFF;        
    Wire.write(payload, 4); 
}
