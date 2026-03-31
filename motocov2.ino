// Hello future programmer! This project has been a huge headache for me,
// Many sleepless nights, a bunch of coffee, and many itterations to get right.
// Due to the dazed state I was in when this was made, the code is not the cleanest or most efficient.
// I have added comments to the best of my ability to explain the logic and flow,
// I'm not exactly sure on what I was doing with this anymore.
// Sorry in advance for any headaches you have because of this!
#include <Wire.h>
#include <avr/wdt.h> // Added Watchdog Timer library

// Pin definitions
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

// I2C and motor parameters
const uint8_t I2C_SLAVE_ADDR = 0x08; 
const uint8_t POLE_PAIRS = 15; // TUNING REQUIRED

// initialize shared variables
volatile uint8_t hallState = 0;
volatile uint8_t previousHallState = 0;
volatile unsigned long lastCommutationTime = 0;
volatile unsigned long commutationInterval = 0;
volatile unsigned int sharedMotorRPM = 0;
volatile int sharedCurrentRaw = 0;

// Local state tracking for optimization
uint8_t lastAppliedState = 0;
int lastAppliedDuty = -1;
int throttleValue = 0;
int pwmDuty = 0;

// Setup function
void setup() {
    // Disable Watchdog immediately upon boot in case of reset loop
    wdt_disable(); 

    pinMode(PIN_THROTTLE, INPUT); pinMode(PIN_CURRENT, INPUT);
    
    pinMode(PIN_HALL_A, INPUT); 
    pinMode(PIN_HALL_B, INPUT); 
    pinMode(PIN_HALL_C, INPUT);
    
    pinMode(PIN_HIN_A, OUTPUT); pinMode(PIN_LIN_A, OUTPUT); pinMode(PIN_SD_A, OUTPUT);
    pinMode(PIN_HIN_B, OUTPUT); pinMode(PIN_LIN_B, OUTPUT); pinMode(PIN_SD_B, OUTPUT);
    pinMode(PIN_HIN_C, OUTPUT); pinMode(PIN_LIN_C, OUTPUT); pinMode(PIN_SD_C, OUTPUT);

    // Default to OFF before enabling timers or interrupts
    disableAllPhases();

    // Set PWM frequency to ~31.25 kHz to prevent audible whine and motor heating
    TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
    TCCR2B = (TCCR2B & 0b11111000) | 0x01; 

   // Wire.begin(I2C_SLAVE_ADDR);
    
    // SAFETY: Prevent I2C bus noise from freezing the main loop
    // Wire.setWireTimeout(3000, true); 
    // Wire.onRequest(i2cRequestEvent); 

    // Enable Pin Change Interrupts for Hall Sensors (Pins 2, 3, 4)
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20);
    updateHallState(); // Prime the initial state

    // SAFETY: Enable Watchdog Timer (250ms timeout)
    // wdt_enable(WDTO_250MS);
}

// Main control loop
void loop() {
    // Pet the watchdog. If this doesn't happen every 250ms, the board resets.
    wdt_reset(); 

    throttleValue = analogRead(PIN_THROTTLE);
    pwmDuty = (throttleValue < 50) ? 0 : map(throttleValue, 50, 1023, 0, 240); 

    // int currentReading = analogRead(PIN_CURRENT); 

    // ATOMIC SNAPSHOT: Freeze interrupts to copy volatile variables safely
    noInterrupts(); 
    unsigned long currentInterval = commutationInterval;
    unsigned long timeSinceLast = micros() - lastCommutationTime;
    uint8_t localHallState = hallState; // Protect against mid-loop state changes
    interrupts();

    unsigned int calcRPM = 0;
    // Timeout RPM to 0 if motor hasn't ticked in 100ms
    if (timeSinceLast < 100000 && currentInterval > 0) { 
        calcRPM = (10000000UL / currentInterval) / POLE_PAIRS;
    }

    noInterrupts();
    sharedMotorRPM = calcRPM;
    sharedCurrentRaw = currentReading;
    interrupts();

    // SAFETY INTERLOCK & COMMUTATION
    // Use the protected 'localHallState' for all logic
    if (pwmDuty == 0 || localHallState == 0 || localHallState == 7) {
        if (lastAppliedDuty != 0 || lastAppliedState != localHallState) {
            disableAllPhases();
            lastAppliedDuty = 0; 
            lastAppliedState = localHallState; 
        }
    } else if (localHallState != lastAppliedState || abs(pwmDuty - lastAppliedDuty) > 2) {
        commute(localHallState, pwmDuty);
        lastAppliedState = localHallState;
        lastAppliedDuty = pwmDuty;
    }
}

// Hall Sensor Interrupt Vector
ISR(PCINT2_vect) {
    updateHallState();
}

void updateHallState() {
    uint8_t hA = digitalRead(PIN_HALL_A);
    uint8_t hB = digitalRead(PIN_HALL_B);
    uint8_t hC = digitalRead(PIN_HALL_C);
    
    hallState = (hA << 2) | (hB << 1) | hC;

    // Only update timing if the state actually changed
    if (hallState != previousHallState) {
        unsigned long now = micros();
        commutationInterval = now - lastCommutationTime;
        lastCommutationTime = now;
        previousHallState = hallState;
    }
}

// Commutation logic based on hall sensor state
void commute(uint8_t state, int duty) {
    switch(state) {
        case 1: // State 1: A+ B- C- (Example)
            digitalWrite(PIN_SD_C, HIGH); digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_SD_A, LOW); digitalWrite(PIN_SD_B, LOW);
            digitalWrite(PIN_LIN_B, HIGH); analogWrite(PIN_HIN_A, duty); break;
        case 2: // State 2: A+ B- C+ (Example)
            digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_SD_B, LOW); digitalWrite(PIN_SD_C, LOW);
            digitalWrite(PIN_LIN_C, HIGH); analogWrite(PIN_HIN_B, duty); break;
        case 3: // State 3: A- B+ C- (Example)
            digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_SD_A, LOW); digitalWrite(PIN_SD_C, LOW);
            digitalWrite(PIN_LIN_C, HIGH); analogWrite(PIN_HIN_A, duty); break;
        case 4: // State 4: A- B+ C+ (Example)
            digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_SD_C, LOW); digitalWrite(PIN_SD_A, LOW);
            digitalWrite(PIN_LIN_A, HIGH); analogWrite(PIN_HIN_C, duty); break;
        case 5: // State 5: A+ B+ C- (Example)
            digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_LIN_A, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_B, 0); 
            digitalWrite(PIN_SD_C, LOW); digitalWrite(PIN_SD_B, LOW);
            digitalWrite(PIN_LIN_B, HIGH); analogWrite(PIN_HIN_C, duty); break;
        case 6: // State 6: A- B- C+ (Example)
            digitalWrite(PIN_SD_C, HIGH); digitalWrite(PIN_LIN_C, LOW); analogWrite(PIN_HIN_C, 0); 
            digitalWrite(PIN_LIN_B, LOW); analogWrite(PIN_HIN_A, 0); 
            digitalWrite(PIN_SD_B, LOW); digitalWrite(PIN_SD_A, LOW);
            digitalWrite(PIN_LIN_A, HIGH); analogWrite(PIN_HIN_B, duty); break;
        default: // Invalid state, disable all phases for safety
            disableAllPhases(); break;
    }
}

// Disable all phases immediately for safety
void disableAllPhases() {
    // Disable SD (Shutdown) pins first to cut power instantly
    digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_SD_C, HIGH);
    // Then clear control signals
    analogWrite(PIN_HIN_A, 0); analogWrite(PIN_HIN_B, 0); analogWrite(PIN_HIN_C, 0);
    digitalWrite(PIN_LIN_A, LOW); digitalWrite(PIN_LIN_B, LOW); digitalWrite(PIN_LIN_C, LOW);
}

void i2cRequestEvent() {
    uint8_t payload[4];
    
    // Read shared variables (safe because loop uses noInterrupts when writing)
    unsigned int txRPM = sharedMotorRPM;
    int txCurrent = sharedCurrentRaw;

    // Bit-shift 16-bit variables into 8-bit array
    payload[0] = (txRPM >> 8) & 0xFF; payload[1] = txRPM & 0xFF;        
    payload[2] = (txCurrent >> 8) & 0xFF; payload[3] = txCurrent & 0xFF;        
    
    Wire.write(payload, 4); 
}
