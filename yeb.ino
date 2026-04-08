const uint8_t PIN_THROTTLE = A0; 

// Hall Sensors
const uint8_t PIN_HALL_A = 2;
const uint8_t PIN_HALL_B = 3;
const uint8_t PIN_HALL_C = 4;

// Gate Driver High/Low Inputs
const uint8_t PIN_HIN_A = 9;  
const uint8_t PIN_HIN_B = 10; 
const uint8_t PIN_HIN_C = 11; 
const uint8_t PIN_LIN_A = 5;
const uint8_t PIN_LIN_B = 6;
const uint8_t PIN_LIN_C = 7;

// Gate Driver Shutdown Pins
const uint8_t PIN_SD_A = 8;
const uint8_t PIN_SD_B = 12;
const uint8_t PIN_SD_C = 13;

// --- HYSTERESIS THRESHOLDS (0-1023) ---
// Adjust these to change the physical "feel" of the pedal
const int THRESHOLD_ON = 600; // Pedal pressed past ~60% -> Motor turns ON
const int THRESHOLD_OFF = 400; // Pedal released below ~40% -> Motor turns OFF

// Global state tracker for the throttle
bool motorEnabled = false;

// Timer variable for our Serial Monitor
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500; // Print every 500 milliseconds (half a second)

void setup() {
    pinMode(PIN_THROTTLE, INPUT); 
    
    // External 5V pulse hall sensors
    pinMode(PIN_HALL_A, INPUT); 
    pinMode(PIN_HALL_B, INPUT); 
    pinMode(PIN_HALL_C, INPUT);
    
    pinMode(PIN_HIN_A, OUTPUT); 
    pinMode(PIN_LIN_A, OUTPUT); 
    pinMode(PIN_SD_A, OUTPUT);
    pinMode(PIN_HIN_B, OUTPUT); 
    pinMode(PIN_LIN_B, OUTPUT); 
    pinMode(PIN_SD_B, OUTPUT);
    pinMode(PIN_HIN_C, OUTPUT); 
    pinMode(PIN_LIN_C, OUTPUT); 
    pinMode(PIN_SD_C, OUTPUT);

    disableAllPhases();

    Serial.begin(115200);
}

void loop() {
    // 1. Read the raw analog throttle voltage
    int rawThrottle = analogRead(PIN_THROTTLE);

    // 2. Apply Hysteresis to determine the ON/OFF state
    if (rawThrottle > THRESHOLD_ON) {
        motorEnabled = true;
    } else if (rawThrottle < THRESHOLD_OFF) {
        motorEnabled = false;
    }

    // 3. Read raw hall states
    uint8_t hA = digitalRead(PIN_HALL_A);
    uint8_t hB = digitalRead(PIN_HALL_B);
    uint8_t hC = digitalRead(PIN_HALL_C);
    
    // Construct the state (1 to 6)
    uint8_t state = (hA << 2) | (hB << 1) | hC;

    // 4. Direct Routing
    if (!motorEnabled || state == 0 || state == 7) {
        disableAllPhases();
    } else {
        commute(state);
    }

    // 5. Non-Blocking Serial Monitor Output
    unsigned long currentMillis = millis();
    if (currentMillis - lastPrintTime >= printInterval) {
        // It has been 500ms since the last print, so we print again and reset the timer
        lastPrintTime = currentMillis;

        Serial.println("====== SYSTEM STATUS ======");
        
        Serial.print("Throttle Raw : "); 
        Serial.println(rawThrottle);
        
        Serial.print("Motor Enabled: "); 
        Serial.println(motorEnabled ? "TRUE" : "FALSE"); // Prints text instead of 1 or 0
        
        Serial.print("Hall State   : "); 
        Serial.println(state);

        Serial.println("--- Output Pin States (1=HIGH, 0=LOW) ---");
        
        Serial.print("Shutdown (SD) -> A: "); Serial.print(digitalRead(PIN_SD_A));
        Serial.print(" | B: "); Serial.print(digitalRead(PIN_SD_B));
        Serial.print(" | C: "); Serial.println(digitalRead(PIN_SD_C));

        Serial.print("High In (HIN) -> A: "); Serial.print(digitalRead(PIN_HIN_A));
        Serial.print(" | B: "); Serial.print(digitalRead(PIN_HIN_B));
        Serial.print(" | C: "); Serial.println(digitalRead(PIN_HIN_C));

        Serial.print("Low In (LIN)  -> A: "); Serial.print(digitalRead(PIN_LIN_A));
        Serial.print(" | B: "); Serial.print(digitalRead(PIN_LIN_B));
        Serial.print(" | C: "); Serial.println(digitalRead(PIN_LIN_C));
        
        Serial.println("===========================\n"); // \n adds an extra blank line for readability
    }
}

void commute(uint8_t state) {
    // Pure digital routing matrix. 100% duty cycle. No PWM.
    switch(state) {
        case 1: 
            digitalWrite(PIN_SD_C, HIGH); digitalWrite(PIN_LIN_C, LOW); digitalWrite(PIN_HIN_C, LOW); 
            digitalWrite(PIN_LIN_A, LOW); digitalWrite(PIN_HIN_B, LOW); 
            digitalWrite(PIN_SD_A, LOW); digitalWrite(PIN_SD_B, LOW);
            digitalWrite(PIN_LIN_B, HIGH); digitalWrite(PIN_HIN_A, HIGH); 
            break;
        case 2: 
            digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_LIN_A, LOW); digitalWrite(PIN_HIN_A, LOW); 
            digitalWrite(PIN_LIN_B, LOW); digitalWrite(PIN_HIN_C, LOW); 
            digitalWrite(PIN_SD_B, LOW); digitalWrite(PIN_SD_C, LOW);
            digitalWrite(PIN_LIN_C, HIGH); digitalWrite(PIN_HIN_B, HIGH); 
            break;
        case 3: 
            digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_LIN_B, LOW); digitalWrite(PIN_HIN_B, LOW); 
            digitalWrite(PIN_LIN_A, LOW); digitalWrite(PIN_HIN_C, LOW); 
            digitalWrite(PIN_SD_A, LOW); digitalWrite(PIN_SD_C, LOW);
            digitalWrite(PIN_LIN_C, HIGH); digitalWrite(PIN_HIN_A, HIGH); 
            break;
        case 4: 
            digitalWrite(PIN_SD_B, HIGH); digitalWrite(PIN_LIN_B, LOW); digitalWrite(PIN_HIN_B, LOW); 
            digitalWrite(PIN_LIN_C, LOW); digitalWrite(PIN_HIN_A, LOW); 
            digitalWrite(PIN_SD_C, LOW); digitalWrite(PIN_SD_A, LOW);
            digitalWrite(PIN_LIN_A, HIGH); digitalWrite(PIN_HIN_C, HIGH); 
            break;
        case 5: 
            digitalWrite(PIN_SD_A, HIGH); digitalWrite(PIN_LIN_A, LOW); digitalWrite(PIN_HIN_A, LOW); 
            digitalWrite(PIN_LIN_C, LOW); digitalWrite(PIN_HIN_B, LOW); 
            digitalWrite(PIN_SD_C, LOW); digitalWrite(PIN_SD_B, LOW);
            digitalWrite(PIN_LIN_B, HIGH); digitalWrite(PIN_HIN_C, HIGH); 
            break;
        case 6: 
            digitalWrite(PIN_SD_C, HIGH); digitalWrite(PIN_LIN_C, LOW); digitalWrite(PIN_HIN_C, LOW); 
            digitalWrite(PIN_LIN_B, LOW); digitalWrite(PIN_HIN_A, LOW); 
            digitalWrite(PIN_SD_B, LOW); digitalWrite(PIN_SD_A, LOW);
            digitalWrite(PIN_LIN_A, HIGH); digitalWrite(PIN_HIN_B, HIGH); 
            break;
        default:
            disableAllPhases(); 
            break;
    }
}

void disableAllPhases() {
    // Assert Shutdown pins HIGH first to sever power instantly
    digitalWrite(PIN_SD_A, HIGH); 
    digitalWrite(PIN_SD_B, HIGH); 
    digitalWrite(PIN_SD_C, HIGH);
    
    // Clear all drive signals
    digitalWrite(PIN_HIN_A, LOW); 
    digitalWrite(PIN_HIN_B, LOW); 
    digitalWrite(PIN_HIN_C, LOW);
    digitalWrite(PIN_LIN_A, LOW); 
    digitalWrite(PIN_LIN_B, LOW); 
    digitalWrite(PIN_LIN_C, LOW);
}
