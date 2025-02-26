#include <Servo.h>
#include <stdint.h>

Servo dir;
Servo esc;

const int MIN_ESC_PULSE_WIDTH = 1000;
const int MAX_ESC_PULSE_WIDTH = 2000;

const int hallPin = 2;          
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = UINT32_MAX;  
volatile bool newPulse = false;
unsigned long prevlogtime = 0;
const long logint = 30;
float rpm = 0, targetRPM = 0;
const int filterSize = 5;
float rpmFilter[filterSize] = {0, 0, 0, 0, 0};
float rpmFiltered = 0;

// ESC and Direction Control Variables
int pulseWidth = 1000;
int dirPulseWidth = 1400;

// PID Control Variables
float Kp = 0.4270462633, Ki = 0, Kd = 0.001;
float error = 0, prevError = 0, integral = 0, derivative = 0;

// ** Lookup Table: PWM (%) -> RPM ** (Based on provided data)
const int pwmValues[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const int rpmValues[] = {0, 1310, 2430, 3538, 4675, 5868, 6996, 8160, 9189, 9575}; 

// Interrupt function for Hall Sensor
void pulseCounter() {                            
    unsigned long currentTime = micros();          
    if (lastPulseTime != 0) {
        pulseInterval = currentTime - lastPulseTime; 
        newPulse = true;                             
    }
    lastPulseTime = currentTime;                   
}

void setup() {
    Serial.begin(115200);
    esc.attach(4);
    dir.attach(7);

    esc.writeMicroseconds(MIN_ESC_PULSE_WIDTH);
    dir.writeMicroseconds(1400);
    delay(1000);
    esc.writeMicroseconds(MAX_ESC_PULSE_WIDTH);

    pinMode(hallPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(hallPin), pulseCounter, RISING);

    for (int i = 0; i < filterSize; i++) {
        rpmFilter[i] = 0;
    }

    Serial.println("Starting...");
}

void loop() {
    if (newPulse && pulseInterval > 0 && pulseInterval < 1000000) { 
        float pulseFrequency = 1000000.0 / pulseInterval;
        rpm = (pulseFrequency * 60.0) / 7;
        rpmFiltered = MA(rpm);
        newPulse = false;
    } else if (millis() - lastPulseTime > 500) {  
        rpm = 0;
        rpmFiltered = MA(0);
    }

    // PID Control Calculation
    error = targetRPM - rpmFiltered;
    integral += error;
    derivative = error - prevError;
    prevError = error;
    
    int pwmAdjustment = Kp * error + Ki * integral + Kd * derivative;
    pulseWidth = constrain(pulseWidth + pwmAdjustment, MIN_ESC_PULSE_WIDTH, MAX_ESC_PULSE_WIDTH);
    esc.writeMicroseconds(pulseWidth);

    unsigned long currentlogint = millis();              
    if (currentlogint - prevlogtime >= logint) {
        prevlogtime = currentlogint;
        Serial.print("Desired:");
        Serial.print(targetRPM);
        Serial.print(" Error:");
        Serial.print(error);
        Serial.print(" Actual:");
        Serial.println(rpmFiltered);
    }

    if (Serial.available() > 0) {
        char command = Serial.read();
        int inputRPM = Serial.parseInt();

        if (command == '+') {
            targetRPM += inputRPM;
        } else if (command == '-') {
            targetRPM -= inputRPM;
        } else if (command == 'c') {
            dirPulseWidth = 1600;
        } else if (command == 'a') {
            dirPulseWidth = 1400;
        }

        targetRPM = constrain(targetRPM, 0, 9575);
        dir.writeMicroseconds(dirPulseWidth);
        Serial.print("Updated Target RPM: ");
        Serial.println(targetRPM);
    }

    delay(10);
}

// ** Function to Convert RPM to Closest PWM (%) **
int rpmToPWM(int targetRPM) {
    for (int i = 0; i < 9; i++) {
        if (targetRPM >= rpmValues[i] && targetRPM <= rpmValues[i + 1]) {
            float ratio = (float)(targetRPM - rpmValues[i]) / (rpmValues[i + 1] - rpmValues[i]);
            return pwmValues[i] + ratio * (pwmValues[i + 1] - pwmValues[i]);
        }
    }
    return 100; // Default to max PWM if above range
}

// ** Moving Average Filter for RPM **
float MA(float rpm) {
    for (int i = 0; i < filterSize - 1; i++) {
        rpmFilter[i] = rpmFilter[i + 1];
    }
    rpmFilter[filterSize - 1] = rpm;
    
    float sum = 0;
    for (int j = 0; j < filterSize; j++) {
        sum += rpmFilter[j];
    }
    return sum / filterSize;
}
