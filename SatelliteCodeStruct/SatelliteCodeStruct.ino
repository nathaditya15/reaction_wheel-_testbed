/******************************************************************************
 * INCLUDES & DEFINITIONS
 ******************************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <stdint.h>  // For UINT32_MAX

/******************************************************************************
 * IMU & ATTITUDE CONTROL
 ******************************************************************************/
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Desired angles in degrees
double setpointRoll  = 0.0;
double setpointPitch = 0.0;
double setpointYaw   = 0.0;

// Actual angles in degrees
double roll, pitch, yaw;

// PD variables for attitude control
double Kp_roll = 1.0, Kd_roll = 0.05;
double Kp_pitch = 1.0, Kd_pitch = 0.05;
double Kp_yaw = 1.0, Kd_yaw = 0.05;

/**
 * @brief A simple PD controller for attitude control
 */
void computePD(double &currentAngle, double setpointDeg, double &outputTorque, double Kp, double Kd) {
  // Static variables to track previous error
  static double lastErrorRoll  = 0.0;
  static double lastErrorPitch = 0.0;
  static double lastErrorYaw   = 0.0;

  // Identify which static variable to use based on the address of outputTorque
  double *lastError;
  if (&outputTorque == &roll) {
    // Not used in this code, just for demonstration
    lastError = &lastErrorRoll;
  } else if (&outputTorque == &pitch) {
    lastError = &lastErrorPitch;
  } else {
    lastError = &lastErrorYaw;
  }

  // Convert setpointDeg to radians for consistency
  double setpointRad = radians(setpointDeg);
  double error = setpointRad - currentAngle;
  double derivative = error - (*lastError);

  outputTorque = Kp * error + Kd * derivative;
  (*lastError) = error;
}

/******************************************************************************
 * REACTION WHEEL DISTRIBUTION
 ******************************************************************************/

// Distribution matrix A for 4 wheels, each at angle beta = 54.73 deg
double beta = 54.73 * (PI / 180);  // Radians
double A[3][4] = {
    {cos(beta),   0,         -cos(beta),   0},
    {0,           cos(beta),  0,          -cos(beta)},
    {sin(beta),   sin(beta),  sin(beta),   sin(beta)}
};

// Penrose pseudoinverse of A
double A_pinv[4][3];
double I = 0.0089;  // Example moment of inertia for each wheel

/**
 * @brief Compute the Moore-Penrose pseudoinverse of A (4x3) using (Aᵀ * A)⁻¹ * Aᵀ
 */
void computePseudoinverse() {
  double A_T[4][3];   // Transpose of A
  double ATA[3][3];   // Aᵀ * A
  double ATA_inv[3][3]; // (Aᵀ * A)⁻¹

  // Compute Aᵀ (Transpose of A)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      A_T[j][i] = A[i][j];
    }
  }

  // Compute Aᵀ * A (3x3 matrix)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ATA[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        ATA[i][j] += A_T[k][i] * A[k][j];
      }
    }
  }

  // Compute the inverse of ATA (3x3 matrix inversion)
  double det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
               ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
               ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

  if (fabs(det) < 1e-6) {
    Serial.println("Matrix inversion failed: determinant is too close to zero.");
    return;
  }

  double invDet = 1.0 / det;

  ATA_inv[0][0] = invDet * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]);
  ATA_inv[0][1] = invDet * (ATA[0][2] * ATA[2][1] - ATA[0][1] * ATA[2][2]);
  ATA_inv[0][2] = invDet * (ATA[0][1] * ATA[1][2] - ATA[0][2] * ATA[1][1]);

  ATA_inv[1][0] = invDet * (ATA[1][2] * ATA[2][0] - ATA[1][0] * ATA[2][2]);
  ATA_inv[1][1] = invDet * (ATA[0][0] * ATA[2][2] - ATA[0][2] * ATA[2][0]);
  ATA_inv[1][2] = invDet * (ATA[0][2] * ATA[1][0] - ATA[0][0] * ATA[1][2]);

  ATA_inv[2][0] = invDet * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);
  ATA_inv[2][1] = invDet * (ATA[0][1] * ATA[2][0] - ATA[0][0] * ATA[2][1]);
  ATA_inv[2][2] = invDet * (ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]);

  // Compute A⁺ = (Aᵀ * A)⁻¹ * Aᵀ (4x3 pseudoinverse)
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      A_pinv[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        A_pinv[i][j] += ATA_inv[k][j] * A_T[i][k];
      }
    }
  }
}

/**
 * @brief Compute the rate of change of angular momentum (H_dot) and reaction wheels (W_dot)
 */
void computeWdotAndHdot(double T[3], double W_dot[4], double H_dot[3]) {
  // H_dot = -T (Newton-meters)
  H_dot[0] = -T[0];
  H_dot[1] = -T[1];
  H_dot[2] = -T[2];

  // W_dot = (1 / I) * A_pinv * H_dot
  // (In practice, each wheel might have its own inertia, so adjust accordingly.)
  for (int i = 0; i < 4; i++) {
    W_dot[i] = 0;
    for (int j = 0; j < 3; j++) {
      W_dot[i] += A_pinv[i][j] * H_dot[j];
    }
  }
}

/******************************************************************************
 * CONVERT RAD/S <-> RPM
 ******************************************************************************/
double radToRPM(double radPerSec) {
  return (radPerSec * 60.0) / (2.0 * PI);
}
double rpmToRad(double rpmVal) {
  return (rpmVal * 2.0 * PI) / 60.0;
}

/******************************************************************************
 * REACTION WHEEL ANGULAR VELOCITY
 ******************************************************************************/
double W[4] = {0.0, 0.0, 0.0, 0.0};  // Current angular velocity of each wheel (rad/s)
double W_dot[4];                    // Rate of change of angular velocity (rad/s^2)
double H_dot[3];                    // Rate of change of angular momentum
unsigned long previousMillis = 0;   // For integration timing

/******************************************************************************
 * BLDC CONTROL & FEEDBACK
 ******************************************************************************/
Servo esc[4];   // ESC signals for 4 motors
Servo dir[4];   // Direction signals for 4 motors

// Hall sensor pins for each motor
const int hallPins[4] = {2, 3, 18, 19}; // Assign appropriate pins
volatile unsigned long lastPulseTime[4] = {0, 0, 0, 0};
volatile unsigned long pulseInterval[4] = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
volatile bool newPulse[4] = {false, false, false, false};

unsigned long prevlogtime = 0;
const long logint = 30;

float rpmMeasured[4]       = {0, 0, 0, 0};  // Raw measured RPM
float rpmFiltered[4]       = {0, 0, 0, 0};  // Moving-average filtered
float rpmMedianFiltered[4] = {0, 0, 0, 0};  // Median-filtered

// Moving Average Filter
const int filterSize = 5;
float rpmFilter[4][filterSize] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};

// Median Filter
const int medianFilterSize = 5;
float rpmMedianBuffer[4][medianFilterSize] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};

/******************************************************************************
 * BLDC MOTOR PID CONTROL
 ******************************************************************************/
// PID constants for each motor
double Kp_motor[4] = {1.0, 1.0, 1.0, 1.0};
double Ki_motor[4] = {0.0, 0.0, 0.0, 0.0};
double Kd_motor[4] = {0.0, 0.0, 0.0, 0.0};

// PID variables for each motor
double integral[4] = {0.0, 0.0, 0.0, 0.0};
double lastError[4] = {0.0, 0.0, 0.0, 0.0};

/**
 * @brief PID controller for BLDC motor speed control
 *
 * @param motorIndex Index of the motor (0-3)
 * @param desiredRPM Desired RPM for the motor
 * @param actualRPM  Actual RPM measured from the Hall sensor
 * @return PWM percentage to apply to the ESC
 */
int computeMotorPID(int motorIndex, double desiredRPM, double actualRPM) {
  double error = desiredRPM - actualRPM;

  integral[motorIndex] += error;

  // Limit integral term to prevent windup (adjust limits as needed)
  if (integral[motorIndex] > 100) integral[motorIndex] = 100;
  if (integral[motorIndex] < -100) integral[motorIndex] = -100;

  double derivative = error - lastError[motorIndex];

  double output = Kp_motor[motorIndex] * error + Ki_motor[motorIndex] * integral[motorIndex] + Kd_motor[motorIndex] * derivative;

  lastError[motorIndex] = error;

  // Convert PID output to PWM percentage (adjust scaling as needed)
  int pwmPercent = (int)output;

  // Clamp PWM percentage to valid range
  if (pwmPercent < 15) pwmPercent = 15;
  if (pwmPercent > 100) pwmPercent = 100;

  return pwmPercent;
}

/******************************************************************************
 * LOOKUP TABLE: PWM PERCENT -> RPM (Filtered)
 *
 * This table is for 24V, 6S, Motor #2. Each entry is:
 *   PWM%  =>  RPM
 * We'll do linear interpolation for intermediate values.
 ******************************************************************************/
const int numPoints = 18;
int pwmArray[numPoints] = {
  15, 20, 25, 30, 35, 40,
  45, 50, 55, 60, 65, 70,
  75, 80, 85, 90, 95, 100
};
int rpmArray[numPoints] = {
  537, 1124, 1595, 2086, 2557, 3043,
  3518, 4005, 4740, 5291, 5760, 6240,
  7111, 7625, 7957, 8403, 8436, 8438
};

/**
 * @brief  Linear interpolation to convert desired RPM to a PWM percentage [15..100].
 *         Clamps if below or above the table range.
 * @param  desiredRPM The target RPM
 * @return The PWM percentage in [15..100]
 */
int rpmToPWM(float desiredRPM) {
  // If below the first entry, return the first PWM
  if (desiredRPM <= rpmArray[0]) {
    return pwmArray[0];
  }
  // If above the last entry, return the last PWM
  if (desiredRPM >= rpmArray[numPoints - 1]) {
    return pwmArray[numPoints - 1];
  }
  // Otherwise, find two adjacent points for interpolation
  for (int i = 0; i < numPoints - 1; i++) {
    if (desiredRPM >= rpmArray[i] && desiredRPM < rpmArray[i + 1]) {
      float rangeRPM = rpmArray[i + 1] - rpmArray[i];
      float ratio    = (desiredRPM - rpmArray[i]) / rangeRPM;

      float rangePWM = pwmArray[i + 1] - pwmArray[i];
      float pwmVal   = pwmArray[i] + ratio * rangePWM;
      return (int)(pwmVal);
    }
  }
  // Fallback
  return pwmArray[numPoints - 1];
}

/**
 * @brief Convert a PWM percentage [0..100] to servo microseconds [1000..2000].
 *        If your table starts at 15% = 1000us, you can clamp the lower bound.
 */
int percentToMicroseconds(int pwmPercent) {
  // We clamp below 15% to 15, since your table starts at 15 => 537 RPM
  if (pwmPercent < 15) pwmPercent = 15;
  // We clamp above 100% to 100
  if (pwmPercent > 100) pwmPercent = 100;

  // Now map 15..100 => 1000..2000
  // E.g. 15% => 1000 µs, 100% => 2000 µs
  // You can adjust if you prefer a different scaling
  return map(pwmPercent, 15, 100, 1000, 2000);
}

/******************************************************************************
 * INTERRUPT SERVICE ROUTINE (Hall Sensor)
 ******************************************************************************/
void pulseCounter0() { pulseCounter(0); }
void pulseCounter1() { pulseCounter(1); }
void pulseCounter2() { pulseCounter(2); }
void pulseCounter3() { pulseCounter(3); }

void pulseCounter(int motorIndex) {
  unsigned long currentTime = micros();
  if (lastPulseTime[motorIndex] != 0) {
    pulseInterval[motorIndex] = currentTime - lastPulseTime[motorIndex];
    newPulse[motorIndex] = true;
  }
  lastPulseTime[motorIndex] = currentTime;
}

/******************************************************************************
 * SETUP
 ******************************************************************************/
void setup() {
  Serial.begin(115200);

  // Attach ESC and direction pins
  // Example: ESC on pins [4..7], direction on pins [8..11]
  // Adjust as needed for your wiring
  for (int i = 0; i < 4; i++) {
    esc[i].attach(4 + i);
    dir[i].attach(8 + i);
    // Initialize them
    esc[i].writeMicroseconds(1000);  // minimal
    dir[i].writeMicroseconds(1400);  // neutral direction
  }

  // Hall sensors for each motor
  pinMode(hallPins[0], INPUT_PULLUP);
  pinMode(hallPins[1], INPUT_PULLUP);
  pinMode(hallPins[2], INPUT_PULLUP);
  pinMode(hallPins[3], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPins[0]), pulseCounter0, RISING);
  attachInterrupt(digitalPinToInterrupt(hallPins[1]), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(hallPins[2]), pulseCounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(hallPins[3]), pulseCounter3, RISING);

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Precompute pseudoinverse
  computePseudoinverse();

  // Initialize filter arrays
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < filterSize; j++) {
      rpmFilter[i][j] = 0.0;
    }
    for (int j = 0; j < medianFilterSize; j++) {
      rpmMedianBuffer[i][j] = 0.0;
    }
  }

  Serial.println("System Initialized.");
}

/******************************************************************************
 * LOOP
 ******************************************************************************/
void loop() {
  /**************************************************************************
   * 1) Get IMU data & compute PD torques
   **************************************************************************/
  sensors_event_t event;
  bno.getEvent(&event);
  roll  = event.orientation.x;  // degrees
  pitch = event.orientation.y;  // degrees
  yaw   = event.orientation.z;  // degrees

  // Convert to radians for PD
  double rollRad  = radians(roll);
  double pitchRad = radians(pitch);
  double yawRad   = radians(yaw);

  // PD outputs (torques)
  double torqueRoll, torquePitch, torqueYaw;
  // We can call computePD for each axis. Adjust as needed:
  // e.g. computePD(rollRad, setpointRoll, torqueRoll, Kp_roll, Kd_roll)
  // but watch out for the static variable usage. Alternatively:
  {
    double errorRoll = radians(setpointRoll) - rollRad;
    static double lastErrorRoll = 0.0;
    double dRoll = errorRoll - lastErrorRoll;
    torqueRoll = Kp_roll * errorRoll + Kd_roll * dRoll;
    lastErrorRoll = errorRoll;
  }
  {
    double errorPitch = radians(setpointPitch) - pitchRad;
    static double lastErrorPitch = 0.0;
    double dPitch = errorPitch - lastErrorPitch;
    torquePitch = Kp_pitch * errorPitch + Kd_pitch * dPitch;
    lastErrorPitch = errorPitch;
  }
  {
    double errorYaw = radians(setpointYaw) - yawRad;
    static double lastErrorYaw = 0.0;
    double dYaw = errorYaw - lastErrorYaw;
    torqueYaw = Kp_yaw * errorYaw + Kd_yaw * dYaw;
    lastErrorYaw = errorYaw;
  }

  // Prepare torque vector T
  double T[3] = {torqueRoll, torquePitch, torqueYaw};

  /**************************************************************************
   * 2) Compute W_dot, H_dot & integrate W
   **************************************************************************/
  computeWdotAndHdot(T, W_dot, H_dot);

  unsigned long currentMillis = millis();
  double deltaT = (currentMillis - previousMillis) / 1000.0; // seconds
  previousMillis = currentMillis;

  // Integrate W_dot to get W (rad/s)
  for (int i = 0; i < 4; i++) {
    W[i] += W_dot[i] * deltaT;
  }

  /**************************************************************************
   * 3) Hall-sensor feedback for each motor
   **************************************************************************/
  for (int i = 0; i < 4; i++) {
    if (newPulse[i] && pulseInterval[i] > 0 && pulseInterval[i] < 1000000UL) {
      float pulseFrequency = 1000000.0f / pulseInterval[i];
      // Example: 7 pulses per revolution => rpm = freq * 60 / 7
      rpmMeasured[i] = (pulseFrequency * 60.0f) / 7.0f;
      newPulse[i] = false;

      // Apply filters
      rpmFiltered[i]       = movingAverageFilter(i, rpmMeasured[i]);
      rpmMedianFiltered[i] = medianFilter(i, rpmMeasured[i]);
    } else if (millis() - lastPulseTime[i] > 500) {
      // If no pulse for 500ms, assume zero RPM
      rpmMeasured[i]       = 0;
      rpmFiltered[i]       = movingAverageFilter(i, 0);
      rpmMedianFiltered[i] = medianFilter(i, 0);
    }
  }

  /**************************************************************************
   * 4) Motor PID control
   **************************************************************************/
  for (int i = 0; i < 4; i++) {
    double desiredRadSec = W[i];
    bool reverse = false;

    // Check sign for direction
    if (desiredRadSec < 0) {
      reverse = true;
      desiredRadSec = -desiredRadSec; // make it positive for RPM
    }

    // Convert rad/s to RPM
    double desiredRPM = radToRPM(desiredRadSec);

    // Compute PWM percentage from PID controller
    int pwmPercent = computeMotorPID(i, desiredRPM, rpmMedianFiltered[i]);

    // Convert that to servo microseconds
    int pwmMicroseconds = percentToMicroseconds(pwmPercent);

    // Direction pin
    if (reverse) {
      // e.g. 2000µs => "reverse"
      dir[i].writeMicroseconds(2000);
    } else {
      // e.g. 1400µs => "forward/neutral"
      dir[i].writeMicroseconds(1400);
    }

    // Send to ESC
    esc[i].writeMicroseconds(pwmMicroseconds);

    /**************************************************************************
     * 5) Logging - Modified to print requested data
     **************************************************************************/
    // Print Motor Number, Motor Speed, Direction
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(", Speed: ");
    Serial.print(rpmMedianFiltered[i]); // Or use rpmFiltered[i], or rpmMeasured[i]
    Serial.print(", Direction: ");
    Serial.println(reverse ? 0 : 1); // 0 for reverse, 1 for forward
  }
  delay(10);
}

/******************************************************************************
 * FILTERS
 ******************************************************************************/

// Moving Average Filter
float movingAverageFilter(int motorIndex, float rpmVal) {
  // Shift
  for (int i = 0; i < filterSize - 1; i++) {
    rpmFilter[motorIndex][i] = rpmFilter[motorIndex][i + 1];
  }
  rpmFilter[motorIndex][filterSize - 1] = rpmVal;

  // Sum
  float sum = 0;
  for (int j = 0; j < filterSize; j++) {
    sum += rpmFilter[motorIndex][j];
  }
  return sum / filterSize;
}

// Median Filter
float medianFilter(int motorIndex, float rpmVal) {
  // Shift
  for (int i = 0; i < medianFilterSize - 1; i++) {
    rpmMedianBuffer[motorIndex][i] = rpmMedianBuffer[motorIndex][i + 1];
  }
  rpmMedianBuffer[motorIndex][medianFilterSize - 1] = rpmVal;

  // Copy & sort
  float temp[medianFilterSize];
  for (int i = 0; i < medianFilterSize; i++) {
    temp[i] = rpmMedianBuffer[motorIndex][i];
  }
  // Simple bubble sort
  for (int i = 0; i < medianFilterSize - 1; i++) {
    for (int j = i + 1; j < medianFilterSize; j++) {
      if (temp[j] < temp[i]) {
        float swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  // Middle element
  return temp[medianFilterSize / 2];
}
