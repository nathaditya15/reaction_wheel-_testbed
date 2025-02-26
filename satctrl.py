import smbus  # For I2C communication with the IMU
import time
import math
import numpy as np  # For matrix operations (NumPy must be installed: pip install numpy)
import RPi.GPIO as GPIO  # For GPIO control (RPi.GPIO must be installed: pip install RPi.GPIO)

###############################################################################
# CONFIGURATION -  ADJUST THESE BASED ON YOUR HARDWARE CONNECTIONS
###############################################################################

# I2C address of the BNO055 IMU (check with i2cdetect if unsure)
BNO055_I2C_ADDR = 0x28  # or 0x29  (check your sensor)

# GPIO pins for ESC and direction control (BCM numbering)
ESC_PINS   = [4, 17, 27, 22]  # Example:  [GPIO4, GPIO17, GPIO27, GPIO22]
DIR_PINS   = [5, 6, 13, 19]  # Example:  [GPIO5, GPIO6, GPIO13, GPIO19]
HALL_PINS  = [23, 24, 25, 12]  # Example: [GPIO23, GPIO24, GPIO25, GPIO12]

# ESC PWM frequency (Hz).  Important for smooth motor control.  Experiment.
ESC_PWM_FREQUENCY = 50  # Standard servo/ESC frequency.  Try higher (e.g., 400)

# Direction signal HIGH/LOW for forward direction
DIR_FORWARD = GPIO.HIGH   # Or GPIO.LOW depending on your wiring

###############################################################################
# IMU REGISTERS (BNO055) -  Refer to the BNO055 datasheet
###############################################################################
BNO055_OPR_MODE   = 0x3D
BNO055_PAGE_ID    = 0x07
BNO055_SYS_TRIGGER = 0x3F
BNO055_EUL_HEADING = 0x1A  # Yaw
BNO055_EUL_ROLL    = 0x1C  # Roll
BNO055_EUL_PITCH   = 0x1E  # Pitch
BNO055_CALIB_STAT  = 0x35
BNO055_TEMP        = 0x34

# Operation Modes
BNO055_OPERATION_MODE_CONFIG = 0x00
BNO055_OPERATION_MODE_IMU    = 0x08  # or  NDOF (0x0C) for sensor fusion

###############################################################################
# GLOBAL VARIABLES
###############################################################################

# IMU & Attitude Control
roll  = 0.0
pitch = 0.0
yaw   = 0.0

# PD gains (Tuning parameters!)
Kp_roll  = 1.0
Kd_roll  = 0.05
Kp_pitch = 1.0
Kd_pitch = 0.05
Kp_yaw   = 1.0
Kd_yaw   = 0.0

lastErrorRoll  = 0.0
lastErrorPitch = 0.0
lastErrorYaw   = 0.0

# Reaction Wheel Distribution
beta = 54.73 * (math.pi / 180)  # Radians
A = np.array([
    [math.cos(beta), 0,          -math.cos(beta), 0],
    [0,          math.cos(beta), 0,          -math.cos(beta)],
    [math.sin(beta), math.sin(beta),  math.sin(beta),  math.sin(beta)]
])
A_pinv = np.linalg.pinv(A)  # Calculate pseudoinverse using NumPy
I = 0.0089  # Example moment of inertia

# Reaction Wheel Angular Velocity
W = [0.0, 0.0, 0.0, 0.0]  # Current angular velocity of each wheel (rad/s)
W_dot = [0.0, 0.0, 0.0, 0.0]  # Rate of change of angular velocity (rad/s^2)
H_dot = [0.0, 0.0, 0.0]  # Rate of change of angular momentum

# BLDC Control & Feedback
pwm = [None] * 4 # PWM objects for each motor
hall_last_pulse_time = [0] * 4
hall_pulse_interval  = [float('inf')] * 4  # Initialize to infinity
hall_new_pulse       = [False] * 4
rpm_measured         = [0.0] * 4
rpm_filtered         = [0.0] * 4
rpm_median_filtered  = [0.0] * 4

# Filter parameters
FILTER_SIZE = 5
MEDIAN_FILTER_SIZE = 5
rpm_filter = [[0.0] * FILTER_SIZE for _ in range(4)]
rpm_median_buffer = [[0.0] * MEDIAN_FILTER_SIZE for _ in range(4)]

# BLDC Motor PID Control
Kp_motor = [1.0] * 4
Ki_motor = [0.0] * 4
Kd_motor = [0.0] * 4
integral = [0.0] * 4
lastError = [0.0] * 4

# PWM/RPM Lookup Table
pwm_array = [15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
rpm_array = [537, 1124, 1595, 2086, 2557, 3043, 3518, 4005, 4740, 5291, 5760, 6240, 7111, 7625, 7957, 8403, 8436, 8438]
num_points = len(pwm_array)

###############################################################################
# I2C COMMUNICATION FUNCTIONS
###############################################################################

bus = smbus.SMBus(1)  # Use I2C bus 1 (check your Raspberry Pi configuration)

def write_byte(register, value):
    bus.write_byte_data(BNO055_I2C_ADDR, register, value)

def read_byte(register):
    return bus.read_byte_data(BNO055_I2C_ADDR, register)

def read_word(register):
    # BNO055 uses little-endian, so read low byte first
    low = read_byte(register)
    high = read_byte(register + 1)
    value = (high << 8) | low
    return value

def read_signed_word(register):
    value = read_word(register)
    if value > 32767:
        value -= 65536
    return value

###############################################################################
# IMU FUNCTIONS
###############################################################################

def bno055_setup():
    """Initializes the BNO055 IMU."""
    write_byte(BNO055_OPR_MODE, BNO055_OPERATION_MODE_CONFIG)
    time.sleep(0.025)  # Wait 25ms
    write_byte(BNO055_SYS_TRIGGER, 0x20) # Reset the MCU
    time.sleep(0.01)
    while read_byte(BNO055_PAGE_ID) != 0:
        time.sleep(0.01)
    write_byte(BNO055_SYS_TRIGGER, 0x00)
    time.sleep(0.01)
    write_byte(BNO055_OPR_MODE, BNO055_OPERATION_MODE_IMU)  # Or NDOF
    time.sleep(0.1) # Wait for mode change

def get_euler_angles():
    """Reads the Euler angles (roll, pitch, yaw) from the IMU."""
    global roll, pitch, yaw
    heading = read_signed_word(BNO055_EUL_HEADING) / 16.0
    roll_raw = read_signed_word(BNO055_EUL_ROLL) / 16.0
    pitch_raw = read_signed_word(BNO055_EUL_PITCH) / 16.0
    roll = roll_raw
    pitch = pitch_raw
    yaw = heading
    return roll, pitch, yaw

###############################################################################
# ATTITUDE CONTROL FUNCTIONS
###############################################################################

def compute_pd(current_angle, setpoint_deg, Kp, Kd, last_error):
    """A simple PD controller for attitude control.  Returns (output_torque, new_last_error)"""
    setpoint_rad = math.radians(setpoint_deg)
    error = setpoint_rad - current_angle
    derivative = error - last_error
    output_torque = Kp * error + Kd * derivative
    return output_torque, error

###############################################################################
# REACTION WHEEL DISTRIBUTION FUNCTIONS
###############################################################################

def compute_wdot_and_hdot(T):
    """Computes the rate of change of angular momentum (H_dot) and reaction wheels (W_dot)."""
    global H_dot, W_dot
    H_dot = [-T[0], -T[1], -T[2]]  # H_dot = -T

    # W_dot = (1 / I) * A_pinv * H_dot  (NumPy handles the matrix multiplication)
    W_dot = (1 / I) * np.dot(A_pinv, H_dot)
    return W_dot, H_dot

###############################################################################
# CONVERT RAD/S <-> RPM
###############################################################################

def rad_to_rpm(rad_per_sec):
    return (rad_per_sec * 60.0) / (2.0 * math.pi)

def rpm_to_rad(rpm_val):
    return (rpm_val * 2.0 * math.pi) / 60.0

###############################################################################
# BLDC CONTROL & FEEDBACK FUNCTIONS
###############################################################################

def hall_callback(channel):
    """Interrupt service routine for Hall sensor."""
    motor_index = HALL_PINS.index(channel)  # Determine which motor triggered
    current_time = time.time()
    if hall_last_pulse_time[motor_index] != 0:
        hall_pulse_interval[motor_index] = current_time - hall_last_pulse_time[motor_index]
        hall_new_pulse[motor_index] = True
    hall_last_pulse_time[motor_index] = current_time

def initialize_esc(esc_pin, dir_pin):
    """Initializes an ESC and sets direction pin."""
    GPIO.setup(esc_pin, GPIO.OUT)
    GPIO.setup(dir_pin, GPIO.OUT)
    pwm_out = GPIO.PWM(esc_pin, ESC_PWM_FREQUENCY)  # Create PWM object
    pwm_out.start(0)  # Start PWM with 0% duty cycle (motor stopped)
    GPIO.output(dir_pin, DIR_FORWARD)  # Set default direction (adjust as needed)
    return pwm_out

def calculate_rpm(interval):
     """Calculates RPM from pulse interval."""
     if interval > 0 and interval < 1: # reasonable time
         frequency = 1.0 / interval
         rpm = (frequency / 3.0) * 60.0  # Assuming 3 pulses per revolution
         return rpm
     else:
         return 0.0

def moving_average_filter(motor_index, new_rpm):
    """Applies a moving average filter to the RPM measurement."""
    global rpm_filter, rpm_filtered
    # Shift the filter window
    for i in range(FILTER_SIZE - 1):
        rpm_filter[motor_index][i] = rpm_filter[motor_index][i + 1]
    rpm_filter[motor_index][FILTER_SIZE - 1] = new_rpm

    # Calculate the average
    rpm_filtered[motor_index] = sum(rpm_filter[motor_index]) / FILTER_SIZE
    return rpm_filtered[motor_index]

def median_filter(motor_index, new_rpm):
    """Applies a median filter to the RPM measurement."""
    global rpm_median_buffer, rpm_median_filtered
    # Shift the filter window
    for i in range(MEDIAN_FILTER_SIZE - 1):
        rpm_median_buffer[motor_index][i] = rpm_median_buffer[motor_index][i + 1]
    rpm_median_buffer[motor_index][MEDIAN_FILTER_SIZE - 1] = new_rpm

    # Calculate the median
    temp_buffer = sorted(rpm_median_buffer[motor_index])
    rpm_median_filtered[motor_index] = temp_buffer[MEDIAN_FILTER_SIZE // 2]
    return rpm_median_filtered[motor_index]

###############################################################################
# BLDC MOTOR PID CONTROL FUNCTIONS
###############################################################################

def compute_motor_pid(motor_index, desired_rpm, actual_rpm):
    """PID controller for BLDC motor speed control."""
    error = desired_rpm - actual_rpm
    integral[motor_index] += error

    # Limit integral term to prevent windup
    integral[motor_index] = max(min(integral[motor_index], 100), -100)

    derivative = error - lastError[motor_index]
    output = Kp_motor[motor_index] * error + Ki_motor[motor_index] * integral[motor_index] + Kd_motor[motor_index] * derivative
    lastError[motor_index] = error

    # Convert PID output to PWM percentage
    pwm_percent = int(output)

    # Clamp PWM percentage to valid range.  The Arduino code clamped to 15-100.  Adjust based on your ESC.
    pwm_percent = max(min(pwm_percent, 100), 15)
    return pwm_percent

###############################################################################
# LOOKUP TABLE FUNCTIONS
###############################################################################

def rpm_to_pwm(desired_rpm):
    """Linear interpolation to convert desired RPM to a PWM percentage [15..100]."""
    if desired_rpm <= rpm_array[0]:
        return pwm_array[0]
    if desired_rpm >= rpm_array[num_points - 1]:
        return pwm_array[num_points - 1]

    for i in range(num_points - 1):
        if desired_rpm >= rpm_array[i] and desired_rpm < rpm_array[i + 1]:
            range_rpm = rpm_array[i + 1] - rpm_array[i]
            ratio = (desired_rpm - rpm_array[i]) / range_rpm
            range_pwm = pwm_array[i + 1] - pwm_array[i]
            pwm_val = pwm_array[i] + ratio * range_pwm
            return int(pwm_val)

    return pwm_array[num_points - 1]  # Fallback

def percent_to_dutycycle(pwm_percent):
    """Converts a PWM percentage [0..100] to a PWM duty cycle [0..100]."""
    #The Arduino code clamped below 15% since the table started at 15% = 537RPM
    pwm_percent = max(min(pwm_percent, 100), 15) #Clamp PWM percentage to valid range
    return pwm_percent # Duty cycle is same as percentage

###############################################################################
# MAIN SETUP
###############################################################################

def setup():
    """Initializes the system."""
    global pwm

    print("System Initializing...")

    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering

    # Initialize ESCs and direction pins
    pwm = [initialize_esc(ESC_PINS[i], DIR_PINS[i]) for i in range(4)]

    # Initialize Hall sensors
    for i in range(4):
        GPIO.setup(HALL_PINS[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Internal pull-up
        GPIO.add_event_detect(HALL_PINS[i], GPIO.RISING, callback=hall_callback)

    # Initialize IMU
    try:
        bno055_setup()
        print("BNO055 IMU Initialized")
    except Exception as e:
        print(f"Error initializing BNO055: {e}")
        exit()

    #Initialize filter arrays (already initialized as global variables)

    print("System Initialized.")

###############################################################################
# MAIN LOOP
###############################################################################

def loop():
    """Main control loop."""
    global roll, pitch, yaw, lastErrorRoll, lastErrorPitch, lastErrorYaw, W, hall_new_pulse
    setpoint_roll = 0.0  #Desired angles in degrees
    setpoint_pitch = 0.0
    setpoint_yaw = 0.0

    previous_millis = time.time() * 1000 # milliseconds

    try:
        while True:
            start_time = time.time()

            # 1) Get IMU data & compute PD torques
            roll, pitch, yaw = get_euler_angles()

            # PD outputs (torques)
            torqueRoll, lastErrorRoll = compute_pd(math.radians(roll), setpoint_roll, Kp_roll, Kd_roll, lastErrorRoll)
            torquePitch, lastErrorPitch = compute_pd(math.radians(pitch), setpoint_pitch, Kp_pitch, Kd_pitch, lastErrorPitch)
            torqueYaw, lastErrorYaw = compute_pd(math.radians(yaw), setpoint_yaw, Kp_yaw, Kd_yaw, lastErrorYaw)

            # Prepare torque vector T
            T = [torqueRoll, torquePitch, torqueYaw]

            # 2) Compute W_dot, H_dot & integrate W
            W_dot, H_dot = compute_wdot_and_hdot(T)

            current_millis = time.time() * 1000 # milliseconds
            delta_t = (current_millis - previous_millis) / 1000.0  # seconds
            previous_millis = current_millis

            # Integrate W_dot to get W (rad/s)
            for i in range(4):
                W[i] += W_dot[i] * delta_t

            # 3) Hall-sensor feedback for each motor
            for i in range(4):
                if hall_new_pulse[i] and hall_pulse_interval[i] > 0 and hall_pulse_interval[i] < 1:
                    # Frequency limit increased to avoid zero division
                    rpm_measured[i] = calculate_rpm(hall_pulse_interval[i])

                    # Apply filters
                    rpm_filtered[i] = moving_average_filter(i, rpm_measured[i])
                    rpm_median_filtered[i] = median_filter(i, rpm_measured[i])
                    hall_new_pulse[i] = False # Reset flag

                # 4) Motor PID Control
                desired_rpm = rad_to_rpm(W[i])  # Convert desired angular velocity to RPM
                pwm_percent = compute_motor_pid(i, desired_rpm, rpm_median_filtered[i]) # Use filtered RPM

                # 5) Lookup table & ESC control
                #pwm_percent = rpm_to_pwm(desired_rpm) #Use lookup table to get initial estimate
                duty_cycle = percent_to_dutycycle(pwm_percent) #Convert percentage to duty cycle

                pwm[i].ChangeDutyCycle(duty_cycle)  # Set ESC speed

                # Print motor data (for debugging)
                #print(f"Motor {i+1}: Desired RPM: {desired_rpm:.2f}, Actual RPM: {rpm_median_filtered[i]:.2f}, PWM: {pwm_percent}, Duty: {duty_cycle:.2f}")

            # Print IMU data
            #print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
            end_time = time.time()
            loop_time = end_time - start_time
            #print(f"Loop time: {loop_time:.4f} seconds")
            time.sleep(0.001)  # Short delay (adjust as needed)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Stop motors and cleanup GPIO
        for p in pwm:
            p.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    setup()
    loop()
