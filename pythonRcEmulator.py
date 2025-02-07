from evdev import InputDevice, categorize, ecodes
import pigpio

# Adjust for your controller
CONTROLLER_PATH = '/dev/input/eventX'  # Replace 'X' with the correct input event ID
controller = InputDevice(CONTROLLER_PATH)

# GPIO Pin Configuration
SERVO_PINS = [17, 18, 27, 22, 23, 24]  # GPIO pins for 6 PWM channels
pi = pigpio.pi()

# Initialize PWM signals
for pin in SERVO_PINS:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(pin, 50)  # 50Hz for servos

# Convert joystick value to PWM pulse width (1000-2000 microseconds)
def joystick_to_pwm(value):
    return int(1500 + (value / 32767) * 500)  # Joystick values are usually -32767 to 32767

# Main loop to read inputs and control servos
try:
    for event in controller.read_loop():
        if event.type == ecodes.EV_ABS:  # Analog stick movement
            if event.code == ecodes.ABS_X:  # Left joystick horizontal
                pwm_value = joystick_to_pwm(event.value)
                pi.set_servo_pulsewidth(SERVO_PINS[0], pwm_value)  # Channel 1
            elif event.code == ecodes.ABS_Y:  # Left joystick vertical
                pwm_value = joystick_to_pwm(event.value)
                pi.set_servo_pulsewidth(SERVO_PINS[1], pwm_value)  # Channel 2
            elif event.code == ecodes.ABS_RX:  # Right joystick horizontal
                pwm_value = joystick_to_pwm(event.value)
                pi.set_servo_pulsewidth(SERVO_PINS[2], pwm_value)  # Channel 3
            elif event.code == ecodes.ABS_RY:  # Right joystick vertical
                pwm_value = joystick_to_pwm(event.value)
                pi.set_servo_pulsewidth(SERVO_PINS[3], pwm_value)  # Channel 4
        elif event.type == ecodes.EV_KEY:  # Button press
            if event.code == ecodes.BTN_A and event.value == 1:  # A button pressed
                pi.set_servo_pulsewidth(SERVO_PINS[4], 2000)  # Channel 5 full forward
            elif event.code == ecodes.BTN_B and event.value == 1:  # B button pressed
                pi.set_servo_pulsewidth(SERVO_PINS[4], 1000)  # Channel 5 full reverse
except KeyboardInterrupt:
    print("Exiting...")
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)  # Stop PWM
    pi.stop()