from machine import Pin, PWM, ADC
import time

# Constants for servo limits and pen positions
SERVO_LIMITS = {
    "shoulder": {"min_duty": 2300, "max_duty": 7500, "min_angle": 0, "max_angle": 180},
    "elbow": {"min_duty": 2300, "max_duty": 7500, "min_angle": 0, "max_angle": 180},
}
PEN_SERVO = {"up": 2300, "down": 3000}
ABSOLUTE_MIN_DUTY = 2300
ABSOLUTE_MAX_DUTY = 7500

def initialize():
    """Initialize PWM for servos, ADC for dials, and buttons."""
    pwm = {
        "shoulder": PWM(Pin(0)),
        "elbow": PWM(Pin(1)),
        "pen": PWM(Pin(2)),
    }
    for servo in pwm.values():
        servo.freq(50)

    dials = {
        "shoulder": ADC(26),
        "elbow": ADC(27),
    }

    # Button setup
    on_off_button = Pin(13, Pin.IN)  # SW4 (GP13) with pull-down resistor
    #pen_button = Pin(12, Pin.IN, Pin.PULL_UP)  # SW3 (GP12)
    pen_button = Pin(12, Pin.IN) 
    # Start pen in the "up" position
    pwm["pen"].duty_u16(PEN_SERVO["up"])

    return pwm, dials, on_off_button, pen_button

def map_value(x, in_min, in_max, out_min, out_max):
    """Map input value from one range to another."""
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def read_dials(dials):
    """Read angles from the dials (potentiometers)."""
    shoulder_raw = dials["shoulder"].read_u16()
    elbow_raw = dials["elbow"].read_u16()
    shoulder_angle = map_value(shoulder_raw, 0, 65535, 0, 180)
    elbow_angle = map_value(elbow_raw, 0, 65535, 0, 180)
    return shoulder_angle, elbow_angle

def angle_to_duty_cycle(angle, servo_name):
    """Convert angle to duty cycle based on servo limits."""
    limits = SERVO_LIMITS[servo_name]
    duty_cycle = int(
        (angle / 180) * (limits["max_duty"] - limits["min_duty"]) + limits["min_duty"]
    )
    return min(max(duty_cycle, ABSOLUTE_MIN_DUTY), ABSOLUTE_MAX_DUTY)

def move_servos(shoulder_angle, elbow_angle, pen_down, pwm):
    """Move servos to the specified angles and pen position."""
    shoulder_duty = angle_to_duty_cycle(shoulder_angle, "shoulder")
    elbow_duty = angle_to_duty_cycle(elbow_angle, "elbow")
    pwm["shoulder"].duty_u16(shoulder_duty)
    pwm["elbow"].duty_u16(elbow_duty)
    pen_position = PEN_SERVO["down"] if pen_down else PEN_SERVO["up"]
    pwm["pen"].duty_u16(pen_position)

# Main program
pwm, dials, on_off_button, pen_button = initialize()
system_active = False  # System starts OFF
pen_down = False       # Pen starts UP
last_on_off_state = 0
last_pen_state = 0
last_button_time = time.ticks_ms()  # For debouncing
previous_angles = {"shoulder": 0, "elbow": 0}  # Track last servo positions

print("System ready. Press SW4 (GP13) to turn system ON/OFF.")
print("When system is ON, press SW3 (GP12) to toggle pen up/down.")

try:
    while True:
        # Check the ON/OFF button (SW4)
        current_on_off_state = on_off_button.value()
        current_time = time.ticks_ms()

        # Toggle system state on button press (debounced)
        if current_on_off_state == 1 and last_on_off_state == 0:
            if time.ticks_diff(current_time, last_button_time) > 300:  # 300ms debounce
                system_active = not system_active
                print(f"System {'ON' if system_active else 'OFF'}")
                
                # Always toggle pen position when any button is pressed
                pen_down = not pen_down
                move_servos(previous_angles["shoulder"], previous_angles["elbow"], pen_down, pwm)
                print(f"Pen {'DOWN' if pen_down else 'UP'}")
                
                last_button_time = current_time

        last_on_off_state = current_on_off_state  # Update last state

        # Check the pen button (SW3)
        current_pen_state = pen_button.value()
        if current_pen_state == 0 and last_pen_state == 1:
            if time.ticks_diff(current_time, last_button_time) > 300:  # 300ms debounce
                pen_down = not pen_down
                
                # Ensure system is active when toggling pen
                system_active = True
                
                # Move servos to current position with new pen state
                move_servos(previous_angles["shoulder"], previous_angles["elbow"], pen_down, pwm)
                print(f"System ON, Pen {'DOWN' if pen_down else 'UP'}")
                last_button_time = current_time

        last_pen_state = current_pen_state  # Update last state

        # Operate system if active
        if system_active:
            # Read angles and move servos
            shoulder_angle, elbow_angle = read_dials(dials)

            # Move servos only if the angles have changed
            if shoulder_angle != previous_angles["shoulder"] or elbow_angle != previous_angles["elbow"]:
                move_servos(shoulder_angle, elbow_angle, pen_down, pwm)
                previous_angles["shoulder"] = shoulder_angle
                previous_angles["elbow"] = elbow_angle

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram terminated.")
finally:
    pwm["pen"].duty_u16(PEN_SERVO["up"])
    for servo in pwm.values():
        servo.deinit()

