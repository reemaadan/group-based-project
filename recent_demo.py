from machine import Pin, PWM, ADC
import time

# constants for servo limits and pen positions
# these define the range of motion for the shoulder and elbow servos in terms of duty cycle (signal strength)
# and angles. for example, a duty cycle of 2300 corresponds to an angle of 0 degrees.
SERVO_LIMITS = {
    "shoulder": {"min_duty": 2300, "max_duty": 7500, "min_angle": 0, "max_angle": 180},
    "elbow": {"min_duty": 2300, "max_duty": 7500, "min_angle": 0, "max_angle": 180},
}
# positions for the pen servo: "up" means the pen is raised, "down" means the pen is touching the surface.
PEN_SERVO = {"up": 2300, "down": 3000}
# absolute duty cycle limits ensure servos do not exceed their physical range
ABSOLUTE_MIN_DUTY = 2300
ABSOLUTE_MAX_DUTY = 7500

def initialize():
    """initialize pwm for servos, adc for dials, and buttons."""
    # create pwm objects for the shoulder, elbow, and pen servos
    pwm = {
        "shoulder": PWM(Pin(0)),  # shoulder servo connected to gpio pin 0
        "elbow": PWM(Pin(1)),     # elbow servo connected to gpio pin 1
        "pen": PWM(Pin(2)),       # pen servo connected to gpio pin 2
    }
    # set the pwm frequency for all servos to 50 hz (standard for hobby servos)
    for servo in pwm.values():
        servo.freq(50)

    # create adc objects for the potentiometers (dials)
    dials = {
        "shoulder": ADC(26),  # shoulder dial connected to adc pin 26
        "elbow": ADC(27),     # elbow dial connected to adc pin 27
    }

    # configure buttons: on_off_button toggles the system, pen_button toggles the pen position
    on_off_button = Pin(13, Pin.IN)  # sw4 (gp13) configured as input
    pen_button = Pin(12, Pin.IN)     # sw3 (gp12) configured as input

    # set the pen to the "up" position by default
    pwm["pen"].duty_u16(PEN_SERVO["up"])

    return pwm, dials, on_off_button, pen_button

def map_value(x, in_min, in_max, out_min, out_max):
    """map input value from one range to another."""
    # this function takes a value `x` in a given range (`in_min` to `in_max`)
    # and maps it proportionally to a new range (`out_min` to `out_max`).
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def read_dials(dials):
    """read angles from the dials (potentiometers)."""
    # read raw adc values from the shoulder and elbow dials
    shoulder_raw = dials["shoulder"].read_u16()  # adc returns a 16-bit value (0 to 65535)
    elbow_raw = dials["elbow"].read_u16()
    # map raw adc values to angles (0 to 180 degrees)
    shoulder_angle = map_value(shoulder_raw, 0, 65535, 0, 180)
    elbow_angle = map_value(elbow_raw, 0, 65535, 0, 180)
    return shoulder_angle, elbow_angle

def angle_to_duty_cycle(angle, servo_name):
    """convert angle to duty cycle based on servo limits."""
    # use servo-specific limits to calculate the appropriate duty cycle
    limits = SERVO_LIMITS[servo_name]
    duty_cycle = int(
        (angle / 180) * (limits["max_duty"] - limits["min_duty"]) + limits["min_duty"]
    )
    # clamp the duty cycle to ensure it stays within absolute safe limits
    return min(max(duty_cycle, ABSOLUTE_MIN_DUTY), ABSOLUTE_MAX_DUTY)

def move_servos(shoulder_angle, elbow_angle, pen_down, pwm):
    """move servos to the specified angles and pen position."""
    # convert angles to duty cycles for the shoulder and elbow servos
    shoulder_duty = angle_to_duty_cycle(shoulder_angle, "shoulder")
    elbow_duty = angle_to_duty_cycle(elbow_angle, "elbow")
    # set the duty cycle for each servo to move it to the desired position
    pwm["shoulder"].duty_u16(shoulder_duty)
    pwm["elbow"].duty_u16(elbow_duty)
    # move the pen to either the "up" or "down" position based on pen_down flag
    pen_position = PEN_SERVO["down"] if pen_down else PEN_SERVO["up"]
    pwm["pen"].duty_u16(pen_position)

# main program starts here
pwm, dials, on_off_button, pen_button = initialize()  # initialize all components
system_active = False  # system starts in the off state
pen_down = False       # pen starts in the "up" position
last_on_off_state = 0  # track the last state of the on/off button
last_pen_state = 0     # track the last state of the pen button
last_button_time = time.ticks_ms()  # store the last button press time for debouncing
previous_angles = {"shoulder": 0, "elbow": 0}  # track last servo positions

# print instructions for the user
print("system ready. press sw4 (gp13) to turn system on/off.")
print("when system is on, press sw3 (gp12) to toggle pen up/down.")

try:
    while True:
        # check the on/off button (sw4)
        current_on_off_state = on_off_button.value()  # read the current state of the button
        current_time = time.ticks_ms()  # get the current time in milliseconds

        # toggle system state on button press (with debounce logic)
        if current_on_off_state == 1 and last_on_off_state == 0:
            if time.ticks_diff(current_time, last_button_time) > 300:  # 300ms debounce
                system_active = not system_active  # toggle the system state
                print(f"system {'on' if system_active else 'off'}")
                
                # always toggle pen position when any button is pressed
                pen_down = not pen_down
                move_servos(previous_angles["shoulder"], previous_angles["elbow"], pen_down, pwm)
                print(f"pen {'down' if pen_down else 'up'}")
                
                last_button_time = current_time  # update the last button press time

        last_on_off_state = current_on_off_state  # update the last state

        # check the pen button (sw3)
        current_pen_state = pen_button.value()  # read the current state of the button
        if current_pen_state == 0 and last_pen_state == 1:
            if time.ticks_diff(current_time, last_button_time) > 300:  # 300ms debounce
                pen_down = not pen_down  # toggle the pen state
                
                # ensure system is active when toggling pen
                system_active = True
                
                # move servos to current position with new pen state
                move_servos(previous_angles["shoulder"], previous_angles["elbow"], pen_down, pwm)
                print(f"system on, pen {'down' if pen_down else 'up'}")
                last_button_time = current_time  # update the last button press time

        last_pen_state = current_pen_state  # update the last state

        # operate system if active
        if system_active:
            # read angles from the dials and move servos accordingly
            shoulder_angle, elbow_angle = read_dials(dials)

            # move servos only if the angles have changed
            if shoulder_angle != previous_angles["shoulder"] or elbow_angle != previous_angles["elbow"]:
                move_servos(shoulder_angle, elbow_angle, pen_down, pwm)
                previous_angles["shoulder"] = shoulder_angle  # update tracked angles
                previous_angles["elbow"] = elbow_angle

        time.sleep(0.01)  # short delay to reduce cpu usage

except KeyboardInterrupt:
    # handle a keyboard interrupt (e.g., ctrl+c) gracefully
    print("\nprogram terminated.")
finally:
    # ensure the pen is raised and pwm is deinitialized when the program exits
    pwm["pen"].duty_u16(PEN_SERVO["up"])
    for servo in pwm.values():
        servo.deinit()

