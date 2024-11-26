from machine import Pin, PWM, ADC
import time

SERVO_LIMITS = {
    "shoulder": {"min_duty": 2300, "max_duty": 7500, "min_angle": 0, "max_angle": 180},
    "elbow": {"min_duty": 2300, "max_duty": 7500, "min_angle": 0, "max_angle": 180}
}

PEN_SERVO = {"up": 2300, "down": 3000}
ABSOLUTE_MIN_DUTY, ABSOLUTE_MAX_DUTY = 2300, 7500

def initialize():
    pwm = {
        "shoulder": PWM(Pin(0)),
        "elbow": PWM(Pin(1)),
        "pen": PWM(Pin(2))
    }
    for servo in pwm.values():
        servo.freq(50)
    
    dials = {
        "shoulder": ADC(26),
        "elbow": ADC(27)
    }
    
    toggle_button = Pin(13, Pin.IN, Pin.PULL_DOWN)
    quit_button = Pin(11, Pin.IN, Pin.PULL_DOWN)
    led = Pin(25, Pin.OUT)
    
    pwm["pen"].duty_u16(PEN_SERVO["up"])
    return pwm, dials, toggle_button, quit_button, led

def read_dials(dials):
    shoulder_raw = dials["shoulder"].read_u16()
    elbow_raw = dials["elbow"].read_u16()
    
    shoulder_angle = (shoulder_raw * 180) // 65535
    elbow_angle = (elbow_raw * 180) // 65535
    
    return shoulder_angle, elbow_angle

def move_servos(shoulder_angle, elbow_angle, pen_down, pwm):
    shoulder_duty = 2300 + (shoulder_angle * (7500 - 2300) // 180)
    elbow_duty = 2300 + (elbow_angle * (7500 - 2300) // 180)
    
    pwm["shoulder"].duty_u16(shoulder_duty)
    pwm["elbow"].duty_u16(elbow_duty)
    pwm["pen"].duty_u16(PEN_SERVO["down"] if pen_down else PEN_SERVO["up"])

def main():
    pwm, dials, toggle_button, quit_button, led = initialize()
    system_on = False
    pen_down = False
    led.value(0)
    
    print("System OFF. Press GP13 to turn ON/OFF")
    
    last_button_state = toggle_button.value()
    
    while True:
        current_button_state = toggle_button.value()
        
        # Detect button press and release cycle
        if current_button_state != last_button_state:
            time.sleep(0.05)  # Debounce
            if current_button_state == 1:  # Button pressed
                system_on = not system_on
                led.value(1 if system_on else 0)
                print(f"System {'ON' if system_on else 'OFF'}")
        
        last_button_state = current_button_state
        
        # Quit button check
        if quit_button.value() == 1:
            print("Quitting...")
            break
        
        if system_on:
            shoulder_angle, elbow_angle = read_dials(dials)
            move_servos(shoulder_angle, elbow_angle, pen_down, pwm)
        
        time.sleep(0.01)
    
    # Cleanup
    pwm["pen"].duty_u16(PEN_SERVO["up"])
    for servo in pwm.values():
        servo.deinit()
    led.value(0)

if __name__ == "__main__":
    main()
