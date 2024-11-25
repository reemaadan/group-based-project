from machine import Pin, PWM, ADC
import time

# Define limits for each servo
SERVO_LIMITS = {
   "shoulder": {
       "min_duty": 2300,
       "max_duty": 7500,
       "min_angle": 0,
       "max_angle": 180
   },
   "elbow": {
       "min_duty": 2300,
       "max_duty": 7500,
       "min_angle": 0,
       "max_angle": 180
   }
}

# Exact pen servo positions - DO NOT MODIFY THESE VALUES
PEN_SERVO = {
   "up": 2300,    # Pen raised from paper
   "down": 3000   # Pen touching paper
}

# Absolute limits - DO NOT MODIFY THESE VALUES
ABSOLUTE_MIN_DUTY = 2300
ABSOLUTE_MAX_DUTY = 7500

def initialize():
   """Initialize PWM for servos, ADC for control dials (potentiometers), and GP13 button"""
   # Initialize servos
   pwm = {
       "shoulder": PWM(Pin(0)),
       "elbow": PWM(Pin(1)),
       "pen": PWM(Pin(2))    # Pen servo on GPIO 2
   }
   for servo in pwm.values():
       servo.freq(50)
   
   # Initialize control dials (potentiometers)
   # Using GPIO 26 (ADC0) and 27 (ADC1) for the control dials
   dials = {
       "shoulder": ADC(26),  # ADC(0) for Pico - shoulder control dial (potentiometer)
       "elbow": ADC(27)      # ADC(1) for Pico - elbow control dial (potentiometer)
   }
   
   # Initialize GP13 button with pull-up resistor for pen control
   button = Pin(13, Pin.IN, Pin.PULL_UP)
   
   # Initialize pen in up position
   pwm["pen"].duty_u16(PEN_SERVO["up"])
   
   return pwm, dials, button

def map_value(x, in_min, in_max, out_min, out_max):
   """Map value from one range to another"""
   return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def read_dials(dials):
   """Read control dial (potentiometer) values and convert to angles"""
   # Read raw values (0-65535 for Pico) from control dials
   shoulder_raw = dials["shoulder"].read_u16()
   elbow_raw = dials["elbow"].read_u16()
   
   # Map control dial values to servo angles
   shoulder_angle = map_value(shoulder_raw, 0, 65535, 
                            SERVO_LIMITS["shoulder"]["min_angle"],
                            SERVO_LIMITS["shoulder"]["max_angle"])
   
   elbow_angle = map_value(elbow_raw, 0, 65535,
                          SERVO_LIMITS["elbow"]["min_angle"],
                          SERVO_LIMITS["elbow"]["max_angle"])
   
   return shoulder_angle, elbow_angle

def angle_to_duty_cycle(angle, servo_name):
   """
   Convert angle to duty cycle using servo-specific limits.
   Enforces absolute duty cycle limits to protect servos.
   """
   limits = SERVO_LIMITS[servo_name]
   duty_cycle = int((angle / (limits["max_angle"] - limits["min_angle"])) * 
                   (limits["max_duty"] - limits["min_duty"]) + 
                   limits["min_duty"])
   
   # Strict enforcement of absolute limits
   if duty_cycle < ABSOLUTE_MIN_DUTY:
       print(f"WARNING: Calculated duty cycle {duty_cycle} below absolute minimum. Limiting to {ABSOLUTE_MIN_DUTY}")
       return ABSOLUTE_MIN_DUTY
   elif duty_cycle > ABSOLUTE_MAX_DUTY:
       print(f"WARNING: Calculated duty cycle {duty_cycle} above absolute maximum. Limiting to {ABSOLUTE_MAX_DUTY}")
       return ABSOLUTE_MAX_DUTY
   
   return duty_cycle

def move_servos(shoulder_angle, elbow_angle, pen_down, pwm):
   """
   Move servos to specified angles and pen position with strict safety limits
   """
   # Convert to duty cycles for arm servos
   shoulder_duty = angle_to_duty_cycle(shoulder_angle, "shoulder")
   elbow_duty = angle_to_duty_cycle(elbow_angle, "elbow")
   
   # Double-check duties are within absolute limits before moving
   if not (ABSOLUTE_MIN_DUTY <= shoulder_duty <= ABSOLUTE_MAX_DUTY):
       print(f"ERROR: Unsafe shoulder duty cycle {shoulder_duty}. Movement prevented.")
       return
       
   if not (ABSOLUTE_MIN_DUTY <= elbow_duty <= ABSOLUTE_MAX_DUTY):
       print(f"ERROR: Unsafe elbow duty cycle {elbow_duty}. Movement prevented.")
       return
   
   # Move arm servos only if duties are safe
   pwm["shoulder"].duty_u16(shoulder_duty)
   pwm["elbow"].duty_u16(elbow_duty)
   
   # Pen servo uses exact values only
   pen_position = PEN_SERVO["down"] if pen_down else PEN_SERVO["up"]
   # Verify pen position is one of the two exact allowed values
   if pen_position not in [PEN_SERVO["up"], PEN_SERVO["down"]]:
       print("ERROR: Invalid pen position requested. Using pen_up position.")
       pen_position = PEN_SERVO["up"]
   
   pwm["pen"].duty_u16(pen_position)

def main():
   # Initialize hardware
   pwm, dials, button = initialize()
   pen_down = False  # Start with pen up
   
   print("System running. Press button to toggle pen up/down.")
   print(f"Using strict servo limits: MIN_DUTY={ABSOLUTE_MIN_DUTY}, MAX_DUTY={ABSOLUTE_MAX_DUTY}")
   print(f"Pen positions: UP={PEN_SERVO['up']}, DOWN={PEN_SERVO['down']}")
   
   try:
       while True:
           # Check button for pen control
           if not button.value():  # Button pressed
               pen_down = not pen_down  # Toggle pen state
               state = "down" if pen_down else "up"
               print(f"Pen toggled: {state}")
               time.sleep(0.2)  # Debounce delay
               
           # Read control dial values and convert to angles
           shoulder_angle, elbow_angle = read_dials(dials)
           # Move servos to new positions with safety limits enforced
           move_servos(shoulder_angle, elbow_angle, pen_down, pwm)
           
           # Small delay to prevent overwhelming the system
           time.sleep(0.01)
           
   except KeyboardInterrupt:
       print("\nProgram terminated by user")
       
   finally:
       # Clean up - ensure pen is up before shutting down
       pwm["pen"].duty_u16(PEN_SERVO["up"])
       for servo in pwm.values():
           servo.deinit()

if __name__ == "__main__":
   main()