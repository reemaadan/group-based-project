from machine import Pin, PWM

# Initializing Raspberry Pi Pico
def initialize():
    """
    Initialize Raspberry Pi Pico and configure GPIO pins for servo control.
    
    - Sets up the PWM controller.
    - Defines and configures GPIO pins for each connected servo.
    - Adjusts the PWM frequency according to servo specifications.
    
    Returns:
        pwm (PWM object): The PWM controller to be used for servo control.
    """
    # Initialize the PWM controller
    pwm = {
        "shoulder": PWM(Pin(0)),  # Example pin for the shoulder servo
        "elbow": PWM(Pin(1))     # Example pin for the elbow servo
    }

    # Set PWM frequency to 50 Hz (standard for servos)
    for servo in pwm.values():
        servo.freq(50)

    return pwm

# Function to Move Servos using PWM Channels
def move_servos(shoulder_angle, elbow_angle, pwm):
    """
    Move servos to specified angles using PWM channels.
    
    Parameters:
        shoulder_angle (int): Target angle for the shoulder servo.
        elbow_angle (int): Target angle for the elbow servo.
        pwm (PWM object): The PWM controller to generate signals for servos.
    """
    # Validate and clamp angles
    shoulder_angle = validate_angle(shoulder_angle)
    elbow_angle = validate_angle(elbow_angle)

    # Convert angles to duty cycles
    duty_cycle1 = angle_to_duty_cycle(shoulder_angle)
    duty_cycle2 = angle_to_duty_cycle(elbow_angle)
    
    # Move servos to the specified angles
    pwm["shoulder"].duty_u16(duty_cycle1)
    pwm["elbow"].duty_u16(duty_cycle2)

# Function to Validate and Clamp Angles
def validate_angle(angle):
    """
    Validate and clamp the servo angle to the range [0, 180].
    
    Parameters:
        angle (int): Target angle for the servo.
    
    Returns:
        int: Validated angle within the range [0, 180].
    """
    if angle < 0:
        print(f"Warning: Angle {angle} is less than 0. Clamping to 0.")
        return 0
    elif angle > 180:
        print(f"Warning: Angle {angle} exceeds 180. Clamping to 180.")
        return 180
    return angle

# Function to Convert Angle to PWM Duty Cycle
def angle_to_duty_cycle(angle):
    """
    Convert a servo angle to the corresponding PWM duty cycle.
    
    Parameters:
        angle (int): The servo angle in degrees (0-180).
    
    Returns:
        int: The duty cycle value required to move the servo to the specified angle.
    """
    servo_min_duty = 2300  # Minimum duty cycle for the servo
    servo_max_duty = 7500  # Maximum duty cycle for the servo
    angle_range = 180      # Maximum angle range of the servo

    # Map the angle to the duty cycle range
    duty_cycle = int((angle / angle_range) * (servo_max_duty - servo_min_duty) + servo_min_duty)
    return duty_cycle

# Example Usage
if __name__ == "__main__":
    # Initialize PWM for servos
    pwm = initialize()

    # Test moving servos with valid and invalid angles
    print("Moving servos to valid angles (90, 45)")
    move_servos(90, 45, pwm)

    print("Testing invalid angles (-30, 200)")
    move_servos(-30, 200, pwm)
