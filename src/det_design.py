from machine import Pin, PWM
import time

# Set the minimum duty cycle in milliseconds
servo_min_duty = 1000
servo_max_duty = 65000
angle_range = 180

def initialize():
    # Initialize the PWM controller
    pwm = {
        "shoulder": PWM(Pin(15)),
        "elbow": PWM(Pin(16)),
        "pen_up_down": PWM(Pin(17))
    }
    for servo in pwm.values():
        servo.freq(50)  # sets our frequency to servo specs
    return pwm

def move_servos(shoulder_angle, elbow_angle, pen_position, pwm):
    """
    Move servos to specified angles using PWM channels.
    
    Parameters:
        shoulder_angle (int): Target angle for the first shoulder servo.
        elbow_angle (int): Target angle for the elbow servo.
        pen_position (bool): True for pen down, False for pen up
        pwm (PWM object): The PWM controller to generate signals for servos.
    """
    # Convert angles to duty cycles
    shoulder_duty = angle_to_duty_cycle(shoulder_angle)
    elbow_duty = angle_to_duty_cycle(elbow_angle)
    
    # Move the shoulder and elbow servos to their specified angle
    pwm["shoulder"].duty_u16(shoulder_duty)
    pwm["elbow"].duty_u16(elbow_duty)
    
    # Control the pen servo (up or down)
    if pen_position:
        # Pen down (move servo to position to draw)
        pwm["pen_up_down"].duty_u16(servo_max_duty)
    else:
        # Pen up (move servo to position to lift pen)
        pwm["pen_up_down"].duty_u16(servo_min_duty)

def angle_to_duty_cycle(angle):
    # Map the angle to the corresponding duty cycle within the servo's range
    duty_cycle = int((angle / angle_range) * (servo_max_duty - servo_min_duty) + servo_min_duty)
    return duty_cycle

if __name__ == "__main__":
    pwm = initialize()
    
    # Test moving servos to specific positions
    while True:
        # Move shoulder and elbow to 90 degrees, pen down
        print("Moving to 90, 90 with pen down")
        move_servos(90, 90, True, pwm)
        time.sleep(2)
        
        # Move shoulder and elbow to 45 degrees, pen up
        print("Moving to 45, 45 with pen up")
        move_servos(45, 45, False, pwm)
        time.sleep(2)
        
        # Move shoulder to 180 degrees, elbow to 0 degrees, pen down
        print("Moving to 180, 0 with pen down")
        move_servos(180, 0, True, pwm)
        time.sleep(2)
        
        # Move shoulder and elbow to 0 degrees, pen up
        print("Moving to 0, 0 with pen up")
        move_servos(0, 0, False, pwm)
        time.sleep(2)
