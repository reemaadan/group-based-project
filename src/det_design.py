from machine import Pin
import time

# Constants for LED brightness levels (for visual feedback)
MIN_BRIGHTNESS = 0
MAX_BRIGHTNESS = 65535
ANGLE_RANGE = 180

def initialize():
    """
    Initialize GPIO pins for LEDs instead of PWM for servos.
    Returns a dictionary of LED pin objects.
    """
    leds = {
        "shoulder": Pin(15, Pin.OUT),  # LED for shoulder movement
        "elbow": Pin(16, Pin.OUT),     # LED for elbow movement
        "pen_up_down": Pin(17, Pin.OUT) # LED for pen position
    }
    return leds

def simulate_movement(shoulder_angle, elbow_angle, pen_position, leds):
    """
    Simulate servo movement using LEDs.
    
    Parameters:
        shoulder_angle (int): Target angle for the shoulder movement
        elbow_angle (int): Target angle for the elbow movement
        pen_position (bool): True for pen down, False for pen up
        leds (dict): Dictionary containing LED Pin objects
    """
    # Shoulder movement simulation
    if shoulder_angle > 0:
        leds["shoulder"].value(1)  # Turn LED on
        print(f"Shoulder LED ON - Angle: {shoulder_angle}")
    else:
        leds["shoulder"].value(0)  # Turn LED off
        print(f"Shoulder LED OFF - Angle: {shoulder_angle}")
    
    # Elbow movement simulation
    if elbow_angle > 0:
        leds["elbow"].value(1)  # Turn LED on
        print(f"Elbow LED ON - Angle: {elbow_angle}")
    else:
        leds["elbow"].value(0)  # Turn LED off
        print(f"Elbow LED OFF - Angle: {elbow_angle}")
    
    # Pen position simulation
    if pen_position:
        leds["pen_up_down"].value(1)  # Turn LED on for pen down
        print("Pen LED ON - Pen Down")
    else:
        leds["pen_up_down"].value(0)  # Turn LED off for pen up
        print("Pen LED OFF - Pen Up")

    # Add a small delay to make LED changes visible
    time.sleep(0.1)

if __name__ == "__main__":
    leds = initialize()
    
    # Test LED patterns
    while True:
        # Simulate shoulder and elbow at 90 degrees, pen down
        print("\nSimulating position: 90, 90, pen down")
        simulate_movement(90, 90, True, leds)
        time.sleep(2)
        
        # Simulate shoulder and elbow at 45 degrees, pen up
        print("\nSimulating position: 45, 45, pen up")
        simulate_movement(45, 45, False, leds)
        time.sleep(2)
        
        # Simulate shoulder at 180 degrees, elbow at 0 degrees, pen down
        print("\nSimulating position: 180, 0, pen down")
        simulate_movement(180, 0, True, leds)
        time.sleep(2)
        
        # Simulate shoulder and elbow at 0 degrees, pen up
        print("\nSimulating position: 0, 0, pen up")
        simulate_movement(0, 0, False, leds)
        time.sleep(2)