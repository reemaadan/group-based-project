from machine import Pin, ADC
import time

class EtchASketch:
    def __init__(self):
        # Setup Output LEDs
        self.led_shoulder = Pin(15, Pin.OUT)
        self.led_elbow = Pin(16, Pin.OUT)
        self.led_pen = Pin(17, Pin.OUT)
        
        # Setup Input Pins
        # Using ADC pins for analog input (potentiometers)
        # On most Pico boards, ADC pins are 26, 27, 28
        self.shoulder_input = ADC(26)  # GPIO 26 - ADC0
        self.elbow_input = ADC(27)     # GPIO 27 - ADC1
        
        # Digital input for pen up/down (button)
        self.pen_button = Pin(28, Pin.IN, Pin.PULL_UP)
        
        # State variables
        self.pen_state = False  # False = up, True = down
        self.last_button_state = True  # Pull-up resistor means button reads True when not pressed
        
    def read_analog_value(self, adc):
        """Convert ADC reading (0-65535) to angle (0-180)"""
        raw = adc.read_u16()
        return int((raw / 65535) * 180)
    
    def check_button(self):
        """Check for button press with debouncing"""
        current_state = self.pen_button.value()
        
        if current_state != self.last_button_state:
            time.sleep(0.05)  # Simple debounce delay
            
            # If button is pressed (goes from high to low due to pull-up)
            if current_state == 0:
                self.pen_state = not self.pen_state  # Toggle pen state
                self.led_pen.value(self.pen_state)   # Update pen LED
                print("Pen state:", "Down" if self.pen_state else "Up")
                
        self.last_button_state = current_state
    
    def update_position_leds(self, shoulder_angle, elbow_angle):
        """Update position LEDs based on current angles"""
        # Shoulder LED brightness
        self.led_shoulder.value(1 if shoulder_angle > 90 else 0)
        
        # Elbow LED brightness
        self.led_elbow.value(1 if elbow_angle > 90 else 0)
        
        # Print position for debugging
        print(f"Position - Shoulder: {shoulder_angle}°, Elbow: {elbow_angle}°")
    
    def run(self):
        """Main loop to continuously read inputs and update outputs"""
        print("Etch-A-Sketch started. Use potentiometers to control position.")
        print("Press button to toggle pen up/down")
        
        try:
            while True:
                # Read current positions from potentiometers
                shoulder_angle = self.read_analog_value(self.shoulder_input)
                elbow_angle = self.read_analog_value(self.elbow_input)
                
                # Check for pen button press
                self.check_button()
                
                # Update LED outputs
                self.update_position_leds(shoulder_angle, elbow_angle)
                
                # Small delay to prevent overwhelming the system
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nProgram stopped by user")
            # Turn off all LEDs
            self.led_shoulder.value(0)
            self.led_elbow.value(0)
            self.led_pen.value(0)

# Create and run the Etch-A-Sketch
if __name__ == "__main__":
    etch = EtchASketch()
    etch.run()