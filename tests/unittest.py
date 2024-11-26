import unittest
from math import pi
import math

# Mock machine module since it won't be available on development machine
class MockPWM:
    def __init__(self, pin):
        self.pin = pin
        self.frequency = 0
        self.duty = 0
    
    def freq(self, frequency):
        self.frequency = frequency
        
    def duty_u16(self, duty):
        self.duty = duty
        
    def deinit(self):
        pass

class MockPin:
    IN = 'in'
    OUT = 'out'
    PULL_UP = 'pull_up'
    
    def __init__(self, pin_num, direction=None, pull=None):
        self.pin_num = pin_num
        self.direction = direction
        self.pull = pull
        self._value = 1
        
    def value(self):
        return self._value

class MockADC:
    def __init__(self, pin):
        self.pin = pin
        self._value = 0
        
    def read_u16(self):
        return self._value
        
    def set_test_value(self, value):
        self._value = value

# Import necessary constants and functions from your main code
SERVO_LIMITS = {
    "shoulder": {
        "min_duty": 2300,
        "max_duty": 7500,
        "min_angle": 45,
        "max_angle": 135
    },
    "elbow": {
        "min_duty": 2300,
        "max_duty": 7500,
        "min_angle": 45,
        "max_angle": 135
    }
}

WORKSPACE_LIMITS = {
    "x_min": -140,
    "x_max": 140,
    "y_min": 50,
    "y_max": 250
}

shoulder_to_elbow = 175
elbow_to_pen = 160

# Import your functions here
# For testing, we'll copy the key functions we want to test

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def inverse_kinematics(x, y):
    try:
        d = math.sqrt(x*x + y*y)
        
        if d > (shoulder_to_elbow + elbow_to_pen) or d < abs(shoulder_to_elbow - elbow_to_pen):
            print(f"Position ({x},{y}) is unreachable")
            return None
        
        a1 = math.acos((x*x + y*y + shoulder_to_elbow*shoulder_to_elbow - elbow_to_pen*elbow_to_pen)/(2*shoulder_to_elbow*math.sqrt(x*x + y*y)))
        a2 = math.atan2(y, x)
        shoulder = math.degrees(a2 + a1)
        
        b1 = math.acos((shoulder_to_elbow*shoulder_to_elbow + elbow_to_pen*elbow_to_pen - x*x - y*y)/(2*shoulder_to_elbow*elbow_to_pen))
        elbow = math.degrees(math.pi - b1)
        
        if (SERVO_LIMITS["shoulder"]["min_angle"] <= shoulder <= SERVO_LIMITS["shoulder"]["max_angle"] and 
            SERVO_LIMITS["elbow"]["min_angle"] <= elbow <= SERVO_LIMITS["elbow"]["max_angle"]):
            return (shoulder, elbow)
        else:
            print(f"Angles out of range: shoulder={shoulder}, elbow={elbow}")
            return None
            
    except:
        print(f"Could not calculate angles for position ({x},{y})")
        return None

def angle_to_duty_cycle(angle, servo_name):
    limits = SERVO_LIMITS[servo_name]
    duty_cycle = int((angle / (limits["max_angle"] - limits["min_angle"])) * 
                    (limits["max_duty"] - limits["min_duty"]) + 
                    limits["min_duty"])
    
    return max(min(duty_cycle, limits["max_duty"]), limits["min_duty"])

class TestEtchASketch(unittest.TestCase):
    
    def test_map_value(self):
        """Test the mapping function"""
        # Test mapping from potentiometer range to x coordinate
        self.assertEqual(map_value(0, 0, 65535, -140, 140), -140)
        self.assertEqual(map_value(65535, 0, 65535, -140, 140), 140)
        self.assertEqual(map_value(32767, 0, 65535, -140, 140), 0)  # Middle point
        
    def test_inverse_kinematics_reachable(self):
        """Test inverse kinematics with reachable positions"""
        # Test a known reachable position
        result = inverse_kinematics(0, 200)  # Straight ahead
        self.assertIsNotNone(result)
        shoulder, elbow = result
        self.assertTrue(45 <= shoulder <= 135)
        self.assertTrue(45 <= elbow <= 135)
        
    def test_inverse_kinematics_unreachable(self):
        """Test inverse kinematics with unreachable positions"""
        # Test point too far away
        self.assertIsNone(inverse_kinematics(1000, 1000))
        # Test point too close
        self.assertIsNone(inverse_kinematics(0, 10))
        
    def test_angle_to_duty_cycle(self):
        """Test conversion of angles to duty cycles"""
        # Test minimum angle
        min_duty = angle_to_duty_cycle(45, "shoulder")
        self.assertEqual(min_duty, 2300)
        
        # Test maximum angle
        max_duty = angle_to_duty_cycle(135, "shoulder")
        self.assertEqual(max_duty, 7500)
        
        # Test middle angle
        mid_duty = angle_to_duty_cycle(90, "shoulder")
        self.assertTrue(2300 < mid_duty < 7500)
        
    def test_workspace_boundaries(self):
        """Test points at workspace boundaries"""
        # Test each corner of the workspace
        corners = [
            (WORKSPACE_LIMITS["x_min"], WORKSPACE_LIMITS["y_min"]),
            (WORKSPACE_LIMITS["x_max"], WORKSPACE_LIMITS["y_min"]),
            (WORKSPACE_LIMITS["x_min"], WORKSPACE_LIMITS["y_max"]),
            (WORKSPACE_LIMITS["x_max"], WORKSPACE_LIMITS["y_max"])
        ]
        
        for x, y in corners:
            result = inverse_kinematics(x, y)
            if result:
                shoulder, elbow = result
                self.assertTrue(45 <= shoulder <= 135)
                self.assertTrue(45 <= elbow <= 135)

    def test_hardware_mocks(self):
        """Test that our mock hardware classes work as expected"""
        # Test PWM
        pwm = MockPWM(0)
        pwm.freq(50)
        pwm.duty_u16(2300)
        self.assertEqual(pwm.frequency, 50)
        self.assertEqual(pwm.duty, 2300)
        
        # Test ADC
        adc = MockADC(26)
        adc.set_test_value(32767)
        self.assertEqual(adc.read_u16(), 32767)
        
        # Test Pin
        pin = MockPin(13, MockPin.IN, MockPin.PULL_UP)
        self.assertEqual(pin.value(), 1)

if __name__ == '__main__':
    unittest.main()