#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import threading

class JoystickPWMNode(Node):
    def __init__(self):
        super().__init__('joystick_serial_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Joint PWM values
        self.joint_pwm_values = [0] * 6
        
        # Currently selected joint (none initially)
        self.selected_joint = None
        
        # Track last sent values to reduce spam
        self.last_sent_values = [0] * 6
        
        # Initialize serial communication FIRST
        self.serial_connection = None
        self.setup_serial()
        
        # Create subscription to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Create timer for publishing PWM values
        self.pwm_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_pwm_values
        )
        
        self.get_logger().info('✅ Joystick PWM Node initialized')
        self.get_logger().info('🎮 Axis 1 (Y-axis): Controls selected joint')
        self.get_logger().info('🔘 Buttons 0-5: Select joints 0-5 (or buttons 7-12 for joints 0-5)')
        self.get_logger().info('⚙️  PWM range: -255 to 255, Center: 0')

    def setup_serial(self):
        """Setup serial connection with better error handling"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1  # Add write timeout
            )
            time.sleep(2)  # Wait for the serial connection to initialize
            
            # Start reader thread like your test script
            self.reader_thread = threading.Thread(
                target=self.serial_reader, 
                daemon=True
            )
            self.reader_thread.start()
            
            # Send status check to ESP32
            self.send_serial_command("STATUS")
            
            self.get_logger().info(f'✅ Serial connection established on {self.serial_port}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Failed to connect to serial port {self.serial_port}: {e}')
            self.get_logger().info('Running in demo mode (no serial output)')
            self.serial_connection = None

    def serial_reader(self):
        """Read and log everything from ESP32 - like your test script"""
        while self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode().strip()
                    if response:
                        self.get_logger().info(f'[ESP32] {response}')
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break
            time.sleep(0.01)

    def send_serial_command(self, command):
        """Send command to serial port with better error handling"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return False
            
        try:
            # Ensure command ends with newline
            if not command.endswith('\n'):
                command += '\n'
                
            # Log what we're sending
            self.get_logger().info(f'[SEND] {command.strip()}')
            
            # Send the command
            self.serial_connection.write(command.encode('utf-8'))
            self.serial_connection.flush()  # Force immediate transmission
            
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            # Try to reconnect
            self.setup_serial()
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error sending command: {e}')
            return False

    def joy_callback(self, msg):
        # Use Y-axis (axis 1) for control
        axis_value = msg.axes[1] if len(msg.axes) > 1 else 0.0
        pwm_value = self.axis_to_pwm(axis_value, apply_deadband=True)
        
        # Check for button presses - support both button mappings
        self.selected_joint = None
        
        # First check buttons 0-5 for joints 0-5
        for i in range(6):
            if len(msg.buttons) > i and msg.buttons[i]:
                self.selected_joint = i
                break
        
        # If no buttons 0-5 pressed, check buttons 7-12 for joints 0-5
        if self.selected_joint is None:
            for i in range(6):
                button_idx = i + 7  # buttons 7-12
                if len(msg.buttons) > button_idx and msg.buttons[button_idx]:
                    self.selected_joint = i
                    break
        
        # Update PWM values
        if self.selected_joint is not None:
            # Reset all joints to 0, then set the selected one
            self.joint_pwm_values = [0] * 6
            self.joint_pwm_values[self.selected_joint] = pwm_value
        else:
            # No button pressed, all joints to 0
            self.joint_pwm_values = [0] * 6

    def axis_to_pwm(self, axis_value, apply_deadband=False):
        # Clamp axis value to valid range
        axis_value = max(-1.0, min(1.0, axis_value))
        
        # Apply deadband if requested
        if apply_deadband:
            deadband = 0.1
            if abs(axis_value) < deadband:
                axis_value = 0.0
        
        # Convert to PWM range (-255 to 255)
        # Invert Y-axis so up is positive
        pwm_value = int(-axis_value * 255)
        return max(-255, min(255, pwm_value))

    def publish_pwm_values(self):
        # Only send if values changed to reduce spam
        if self.joint_pwm_values != self.last_sent_values:
            # Format as serial command: "SET" followed by space-separated values
            command = f"SET {' '.join(map(str, self.joint_pwm_values))}"
            
            # Send to serial port
            if self.serial_connection:
                success = self.send_serial_command(command)
                if success:
                    # Log active joints
                    active_joints = [(i, pwm) for i, pwm in enumerate(self.joint_pwm_values) if pwm != 0]
                    if active_joints:
                        for joint_id, pwm in active_joints:
                            direction = "FWD" if pwm > 0 else "REV"
                            self.get_logger().info(f'[CONTROL] Joint {joint_id}: {direction} {abs(pwm)}/255')
                    elif any(self.last_sent_values):
                        self.get_logger().info('[CONTROL] All joints stopped')
            else:
                # Demo mode
                self.get_logger().info(f'[DEMO] {command}')
            
            self.last_sent_values = self.joint_pwm_values.copy()

    def destroy_node(self):
        # Clean shutdown - stop all motors
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.get_logger().info('Stopping all motors...')
                self.send_serial_command("STOP")
                time.sleep(0.1)
                self.serial_connection.close()
                self.get_logger().info('Serial connection closed')
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickPWMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
