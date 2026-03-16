#!/usr/bin/env python3
"""
Keyboard Speed Controller
Real-time control of velocity simulator for testing different speeds.
Press keys to adjust speed multiplier during runtime.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import tty
import termios
import threading


class KeyboardSpeedController(Node):
    def __init__(self):
        super().__init__('keyboard_speed_controller')
        
        # Publisher for speed multiplier
        self.speed_pub = self.create_publisher(Float32, '/speed_multiplier', 10)
        
        self.speed_multiplier = 1.0
        self.speed_step = 0.1
        
        # Print instructions
        self.print_instructions()
        
        # Start keyboard listener in separate thread
        self.running = True
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()
        
        # Timer to publish speed
        self.timer = self.create_timer(0.1, self.publish_speed)
    
    def print_instructions(self):
        print("\n" + "="*60)
        print("  KEYBOARD SPEED CONTROLLER")
        print("="*60)
        print("  UP ARROW    : Increase speed (+0.1x)")
        print("  DOWN ARROW  : Decrease speed (-0.1x)")
        print("  RIGHT ARROW : Large increase (+0.5x)")
        print("  LEFT ARROW  : Large decrease (-0.5x)")
        print("  1-9         : Set speed to 0.1x - 0.9x")
        print("  0           : Set speed to 1.0x (normal)")
        print("  s           : Stop (0.0x)")
        print("  f           : Fast (2.0x)")
        print("  q / ESC     : Quit")
        print("="*60)
        print(f"  Current speed: {self.speed_multiplier:.2f}x")
        print("="*60 + "\n")
    
    def keyboard_loop(self):
        """Read keyboard input in non-blocking mode."""
        # Save terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(fd)
            while self.running:
                ch = sys.stdin.read(1)
                
                # Handle arrow keys (escape sequences)
                if ch == '\x1b':
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A':  # Up arrow
                            self.speed_multiplier += self.speed_step
                        elif ch3 == 'B':  # Down arrow
                            self.speed_multiplier = max(0.0, self.speed_multiplier - self.speed_step)
                        elif ch3 == 'C':  # Right arrow
                            self.speed_multiplier += 0.5
                        elif ch3 == 'D':  # Left arrow
                            self.speed_multiplier = max(0.0, self.speed_multiplier - 0.5)
                    else:
                        # ESC key
                        self.running = False
                        break
                
                # Number keys
                elif ch in '123456789':
                    self.speed_multiplier = int(ch) * 0.1
                elif ch == '0':
                    self.speed_multiplier = 1.0
                
                # Letter commands
                elif ch.lower() == 's':
                    self.speed_multiplier = 0.0
                elif ch.lower() == 'f':
                    self.speed_multiplier = 2.0
                elif ch.lower() == 'q':
                    self.running = False
                    break
                
                # Display current speed
                print(f"\r  Current speed: {self.speed_multiplier:.2f}x ", end='', flush=True)
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("\n")
    
    def publish_speed(self):
        """Publish current speed multiplier."""
        msg = Float32()
        msg.data = self.speed_multiplier
        self.speed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSpeedController()
    
    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    print("\n🛑 Keyboard controller stopped")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()