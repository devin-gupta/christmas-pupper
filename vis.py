import pygame
import sys
import time
import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PupperVisualization(Node):
    def __init__(self):
        super().__init__('pupper_visualization_node')
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'visualization_topic',
            self.visualization_callback,
            10)
        
        self.get_logger().info('vis node started and waiting for queries...')
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Pupper Face")
        self.clock = pygame.time.Clock()

        # Animation states
        self.eye_state = 'open'
        self.look_direction = 'center'  # New state for eye direction
        self.blink_timer = 0
        self.blink_duration = 10
        self.next_blink = random.randint(7 * 60, 13 * 60)
        
        # Offset for looking left/right (in pixels)
        self.eye_offset_x = 0
        
    def visualization_callback(self, msg):
        if msg.data == 'blink':
            self.eye_state = 'closed'
            self.blink_timer = self.next_blink  # Trigger immediate blink
        elif msg.data == 'look_left':
            self.look_direction = 'left'
            self.eye_offset_x = -30  # Adjust this value as needed
        elif msg.data == 'look_right':
            self.look_direction = 'right'
            self.eye_offset_x = 30   # Adjust this value as needed
        elif msg.data == 'look_center':
            self.look_direction = 'center'
            self.eye_offset_x = 0

    def draw_face(self):
        """Draws stylized cartoon eyes."""
        # Colors
        WHITE_BG = (255, 255, 255)
        WHITE = (255, 255, 255)
        BLACK = (0, 0, 0)
        COPPER = (184, 115, 51)
        TURQUOISE = (64, 224, 208)
        
        self.screen.fill(WHITE_BG)
        
        screen_width = self.screen.get_width()
        screen_height = self.screen.get_height()
        center_x = screen_width // 2
        center_y = screen_height // 2
        
        base_eye_size = int(screen_width * 0.24)
        eye_spacing = base_eye_size * 0.8
        
        if self.eye_state == 'open':
            for y_offset in [-eye_spacing, eye_spacing]:
                # Add eye_offset_x to center_x for left/right looking
                current_x = center_x + self.eye_offset_x
                
                # Draw eyes with offset
                pygame.draw.circle(self.screen, COPPER, 
                                 (current_x, center_y + y_offset), base_eye_size)
                
                pygame.draw.circle(self.screen, WHITE, 
                                 (current_x, center_y + y_offset), int(base_eye_size * 0.9))
                
                pygame.draw.circle(self.screen, BLACK, 
                                 (current_x, center_y + y_offset), int(base_eye_size * 0.9), 2)
                
                pygame.draw.circle(self.screen, TURQUOISE, 
                                 (current_x, center_y + y_offset), int(base_eye_size * 0.7))
                
                pygame.draw.circle(self.screen, BLACK, 
                                 (current_x, center_y + y_offset), int(base_eye_size * 0.36))
                
                pygame.draw.circle(self.screen, BLACK, 
                                 (current_x, center_y + y_offset), int(base_eye_size * 0.31))
                
                pygame.draw.circle(self.screen, WHITE, 
                                 (current_x - int(base_eye_size * 0.2), 
                                  center_y + y_offset - int(base_eye_size * 0.2)), 
                                  int(base_eye_size * 0.25))
                
                pygame.draw.circle(self.screen, WHITE, 
                                 (current_x + int(base_eye_size * 0.15), 
                                  center_y + y_offset + int(base_eye_size * 0.15)), 
                                  int(base_eye_size * 0.1))
                
        elif self.eye_state == 'closed':
            for y_offset in [-eye_spacing, eye_spacing]:
                current_x = center_x + self.eye_offset_x
                
                pygame.draw.circle(self.screen, COPPER, 
                                 (current_x, center_y + y_offset), base_eye_size)
                
                pygame.draw.circle(self.screen, WHITE, 
                                 (current_x, center_y + y_offset), int(base_eye_size * 0.9))
                
                pygame.draw.arc(self.screen, BLACK,
                              (current_x - int(base_eye_size * 0.6), 
                               center_y + y_offset - int(base_eye_size * 0.3), 
                               base_eye_size * 1.2, base_eye_size * 0.6),
                              0, math.pi, 4)

    def run(self):
        try:
            while rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_q:
                            return

                # Handle automatic blinking
                self.blink_timer += 1
                if self.blink_timer == self.next_blink:
                    self.eye_state = 'closed'
                elif self.blink_timer == self.next_blink + self.blink_duration:
                    self.eye_state = 'open'
                    self.blink_timer = 0
                    self.next_blink = random.randint(7 * 60, 13 * 60)

                # Draw and update display
                self.draw_face()
                pygame.display.flip()
                self.clock.tick(60)

                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)

        finally:
            pygame.quit()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vis_node = PupperVisualization()
    vis_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()