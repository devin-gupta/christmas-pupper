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
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Pupper Face")
        self.clock = pygame.time.Clock()

        # Animation states
        self.eye_state = 'open'
        self.mouth_state = 'smile'
        self.look_direction = 0
        self.blink_timer = 0
        self.blink_duration = 10
        self.next_blink = random.randint(2 * 60, 4 * 60)
        
        # Eye movement variables
        self.movement_target = 0
        self.movement_speed = 0.1
        self.look_timer = 0
        self.look_duration = 30
        self.is_looking = False

        # Eyebrow movement variables
        self.eyebrow_offset = 0
        self.eyebrow_target = 0
        self.eyebrow_speed = 0.2
        self.eyebrow_max_offset = int(self.screen.get_width() * 0.4)
        self.eyebrow_timer = 0
        self.eyebrow_duration = 30
        self.is_eyebrow_moving = False

    def blink(self):
        """Trigger a blink"""
        self.eye_state = 'closed'
        self.blink_timer = self.next_blink

    def look_up(self):
        """Make eyes look up"""
        self.movement_target = 1
        self.is_looking = True
        self.look_timer = 0

    def look_down(self):
        """Make eyes look down"""
        self.movement_target = -1
        self.is_looking = True
        self.look_timer = 0

    def eyebrow_raise(self):
        """Raise the eyebrows"""
        self.eyebrow_target = self.eyebrow_max_offset
        self.is_eyebrow_moving = True
        self.eyebrow_timer = 0

    def visualization_callback(self, msg):
        if msg.data == 'blink':
            self.blink()
        elif msg.data == 'look_up':
            self.look_up()
        elif msg.data == 'look_down':
            self.look_down()
        elif msg.data == 'eyebrow_raise':
            self.eyebrow_raise()

    def draw_face(self):
        """Draws the complete face with eyes, eyebrows, and nose"""
        # Colors
        WHITE = (255, 255, 255)
        BLACK = (0, 0, 0)
        WARM_BROWN = (139, 69, 19)
        TAN_BG = (210, 180, 140)
        RED = (255, 0, 0)
        LIGHT_RED = (255, 150, 150)
        
        # Screen dimensions
        center_x = self.screen.get_width() // 2
        center_y = self.screen.get_height() // 2
        base_width = self.screen.get_width() // 8
        base_height = self.screen.get_height() // 8
        eye_spacing = base_height
        eye_offset_x = base_width // 2

        # Fill background
        self.screen.fill(TAN_BG)
        
        # Draw eyes
        if self.eye_state == 'open':
            for y_offset in [-eye_spacing, eye_spacing]:
                current_x = center_x + eye_offset_x
                
                # Helper function to create rect for ellipse
                def get_eye_rect(size_scale, move_with_look=False):
                    width = int(base_width * size_scale)
                    height = int(base_height * size_scale)
                    x_pos = current_x - width//2
                    y_pos = (center_y + y_offset) - height//2
                    if move_with_look:  # Only move pupils vertically
                        y_pos += int(base_height * 0.15 * self.look_direction)
                    return pygame.Rect(
                        x_pos,
                        y_pos,
                        width,
                        height
                    )
                
                # White background (stays fixed)
                pygame.draw.ellipse(self.screen, WHITE, get_eye_rect(0.9))
                
                # Black outline (stays fixed)
                pygame.draw.ellipse(self.screen, BLACK, get_eye_rect(0.9), 5)
                
                # Brown iris (stays fixed)
                pygame.draw.ellipse(self.screen, WARM_BROWN, get_eye_rect(0.7))
                
                # Black pupils (move with look_direction)
                pygame.draw.ellipse(self.screen, BLACK, get_eye_rect(0.36, True))
                pygame.draw.ellipse(self.screen, BLACK, get_eye_rect(0.31, True))
                
                # Main light reflection (stays fixed)
                reflection_size = int(base_width * 0.10)
                pygame.draw.circle(self.screen, WHITE, 
                                 (current_x - int(base_width * 0.2),
                                  center_y + y_offset - int(base_height * 0.2)),
                                  reflection_size)
                
                # Small secondary reflection (stays fixed)
                small_reflection = int(base_width * 0.08)
                pygame.draw.circle(self.screen, WHITE, 
                                 (current_x + int(base_width * 0.15),
                                  center_y + y_offset + int(base_height * 0.15)),
                                  small_reflection)
        
        # Draw eyebrows with increased spacing
        eyebrow_thickness = 26
        outline_thickness = 6
        eyebrow_length = int(base_height * 0.6)
        eyebrow_offset_x = int(base_width * 1) + self.eyebrow_offset  # Added eyebrow_offset
        eyebrow_spacing = int(eye_spacing * 1.3)
        
        for y_offset in [-eyebrow_spacing, eyebrow_spacing]:
            # Calculate start and end points for each eyebrow
            start_x = center_x + eyebrow_offset_x
            start_y = (center_y + y_offset)
            
            if y_offset < 0:  # Top eyebrow goes down
                end_x = start_x + int(base_width * 0.1)
                end_y = start_y + eyebrow_length
            else:  # Bottom eyebrow goes up
                end_x = start_x + int(base_width * 0.1)
                end_y = start_y - eyebrow_length
            
            # Draw the black outline first (thicker)
            pygame.draw.line(self.screen, BLACK, 
                            (start_x, start_y), 
                            (end_x, end_y), 
                            outline_thickness + eyebrow_thickness)
            
            # Draw the brown eyebrow on top (slightly thinner)
            pygame.draw.line(self.screen, WARM_BROWN, 
                            (start_x, start_y), 
                            (end_x, end_y), 
                            eyebrow_thickness)
        
        # Draw nose with highlight
        nose_size = int(base_width * 0.4)
        nose_x = center_x + eye_offset_x
        nose_y = center_y
        
        # Draw the red nose circle
        pygame.draw.circle(self.screen, RED, (nose_x, nose_y), nose_size)
        
        # Draw black outline for nose
        pygame.draw.circle(self.screen, BLACK, (nose_x, nose_y), nose_size, 4)
        
        # Add highlight to nose
        highlight_size = int(nose_size * 0.3)
        highlight_offset = int(nose_size * 0.25)
        pygame.draw.circle(self.screen, LIGHT_RED,
                          (nose_x - highlight_offset, nose_y - highlight_offset),
                          highlight_size)

    def run(self):
        try:
            while rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_q:
                            return

                # Update look timer and handle auto-return
                if self.is_looking:
                    self.look_timer += 1
                    if self.look_timer >= self.look_duration:
                        self.movement_target = 0
                        self.is_looking = False

                # Update eyebrow timer and handle auto-return
                if self.is_eyebrow_moving:
                    self.eyebrow_timer += 1
                    if self.eyebrow_timer >= self.eyebrow_duration:
                        self.eyebrow_target = 0
                        self.is_eyebrow_moving = False

                # Smoothly move eyes toward target
                if self.look_direction < self.movement_target:
                    self.look_direction = min(self.movement_target, 
                                           self.look_direction + self.movement_speed)
                elif self.look_direction > self.movement_target:
                    self.look_direction = max(self.movement_target, 
                                           self.look_direction - self.movement_speed)

                # Smoothly move eyebrows toward target
                if self.eyebrow_offset < self.eyebrow_target:
                    self.eyebrow_offset = min(self.eyebrow_target, 
                                           self.eyebrow_offset + self.eyebrow_speed)
                elif self.eyebrow_offset > self.eyebrow_target:
                    self.eyebrow_offset = max(self.eyebrow_target, 
                                           self.eyebrow_offset - self.eyebrow_speed)

                # Handle automatic blinking
                self.blink_timer += 1
                if self.blink_timer == self.next_blink:
                    self.eye_state = 'closed'
                elif self.blink_timer == self.next_blink + self.blink_duration:
                    self.eye_state = 'open'
                    self.blink_timer = 0
                    self.next_blink = random.randint(2 * 60, 4 * 60)

                # Draw face and update display
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
