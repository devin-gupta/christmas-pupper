import pygame
import sys
import time
import math
import random

def draw_face(screen, eye_state, mouth_state, look_direction=0, eyebrow_offset=0):
    """Draws stylized cartoon eyes."""
    # Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    WARM_BROWN = (139, 69, 19)     # Saddle Brown (for iris)
    RED = (255, 0, 0)
    TAN_BG = (210, 180, 140)       # Changed to warmer tan color
    # Other options you could try:
    # TAN_BG = (205, 175, 149)     # Slightly warmer option
    # TAN_BG = (195, 176, 145)     # Another warm option
    # TAN_BG = (200, 155, 130)     # Even warmer option
    
    screen.fill(TAN_BG)
    
    screen_width = screen.get_width()
    screen_height = screen.get_height()
    center_x = screen_width // 2
    center_y = screen_height // 2
    
    base_width = int(screen_width * 0.24)  # Width of the eye
    base_height = int(base_width * 0.8)    # Height is 80% of width
    eye_spacing = base_width * 0.8
    eye_offset_x = int(base_width * 0.4)
    
    # Add LIGHT_RED color for nose highlight
    LIGHT_RED = (255, 150, 150)  # Lighter shade of red
    
    # Draw eyes
    if eye_state == 'open':
        for y_offset in [-eye_spacing, eye_spacing]:
            current_x = center_x + eye_offset_x
            
            # Helper function to create rect for ellipse
            def get_eye_rect(size_scale, move_with_look=False):
                width = int(base_width * size_scale)
                height = int(base_height * size_scale)
                x_pos = current_x - width//2
                y_pos = (center_y + y_offset) - height//2
                if move_with_look:  # Only move pupils vertically
                    y_pos += int(base_height * 0.15 * look_direction)  # Changed from 0.3 to 0.15
                return pygame.Rect(
                    x_pos,
                    y_pos,
                    width,
                    height
                )
            
            # White background (stays fixed)
            pygame.draw.ellipse(screen, WHITE, get_eye_rect(0.9))
            
            # Black outline (stays fixed)
            pygame.draw.ellipse(screen, BLACK, get_eye_rect(0.9), 5)
            
            # Brown iris (stays fixed)
            pygame.draw.ellipse(screen, WARM_BROWN, get_eye_rect(0.7))
            
            # Black pupils (move with look_direction)
            pygame.draw.ellipse(screen, BLACK, get_eye_rect(0.36, True))
            pygame.draw.ellipse(screen, BLACK, get_eye_rect(0.31, True))
            
            # Main light reflection (stays fixed)
            reflection_size = int(base_width * 0.10)
            pygame.draw.circle(screen, WHITE, 
                             (current_x - int(base_width * 0.2),
                              center_y + y_offset - int(base_height * 0.2)),
                              reflection_size)
            
            # Small secondary reflection (stays fixed)
            small_reflection = int(base_width * 0.08)
            pygame.draw.circle(screen, WHITE, 
                             (current_x + int(base_width * 0.15),
                              center_y + y_offset + int(base_height * 0.15)),
                              small_reflection)
            
    elif eye_state == 'closed':
        for y_offset in [-eye_spacing, eye_spacing]:
            current_x = center_x + eye_offset_x
            
            # White background - vertical oval
            pygame.draw.ellipse(screen, WHITE, 
                              pygame.Rect(
                                  current_x - int(base_width * 0.15),  # Narrower width
                                  (center_y + y_offset) - int(base_height * 0.35),  # Taller height
                                  int(base_width * 0.3),   # Narrow
                                  int(base_height * 0.7)   # Tall
                              ))
            
            # Left curved line
            pygame.draw.arc(screen, BLACK,
                          (current_x - int(base_width * 0.15),
                           center_y + y_offset - int(base_height * 0.4),
                           base_width * 0.6,
                           base_height * 0.8),
                          math.pi/2, 3*math.pi/2, 4)  # Left side curve
            
            # Right curved line
            pygame.draw.arc(screen, BLACK,
                          (current_x - int(base_width * 0.45),
                           center_y + y_offset - int(base_height * 0.4),
                           base_width * 0.6,
                           base_height * 0.8),
                          -math.pi/2, math.pi/2, 4)  # Right side curve
        
    # Draw eyebrows with increased spacing
    eyebrow_thickness = 26
    outline_thickness = 6
    eyebrow_length = int(base_height * 0.6)
    eyebrow_offset_x = int(base_width * 1) + eyebrow_offset
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
        pygame.draw.line(screen, BLACK, 
                        (start_x, start_y), 
                        (end_x, end_y), 
                        outline_thickness + eyebrow_thickness)
        
        # Draw the brown eyebrow on top (slightly thinner)
        pygame.draw.line(screen, WARM_BROWN, 
                        (start_x, start_y), 
                        (end_x, end_y), 
                        eyebrow_thickness)
    
    # Draw nose with highlight
    nose_size = int(base_width * 0.4)
    nose_x = center_x - eye_offset_x
    nose_y = center_y
    
    # Draw the red nose circle
    pygame.draw.circle(screen, RED, (nose_x, nose_y), nose_size)
    
    # Draw black outline for nose
    pygame.draw.circle(screen, BLACK, (nose_x, nose_y), nose_size, 4)
    
    # Add highlight to nose
    highlight_size = int(nose_size * 0.3)
    highlight_offset = int(nose_size * 0.25)
    pygame.draw.circle(screen, LIGHT_RED,
                      (nose_x - highlight_offset, nose_y - highlight_offset),
                      highlight_size)

def main():
    pygame.init()
    
    # Initialize the display
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.display.set_caption("Pupper Face")

    clock = pygame.time.Clock()

    # Animation states
    eye_state = 'open'
    mouth_state = 'smile'
    
    running = True
    blink_timer = 0
    blink_duration = 10
    next_blink = random.randint(2 * 60, 4 * 60)

    # Eye movement variables
    look_direction = 0
    movement_speed = 0.1
    movement_target = 0
    look_timer = 0
    look_duration = 30
    is_looking = False

    # Add eyebrow movement variables
    eyebrow_offset = 0  # Current offset
    eyebrow_target = 0  # Target offset
    eyebrow_speed = 3  # Speed of movement
    eyebrow_max_offset = int(screen.get_width() * 0.8)  # Maximum right movement
    eyebrow_timer = 0
    eyebrow_duration = 30  # How long to hold the position
    is_eyebrow_moving = False
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_l and not is_looking:
                    movement_target = 1
                    is_looking = True
                    look_timer = 0
                elif event.key == pygame.K_r and not is_looking:
                    movement_target = -1
                    is_looking = True
                    look_timer = 0
                elif event.key == pygame.K_e and not is_eyebrow_moving:
                    eyebrow_target = eyebrow_max_offset
                    is_eyebrow_moving = True
                    eyebrow_timer = 0

        # Update look timer and handle auto-return
        if is_looking:
            look_timer += 1
            if look_timer >= look_duration:
                movement_target = 0
                is_looking = False

        # Update eyebrow timer and handle auto-return
        if is_eyebrow_moving:
            eyebrow_timer += 1
            if eyebrow_timer >= eyebrow_duration:
                eyebrow_target = 0
                is_eyebrow_moving = False

        # Smoothly move eyes toward target
        if look_direction < movement_target:
            look_direction = min(movement_target, look_direction + movement_speed)
        elif look_direction > movement_target:
            look_direction = max(movement_target, look_direction - movement_speed)

        # Smoothly move eyebrows toward target
        if eyebrow_offset < eyebrow_target:
            eyebrow_offset = min(eyebrow_target, eyebrow_offset + eyebrow_speed)
        elif eyebrow_offset > eyebrow_target:
            eyebrow_offset = max(eyebrow_target, eyebrow_offset - eyebrow_speed)

        # Update animation states
        blink_timer += 1
        if blink_timer == next_blink:
            eye_state = 'closed'
        elif blink_timer == next_blink + blink_duration:
            eye_state = 'open'
            blink_timer = 0
            next_blink = random.randint(2 * 60, 4 * 60)

        # Draw face with current look_direction and eyebrow_offset
        draw_face(screen, eye_state, mouth_state, look_direction, eyebrow_offset)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()