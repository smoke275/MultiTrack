"""
Unicycle model simulation with Kalman filter monitoring system for state estimation and prediction.
"""
import pygame
import numpy as np
import sys
from math import sin, cos, pi
from kalman_filter import UnicycleKalmanFilter

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)

# Monitoring system settings
SHOW_PREDICTIONS = True  # Show Kalman filter predictions
PREDICTION_STEPS = 20    # Number of steps to predict into the future
SHOW_UNCERTAINTY = True  # Show uncertainty ellipse
PREDICTION_COLOR = CYAN  # Color for predictions
UNCERTAINTY_COLOR = (100, 100, 255, 100)  # Light blue with transparency

class UnicycleModel:
    def __init__(self):
        # State: [x, y, theta, v]
        self.state = np.array([WIDTH//2, HEIGHT//2, 0.0, 0.0])
        # Control inputs: [v, omega] (linear and angular velocity)
        self.controls = np.array([0.0, 0.0])
        # Process noise parameters
        self.noise_pos = 2.0
        self.noise_angle = 0.1
        
        # Initialize Kalman filter for monitoring
        self.kalman_filter = UnicycleKalmanFilter(self.state, dt=0.1)
        self.kalman_predictions = []
        self.last_measurement_time = 0
        self.measurement_interval = 0.5  # How often to take measurements (seconds)
        self.prediction_horizon = PREDICTION_STEPS
        
        # Add measurement noise - for realistic monitoring (bad sensor)
        self.measurement_noise_pos = 25.0  # Increased from 5.0 to 25.0
        self.measurement_noise_angle = 0.5  # Increased from 0.1 to 0.5
    
    def update(self, dt=0.1, elapsed_time=0):
        v, omega = self.controls
        theta = self.state[2]
        
        # Unicycle model dynamics
        self.state[0] += v * cos(theta) * dt
        self.state[1] += v * sin(theta) * dt
        self.state[2] += omega * dt
        self.state[3] = v  # Update velocity state
        
        # Normalize angle to [-pi, pi]
        self.state[2] = (self.state[2] + pi) % (2 * pi) - pi
        
        # Boundary conditions
        self.state[0] = np.clip(self.state[0], 0, WIDTH)
        self.state[1] = np.clip(self.state[1], 0, HEIGHT)
        
        # Update Kalman filter monitoring system
        if elapsed_time - self.last_measurement_time >= self.measurement_interval:
            # Create a noisy measurement
            noisy_measurement = np.array([
                self.state[0] + np.random.normal(0, self.measurement_noise_pos),
                self.state[1] + np.random.normal(0, self.measurement_noise_pos),
                self.state[2] + np.random.normal(0, self.measurement_noise_angle)
            ])
            
            # Update Kalman filter with measurement and control input
            self.kalman_filter.update(noisy_measurement, self.controls)
            self.last_measurement_time = elapsed_time
            
            # Generate predictions for visualization
            self.kalman_predictions = self.kalman_filter.predict(self.controls, self.prediction_horizon)
        else:
            # Just update the filter prediction without measurement
            self.kalman_filter.update(None, self.controls)
    
    def set_controls(self, linear_vel, angular_vel):
        self.controls = np.array([linear_vel, angular_vel])
    
    def draw(self, screen):
        # Draw Kalman filter monitoring visualizations
        if SHOW_PREDICTIONS and self.kalman_predictions:
            # Draw prediction path
            for i in range(1, len(self.kalman_predictions)):
                pred_prev = self.kalman_predictions[i-1]
                pred_curr = self.kalman_predictions[i]
                
                # Draw line from previous prediction to current
                pygame.draw.line(screen, PREDICTION_COLOR, 
                                (pred_prev[0], pred_prev[1]), 
                                (pred_curr[0], pred_curr[1]), 2)
                
                # Draw small circle at each prediction point
                if i % 5 == 0:  # Draw every 5th point to avoid clutter
                    pygame.draw.circle(screen, PREDICTION_COLOR, 
                                      (int(pred_curr[0]), int(pred_curr[1])), 3)
            
            # Draw final predicted position
            if len(self.kalman_predictions) > 1:
                final_pred = self.kalman_predictions[-1]
                pygame.draw.circle(screen, YELLOW, 
                                  (int(final_pred[0]), int(final_pred[1])), 5)
        
        # Draw uncertainty ellipse
        if SHOW_UNCERTAINTY:
            # Get uncertainty ellipse points from Kalman filter
            ellipse_points = self.kalman_filter.get_prediction_ellipse(0.95)
            
            # Convert points to integers for pygame
            ellipse_points_int = [(int(p[0]), int(p[1])) for p in ellipse_points]
            
            # Draw ellipse as polygon
            if len(ellipse_points_int) > 2:
                pygame.draw.polygon(screen, UNCERTAINTY_COLOR, ellipse_points_int, 1)
        
        # Draw Kalman filter estimated position
        kf_x, kf_y = self.kalman_filter.state[0], self.kalman_filter.state[1]
        kf_theta = self.kalman_filter.state[2]
        
        # Draw estimated position as circle with direction indicator
        pygame.draw.circle(screen, GREEN, (int(kf_x), int(kf_y)), 8, 2)
        end_x = kf_x + 15 * cos(kf_theta)
        end_y = kf_y + 15 * sin(kf_theta)
        pygame.draw.line(screen, GREEN, (kf_x, kf_y), (end_x, end_y), 2)
        
        # Draw unicycle agent
        x, y, theta, _ = self.state
        radius = 10
        pygame.draw.circle(screen, RED, (int(x), int(y)), radius)
        end_x = x + radius * cos(theta)
        end_y = y + radius * sin(theta)
        pygame.draw.line(screen, WHITE, (x, y), (end_x, end_y), 2)

# Main simulation loop
def run_simulation():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Unicycle Model Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Arial', 16)
    
    model = UnicycleModel()
    
    # Display options
    show_fps = True
    
    # Monitoring options
    global SHOW_PREDICTIONS, SHOW_UNCERTAINTY
    
    # Performance monitoring
    frame_times = []
    
    # Time tracking
    start_time_ms = pygame.time.get_ticks()
    
    running = True
    while running:
        frame_start_time = pygame.time.get_ticks()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_f:
                    show_fps = not show_fps
                elif event.key == pygame.K_k:
                    # Toggle Kalman filter prediction display
                    SHOW_PREDICTIONS = not SHOW_PREDICTIONS
                elif event.key == pygame.K_u:
                    # Toggle uncertainty ellipse display
                    SHOW_UNCERTAINTY = not SHOW_UNCERTAINTY
        
        # Get keyboard input to control the unicycle
        keys = pygame.key.get_pressed()
        linear_vel = 0
        angular_vel = 0
        
        if keys[pygame.K_UP]:
            linear_vel = 50
        if keys[pygame.K_DOWN]:
            linear_vel = -50
        # Reversed controls for left and right
        if keys[pygame.K_RIGHT]:
            angular_vel = 1.0
        if keys[pygame.K_LEFT]:
            angular_vel = -1.0
            
        model.set_controls(linear_vel, angular_vel)
        
        # Calculate elapsed time in seconds
        elapsed_time = (pygame.time.get_ticks() - start_time_ms) / 1000.0
        
        # Update model with elapsed time for Kalman filter timing
        model.update(elapsed_time=elapsed_time)
        
        # Clear screen
        screen.fill(BLACK)
        
        # Draw model
        model.draw(screen)
        
        # Calculate FPS
        frame_end_time = pygame.time.get_ticks()
        frame_time = frame_end_time - frame_start_time
        frame_times.append(frame_time)
        if len(frame_times) > 30:
            frame_times.pop(0)
        avg_frame_time = sum(frame_times) / len(frame_times)
        fps = int(1000 / max(1, avg_frame_time))
        
        # Display info
        info_text = [
            f"Controls: Arrow keys to move, ESC to quit",
            f"Position: ({int(model.state[0])}, {int(model.state[1])}), Heading: {model.state[2]:.2f}",
            f"F: Toggle FPS | K: Toggle Kalman predictions | U: Toggle uncertainty",
            f"Kalman estimate: ({int(model.kalman_filter.state[0])}, {int(model.kalman_filter.state[1])})"
        ]
        
        if show_fps:
            info_text.append(f"FPS: {fps} (Avg frame time: {avg_frame_time:.1f}ms)")
        
        for i, text in enumerate(info_text):
            text_surf = font.render(text, True, WHITE)
            screen.blit(text_surf, (10, HEIGHT - 110 + i*20))
        
        # Draw title
        title_text = "Unicycle Model Simulation"
        title = font.render(title_text, True, WHITE)
        screen.blit(title, (WIDTH//2 - title.get_width()//2, 10))
        
        # Update display
        pygame.display.flip()
        
        # Control framerate
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    run_simulation()
