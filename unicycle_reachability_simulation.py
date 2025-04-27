"""
Unicycle model simulation with Kalman filter monitoring system for state estimation and prediction.
Also includes an MPPI-controlled follower agent that tracks the main agent.
"""
import pygame
import numpy as np
import sys
from math import sin, cos, pi, sqrt
from multitrack.filters.kalman_filter import UnicycleKalmanFilter
from multitrack.controllers.mppi_controller import MPPIController, DEVICE_INFO
from multitrack.utils.config import *

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
ORANGE = (255, 165, 0)  # Color for the follower agent

# Monitoring system settings
SHOW_PREDICTIONS = True  # Show Kalman filter predictions
PREDICTION_STEPS = 20    # Number of steps to predict into the future
SHOW_UNCERTAINTY = True  # Show uncertainty ellipse
PREDICTION_COLOR = CYAN  # Color for predictions
UNCERTAINTY_COLOR = (100, 100, 255, 100)  # Light blue with transparency

# Follower agent settings
SHOW_MPPI_PREDICTIONS = True  # Show MPPI predictions
MPPI_PREDICTION_COLOR = (255, 100, 100)  # Light red
FOLLOWER_ENABLED = True  # Enable/disable follower agent

class UnicycleModel:
    def __init__(self):
        # State: [x, y, theta, v]
        self.state = np.array([WIDTH//2, HEIGHT//2, 0.0, 0.0])
        # Control inputs: [v, omega] (linear and angular velocity)
        self.controls = np.array([0.0, 0.0])
        # Process noise parameters
        self.noise_pos = KF_PROCESS_NOISE_POS
        self.noise_angle = KF_PROCESS_NOISE_ANGLE
        
        # Initialize Kalman filter for monitoring
        self.kalman_filter = UnicycleKalmanFilter(self.state, dt=0.1)
        self.kalman_predictions = []
        self.last_measurement_time = 0
        self.measurement_interval = KF_MEASUREMENT_INTERVAL
        self.prediction_horizon = PREDICTION_STEPS
        
        # Add measurement noise - for realistic monitoring
        self.measurement_noise_pos = KF_MEASUREMENT_NOISE_POS
        self.measurement_noise_angle = KF_MEASUREMENT_NOISE_ANGLE
    
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

class FollowerAgent:
    def __init__(self, initial_state=None, target_distance=FOLLOWER_TARGET_DISTANCE):
        """
        Follower agent that uses an MPPI controller to follow a target
        
        Parameters:
        - initial_state: [x, y, theta, v] or None for random initialization
        - target_distance: Desired following distance
        """
        # Initialize state
        if initial_state is None:
            # Random initialization away from the center
            x = np.random.uniform(WIDTH * 0.2, WIDTH * 0.8)
            y = np.random.uniform(HEIGHT * 0.2, HEIGHT * 0.8)
            theta = np.random.uniform(-pi, pi)
            self.state = np.array([x, y, theta, 0.0])
        else:
            self.state = initial_state.copy()
        
        # Control inputs
        self.controls = np.array([0.0, 0.0])
        
        # Initialize MPPI controller
        self.mppi = MPPIController(horizon=MPPI_HORIZON, samples=MPPI_SAMPLES, dt=0.1)
        
        # Last predicted trajectory from MPPI
        self.predicted_trajectory = None
        
        # Target distance to maintain from the leader
        self.target_distance = target_distance
        
        # History of states for visualization
        self.history = []
        self.max_history = 20
    
    def update(self, dt, leader_state, obstacles=None):
        """
        Update follower agent state based on leader position
        
        Parameters:
        - dt: Time step
        - leader_state: State of the leader agent [x, y, theta, v]
        - obstacles: List of obstacle positions [(x, y, radius), ...] or None
        """
        # Generate target trajectory (follow leader at a distance)
        target_trajectory = self._generate_target_trajectory(leader_state)
        
        # Compute optimal control using MPPI
        optimal_control, predicted_trajectory = self.mppi.compute_control(
            self.state, target_trajectory, obstacles)
        
        # Store predicted trajectory for visualization
        self.predicted_trajectory = predicted_trajectory
        
        # Apply control
        self.controls = optimal_control
        
        # Update state using unicycle dynamics
        x, y, theta, _ = self.state
        v, omega = self.controls
        
        # Unicycle model dynamics
        x += v * cos(theta) * dt
        y += v * sin(theta) * dt
        theta += omega * dt
        
        # Normalize angle to [-pi, pi]
        theta = (theta + pi) % (2 * pi) - pi
        
        # Boundary conditions
        x = np.clip(x, 0, WIDTH)
        y = np.clip(y, 0, HEIGHT)
        
        # Update state
        self.state = np.array([x, y, theta, v])
        
        # Add current state to history
        self.history.append(self.state.copy())
        if len(self.history) > self.max_history:
            self.history.pop(0)
    
    def _generate_target_trajectory(self, leader_state):
        """
        Generate a target trajectory to follow leader at a specified distance
        
        Parameters:
        - leader_state: State of the leader agent [x, y, theta, v]
        
        Returns:
        - target_trajectory: Array of target states
        """
        lx, ly, ltheta, lv = leader_state
        
        # Target position is behind the leader at the specified distance
        tx = lx - self.target_distance * cos(ltheta)
        ty = ly - self.target_distance * sin(ltheta)
        
        # Target heading should match the leader
        ttheta = ltheta
        
        # Target velocity should match the leader
        tv = lv
        
        # For simplicity, repeat the target state for the entire horizon
        target_state = np.array([tx, ty, ttheta, tv])
        target_trajectory = np.tile(target_state, (self.mppi.horizon + 1, 1))
        
        return target_trajectory
    
    def draw(self, screen):
        """Draw the follower agent and its predicted trajectory"""
        # Draw trajectory history as a fading trail
        if len(self.history) > 1:
            for i in range(1, len(self.history)):
                alpha = int(255 * i / len(self.history))
                color = (min(255, ORANGE[0]), 
                        min(255, ORANGE[1]), 
                        min(255, ORANGE[2]))
                
                prev_pos = self.history[i-1][:2]
                curr_pos = self.history[i][:2]
                
                pygame.draw.line(screen, color, 
                            (int(prev_pos[0]), int(prev_pos[1])), 
                            (int(curr_pos[0]), int(curr_pos[1])), 2)
        
        # Draw MPPI predicted trajectory
        if SHOW_MPPI_PREDICTIONS and self.predicted_trajectory is not None:
            for i in range(1, len(self.predicted_trajectory)):
                pred_prev = self.predicted_trajectory[i-1]
                pred_curr = self.predicted_trajectory[i]
                
                # Draw line from previous prediction to current
                pygame.draw.line(screen, MPPI_PREDICTION_COLOR, 
                                (int(pred_prev[0]), int(pred_prev[1])), 
                                (int(pred_curr[0]), int(pred_curr[1])), 1)
                
                # Draw small circle at each prediction point (less frequently to avoid clutter)
                if i % 3 == 0:
                    pygame.draw.circle(screen, MPPI_PREDICTION_COLOR, 
                                    (int(pred_curr[0]), int(pred_curr[1])), 2)
        
        # Draw follower agent
        x, y, theta, _ = self.state
        radius = 10
        pygame.draw.circle(screen, ORANGE, (int(x), int(y)), radius)
        end_x = x + radius * cos(theta)
        end_y = y + radius * sin(theta)
        pygame.draw.line(screen, WHITE, (x, y), (end_x, end_y), 2)

# Main simulation loop
def run_simulation():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Unicycle Model with MPPI Follower")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Arial', 16)
    
    # Monitoring options
    global SHOW_PREDICTIONS, SHOW_UNCERTAINTY, SHOW_MPPI_PREDICTIONS, FOLLOWER_ENABLED
    
    model = UnicycleModel()
    follower = FollowerAgent(target_distance=100.0) if FOLLOWER_ENABLED else None
    
    # Display options
    show_fps = True
    
    # Performance monitoring
    frame_times = []
    last_mppi_update_time = 0
    mppi_update_interval = 1.0  # Update MPPI stats every second
    
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
                elif event.key == pygame.K_m:
                    # Toggle MPPI predictions display
                    SHOW_MPPI_PREDICTIONS = not SHOW_MPPI_PREDICTIONS
                elif event.key == pygame.K_t:
                    # Toggle follower agent
                    FOLLOWER_ENABLED = not FOLLOWER_ENABLED
                    if FOLLOWER_ENABLED and follower is None:
                        follower = FollowerAgent(target_distance=100.0)
                    if not FOLLOWER_ENABLED:
                        follower = None
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    # Increase following distance
                    if follower:
                        follower.target_distance = min(FOLLOWER_MAX_DISTANCE, follower.target_distance + 10)
                elif event.key == pygame.K_MINUS:
                    # Decrease following distance
                    if follower:
                        follower.target_distance = max(FOLLOWER_MIN_DISTANCE, follower.target_distance - 10)
                elif event.key == pygame.K_r:
                    # Reset follower position (random)
                    if follower:
                        follower = FollowerAgent(target_distance=follower.target_distance)
        
        # Get keyboard input to control the unicycle
        keys = pygame.key.get_pressed()
        linear_vel = 0
        angular_vel = 0
        
        if keys[pygame.K_UP]:
            linear_vel = LEADER_LINEAR_VEL
        if keys[pygame.K_DOWN]:
            linear_vel = -LEADER_LINEAR_VEL
        # Reversed controls for left and right
        if keys[pygame.K_RIGHT]:
            angular_vel = LEADER_ANGULAR_VEL
        if keys[pygame.K_LEFT]:
            angular_vel = -LEADER_ANGULAR_VEL
            
        model.set_controls(linear_vel, angular_vel)
        
        # Calculate elapsed time in seconds
        elapsed_time = (pygame.time.get_ticks() - start_time_ms) / 1000.0
        
        # Update model with elapsed time for Kalman filter timing
        model.update(elapsed_time=elapsed_time)
        
        # Update follower agent
        if follower:
            # Use Kalman filter estimate instead of actual state
            kalman_state = np.array([
                model.kalman_filter.state[0],  # x from Kalman filter
                model.kalman_filter.state[1],  # y from Kalman filter
                model.kalman_filter.state[2],  # theta from Kalman filter
                model.state[3]                 # v (keep original velocity)
            ])
            follower.update(dt=0.1, leader_state=kalman_state)
        
        # Get current time to update MPPI stats at regular intervals
        current_time = elapsed_time
        
        # Update MPPI performance statistics
        mppi_stats = None
        if follower and current_time - last_mppi_update_time >= mppi_update_interval:
            mppi_stats = follower.mppi.get_computation_stats()
            last_mppi_update_time = current_time
        
        # Clear screen
        screen.fill(BLACK)
        
        # Draw model
        model.draw(screen)
        
        # Draw follower agent
        if follower:
            follower.draw(screen)
            
            # Draw line from leader to follower
            leader_pos = (int(model.state[0]), int(model.state[1]))
            follower_pos = (int(follower.state[0]), int(follower.state[1]))
            pygame.draw.line(screen, (50, 50, 50), leader_pos, follower_pos, 1)
        
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
            f"Leader: ({int(model.state[0])}, {int(model.state[1])}), Heading: {model.state[2]:.2f}",
            f"F: Toggle FPS | K: Kalman viz | U: Uncertainty | M: MPPI viz",
            f"T: Toggle follower | +/-: Adjust follow distance | R: Reset follower",
            f"Computing device: {DEVICE_INFO}"  # Add device info to display
        ]
        
        if follower:
            info_text.append(f"Follower: ({int(follower.state[0])}, {int(follower.state[1])}), Target dist: {follower.target_distance:.1f}")
            
            # Display MPPI performance statistics if available
            if mppi_stats:
                info_text.append(f"MPPI compute: {mppi_stats['last_time']:.1f}ms (avg: {mppi_stats['avg_time']:.1f}ms)")
                cache_hits = sum(1 for entry in follower.mppi.computation_cache if entry is not None)
                info_text.append(f"Cache: {cache_hits}/{MPPI_CACHE_SIZE} | Batch size: {follower.mppi.batch_size}")
        
        if show_fps:
            info_text.append(f"FPS: {fps} (Avg frame time: {avg_frame_time:.1f}ms)")
        
        # Calculate text panel height to ensure all text is visible
        text_panel_height = len(info_text) * 20 + 10  # 20px per line + 10px padding
        
        # Draw text background for better readability
        pygame.draw.rect(screen, (0, 0, 0, 180), (0, HEIGHT - text_panel_height, WIDTH, text_panel_height))
        
        # Display text with adjusted starting position to ensure all lines are visible
        for i, text in enumerate(info_text):
            text_surf = font.render(text, True, WHITE)
            screen.blit(text_surf, (10, HEIGHT - text_panel_height + 5 + i*20))
        
        # Draw title
        title_text = "Unicycle Model with MPPI Follower"
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
