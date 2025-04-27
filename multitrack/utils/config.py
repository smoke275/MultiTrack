"""
Constants and configuration settings for the unicycle simulation and controllers.
This file centralizes all configurable parameters to make adjustments easier.
"""

# Screen dimensions
WIDTH = 800
HEIGHT = 600

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
ORANGE = (255, 165, 0)  # Color for the follower agent

# Leader agent settings
LEADER_LINEAR_VEL = 50.0        # Maximum linear velocity for leader
LEADER_ANGULAR_VEL = 1.0        # Maximum angular velocity for leader

# Follower agent settings
FOLLOWER_ENABLED = True                  # Enable/disable follower agent
FOLLOWER_TARGET_DISTANCE = 100.0         # Default following distance
FOLLOWER_LINEAR_VEL_MIN = 0.0            # Minimum linear velocity (changed from -50.0 to enforce forward-only motion)
FOLLOWER_LINEAR_VEL_MAX = 50.0           # Maximum linear velocity
FOLLOWER_ANGULAR_VEL_MIN = -0.6          # Minimum angular velocity (less agile)
FOLLOWER_ANGULAR_VEL_MAX = 0.6           # Maximum angular velocity (less agile)
FOLLOWER_LINEAR_NOISE_SIGMA = 10.0       # Noise std dev for linear velocity in MPPI
FOLLOWER_ANGULAR_NOISE_SIGMA = 0.3       # Noise std dev for angular velocity in MPPI
FOLLOWER_SAFETY_DISTANCE = 30.0          # Minimum safety distance for collision avoidance
FOLLOWER_MIN_DISTANCE = 50.0             # Minimum allowed following distance
FOLLOWER_MAX_DISTANCE = 200.0            # Maximum allowed following distance

# MPPI controller settings
MPPI_HORIZON = 20                       # Reduced from 30 for better performance
MPPI_SAMPLES = 300                      # Reduced from 1000 for better performance
MPPI_LAMBDA = 0.05                      # Temperature for softmax weighting - decreased for smoother control
MPPI_WEIGHT_POSITION = 1.0              # Weight for position tracking
MPPI_WEIGHT_HEADING = 0.5               # Weight for heading alignment
MPPI_WEIGHT_CONTROL = 0.1               # Weight for control effort - increased for smoother control
MPPI_WEIGHT_COLLISION = 10.0            # Weight for collision avoidance
MPPI_WEIGHT_FORWARD = 0.3               # Weight for forward direction incentive
MPPI_USE_GPU = True                     # Enable GPU acceleration
MPPI_GPU_BATCH_SIZE = 100               # Process samples in batches for better GPU memory management
MPPI_USE_ASYNC = True                   # Use asynchronous computation where possible
MPPI_CACHE_SIZE = 5                     # Cache recent computations for reuse

# Kalman filter settings
KF_MEASUREMENT_INTERVAL = 0.5           # How often to take measurements (seconds)
KF_MEASUREMENT_NOISE_POS = 2.0          # Position measurement noise
KF_MEASUREMENT_NOISE_ANGLE = 0.1        # Angle measurement noise
KF_PROCESS_NOISE_POS = 2.0              # Process noise for position
KF_PROCESS_NOISE_ANGLE = 0.1            # Process noise for angle

# Visualization settings
SHOW_PREDICTIONS = True                 # Show Kalman filter predictions
PREDICTION_STEPS = 20                   # Number of steps to predict into future
SHOW_UNCERTAINTY = True                 # Show uncertainty ellipse
SHOW_MPPI_PREDICTIONS = True            # Show MPPI predictions
PREDICTION_COLOR = CYAN                 # Color for Kalman predictions
UNCERTAINTY_COLOR = (100, 100, 255, 100)  # Color for uncertainty ellipse (with alpha)
MPPI_PREDICTION_COLOR = (255, 100, 100)   # Color for MPPI predictions