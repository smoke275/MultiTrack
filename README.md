# MultiTrack

A unicycle model simulation with Kalman filter-based monitoring system for state estimation and prediction, plus an MPPI-controlled follower agent.

## Overview

This project simulates a unicycle model with both Kalman filter state estimation and a Model Predictive Path Integral (MPPI) controller. It demonstrates how these techniques can be used to track, predict, and control autonomous agents in a dynamic environment.

## Features

- Unicycle model simulation with simple dynamics
- Extended Kalman filter implementation for state estimation
- Real-time visualization of state predictions and uncertainty
- MPPI controller for optimal trajectory planning and following
- Automated follower agent that tracks the user-controlled agent
- Configurable measurement uncertainty and following distance
- Interactive control using keyboard

## Files

- `unicycle_reachability_simulation.py`: Main simulation file with pygame visualization
- `kalman_filter.py`: Implementation of the Extended Kalman Filter
- `mppi_controller.py`: Implementation of the Model Predictive Path Integral controller
- `create_conda_env.sh`: Script to create conda environment
- `environment.yml`: Conda environment configuration

## Controls

- **Arrow keys**: Control the leader unicycle (Up/Down for forward/backward, Left/Right for turning)
- **K**: Toggle Kalman filter prediction display
- **U**: Toggle uncertainty ellipse display
- **M**: Toggle MPPI prediction display
- **T**: Toggle follower agent
- **+/-**: Increase/decrease follower distance
- **R**: Reset follower position (random)
- **F**: Toggle FPS display
- **ESC**: Quit

## Requirements

- Python 3.6+
- NumPy
- SciPy
- Pygame

## Setup

You can set up the environment using conda:

```bash
./create_conda_env.sh
```

Or manually:

```bash
conda env create -f environment.yml
conda activate multitrack
```

## Running the Simulation

```bash
python unicycle_reachability_simulation.py
```

## Methodology

### Kalman Filter

The Kalman filter provides optimal state estimation for the unicycle, taking into account both the model predictions and noisy sensor measurements. The implementation uses an Extended Kalman Filter to handle the nonlinear unicycle dynamics.

### MPPI Controller

The Model Predictive Path Integral (MPPI) controller optimizes control inputs by:

1. Sampling multiple possible control sequences
2. Evaluating their costs based on tracking error, control effort, and collision avoidance
3. Computing a weighted average of sampled controls, favoring lower-cost trajectories
4. Applying the optimized control to the follower agent

This sampling-based approach efficiently handles the non-linear dynamics and constraints of the unicycle model.