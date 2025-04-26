# MultiTrack

A unicycle model simulation with Kalman filter-based monitoring system for state estimation and prediction.

## Overview

This project simulates a unicycle model with an implemented Kalman filter for state estimation and prediction. It demonstrates how Kalman filters can be used to track and predict the movement of systems with noisy measurements.

## Features

- Unicycle model simulation with simple dynamics
- Extended Kalman filter implementation for state estimation
- Real-time visualization of state predictions and uncertainty
- Configurable measurement uncertainty to simulate different sensor qualities
- Interactive control using keyboard arrow keys

## Files

- `unicycle_reachability_simulation.py`: Main simulation file with pygame visualization
- `kalman_filter.py`: Implementation of the Extended Kalman Filter for the unicycle model
- `create_conda_env.sh`: Script to create conda environment
- `environment.yml`: Conda environment configuration

## Controls

- Arrow keys: Control the unicycle (Up/Down for forward/backward, Left/Right for turning)
- K: Toggle Kalman filter prediction display
- U: Toggle uncertainty ellipse display
- F: Toggle FPS display
- ESC: Quit

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