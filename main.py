#!/usr/bin/env python3
"""
Main entry point for the MultiTrack simulation.
This script launches the unicycle simulation with Kalman filter and MPPI controller.
"""
from multitrack.utils.config import *
from unicycle_reachability_simulation import run_simulation

if __name__ == "__main__":
    print(f"Starting MultiTrack simulation...")
    run_simulation()