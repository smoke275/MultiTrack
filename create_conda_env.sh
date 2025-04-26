#!/bin/bash
# Script to create and activate a conda environment for this project using environment.yml

# Create the conda environment from environment.yml
conda env create -f environment.yml

echo "Conda environment created from environment.yml."
echo "To activate it, run:"
echo "  conda activate multitrack-env"