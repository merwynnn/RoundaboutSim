# Roundabout Traffic Simulation

## Overview
A Python-based traffic simulation system modeling vehicle flow through roundabout intersections using pygame and dearpygui. The simulation processes traffic flow configurations from Excel files and supports various simulation parameters.

## Current Status
- **Workflow**: Running (Roundabout Simulation)
- **Status**: Active and processing simulations
- **Last Updated**: December 22, 2025

## Project Structure
- `main.py` - Primary entry point for the simulation
- `Simulator.py` - Core simulation engine
- `Car.py` - Vehicle behavior and properties
- `Road.py` - Road network and connections
- `Intersections.py` - Roundabout and intersection logic
- `FlowManager.py` - Traffic flow management
- `SpatialGrid.py` - Spatial partitioning for optimization
- `Camera.py` - Viewport and visualization
- `Constants.py` - Configuration constants
- `Assets/` - Car sprite images (red, blue, black)
- `configs/` - Excel configuration files for traffic flow scenarios

## Dependencies
- Python 3.8
- pygame 2.1.2 - Graphics and simulation framework
- dearpygui 1.6.2 - GUI library

## Configuration
- Traffic flow rates configurable via Excel files in `configs/`
- Supports grid-based roundabout networks
- Adjustable multipliers for flow testing

## Running the Project
```bash
python main.py
```

The simulation loads traffic configurations and runs various test scenarios with different multipliers.
