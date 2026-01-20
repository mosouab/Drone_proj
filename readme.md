# Drone Project

This project provides a containerized development environment for drone simulation using PX4, ROS Noetic, Gazebo, and MAVROS.

## Prerequisites

- Docker and Docker Compose installed
- WSLg (for GUI support on Windows) or X11 server

## Quick Start

### 1. Build and Run the Container

Navigate to the docker directory and start the container:

```bash
cd docker
docker-compose up -d
```

### 2. Enter the Container

```bash
docker exec -it drone_dev_env bash
```

### 3. Launch the PX4 SITL Simulation

Inside the container, navigate to the PX4-Autopilot directory and launch the simulation:

```bash
cd /root/workspace/PX4-Autopilot
make px4_sitl gazebo
```

This will start:
- PX4 SITL (Software-In-The-Loop) simulator
- Gazebo simulation environment
- MAVLink communication on default ports (UDP 14550, 14540)

### 4. Run Drone Control Scripts

The workspace contains several control utilities in `/root/workspace/bin/`:
- `drone_ctrl` - Manual drone control
- `drone_land` - Landing command
- `rtl` - Return to launch
- `waypoints` - Waypoint navigation

You can also use the source files in `/root/workspace/src/` for custom control programs.

## Connecting External Ground Control Station

The container uses host network mode, allowing you to connect QGroundControl or Mission Planner from your host machine on the default MAVLink port (UDP 14550).

## Stopping the Container

```bash
docker-compose down
```
