# Description

This is a Project to explore the possibilities of state of the art Autonomous Navigation Implementations utilizing Computer Vision.

***Core Components***
 1. Jetson Orin Nano 8gb: Onboard Computation Tasks
 2. ArkV6 with Pab Jetson Carrier: Open Source Flightcontroller (PX4)
 3. Intel Realsense D435: DepthCamera 

## Dataflow

DepthCamera --> State Estimation (Vio) + Pointcloud --> Pathfinding
