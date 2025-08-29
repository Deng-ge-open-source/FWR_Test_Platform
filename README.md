# FWR_Test_Platform

The **Flapping Wing Rotor (FWR) Physically Driven Digital Twin (PDDT) Test Platform** is developed as part of my PhD research project.
 It serves as an experimental system for reproducing and validating the hybrid flapping wing rotor concept through both hardware testing and digital twin simulation.

------

## Overview

The platform consists of two major components:

- **Rotor Head**
- **Experimental Measurement Platform**

Figures 2 and 3 illustrate the detailed structural layout and component arrangement of the system.

------

## Project Folder Structure

1. **Rotor_head/**
    Contains the main controller program for operating the Rotor Head.
2. **Measurement_Platform/**
    Contains the main controller program for data acquisition on the Measurement Platform.
3. **Mechanical_structure/**
    Provides 3D-printable mechanical design files for fabricating the entire test platform.
4. **Simulink/**
    Includes Simulink models for PC-side operation. These models handle signal exchange and command execution between the Rotor Head and the Measurement Platform, enabling the implementation of the PDDT framework.
   - Recommended environment: **MATLAB 2022a**

------

## Deployment Instructions

1. **Controller Programs**
    All controller codes should be directly flashed onto the **ESP32 Arduino** boards.
2. **Simulink Programs**
    Run the Simulink models using **MATLAB 2022a**.
3. **UDP Communication Setup**
    Configure the wireless IP addresses and ports for both the PC and the main controllers in the UDP section of the code (refer to program comments). Once configured, wireless communication and real-time control can be established.

------

## Troubleshooting

If you encounter any issues, please submit them via **GitHub Issues**, and I will do my best to provide support.

------

## Figures



![1](C:\Users\s336079\OneDrive - Cranfield University\Documents\GitHub\FWR_Test_Platform\Pic\1.png)

Figure 1:FWR MAV Test Platform

![2](C:\Users\s336079\OneDrive - Cranfield University\Documents\GitHub\FWR_Test_Platform\Pic\2.png)

Figure 2:Rotor head on FWR MAV Test Platform

![3](C:\Users\s336079\OneDrive - Cranfield University\Documents\GitHub\FWR_Test_Platform\Pic\3.png)

Figure 3:Measurement platform on FWR MAV Test Platform
