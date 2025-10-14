# FIRE_CP_Modelica
### Cyber-Physical System Models in Modelica
This repository provides **cyber–physical models of robotic vehicles**, including both **aerial (drone)** and **ground (rover)** systems, developed in **Modelica**.  
Each model integrates both *physical dynamics* (chassis, actuators, and sensors) and *cyber components* (control, sensor fusion, communication) to enable closed-loop simulation, reachability analysis, and vulnerability assessment.

---

## Supported Models

### 1. Rover
The rover model captures full **vehicle dynamics**, including suspension, tire–ground interaction, and drive–motor coupling.  
On the cyber side, it incorporates **embedded control algorithms**, **sensor fusion**, and **communication delay/fault injection interfaces**, enabling emulations of **cyber–physical vulnerabilities**.

This model is based on and extends the approaches described in:  
- [Ref. 1] *[Paper title 1]*  
- [Ref. 2] *[Paper title 2]*  

---

### 2. Drone

## Key Features
- Modular Modelica models for *cyber–physical integration*   
- Includes *attack and fault injection hooks* for CPS vulnerability analysis*  

---

## Applications
- Cyber-physical vulnerability assessment  
- Multi-fidelity simulation studies  
- Formal reachability and risk analysis  

---



### Acknowledgement

[//]: <> ### Citation If you use this repository, please cite the relevant rover or drone model papers: ```bibtex@article{<Our Paper>,  title   = {...},  author  = {...},  journal = {...},  year    = {...}}
