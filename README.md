# FIRE_CP_Modelica
### Cyber-Physical System Models in Modelica
This repository provides **cyber–physical models of robotic vehicles**, including both **aerial (drone)** and **ground (rover)** systems, developed in **Modelica**.  
Each model integrates both *physical dynamics* (chassis, actuators, and sensors) and *cyber components* (control, sensor fusion, communication) to enable closed-loop simulation, reachability analysis, and vulnerability assessment.

---

## Supported Models

### 1. Rover
The rover model captures full **vehicle dynamics**, including suspension, tire–ground interaction, and drive–motor coupling. On the cyber side, it incorporates **embedded control algorithms**, **sensor fusion**, and **communication delay/fault injection interfaces**, enabling emulations of **cyber–physical vulnerabilities**. For vehicle parameters, please refer to the in-line comments and explanations in the Modelica source files.

This model is based on and extends the approaches described in:  
- [Ref. 1] (Chassis Dynamics) *X. Yang, “Improvements in vehicle handling and stability by a novel wheel slip coordination control scheme,” IJVD, vol. 62, no. 2/3/4, p. 206, 2013, doi: 10.1504/IJVD.2013.052702.*
- [Ref. 2] (Tire-Ground Interaction) *T.-Y. Kim, S. Jung, and W.-S. Yoo, “Advanced slip ratio for ensuring numerical stability of low-speed driving simulation: Part II—lateral slip ratio,” Proceedings of the Institution of Mechanical Engineers, Part D: Journal of Automobile Engineering, vol. 233, no. 11, pp. 2903–2911, Sep. 2019, doi: 10.1177/0954407018807040.*
- [Ref. 3] (Vehicle Rollover) *M. Doumiati, Victorino ,A., Charara ,A., and D. and Lechner, “Lateral load transfer and normal forces estimation for vehicle safety: experimental test,” Vehicle System Dynamics, vol. 47, no. 12, pp. 1511–1533, Dec. 2009, doi: 10.1080/00423110802673091.*
- [Ref. 4] (Sensor Fusion) *S. O. H. Madgwick, “An eﬃcient orientation ﬁlter for inertial and inertial/magnetic sensor arrays,” 2010.*

---

### 2. Drone
The drone model represents a **multi-fidelity quadrotor system** with both **low-fidelity (Lo-Fi)** and **high-fidelity (Hi-Fi)** variants.  
It integrates physical models for **rotor thrust**, **aerodynamic coupling**, and **6-DOF dynamics**, along with cyber modules for **state estimation**, **attitude and position control**, and **networked feedback**.

## Key Features
- Modular Modelica models for *cyber–physical integration*   
- Includes *attack and fault injection hooks* for CPS vulnerability analysis*  
- Compatible with *Python-based orchestration frameworks* (e.g., *CP-Glimpse*, *CyPhER*)  

---

## Applications
- Cyber-physical vulnerability assessment  
- Multi-fidelity simulation studies  
- Formal reachability and risk analysis  

---

### Acknowledgement
TBF.
---

### Citation
If you use this repository, please cite the relevant papers:
```bibtex
@article{<Paper>,
  title   = {TBD},
  author  = {TBD},
  journal = {TBD},
  year    = {TBD}
}