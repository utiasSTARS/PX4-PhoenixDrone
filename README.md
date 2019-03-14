## Phoeix Drone PX4 Pro Drone Autopilot ##

This repository is built upon the original PX4 autopilot firmware repository: https://github.com/PX4/Firmware. It holds the flight control solution for the Phoeix Drone, with the main applications located in the `src/modules` directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.


**If you use this software in academic work, please cite PX4 and our paper as per the README in the top level repository.**

To compile the Phoeix Drone flight code for hardware (PixRacer), execute

```make nuttx_px4fmu-v4_tailsitter```

To upload the firmware onto the hardware, execute

```make nuttx_px4fmu-v4_tailsitter upload```

To run SITL (Software-in-the-Loop) simulation, execute the following after all the necessary prerequisites has been installed:

```make posix_sitl_tailsitter gazebo_STARS_TS```

Our main contributions are:
- `src/modules/mc_att_control`: Simplified position controller for Phoeix Drone position control
- `src/modules/tc_att_control`: A custom attitude controller for the Phoeix Drone with the control strategy outlined in our paper
- `drivers/tsfmu`: A custom daemon inferring motor rotation speed from ESC sync pulses and regulating PWM signals delivered to ESC to enhance tracking of motor speed

## Flight Hardware
We tested the software on a PixRacer (PX4FMUV4) flight computer. However, since the entire Pixhawk family uses the same ARM architecture, the flight code should be compatiable with all flight hardware supported by PX4. Our custom motor driver `tsfmu` needs to be modified such that the motor pulse signal PIN matches the specific hardware design and attention should also be paid to ensure there is no conflict between timer resources when porting to other platforms.
