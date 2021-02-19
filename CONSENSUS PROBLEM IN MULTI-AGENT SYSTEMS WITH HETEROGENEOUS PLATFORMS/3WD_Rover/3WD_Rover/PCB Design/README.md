This is the PCB design for the physical platform. To edit this file, Altium can be used. Altium is availabe in all CAT Suites in Ingkarni Wardli.

The new PCB version with additional IMU module for GY-521 connection, and three MS switches control the step mode of motor driver. For current case, the MS 1and MS3 set to low, MS2 set to high, so the rover is quarter-step. To change the step mode, the ratio in SetStepperSpeed() has to be adjusted. 

However, the schematic sheet has not been updated yet, and you can keep editing on it