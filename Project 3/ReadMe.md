To run the code:

* Ensure "Highway test.xlsx" and "Urban Driving.xlsx" are in the same folder as the MATLAB script.  
* Open Simulink model "Project3Week1\_notireslip" to ensure it is accessible.  
* Run the MATLAB script and when prompted:  
  * Enter "yes" for the Highway Test  
  * Enter "no" for the Urban Driving Test  
* The script will execute the Simulink model and generate plots comparing actual vs. target velocity with ±3 mph bounds.

This program models a PID-controlled vehicle speed tracking system for an EPA drive test. The MATLAB script begins by asking the user to select between a highway or urban driving cycle, then loads the corresponding dataset from an Excel spreadsheet. It extracts time and velocity data, converting the reference speed from mph to m/s. Using interpolation, the script creates a function that dynamically retrieves the reference velocity during simulation. Then vehicle parameters, including wheel radius, mass, wheel and driveline inertia, aerodynamic properties, and maximum torque, are defined along with environmental constants such as air density.

The PID controller gains are specified in the script for easy iteration and tuning. The proportional, integral, and derivative gains are adjusted to achieve an error margin within ±3 mph (ended up being less than 0.39mph). The Simulink model, "Project3Week1\_notireslip," is then executed, where the driver model generates a torque command based on speed error. The vehicle dynamics subsystem converts this torque into wheel force, accounts for aerodynamic drag, and computes acceleration using Acceleration=Torque/Total Mass. The system integrates acceleration to update vehicle speed.

The current driver model assumes both positive and negative torque control, allowing acceleration and braking within a single command. However, it can be easily adjusted to separate powertrain and braking torque based on the error sign for the next few weeks. 

After the simulation, MATLAB extracts the actual velocity and time data from Simulink’s output dataset. The script computes the target velocity at each time step and generates two key plots: one displaying actual velocity over time and another comparing actual velocity against the target velocity with ±3 mph tolerance bands. This visualization ensures that the model stays within acceptable error margins and aids in further PID tuning.

