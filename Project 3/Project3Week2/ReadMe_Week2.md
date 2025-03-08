To run the code:

* Ensure "Highway test.xlsx" and "Urban Driving.xlsx" are in the same folder as the MATLAB script.  
* Open Simulink model "Project3Week1\_notireslip" to ensure it is accessible.  
* Run the MATLAB script and when prompted:  
  * Enter "yes" for the Highway Test  
  * Enter "no" for the Urban Driving Test

\*The codes are called “week1” because I did not want to mess something up by changing the file name \- Sorry for the confusion\*

This week, we added the powertrain and energy management models. First, we separated brake torque and motor torque in the driver model by modifying the alpha signal with an if statement: if alpha was negative, a brake torque command was sent directly to the vehicle dynamics subfunction; if positive, the system applied motor torque.

Next, we developed a powertrain sub function to model the single-speed reduction. This function multiplies the input torque from the driver subsystem by the gear ratio, passing the result to the vehicle dynamics module. To get angular velocity and acceleration, we divided linear speed and linear acceleration by the tire radius and then applied the same gear reduction to compute motor angular velocity and angular acceleration. Additionally, the effective inertia of the system was adjusted based on the gear ratio, to ensure accurate dynamic behavior in the vehicle model.

An energy management sub function was added to calculate motor power by multiplying motor torque and angular velocity. This power was then integrated over time to determine the total energy consumption in Joules. Finally, the model exported the computed energy data to the workspace, where it was plotted as energy consumption vs. time. The total energy consumed at 765 seconds—the end of the EPA test cycle—represents the final energy value.

