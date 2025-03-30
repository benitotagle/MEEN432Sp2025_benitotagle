To run the code:

* Ensure "Highway test.xlsx" and "Urban Driving.xlsx" are in the same folder as the MATLAB script.  
* Open Simulink model "Project3Week1\_notireslip" to ensure it is accessible.  
* Run the MATLAB script and when prompted:  
  * Enter "yes" for the Highway Test  
  * Enter "no" for the Urban Driving Test

\*The codes are called “week1” because I did not want to mess something up by changing the file name \- Sorry for the confusion\*

This week, we completed the project by implementing the full battery and motor sub-functions. A battery sub-function was created that calculates how much power the battery can realistically provide using its voltage and internal resistance. This was done using the equation Pmax=V24R, which estimates the peak power based on battery limitations. That power was then used to determine the maximum torque the motor could deliver at that moment.

For the motor side, we used a provided torque-speed curve. This curve defines the motor’s maximum torque at different RPM values and was implemented using a 1-D lookup table in Simulink. In the special case of zero or near-zero motor speed, we implemented stall torque by setting the available torque equal to the torque limit from the curve at 0 RPM, allowing the motor to deliver full torque at stall just like a real EV motor.

Since we already had energy consumption calculated in joules from Week 2, we used that value to estimate the state of charge (SOC) over time. The SOC was calculated using the equation SOC=SOCinitial-Econsumed3600\*V\*Capacity, where energy consumed is in joules, voltage is in volts, and battery capacity is in amp-hours. This provided a clean and accurate method to track battery depletion over the duration of the drive cycle.

We also implemented regenerative braking by modifying the driver model. When the control signal (alpha) was negative, the system first attempted to apply negative torque via the motor (regenerative braking), and if additional braking was needed, it used the friction brakes. The regenerative braking torque was limited to 60% of the available motor torque at that moment. This value was based on general electric vehicle data and serves as a rough but reasonable approximation, since regen torque is typically smaller than maximum drive torque in real-world applications.

Overall, this final week completed the energy and torque management aspects of the model. The vehicle now realistically simulates electric motor torque behavior, power constraints from the battery, SOC tracking, and regenerative braking based on researched limitations and real-world torque-speed data.

