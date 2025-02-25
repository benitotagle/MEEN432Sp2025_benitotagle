Our final model for project two is included in the files above. To run the code, follow the steps below:

1. Open and run the file Project2Week1.m in order to generate the track and vehicle parameters  
2. Open and run the Simulink function Project2FinalModel. This will generate the path the car takes  
3. Open the VisualizeSim.m function. In the command window, input the following code to load the Simulink output variables into the workspace:   
   * X\_sim \= out.X\_sim;    
   * Y\_sim \= out.Y\_sim;    
   * psi\_sim \= out.psi\_sim;    
   * assignin('base', 'X\_sim', X\_sim);  
   * assignin('base', 'Y\_sim', Y\_sim);  
   * assignin('base', 'psi\_sim', psi\_sim);  
   * disp('Extracted Simulink output.');  
4. Run the VisualizeSim.m function. It will display an animation of the car driving the path detailed in the sim  
5. Run the raceStat.m file for the race statistics

The project utilizes a combination of simulink and matlab to simulate a race car going around a track. The code initializes the track and vehicle using Project2Week1.m. The code generates an oval track with waypoints that the car must hit to complete its path. The track has boundaries that the car must stay inside. Lastly, the code establishes vehicle parameters such as mass, wheelbase, and tire cornering stiffness. These values are used by Project2FinalModel.slx, which simulates the car driving around the track for a set amount of time. The simulation contains three main subsystems:

1. Driver block  
   * Determines the steering angle and velocity of the car  
   * Determines the next waypoint using the look-ahead approach specified in lecture  
   * Implements a PI controller to correct lateral error  
2. Lateral dynamics block  
   * Computes the tire slip angles, lateral tire forces, and yaw acceleration (`omega_dot`) using a bicycle model  
   * Updates the yaw rate (`omega`) and lateral velocity (`Vy`) based on the computed forces.  
   * Integrates yaw rate (`omega`) to determine the vehicle’s heading angle (`psi`).  
3. Inertial frame block  
   * Converts frame to global coordinates (X, Y)  
   * Uses the heading angle to transform into velocity components  
   * Integrates velocity to determine the vehicle position along the track in X Y coordinates

Variables such as X and Y are saved in the workspace from the simulink and can be used in our next file, which is VisualizeSim.m. This code combines the initial MATLAB file outputs and the Simulink outputs to display how the car drives around the set track. The code initializes the track boundaries specified in Project2Week1, checks if Simulink data exists, and then animates the vehicle motion by plotting x-y coordinates and rotating the vehicle using psi.

The project underwent significant changes compared to the previous week. Majority of the files were redone due to difficulties implementing the Simulink model from week two and the MATLAB code from week 1\. We also utilized a separate code to visualize the car driving around the track rather than trying to implement that into the code for Project2Week1.m; however, the model doesn’t stay on the track. This can be due to incorrect tuning of PID gains, or due to not looking at future deviations within steering angle correctly. At higher speeds, the car might not be able to 

